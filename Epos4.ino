/*******************************************************************************************************/
/*       Maxon Epos 4 communication for Arduino                                                     
         Apr 21 2018 Denis Gayraud  dgayraud@club-internet.fr                                        
         EPOS4 doumentation:                                                                         
 http://academy.maxonjapan.co.jp/wp-content/uploads/manual/epos4/EPOS4-Communication-Guide-En.pdf    
 http://academy.maxonjapan.co.jp/wp-content/uploads/manual/epos4/EPOS4-Firmware-Specification-En.pdf
 
 Copyright 2018  Denis Gayraud (dgayraud[at] club-internet [dot] fr)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.*/
/*******************************************************************************************************/

#include <SoftwareSerial.h>  // For Arduino uno
SoftwareSerial SerialEpcos(2, 3); // RX, TX for  arduino uno
//#define SerialEpcos  Serial1 // pro micro or leonardo 
#define ReceiveTimeOut 3000 // Maximum for waiting answer from EPOS4 in milli second
#define MaxLenFrame 10  // max number of words in frame

byte DataRead[264];
byte ReadOpcode;
byte len;
int pDataRead;
word rxCRC;
unsigned long CommErrorCode;
enum read_status {
  RX_DLE,
  RX_STX,
  RX_OPCODE,
  RX_LEN,
  RX_DATA,
  RX_CRC_LOW,
  RX_CRC_HIGH,
  RX_DONE,
  ESC_OPCODE,
  ESC_LEN,
  ESC_DATA,
  ESC_CRC_LOW,
  ESC_CRC_HIGH
}read_st;


/*****************************************************************************************************/
/*  CalcFieldCRC : code from EPOS4-Communication-Guide-En.pdf                                        */
/*****************************************************************************************************/
word CalcFieldCRC(word* pDataArray, word numberOfints)
{
  word shifter, c;
  word carry;
  word CRC = 0;
  //Calculate pDataArray Word by Word
  while(numberOfints--)
    {
      shifter = 0x8000;          //Initialize BitX to Bit15 
      c = *pDataArray++;         //Copy next DataWord to c
      do
      {
        carry = CRC & 0x8000;    //Check if Bit15 of CRC is set 
        CRC <<= 1;               //CRC = CRC * 2   
        if(c & shifter) CRC++;   //CRC = CRC + 1, if BitX is set in c 
        if(carry) CRC ^= 0x1021; //CRC = CRC XOR G(x), if carry is true 
        shifter >>= 1;           //Set BitX to next lower Bit, shifter = shifter/2
      } while(shifter);
    }
  return CRC;
}

inline void SerialEpcosStuffing(byte BYTE)
{
  if (BYTE==0x90) SerialEpcos.write(BYTE);
  SerialEpcos.write(BYTE);
}

void SendFrame(byte OpCode,word* pDataArray,byte numberOfwords)
{
  word CRC;
  word pDataCRC[MaxLenFrame+2];
  pDataCRC[0] = OpCode | (numberOfwords<<8);
  for (int i=0; i<numberOfwords ; i++)
  {
    pDataCRC[i+1]=pDataArray[i];
  }
  pDataCRC[numberOfwords+1]=0x0000;
  CRC=CalcFieldCRC(pDataCRC, (word)(numberOfwords+2));
  SerialEpcos.write(0x90);  // DLE=Data Link Escape 
  SerialEpcos.write(0x02);  // STX=Start of Text
  SerialEpcosStuffing(OpCode);
  SerialEpcosStuffing(numberOfwords);
  for (int i=0; i<numberOfwords ; i++)
  {
    SerialEpcosStuffing(lowByte(pDataArray[i]));
    SerialEpcosStuffing(highByte(pDataArray[i]));
  }
  SerialEpcosStuffing(lowByte(CRC));
  SerialEpcosStuffing(highByte(CRC));
}

bool WriteObject(word Index, byte SubIndex,word* pArray)
{
  word data[4];
  data[0]= 0x01 | (lowByte(Index)<<8);  // NodeId=1
  data[1]= highByte(Index)|(((word)SubIndex )<< 8);
  data[2]=pArray[0];
  data[3]=pArray[1];
  SendFrame(0x68,data,(byte)4);
  return ReadFrame();
}

bool ReadObject(word Index, byte SubIndex)
{
  word data[2];
  data[0]= 0x01 | (lowByte(Index)<<8);  // NodeId=1
  data[1]= highByte(Index)|(((word)SubIndex )<< 8);
  SendFrame(0x60,data,(byte)2);
  return ReadFrame();
}

// For debuging:
void print_rcv_data()
{
      //todo check CRC and print data
    Serial. print("OPcode: ");
    Serial. print(ReadOpcode,HEX);
    Serial. print(" len: ");
    Serial. print(len);
    Serial. print(" Data:");
    for (int i=0; i<2*len ; i++)
    {
       Serial.print(DataRead[i],HEX);
       Serial.print(" , ");
    }
    Serial.print("CRC: ");   
    Serial.println(rxCRC, HEX); 
}

bool ReadFrame()
{
  int incomingByte = 0;   // for incoming SerialEpcos data
  unsigned long timer=millis();
  read_st=RX_DLE;
  word pDataCRC[MaxLenFrame+2];
  CommErrorCode=0;
  while ((read_st!=RX_DONE) and (millis()-timer < ReceiveTimeOut))
  {
    if (SerialEpcos.available() > 0)
    {
      incomingByte = SerialEpcos.read();
//      Serial.print("Read: ");
//      Serial.println(incomingByte, HEX); 
//      Serial.print("read_st: ");
//      Serial.println(read_st); 
      
      switch (read_st)
      {
        case RX_DLE:
           if (incomingByte==0x90) read_st=RX_STX;
           pDataRead=0;
           len=0;
           break;
        case RX_STX:
           if (incomingByte==0x02) read_st=RX_OPCODE;
           else read_st=RX_DLE;
           break;
        case RX_OPCODE:
           if ( incomingByte == 0x90)
           {
            read_st=ESC_OPCODE;
            break;
           }
           else
           {
             ReadOpcode=incomingByte;
             read_st=RX_LEN;
           }
           break;
        case RX_LEN:
           len=incomingByte;
           if ( incomingByte == 0x90)
           {
            read_st=ESC_LEN;
            break;
           }
           else read_st=RX_DATA;
           break;
        case RX_DATA:
           if ( incomingByte == 0x90)
           {
            read_st=ESC_DATA;
            break;
           }
           else
           {
             DataRead[pDataRead]=incomingByte;
             pDataRead++;
           }
           if ( pDataRead== (2*len) ) read_st=RX_CRC_LOW;
           break;
        case RX_CRC_LOW:
           rxCRC=incomingByte; 
           if ( incomingByte == 0x90) read_st=ESC_CRC_LOW;
           else read_st=RX_CRC_HIGH;
           break;
        case RX_CRC_HIGH:
           rxCRC +=incomingByte<<8;
           if ( incomingByte == 0x90) read_st=ESC_CRC_HIGH;
           else read_st=RX_DONE;
           break; 
        case ESC_OPCODE:
           if (incomingByte== 0x02)
           {
             read_st=RX_OPCODE;
             break;
           }
           else if (incomingByte== 0x90)
           {
             ReadOpcode=incomingByte;
             read_st=RX_LEN;
             break;
           }
           else Serial.println ("Protocol error: single DLE");   
           break;
        case ESC_LEN:
           if (incomingByte== 0x90)
           {
             read_st=RX_DATA;
             break;
           }
           if (incomingByte== 0x02)
           {
             read_st=RX_OPCODE;
             break;
           }
           Serial.println ("Protocol error: Escape len error");
           break;
        case ESC_DATA:
           if ( incomingByte == 0x90)
           {
             DataRead[pDataRead]=incomingByte;
             pDataRead++;
             read_st=RX_DATA;
             if ( pDataRead== (2*len) ) read_st=RX_CRC_LOW;
             break;
           }
           if (incomingByte== 0x02)
           {
             read_st=RX_OPCODE;
             break;
           }
           Serial.println ("Protocol error: Escape data error");
           break;
           
        case ESC_CRC_LOW:
           if ( incomingByte == 0x90) read_st=RX_CRC_HIGH;
           else if (incomingByte== 0x02) read_st=RX_OPCODE;
           else Serial.println ("Protocol error: Escape crc l error");
           break;
        case ESC_CRC_HIGH:
           if ( incomingByte == 0x90) read_st=RX_DONE;
           else if (incomingByte== 0x02) read_st=RX_OPCODE;
           else Serial.println ("Protocol error: Escape crc h error");
           break;
        default:
           break;
      }
    }
  }
// Check Time out:
  if (millis()-timer >= ReceiveTimeOut)
  {
    Serial.println("Serial Time out");
    return false;
  }
// check CRC:
  pDataCRC[0] = ReadOpcode | ((word)len)<<8;
  for (int i=0 ;  i< len ;i++ )
  {
    pDataCRC[i+1]= DataRead[2*i] | (((word)DataRead[2*i+1])<<8);
  }
  pDataCRC[len+1]=0x0000;
  if (CalcFieldCRC(pDataCRC, (word)(len+2))!= rxCRC)
  {
    Serial.println("Error CRC");
    return false;
  }
//  Serial.print("RCV CRC: ");Serial.println (CalcFieldCRC(pDataCRC, (word)(len+2)),HEX);
// Check communication error code
  CommErrorCode= DataRead[0] | (((word)DataRead[1])<<8) | (((unsigned long)DataRead[2])<<16) | (((unsigned long)DataRead[3])<<24);
  if (CommErrorCode!=0)
  {
    Serial.print( "Communication Error Code:");
    Serial.println(CommErrorCode,HEX);
    return false;
  }
//  print_rcv_data(); 
return true;
}


word ReadStatusWord()
{
  word statusWord;
  ReadObject(0x6041,0);
  statusWord = DataRead[4] + (((word) DataRead[5])<<8);
  Serial.print("Statusword: ");
  Serial.println(statusWord,HEX);
  return (statusWord);
}

/* This code enable driver, run motor for 5s and stop.  
 Motor configuration must be setup with EPOS studio and saved in controller*/        
void setup()
{
  word data[4];
    // start serial port at 9600 bps and wait for port to open:
  Serial.begin(9600);
  while (!Serial) ; // wait for serial port to connect. Needed for native USB port only
  SerialEpcos.begin(9600);
  
  // send 1000 in profile acc:
  //Serial.println("send 1000 in profile acc");
  data[0]=1000;
  data[1]=0x0000;  
  WriteObject(0x6083,0,data);
  // send 1000 in profile  dec:
  //Serial.println("send 1000 in profile dec");
  WriteObject(0x6084,0,data);
  // Send 1000 in taget velocity
  //Serial.println("send 1000 in taget velocity");
  data[0]=1000;
  WriteObject (0x60FF,0,data);
  //Todo: wait for ready to switch on
  // if not ready to switch on : send shutdown command
  if (!(ReadStatusWord() & 0x01))
  {
   Serial.println("Write 06 Controlword");
   data[0]=0x0006;
   WriteObject (0x6040,0,data);    
  }
  // if Ready to switch on : enable
  if (ReadStatusWord() & 0x01) // or 0x20 or 0x21?
  {
   Serial.println("Write 0F in  Controlword");
   data[0]=0x000F;
   WriteObject (0x6040,0,data);    
  }
  ReadStatusWord();
  // Start operation command:
  Serial.println("Write FF in Controlword");
  data[0]=0x00FF;
  WriteObject (0x6040,0,data);
  ReadStatusWord();
  
  delay(5000);
   //Send Halt
  data[0]=0x01FF;
  WriteObject (0x6040,0,data);
  ReadStatusWord();

}

void loop()
{
  // doc example
  //ReadObject(0x30B0,0);
  
}

