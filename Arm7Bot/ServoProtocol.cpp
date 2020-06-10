/*
   
*/

#include <stddef.h>
#include "ServoProtocol.h"



ServoProtocol::ServoProtocol()
{
  IOTimeOut = 2;
  pSerial = NULL;

}


void ServoProtocol::Host2Servo(u8 *DataL, u8* DataH, int Data)
{
  *DataL = (Data >> 8);
  *DataH = (Data & 0xff);
}


int ServoProtocol::Servo2Host(u8 DataL, u8 DataH)
{
  int Data;
  Data = DataL;
  Data <<= 8;
  Data |= DataH;
  return Data;
}

void ServoProtocol::writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun)
{
  u8 msgLen = 2;
  u8 bBuf[6];
  u8 CheckSum = 0;
  bBuf[0] = 0xff;
  bBuf[1] = 0xff;
  bBuf[2] = ID;
  bBuf[4] = Fun;
  if (nDat) {
    msgLen += nLen + 1;
    bBuf[3] = msgLen;
    bBuf[5] = MemAddr;
    writeServo(bBuf, 6);

  } else {
    bBuf[3] = msgLen;
    writeServo(bBuf, 5);
  }
  CheckSum = ID + msgLen + Fun + MemAddr;
  u8 i = 0;
  if (nDat) {
    for (i = 0; i < nLen; i++) {
      CheckSum += nDat[i];
    }
  }
  writeServo(nDat, nLen);
  writeServo(~CheckSum);
}

// Gerneral Write
//
int ServoProtocol::genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
  flushServo();
  writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
  return Ack(ID);
}

//
//
int ServoProtocol::regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
  flushServo();
  writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
  return Ack(ID);
}

//
//
void ServoProtocol::snycWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen)
{
  u8 mesLen = ((nLen + 1) * IDN + 4);
  u8 Sum = 0;
  u8 bBuf[7];
  bBuf[0] = 0xff;
  bBuf[1] = 0xff;
  bBuf[2] = 0xfe;
  bBuf[3] = mesLen;
  bBuf[4] = INST_SYNC_WRITE;
  bBuf[5] = MemAddr;
  bBuf[6] = nLen;
  writeServo(bBuf, 7);

  Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
  u8 i, j;
  for (i = 0; i < IDN; i++) {
    writeServo(ID[i]);
    writeServo(nDat, nLen);
    Sum += ID[i];
    for (j = 0; j < nLen; j++) {
      Sum += nDat[j];
    }
  }
  writeServo(~Sum);
}

int ServoProtocol::writeByte(u8 ID, u8 MemAddr, u8 bDat)
{
  flushServo();
  writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
  return Ack(ID);
}

int ServoProtocol::writeWord(u8 ID, u8 MemAddr, u16 wDat)
{
  flushServo();
  u8 buf[2];
  Host2Servo(buf + 0, buf + 1, wDat);
  writeBuf(ID, MemAddr, buf, 2, INST_WRITE);
  return Ack(ID);
}

// Enable 0-forceless,  1-normal servo(default),  2-protection
int ServoProtocol::setServoTorque(u8 ID, u8 Enable)
{
  return writeByte(ID, P_TORQUE_ENABLE, Enable);
}

int ServoProtocol::writePos(u8 ID, u16 Position, u16 Time, u16 Speed, u8 Fun)
{
  flushServo();
  u8 buf[6];
  Host2Servo(buf + 0, buf + 1, Position);
  Host2Servo(buf + 2, buf + 3, Time);
  Host2Servo(buf + 4, buf + 5, Speed);
  writeBuf(ID, P_GOAL_POSITION_L, buf, 6, Fun);
  return Ack(ID);
}

int ServoProtocol::setServoPos(u8 ID, u16 Position)
{
  flushServo();
  u8 buf[2];
  Host2Servo(buf + 0, buf + 1, Position);
  writeBuf(ID, P_GOAL_POSITION_L, buf, 2, INST_WRITE);
  return Ack(ID);
}

int ServoProtocol::setServoTime(u8 ID, u16 Time)
{
  flushServo();
  u8 buf[2];
  Host2Servo(buf + 0, buf + 1, Time);
  writeBuf(ID, P_GOAL_TIME_L, buf, 2, INST_WRITE);
  return Ack(ID);
}

int ServoProtocol::setServoSpeed(u8 ID, u16 Speed)
{
  flushServo();
  u8 buf[2];
  Host2Servo(buf + 0, buf + 1, Speed);
  writeBuf(ID, P_GOAL_SPEED_L, buf, 2, INST_WRITE);
  return Ack(ID);
}


//
int ServoProtocol::WritePos(u8 ID, u16 Position, u16 Time, u16 Speed)
{
  return writePos(ID, Position, Time, Speed, INST_WRITE);
}


//
int ServoProtocol::RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed)
{
  return writePos(ID, Position, Time, Speed, INST_REG_WRITE);
}

void ServoProtocol::RegWriteAction()
{
  writeBuf(0xfe, 0, NULL, 0, INST_ACTION);
}


//
void ServoProtocol::SyncWritePos(u8 ID[], u8 IDN, u16 Position, u16 Time, u16 Speed)
{
  u8 buf[6];
  Host2Servo(buf + 0, buf + 1, Position);
  Host2Servo(buf + 2, buf + 3, Time);
  Host2Servo(buf + 4, buf + 5, Speed);
  snycWrite(ID, IDN, P_GOAL_POSITION_L, buf, 6);
}


//
int ServoProtocol::Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen)
{
  flushServo();
  writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
  u8 bBuf[5];
  if (readServo(bBuf, 5) != 5) {
    return 0;
  }
  int Size = readServo(nData, nLen);
  if (readServo(bBuf, 1)) {
    return Size;
  }
  return 0;
}


int ServoProtocol::readByte(u8 ID, u8 MemAddr)
{
  u8 bDat;
  int Size = Read(ID, MemAddr, &bDat, 1);
  if (Size != 1) {
    return -1;
  } else {
    return bDat;
  }
}


int ServoProtocol::readWord(u8 ID, u8 MemAddr)
{
  u8 nDat[2];
  int Size;
  u16 wDat;
  Size = Read(ID, MemAddr, nDat, 2);
  if (Size != 2)
    return -1;
  wDat = Servo2Host(nDat[0], nDat[1]);
  return wDat;
}

//
int ServoProtocol::readServoPos(u8 ID)
{
  return readWord(ID, P_PRESENT_POSITION_L);
}


//
int ServoProtocol::readServoVoltage(u8 ID)
{
  return readByte(ID, P_PRESENT_VOLTAGE);
}

//
int ServoProtocol::readServoTemper(u8 ID)
{
  return readByte(ID, P_PRESENT_TEMPERATURE);
}


//
int	ServoProtocol::Ack(u8 ID)
{
  if (ID != 0xfe) {
    u8 buf[6];
    u8 Size = readServo(buf, 6);
    if (Size != 6) {
      return 0;
    }
  }
  return 1;
}




int ServoProtocol::readServo(unsigned char *nDat, int nLen)
{
  int Size = 0;
  int ComData;
  unsigned long t_begin = millis();
  unsigned long t_user;
  while(1){
    ComData = pSerial->read();
    if(ComData!=-1){
      if(nDat){
        nDat[Size] = ComData;
      }
      Size++;
      t_begin = millis();
    }
    if(Size>=nLen){
      break;
    }
    t_user = millis() - t_begin;
    if(t_user>IOTimeOut){
      break;
    }
  }
  return Size;
}

int ServoProtocol::writeServo(unsigned char *nDat, int nLen)
{
  if(nDat==NULL){
    return 0;
  }
  return pSerial->write(nDat, nLen);
}

int ServoProtocol::writeServo(unsigned char bDat)
{
  return pSerial->write(&bDat, 1);
}

void ServoProtocol::flushServo()
{
  while(pSerial->read()!=-1);
}
