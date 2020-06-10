/****************************************************
/* 7Bot class for Arduino platform
/* Author: Jerry Peng
/* Date: 26 April 2019
/* 
/* Version 2.0
/* www.7bot.cc
/*  
/* Description: 
/* 
/*
/***************************************************/

#ifndef _SERVOPROTOCOL_H
#define _SERVOPROTOCOL_H


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

typedef		char			s8;
typedef		unsigned char	u8;
typedef		unsigned short	u16;
typedef		short			s16;
typedef		unsigned long	u32;
typedef		long			s32;

class ServoProtocol {

  public:

    unsigned long int IOTimeOut;//
    HardwareSerial *pSerial;    // serial pointer

    ServoProtocol();

    int setServoTorque(u8 ID, u8 Enable); // set torque status,  Enable: 0-protection,  1-normal servo(default),  2-forceless
    int setServoPos(u8 ID, u16 Position); // set pose
    int setServoTime(u8 ID, u16 Time);    // set motion duration
    int setServoSpeed(u8 ID, u16 Speed);  // set motion speed
    int readServoPos(u8 ID);              // read  pose
    int readServoVoltage(u8 ID);          // read voltage
    int readServoTemper(u8 ID);           // read temperature

  protected:

  private:
    void writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
    int writePos(u8 ID, u16 Position, u16 Time, u16 Speed, u8 Fun);
    void Host2Servo(u8 *DataL, u8* DataH, int Data);
    int	Servo2Host(u8 DataL, u8 DataH);
    int	Ack(u8 ID);       // will respone

    int writeServo(unsigned char *nDat, int nLen);
    int readServo(unsigned char *nDat, int nLen);
    int writeServo(unsigned char bDat);
    void flushServo();  // refresh interface buffer

    int writeByte(u8 ID, u8 MemAddr, u8 bDat);
    int writeWord(u8 ID, u8 MemAddr, u16 wDat);
    int Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen);
    int readByte(u8 ID, u8 MemAddr);
    int readWord(u8 ID, u8 MemAddr);

    int genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);
    int regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);
    void snycWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen);

    int WritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);
    int RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);
    void RegWriteAction();
    void SyncWritePos(u8 ID[], u8 IDN, u16 Position, u16 Time, u16 Speed = 0);


    //register Address

#define P_ID 5
#define P_MAX_TORQUE_L 16
#define P_MAX_TORQUE_H 17
#define P_ALARM_SHUTDOWN 20
#define P_COMPLIANCE_P 21
#define P_TORQUE_ENABLE 40
#define P_GOAL_POSITION_L 42
#define P_GOAL_POSITION_H 43
#define P_GOAL_TIME_L 44
#define P_GOAL_TIME_H 45
#define P_GOAL_SPEED_L 46
#define P_GOAL_SPEED_H 47
#define P_LOCK 48
#define P_PRESENT_POSITION_L 56
#define P_PRESENT_POSITION_H 57
#define P_PRESENT_SPEED_L 58
#define P_PRESENT_SPEED_H 59
#define P_PRESENT_LOAD_L 60
#define P_PRESENT_LOAD_H 61
#define P_PRESENT_VOLTAGE 62
#define P_PRESENT_TEMPERATURE 63
#define P_REGISTERED_INSTRUCTION 64


    //Instruction:
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05

#define INST_SYNC_WRITE 0x83

};
#endif
