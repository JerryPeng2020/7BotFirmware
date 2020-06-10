/****************************************************
/* 7Bot class for Arduino platform
/* Author: Jerry Peng
/* Date: June 9th, 2020
/* 
/* Version 2.2
/* www.7bot.cc
/*  
/* Description: 
    1. Change comunication command to register read and write version.
    2. Reduce pose control commands from 2 bytes to 1 byte. 
    3. Add CRC to ensure command transmission.

Changes:




需要优化的地方：
（一）硬件
1. 加机械限位，防止joint[0]，套圈。 同时说明书中加注意事项说明，通过servo[0]开孔看线判断有没有套圈

（二）固件：
1. 增加软件握手check机制
2. WiFi/BT 功能及配置
3. 增加离线按键操作功能。用eeprom class或write
4. 增加IK
5. 将舵机运动范围适当扩大？？？ 暂缓，除非建模上大改

/***************************************************/

#ifndef _ARM7BOT_H
#define _ARM7BOT_H

#include "ServoProtocol.h"
#include "EEPROM.h"

#define SERVO_NUM 7

/* Hardware Pins */
#define PUMP 32     // pump
#define VALVE 33    // valve
#define BUZZER 25   // buzzer

#define BUTTON1 14  // button1
#define BUTTON2 27  // button2
#define BUTTON3 26  // button3

/* System Parameters */
#define WAKEUP_TIME 2000  // 7Bot wake up time,  Unit: ms
#define INIT_SPEED 600  // after 7Bot wake up reset normal motion speed
#define INIT_TIME 1000  // after 7Bot wake up reset normal motion time

// Device ROM Data
#define DEVICE_TYPE 7
#define VERSION 22   // version 2.2

// EEPROM 
#define EEPROM_SIZE 9

// Command Header
#define BEGIN_FLAG_0 0xAA
#define BEGIN_FLAG_1 0x77

/* Register ID */
// ROM
#define DEVICE_TYPE_ID 0
#define VERSION_ID 1
#define CHIP_ID 2
// EEPROM
#define EEPROM_ID 11
#define DEVICE_ID 11
#define BAUDRATE_ID 12
#define OFFSET_ID 13
// RAM 
#define EEPROM_LOCK_ID 28
#define MOTOR_STATUS_ID 29
#define EFFECTOR_ID 30
#define VACUUM_ID 31
#define SPEED_ID 32
#define TIME_ID 39
#define POSITION_ID 46
#define POSITION_FEEDBACK_FREQ_ID 53
#define POSITION_FEEDBACK_ID 54



class Arm7Bot : public ServoProtocol {

  public:

    boolean wakeupFlage = false;
    int wakeupBeginTime = 0;

    uint64_t chipID;
    int posG[SERVO_NUM];
    int posG_pre[SERVO_NUM];
    int servoPos[SERVO_NUM];

    float posD[SERVO_NUM];

    boolean newPos[SERVO_NUM];

    const boolean reverse[SERVO_NUM] = {false, true, true, true, true, true, false};
    //degree to value, for servo ID(0~3): 850(value) equal 180(degree)  for servo ID(4~6): 625(value) equal 180(degree)
    const float dToV[SERVO_NUM] = {4.722, 4.722, 4.722, 4.722, 3.472, 3.472, 3.472}; // degree to value 
    const float vToD[SERVO_NUM] = {0.212, 0.212, 0.212, 0.212, 0.288, 0.288, 0.288}; // value to degree


    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* System Offsets */
    /* Mechanical Structure  (servoPos VS [Geometrical Angle]) */
    // offset = pos - angle*dToV;
    // !!!! actually, angle here means: 90(degree) angle should be 90*dToV, 425 for servo ID(0~3), 312 for servo ID(4~6)
    // Unit: Raw Value
    const int sysOffsets[SERVO_NUM] = {86, 20, 141, 86, 199, 199, 730}; 

    // offsets of induvidual robotic arm, Unit:Degree
    int individualOffsets[SERVO_NUM] = {0, 0, 0, 0, 0, 0, 0}; 

    //standard calibration pose (Structure Angle) Unit: Degree
    int initPos[SERVO_NUM] = {90,  90,  65,  90,  90,  90, 80};

    void setJointAngle(u8 ID, int angle);

    // UART
    bool beginFlag = false;
    bool haveInstruction = false;
    int instruction = 0;
    int cnt = 0;
    int dataBuf[30];
    int rxBuf_pre;

    // CRC
    void sendData(uint8_t *data, int len);
    uint16_t CRC16_MODBUS(uint8_t *data, int len);
    uint8_t InvertUint8( uint8_t dBuf);
    uint16_t InvertUint16( uint16_t dBuf);

    // command register: Verion 2.2
    uint8_t comReg[64];


    Arm7Bot();
    void init();
    void receiveCommand();
    void poseFeedback(); // motor pose feedback
    void vacuum(bool status);   // true: open,  false:close

};



#endif
