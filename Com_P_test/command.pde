
// 7Bot V2.2 command protocol

/* Register ID */
// ROM
int DEVICE_TYPE_ID = 0;
int VERSION_ID = 1;
int CHIP_ID = 2;
// EEPROM
int EEPROM_ID = 11;
int DEVICE_ID = 11;
int BAUDRATE_ID = 12;
int OFFSET_ID = 13;
// RAM 
int EEPROM_LOCK_ID = 28;
int MOTOR_STATUS_ID = 29;
int EFFECTOR_ID = 30;
int VACUUM_ID = 31;
int SPEED_ID = 32;
int TIME_ID = 39;
int POSITION_ID = 46;
int POSITION_FEEDBACK_FREQ_ID = 53;
int POSITION_FEEDBACK_ID = 54;


/* Register Functions */
void readReg(int addr, int len) {
  int command[] = new int[5];
  command[0] = 0xAA;
  command[1] = 0x77;
  command[2] = 0x03;
  command[3] = addr;
  command[4] = len;
  sendCommand(command);
}

void writeReg(int addr, int len, int data[]) {
  int command[] = new int[len+5];
  command[0] = 0xAA;
  command[1] = 0x77;
  command[2] = 0x06;
  command[3] = addr;
  command[4] = len;
  for (int i=0; i<len; i++)
    command[i+5] = data[i];
  sendCommand(command);
}

void writeReg(int addr, int data) {
  int command[] = new int[6];
  command[0] = 0xAA;
  command[1] = 0x77;
  command[2] = 0x06;
  command[3] = addr;
  command[4] = 1;
  command[5] = data;
  sendCommand(command);
}


/* Get Functions */
void getDeviceCode() {
  readReg(DEVICE_TYPE_ID, 1);
}

void getVersionCode() {
  readReg(VERSION_ID, 1);
}

void getChipID() {
  readReg(CHIP_ID, 6);
}

void getDeviceID() {
  readReg(DEVICE_ID, 1);
}

void getBaudrate() {
  readReg(BAUDRATE_ID, 1);
}

void getOffset() {
  readReg(OFFSET_ID, 7);
}


/* Set Functions */
void setDeviceID(int data) {
  writeReg(DEVICE_ID, data);
}

void setBaudrate(int data) {
  writeReg(BAUDRATE_ID, data);
}

void setOffset(int offset[]) {
  writeReg(OFFSET_ID, 7, offset);
}

// set EEPROM write lock: 0-lock off(enable writting), 1-lock on(protect EEPROM)
void setLock(int data) {
  writeReg(EEPROM_LOCK_ID, data);
}

// set arm force status: 0-protect, 1-servo, 2-forceless
void setStatus(int data) {
  writeReg(MOTOR_STATUS_ID, data);
}

void setEffector(int data) {
  writeReg(EFFECTOR_ID, data);
}

void setVacuum(int data) {
  writeReg(VACUUM_ID, data);
}

// set motion Speed (0~100)
void setSpeed(int Speed) {
  int speeds[] = {Speed, Speed, Speed, Speed, Speed, Speed, Speed};
  writeReg(SPEED_ID, 7, speeds);
}

// set motion Time (Unit:100ms)
// !!!! do not work well yet
void setTime(int Time) {
  int times[] = {Time, Time, Time, Time, Time, Time, Time};
  writeReg(TIME_ID, 7, times);
}

// set motion Position
void setPos(int jointPos[]) {
  writeReg(POSITION_ID, 7, jointPos);
}

void setFeedbackFreq(int data) {
  writeReg(POSITION_FEEDBACK_FREQ_ID, data);
}


// get read register feedback  

//UART
int[] dataBuf = new int[60];
boolean beginFlag = false;
int instruction = 0;
int cnt = 0;
int rxBuf_pre;
int BEGIN_FLAG_0 = 0xAA;
int BEGIN_FLAG_1 = 0x77;
int comReg[] = new int[64];

void serialEvent(Serial myPort) {
  while (myPort.available () > 0) {

    // read data
    int rxBuf = myPort.read();
//    print(rxBuf); print(", ");


    if (!beginFlag)
    {
      beginFlag = (rxBuf_pre == BEGIN_FLAG_0 && rxBuf == BEGIN_FLAG_1) ? true : false; // Beginning Flag
    } else
    {
      if (instruction == 0) instruction = rxBuf;
      else {
        switch (instruction) {

        // Read register
        case 0x03:
          dataBuf[cnt++] = rxBuf;
          // get data length
          if (cnt >= 2)
          {
            if (cnt >= dataBuf[1] + 4)
            {
              beginFlag = false;
              instruction = 0;
              cnt = 0;
              int addr = dataBuf[0];
              int num = dataBuf[1];
              // data read
              int data_read[] = new int[5 + num];
              data_read[0] = 0xAA;
              data_read[1] = 0x77;
              data_read[2] = 0x03;
              data_read[3] = addr;
              data_read[4] = num;

              for (int i = 0; i < num; i++)
              {
                data_read[5 + i] = dataBuf[2 + i];
              }
              // CRC check
              int crc = CRC16_MODBUS(data_read, num + 5);
              int high = crc / 256;
              int low = crc % 256;

              // CRC check pass, write reg
              if (low == dataBuf[num + 2] && high == dataBuf[num + 3])
              {
                // update comReg
                for (int i = addr; i < addr + num; i++)
                {
                  comReg[i] = dataBuf[2 + i - addr];
                  // print(i+", ");
                   if(i >= POSITION_FEEDBACK_ID && i < POSITION_FEEDBACK_ID + SERVO_NUM) {
                     // print(i-POSITION_FEEDBACK_ID+1, ":", comReg[i], ",");
                   }
                }
                // println();

              }
            }
          }
          break;   

        default:
          beginFlag = false;
          instruction = 0;
          cnt = 0;

          break;
        }
      }
    }
    

    rxBuf_pre = rxBuf;
  }
}






//////////////////////////////////////////////////////////////////////////////////////
/* CRC */
//////////////////////////////////////////////////////////////////////////////////////



void sendCommand(int[] Msg) {
  int crc = CRC16_MODBUS(Msg, Msg.length);
  int high = crc/256;
  int low = crc%256;

  for (int i=0; i<Msg.length; i++) {
    myPort.write(Msg[i]);
  }
  
  myPort.write(low);
  myPort.write(high);
  
//  println("CRC send: ", low, high);
}


int CRC16_MODBUS( int _dataMsg[], int _dataLen)  
{  
  int wCRCin = 0xFFFF;  
  int wCPoly = 0x8005;  
  int wbyte = 0;  
  int j = 0;
  while (_dataLen > 0)     
  {  
    _dataLen--;
    wbyte = _dataMsg[j++];  
    wbyte = InvertUint8(wbyte); 
    wCRCin ^= (wbyte << 8);  
    for (int i = 0; i < 8; i++)  
    {  
      if ((wCRCin & 0x8000) != 0)  
        wCRCin = char((wCRCin << 1) ^ wCPoly);  
      else  
      wCRCin = char(wCRCin << 1);
    }
  }  
  wCRCin = InvertUint16(wCRCin);  
  return (wCRCin) ;
}  


int InvertUint16( int dBuf)  
{  
  int i;  
  int tmp;  
  tmp = 0;  
  for (i=0; i< 16; i++)  
  {  
    if ((dBuf & (1 << i)) != 0)  
      tmp|=1<<(15 - i);
  }  
  return tmp;
}



int InvertUint8(int dBuf)  
{  
  int i;  
  int tmp = 0;
  for (i=0; i< 8; i++)  
  {  
    if ((dBuf & (1 << i)) != 0)  
      tmp |= 1<<(7-i);
  }  
  return tmp;
}  
