#include "Arm7Bot.h"

/* Constructor */
Arm7Bot::Arm7Bot() {}

// initialize 7Bot
void Arm7Bot::init()
{

  /* Read ROM data to comReg(command register) */
  comReg[DEVICE_TYPE_ID] = DEVICE_TYPE;
  comReg[VERSION_ID] = VERSION;

  // Get chip ID (MAC address), length = 6 bytes
  chipID = ESP.getEfuseMac();
  for (int i = 0; i < 6; i++)
    comReg[CHIP_ID + i] = chipID >> (40 - i * 8);

  /* EEPROM init and Read EEPROM data to comReg */
  //
  // indivudual EEPROM: will set every uninitialized byte(0x00) to 0xFF.
  //                    If they these bytes have been set to a value(include 0x00),
  //                    the stored data will not change anymore.
  if (!EEPROM.begin(EEPROM_SIZE))
  { /*  Serial.println("failed to initialise EEPROM"); */
  }
  else
  { /* Serial.println("Successfully initialise EEPROM"); */
  }

  // read EEPROM data to comReg
  for (int i = 0; i < EEPROM_SIZE; i++)
  {
    comReg[EEPROM_ID + i] = EEPROM.read(i);
  }

  // Device Serial Port Init: read EEPROM(1) & setup serial port
  if (comReg[BAUDRATE_ID] == 0)
    Serial.begin(115200);
  else if (comReg[BAUDRATE_ID] == 3)
    Serial.begin(1000000);
  //  else if(buadrateIndex == 7) Serial.begin(9600);
  else
    Serial.begin(115200);

  /*   
  for (int i = 0; i < EEPROM_SIZE; i++)
  {
    Serial.print(comReg[EEPROM_ID + i]); Serial.print(", ");
  }
  Serial.println();
*/

  // read individual offsets from EEPROM via comReg
  for (int i = 0; i < SERVO_NUM; i++)
  {
    int offset = comReg[OFFSET_ID + i];
    if (offset != 255 && offset != 0)
    {
      if (offset >= 128)
        offset = -1 * (offset - 128);
      individualOffsets[i] = offset;
    }
  }

  /* Init comReg RAM data */
  comReg[EEPROM_LOCK_ID] = 1;  // eepromLock, 0-disable EEPROM write protect lock(can write),  1-enable it
  comReg[MOTOR_STATUS_ID] = 1; // motor status: 0-protect, 1-servo, 2-forceless
  comReg[EFFECTOR_ID] = 0;     // endEffectorMode: 0-vacuum, 1-gripper;
  comReg[VACUUM_ID] = 0;       // vacuum status: 0-release, 1-grip;
  for (int i = 0; i < SERVO_NUM; i++)
    comReg[SPEED_ID + i] = INIT_SPEED;
  for (int i = 0; i < SERVO_NUM; i++)
    comReg[TIME_ID + i] = INIT_TIME;
  for (int i = 0; i < SERVO_NUM; i++)
    comReg[POSITION_ID + i] = initPos[i];
  for (int i = 0; i < SERVO_NUM; i++)
    comReg[POSITION_FEEDBACK_ID + i] = 0; // joint angle (read only to master)
  comReg[POSITION_FEEDBACK_FREQ_ID] = 0;  // pose feedback frequency

  /* Init 7Bot Components */
  // init Servo Serial Port
  Serial2.begin(1000000);
  pSerial = &Serial2;
  // init input buttons
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);
  // init buzzer pin
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  // init pump & valve pins
  pinMode(PUMP, OUTPUT);
  digitalWrite(PUMP, LOW);
  pinMode(VALVE, OUTPUT);
  digitalWrite(VALVE, LOW);

  for (int i = 0; i < SERVO_NUM; i++)
  {
    setServoTorque(i, 1);
    setServoTime(i, WAKEUP_TIME);
    setJointAngle(i, initPos[i]);
  }
  wakeupBeginTime = millis();
}

// scan and receive command from USB-UART port
void Arm7Bot::receiveCommand()
{
  // after 7Bot wakeup, reset its motion speed
  if (!wakeupFlage)
  {
    if (millis() - wakeupBeginTime > WAKEUP_TIME)
    {
      wakeupFlage = true; // 7Bot already waken up
      for (int i = 0; i < SERVO_NUM; i++)
      {
        setServoSpeed(i, INIT_SPEED);
      }
    }
  }

  /*  recive commands（V2.2) */
  while (Serial.available() > 0)
  {
    // read data
    int rxBuf = Serial.read();
    if (!beginFlag)
    {
      beginFlag = (rxBuf_pre == BEGIN_FLAG_0 && rxBuf == BEGIN_FLAG_1) ? true : false; // Beginning Flag
    }
    else
    {
      if (instruction == 0)
        instruction = rxBuf;
      else
      {
        switch (instruction)
        {

        // Read register
        case 0x03:
          dataBuf[cnt++] = rxBuf;
          if (cnt >= 4)
          {
            beginFlag = false;
            instruction = 0;
            cnt = 0;
            uint8_t data_read[] = {0xAA, 0x77, 0x03, dataBuf[0], dataBuf[1]};
            uint16_t crc = CRC16_MODBUS(data_read, 5);
            int high = crc / 256;
            int low = crc % 256;
            int addr = dataBuf[0]; // begin address
            int num = dataBuf[1];  // reg number

            // Serial.write(num);   // num debug

            // CRC check pass, send reg data
            if (low == dataBuf[2] && high == dataBuf[3])
            {
              // prepare data
              uint8_t data_send[5 + num];
              data_send[0] = 0xAA;
              data_send[1] = 0x77;
              data_send[2] = 0x03;
              data_send[3] = addr;
              data_send[4] = num;
              for (int i = 0; i < num; i++)
              {
                data_send[5 + i] = comReg[addr + i];
              }

              // send data with CRC inside
              sendData(data_send, num + 5);
            }
          }
          break;

        // write reg
        case 0x6:
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
              uint8_t data_read[5 + num];
              data_read[0] = 0xAA;
              data_read[1] = 0x77;
              data_read[2] = 0x06;
              data_read[3] = addr;
              data_read[4] = num;

              for (int i = 0; i < num; i++)
              {
                data_read[5 + i] = dataBuf[2 + i];
              }
              // CRC check
              uint16_t crc = CRC16_MODBUS(data_read, num + 5);
              int high = crc / 256;
              int low = crc % 256;

              // CRC check pass, write reg
              if (low == dataBuf[num + 2] && high == dataBuf[num + 3])
              {
                // write comReg
                for (int i = addr; i < addr + num; i++)
                {
                  // update writeable addr data
                  if (i >= EEPROM_ID && i <= POSITION_FEEDBACK_FREQ_ID)
                    comReg[i] = dataBuf[2 + i - addr];
                  // immediate execute command
                  if (i == MOTOR_STATUS_ID)
                    for (int j = 0; j < SERVO_NUM; j++)
                      setServoTorque(j, comReg[MOTOR_STATUS_ID]);
                  if (i == VACUUM_ID)
                    vacuum(comReg[VACUUM_ID]);
                  for (int j = 0; j < SERVO_NUM; j++)
                  { // can be optimized
                    if (i == TIME_ID + j)
                      setServoTime(j, comReg[TIME_ID + j] * 100); // Time is less important than speed Setting
                    if (i == SPEED_ID + j)
                      setServoSpeed(j, comReg[SPEED_ID + j] * 10);
                    if (i == POSITION_ID + j)
                      setJointAngle(j, comReg[POSITION_ID + j]);
                  }
                }

                // if need write EEPROM, check lock frist and then write EEPROM
                if (comReg[EEPROM_LOCK_ID] == 0)
                {
                  for (int i = EEPROM_ID; i < EEPROM_ID + EEPROM_SIZE; i++)
                    if (i >= addr && i < addr + num)
                      EEPROM.write(i - EEPROM_ID, comReg[i]);
                }
                EEPROM.commit();
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

// send feedback to USB-UART port
void Arm7Bot::poseFeedback()
{
  // prepare data
  uint8_t data_send[5 + SERVO_NUM];
  data_send[0] = 0xAA;
  data_send[1] = 0x77;
  data_send[2] = 0x03;
  data_send[3] = POSITION_FEEDBACK_ID;
  data_send[4] = SERVO_NUM;

  for (int i = 0; i < SERVO_NUM; i++)
  {
    int posRead = readServoPos(i);                                                         // read potentiometer 10 bits raw data
    float posTmp = float(posRead - sysOffsets[i]) * vToD[i] + float(individualOffsets[i]); // convert raw data to angular data
    // range limit to 0~180, can be extend a little bit later!!！
    if (posTmp < 0)
      posTmp = 0;
    else if (posTmp > 180)
      posTmp = 180;
    if (reverse[i])
      comReg[POSITION_FEEDBACK_ID + i] = 180 - round(posTmp);
    else
      comReg[POSITION_FEEDBACK_ID + i] = round(posTmp);
    data_send[5 + i] = comReg[POSITION_FEEDBACK_ID + i];
  }
  sendData(data_send, SERVO_NUM + 5);
}

// vacuum control
void Arm7Bot::vacuum(bool status)
{
  if (status)
  {
    digitalWrite(PUMP, HIGH);
    digitalWrite(VALVE, LOW);
  }
  else
  {
    digitalWrite(PUMP, LOW);
    digitalWrite(VALVE, HIGH);
  }
}

// Joint angle set, involve in servo offsets compensation
// ID: 0~6
void Arm7Bot::setJointAngle(u8 ID, int angle)
{
  newPos[ID] = false; // valid command detection
  posG[ID] = angle;
  if (posG_pre[ID] != posG[ID])
    newPos[ID] = true;
  posG_pre[ID] = posG[ID];
  // only sent command to servo when new pose received
  if (newPos[ID])
  {
    float posTmp = float(posG[ID]);
    if (reverse[ID])
      posTmp = 180.0 - float(posG[ID]);
    servoPos[ID] = round(posTmp * dToV[ID] + float(sysOffsets[ID]) - float(individualOffsets[ID]) * dToV[ID]); // Unit: value
    if (servoPos[ID] < 0)
      servoPos[ID] = 0;
    else if (servoPos[ID] > 1023)
      servoPos[ID] = 1023;
    setServoPos(ID, servoPos[ID]);
  }
}

// Be care!!!! sizeof(data) do not work properly, maybe because of
// the uint8_t data is not treat correctly in sizeof().
void Arm7Bot::sendData(uint8_t *data, int len)
{
  uint16_t crc = CRC16_MODBUS(data, len);
  int high = crc / 256;
  int low = crc % 256;
  for (int i = 0; i < len; i++)
    Serial.write(data[i]);
  Serial.write(low);
  Serial.write(high);
}

uint16_t Arm7Bot::CRC16_MODBUS(uint8_t *data, int len)
{
  uint16_t wCRCin = 0xFFFF;
  uint16_t wCPoly = 0x8005;
  uint8_t wbyte = 0;
  int j = 0;
  while (len > 0)
  {
    len--;
    wbyte = data[j++];
    wbyte = InvertUint8(wbyte);
    wCRCin ^= (wbyte << 8);
    for (int i = 0; i < 8; i++)
    {
      if ((wCRCin & 0x8000) != 0)
        wCRCin = uint16_t((wCRCin << 1) ^ wCPoly);
      else
        wCRCin = uint16_t(wCRCin << 1);
    }
  }
  wCRCin = InvertUint16(wCRCin);
  return (wCRCin);
}

uint8_t Arm7Bot::InvertUint8(uint8_t dBuf)
{
  int i;
  uint8_t tmp = 0;
  for (i = 0; i < 8; i++)
  {
    if ((dBuf & (1 << i)) != 0)
      tmp |= 1 << (7 - i);
  }
  return tmp;
}

uint16_t Arm7Bot::InvertUint16(uint16_t dBuf)
{
  int i;
  uint16_t tmp;
  tmp = 0;
  for (i = 0; i < 16; i++)
  {
    if ((dBuf & (1 << i)) != 0)
      tmp |= 1 << (15 - i);
  }
  return tmp;
}