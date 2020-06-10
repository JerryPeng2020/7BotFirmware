

/*

 V2.2 Serial Port Communication Test (CRC)
 2020-06-08
 
 */

int BAUD_RATE = 115200;
int SERVO_NUM = 7;

import processing.serial.*;
Serial myPort;  // Create object from Serial class

int[] poseFeedback = new int[SERVO_NUM];


void setup() 
{
  myPort = new Serial(this, Serial.list()[2], BAUD_RATE); // UART port
  delay(2000);
    setStatus(2);
  //  sendCommand(testData);
  
  setFeedbackFreq(30);
  
  //int offset[] = {0,0,0,0,0,0,0};
  //setLock(0);
  //setOffset(offset);
  //setLock(1);
}

void draw()
{
  //int pos1[] = {90, 90, 60, 90, 90, 90, 90};
  //setPos(pos1);
  //delay(2000);
  //int pos2[] = {50, 110, 60, 90, 90, 90, 90};
  //setPos(pos2);
  //delay(2000);
}
