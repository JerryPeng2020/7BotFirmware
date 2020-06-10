/****************************************************
  /* 7Bot Arduino example
  /* Author: Jerry Peng
  /* Date: 26 April 2019
  /*
  /* Version 2.0
  /* www.7bot.cc
  /*
  /* Description: this example demonstrate how to read 7Bot servo pose， Buad Rate:115200
  /*
  /*
  /***************************************************/



#include <Arm7Bot.h>

Arm7Bot arm;

void setup()
{
  // initialize 7Bot
  arm.init();

  delay(1000);
  arm.setServoTorque(0, 2);  // set servo(ID=0) to forceless torque status
}


void loop()
{
  
  int pose = arm.readServoPos(0);    // read servo(ID=0) pose
  Serial.println(pose);        // print pose value to Arduino Serial Monitor
  delay(100);
  
}
