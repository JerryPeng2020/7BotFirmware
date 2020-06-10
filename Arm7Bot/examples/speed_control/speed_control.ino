/****************************************************
  /* 7Bot Arduino example
  /* Author: Jerry Peng
  /* Date: 26 April 2019
  /*
  /* Version 2.0
  /* www.7bot.cc
  /*
  /* Description: this example demonstrate how to set 7Bot motion speed
  /*
  /*
  /***************************************************/



#include <Arm7Bot.h>

Arm7Bot arm;

void setup()
{
  // initialize 7Bot
  arm.init();
}


void loop()
{
  arm.setServoSpeed(0, 300);     // set servo(ID=0), motion speed to value 300
  
  arm.setServoPos(0, 300);  // set servo(ID=0), pose to 300
  delay(2000);
  arm.setServoPos(0, 700);  // set servo(ID=0), pose to 700
  delay(2000);


  arm.setServoSpeed(0, 700);     // set servo(ID=0), motion speed to value 700
  
  arm.setServoPos(0, 300);  // set servo(ID=0), pose to 300
  delay(1000);
  arm.setServoPos(0, 700);  // set servo(ID=0), pose to 700
  delay(1000);
  
}
