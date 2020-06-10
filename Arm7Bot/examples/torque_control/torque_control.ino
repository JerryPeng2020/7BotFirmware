/****************************************************
  /* 7Bot Arduino example
  /* Author: Jerry Peng
  /* Date: 26 April 2019
  /*
  /* Version 2.0
  /* www.7bot.cc
  /*
  /* Description: this example demonstrate how to set 7Bot servo torque
  /*
  /*
  /***************************************************/



#include <Arm7Bot.h>

Arm7Bot arm;

void setup()
{
  // initialize 7Bot
  arm.init();

  // wait for 2 seconds
  delay(2000);

  // change all servos torque status to forceless
  for (int i = 0; i < SERVO_NUM; i++)
    arm.setServoTorque(i, 0);         // status: 0-protection,  1-normal servo(default),  2-forceless
}


void loop()
{

}
