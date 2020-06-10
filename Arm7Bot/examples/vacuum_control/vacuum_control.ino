/****************************************************
  /* 7Bot Arduino example
  /* Author: Jerry Peng
  /* Date: 26 April 2019
  /*
  /* Version 2.0
  /* www.7bot.cc
  /*
  /* Description: this example demonstrate how to control 7Bot vacuum status
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
  arm.vacuum(true);  // open vacuum
  delay(2000);
  arm.vacuum(false); // close vacuum
  delay(2000);

}
