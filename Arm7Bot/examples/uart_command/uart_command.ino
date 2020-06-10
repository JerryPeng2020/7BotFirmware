/****************************************************
  /* 7Bot Arduino example
  /* Author: Jerry Peng
  /* Date: June 8th, 2020
  /*
  /* Version 2.2
  /* www.7bot.cc
  /*
  /* Description: this example enable 7Bot receive command and
  /* send feedback through USB-UART port. The command is compatible with 7Bot Version 1.0
  /*
  /*
  /***************************************************/



#include <Arm7Bot.h>

Arm7Bot arm;
long timeBuf;

void setup()
{

  arm.init();
}


void loop()
{

  arm.receiveCommand();

  if (arm.comReg[POSITION_FEEDBACK_FREQ_ID] != 0)
    if (millis() - timeBuf > 1000/arm.comReg[POSITION_FEEDBACK_FREQ_ID]) {
      timeBuf = millis();
      arm.poseFeedback();
    }

}
