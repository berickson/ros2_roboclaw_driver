#pragma once

#include "roboclaw_cmd.h"

class CmdSetSerialTimeout : public Cmd {
 public:
  CmdSetSerialTimeout(RoboClaw &roboclaw, uint8_t timeout_tenths)
      : Cmd(roboclaw, "SetSerialTimeout", RoboClaw::kNone),
        timeout_tenths_(timeout_tenths) {}

  void send() override {
    roboclaw_.appendToWriteLog("SetSerialTimeout: timeout_tenths: %u, WROTE: ",
                               timeout_tenths_);
    
    // Command format: [Address, Command(14), Timeout Byte, CRC]
    roboclaw_.writeN2(true, 3, roboclaw_.portAddress_,
                      RoboClaw::SETSERIALTIMEOUT,
                      timeout_tenths_);
  }

 private:
  uint8_t timeout_tenths_;
};
