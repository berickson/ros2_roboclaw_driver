#pragma once

#include "roboclaw_cmd.h"

class CmdReadMainBatteryVoltage : public Cmd {
 public:
  CmdReadMainBatteryVoltage(RoboClaw &roboclaw, float &voltage)
      : Cmd(roboclaw, "ReadLMainBatteryVoltage", RoboClaw::kNone),
        voltage_(voltage) {}
  void send() override {
    roboclaw_.appendToWriteLog("ReadLMainBatteryVoltage: WROTE: ");
    float result =
        ((float)roboclaw_.get2ByteCommandResult2(RoboClaw::GETMBATT)) / 10.0;
    voltage_ = result;
    roboclaw_.appendToReadLog(", RESULT: %f", result);
  }

 private:
  float &voltage_;
};
