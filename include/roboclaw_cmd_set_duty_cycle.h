#pragma once

#include "roboclaw_cmd.h"

class CmdSetDutyCycle : public Cmd {
 public:
  CmdSetDutyCycle(RoboClaw &roboclaw, int16_t m1_duty, int16_t m2_duty)
      : Cmd(roboclaw, "SetDutyCycle", RoboClaw::kNone),
        m1_duty_(m1_duty),
        m2_duty_(m2_duty) {}
  
  void send() override {
    roboclaw_.appendToWriteLog(
        "SetDutyCycle: m1_duty: %d, m2_duty: %d, WROTE: ",
        m1_duty_, m2_duty_);
    
    // kMIXEDDUTY command 34 expects two signed 16-bit duty cycle values (-32767 to +32767)
    roboclaw_.writeN2(true, 6, 
                      roboclaw_.portAddress_,
                      RoboClaw::kMIXEDDUTY,
                      (uint8_t)(m1_duty_ >> 8),
                      (uint8_t)(m1_duty_),
                      (uint8_t)(m2_duty_ >> 8),
                      (uint8_t)(m2_duty_)
    );
  }

 private:
  int16_t m1_duty_;
  int16_t m2_duty_;
};
