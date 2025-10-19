#pragma once

#include "roboclaw_cmd.h"

class CmdDoM1M2DriveSpeedAccel : public Cmd {
 public:
  CmdDoM1M2DriveSpeedAccel(
      RoboClaw &roboclaw, uint32_t accel_quad_pulses_per_second,
      int32_t m1_speed_quad_pulses_per_second,
      int32_t m2_speed_quad_pulses_per_second)
      : Cmd(roboclaw, "DoM1M2DriveSpeedAccel", RoboClaw::kNone),
        accel_quad_pulses_per_second_(accel_quad_pulses_per_second),
        m1_speed_quad_pulses_per_second_(m1_speed_quad_pulses_per_second),
        m2_speed_quad_pulses_per_second_(m2_speed_quad_pulses_per_second) {}
  void send() override {
    roboclaw_.appendToWriteLog(
        "M1M2WithSignedSpeedAccel: accel: %d, m1Speed: %d, m2Speed: %d, WROTE: ",
        accel_quad_pulses_per_second_, m1_speed_quad_pulses_per_second_,
        m2_speed_quad_pulses_per_second_);
    roboclaw_.writeN2(true, 14, roboclaw_.portAddress_,
                      RoboClaw::MIXEDSPEEDACCEL,
                      SetDWORDval(accel_quad_pulses_per_second_),
                      SetDWORDval(m1_speed_quad_pulses_per_second_),
                      SetDWORDval(m2_speed_quad_pulses_per_second_)
    );
  }

 private:
  uint32_t accel_quad_pulses_per_second_;
  int32_t m1_speed_quad_pulses_per_second_;
  int32_t m2_speed_quad_pulses_per_second_;
};
