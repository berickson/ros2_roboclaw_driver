#pragma once

#include "roboclaw_cmd.h"

class CmdReadMotorCurrents : public Cmd {
 public:
  CmdReadMotorCurrents(RoboClaw &roboclaw,
                       RoboClaw::TMotorCurrents &motorCurrents)
      : Cmd(roboclaw, "ReadMotorCurrents", RoboClaw::kNone),
        motorCurrents_(motorCurrents) {}
  
  void send() override {
    try {
      roboclaw_.appendToWriteLog("ReadMotorCurrents: WROTE: ");

      unsigned long currentPair =
          roboclaw_.getUlongCommandResult2(RoboClaw::GETCURRENTS);
      motorCurrents_.m1Current = ((int16_t)(currentPair >> 16)) * 0.010;
      motorCurrents_.m2Current = ((int16_t)(currentPair & 0xFFFF)) * 0.010;
      roboclaw_.appendToReadLog(", RESULT m1 current: %3.4f, m2 current: %3.4f",
                                motorCurrents_.m1Current,
                                motorCurrents_.m2Current);

      // Add current samples to history and calculate averages
      roboclaw_.addCurrentSample(motorCurrents_.m1Current, motorCurrents_.m2Current);
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadMotorCurrents] Uncaught exception in send() !!!");
      throw;
    }
  }
  
  void process() override {
    // Process current readings and handle over-current (called after mutex released)
    if (roboclaw_.current_protection_state_ == RoboClaw::NORMAL) {
      // Check if averaged current exceeds threshold
      if (roboclaw_.m1_current_average_ > roboclaw_.maxM1Current_) {
        roboclaw_.motorAlarms_ |= RoboClaw::kM1_OVER_CURRENT_ALARM;
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadMotorCurrents] Motor 1 over current (avg). Max "
            "allowed: %6.3f, found: %6.3f",
            roboclaw_.maxM1Current_, roboclaw_.m1_current_average_);
        // Safe to send stop command here - mutex is released
        roboclaw_.doMixedSpeedAccel(0, 0, 0);
        roboclaw_.transitionState(RoboClaw::OVER_CURRENT_WARNING, 
                                   "M1 average current exceeded");
      } else if (roboclaw_.m2_current_average_ > roboclaw_.maxM2Current_) {
        roboclaw_.motorAlarms_ |= RoboClaw::kM2_OVER_CURRENT_ALARM;
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadMotorCurrents] Motor 2 over current (avg). Max "
            "allowed: %6.3f, found: %6.3f",
            roboclaw_.maxM2Current_, roboclaw_.m2_current_average_);
        // Safe to send stop command here - mutex is released
        roboclaw_.doMixedSpeedAccel(0, 0, 0);
        roboclaw_.transitionState(RoboClaw::OVER_CURRENT_WARNING, 
                                   "M2 average current exceeded");
      } else {
        // Clear alarm flags when current is normal
        roboclaw_.motorAlarms_ &= ~(RoboClaw::kM1_OVER_CURRENT_ALARM | 
                                    RoboClaw::kM2_OVER_CURRENT_ALARM);
      }
    } else if (roboclaw_.current_protection_state_ == RoboClaw::OVER_CURRENT_WARNING) {
      // Check if we can start recovery (no non-zero cmd_vel for recovery timeout)
      if (roboclaw_.recovery_timeout_seconds_ > 0.0) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<float>(now - roboclaw_.last_nonzero_cmd_vel_time_).count();
        if (elapsed >= roboclaw_.recovery_timeout_seconds_) {
          roboclaw_.recovery_start_time_ = now;
          roboclaw_.transitionState(RoboClaw::RECOVERY_WAITING, 
                                     "no non-zero cmd_vel, starting recovery wait");
        }
      }
    } else if (roboclaw_.current_protection_state_ == RoboClaw::RECOVERY_WAITING) {
      // Check if we can recover: current is low
      if (roboclaw_.m1_current_average_ > roboclaw_.maxM1Current_ ||
          roboclaw_.m2_current_average_ > roboclaw_.maxM2Current_) {
        RCUTILS_LOG_WARN(
            "[RoboClaw::CmdReadMotorCurrents] Current still high during recovery wait. "
            "M1: %6.3f, M2: %6.3f",
            roboclaw_.m1_current_average_, roboclaw_.m2_current_average_);
        roboclaw_.transitionState(RoboClaw::OVER_CURRENT_WARNING, 
                                   "Current still too high");
      } else {
        // Current is low, we can transition back to normal
        roboclaw_.transitionState(RoboClaw::NORMAL, "current normal, recovery complete");
      }
    }
  }

 private:
  RoboClaw::TMotorCurrents &motorCurrents_;
};
