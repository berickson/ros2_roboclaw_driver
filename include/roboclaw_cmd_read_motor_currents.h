#pragma once

#include "roboclaw_cmd.h"

class CmdReadMotorCurrents : public Cmd {
 public:
  CmdReadMotorCurrents(RoboClaw &roboclaw,
                       RoboClaw::TMotorCurrents &motorCurrents)
      : Cmd(roboclaw, "ReadMotorCurrents", RoboClaw::kNone),
        motorCurrents_(motorCurrents) {}
  
  void send() override {
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
  }
  
  void process() override {
    // Process current readings and handle over-current (called after mutex released)
    const float m1_average = roboclaw_.m1_current_average_.load();
    const float m2_average = roboclaw_.m2_current_average_.load();
    const float m1_max = roboclaw_.maxM1Current_.load();
    const float m2_max = roboclaw_.maxM2Current_.load();
    const bool m1_over = m1_average > m1_max;
    const bool m2_over = m2_average > m2_max;

    const auto state = static_cast<drive_logic::ProtectionState>(
        roboclaw_.current_protection_state_.load());
    const double seconds_since_nonzero_cmd = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - roboclaw_.getLastNonzeroCmdVelTime()).count();
    const auto next = drive_logic::nextProtectionState(
        state, m1_over || m2_over, seconds_since_nonzero_cmd,
        roboclaw_.recovery_timeout_seconds_);

    if (state == drive_logic::kNormal && next == drive_logic::kOverCurrentWarning) {
      roboclaw_.motorAlarms_ |= m1_over ? RoboClaw::kM1_OVER_CURRENT_ALARM
                                        : RoboClaw::kM2_OVER_CURRENT_ALARM;
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadMotorCurrents] Motor %d over current (avg). Max "
          "allowed: %6.3f, found: %6.3f",
          m1_over ? 1 : 2, m1_over ? m1_max : m2_max,
          m1_over ? m1_average : m2_average);
      // Enter the fault state before stopping, so cmd_vel is blocked even if
      // the stop command fails
      roboclaw_.transitionState(RoboClaw::OVER_CURRENT_WARNING,
                                m1_over ? "M1 average current exceeded"
                                        : "M2 average current exceeded");
      roboclaw_.doMixedSpeedAccel(0, 0, 0);
    } else if (state == drive_logic::kNormal) {
      // Clear alarm flags when current is normal
      roboclaw_.motorAlarms_ &= ~(RoboClaw::kM1_OVER_CURRENT_ALARM |
                                  RoboClaw::kM2_OVER_CURRENT_ALARM);
      // Debug: periodically log current vs limits
      static int log_counter = 0;
      if (roboclaw_.do_debug_ && (++log_counter % 50) == 0) {
        RCUTILS_LOG_INFO(
            "[RoboClaw::CmdReadMotorCurrents] M1: %.2fA/%.2fA, M2: %.2fA/%.2fA",
            m1_average, m1_max, m2_average, m2_max);
      }
    } else if (next == drive_logic::kRecoveryWaiting) {
      roboclaw_.transitionState(RoboClaw::RECOVERY_WAITING,
                                "no non-zero cmd_vel, starting recovery wait");
    } else if (state == drive_logic::kRecoveryWaiting && next == drive_logic::kOverCurrentWarning) {
      RCUTILS_LOG_WARN(
          "[RoboClaw::CmdReadMotorCurrents] Current still high during recovery wait. "
          "M1: %6.3f, M2: %6.3f",
          m1_average, m2_average);
      roboclaw_.transitionState(RoboClaw::OVER_CURRENT_WARNING, "Current still too high");
    } else if (state == drive_logic::kRecoveryWaiting && next == drive_logic::kNormal) {
      roboclaw_.transitionState(RoboClaw::NORMAL, "current normal, recovery complete");
    }
  }

 private:
  RoboClaw::TMotorCurrents &motorCurrents_;
};
