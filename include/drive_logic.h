#pragma once

// Pure functions for differential drive kinematics and safety decisions.
// Kept free of ROS and serial dependencies so they can be unit tested.

#include <algorithm>
#include <cstddef>

namespace drive_logic {

// Wheel speeds in m/s. Left is m1, right is m2.
struct WheelSpeeds {
  double left;
  double right;
};

// Body motion: forward speed in m/s, turn rate in rad/s (positive = CCW).
struct BodyVelocity {
  double linear;
  double angular;
};

inline WheelSpeeds wheelSpeedsFromTwist(double linear, double angular,
                                        double wheel_separation) {
  const double turn = angular * wheel_separation / 2.0;
  return {linear - turn, linear + turn};
}

inline BodyVelocity twistFromWheelSpeeds(double left, double right,
                                         double wheel_separation) {
  return {(left + right) / 2.0, (right - left) / wheel_separation};
}

// Number of current samples to average. A window of 0 disables filtering.
// Any positive window keeps at least one sample, so a window shorter than
// the sample period can't leave the average stuck at zero.
inline size_t currentBufferSize(double window_seconds, double sample_rate_hz) {
  if (window_seconds <= 0.0) return 0;
  const double capped_window = std::min(window_seconds, 10.0);
  return std::max<size_t>(1, static_cast<size_t>(capped_window * sample_rate_hz));
}

// Values match RoboClaw::OverCurrentState.
enum ProtectionState {
  kNormal = 0,
  kOverCurrentWarning = 1,
  kRecoveryWaiting = 2,
};

// Over-current state machine. Motors are stopped in kOverCurrentWarning and
// kRecoveryWaiting. A recovery timeout of 0 disables auto-recovery; the fault
// then latches until cleared explicitly.
inline ProtectionState nextProtectionState(ProtectionState state, bool over_limit,
                                           double seconds_since_nonzero_cmd,
                                           double recovery_timeout_seconds) {
  switch (state) {
    case kNormal:
      return over_limit ? kOverCurrentWarning : kNormal;
    case kOverCurrentWarning:
      if (recovery_timeout_seconds > 0.0 &&
          seconds_since_nonzero_cmd >= recovery_timeout_seconds) {
        return kRecoveryWaiting;
      }
      return kOverCurrentWarning;
    case kRecoveryWaiting:
      return over_limit ? kOverCurrentWarning : kNormal;
  }
  return state;
}

// True when motors were commanded to move but no cmd_vel has arrived within
// the timeout. A timeout of 0 disables the watchdog.
inline bool watchdogExpired(bool moving, double seconds_since_cmd,
                            double timeout_seconds) {
  return moving && timeout_seconds > 0.0 && seconds_since_cmd > timeout_seconds;
}

}  // namespace drive_logic
