#include <gtest/gtest.h>

#include "drive_logic.h"

using namespace drive_logic;

constexpr double kSeparation = 0.414;

// --- Kinematics

TEST(WheelSpeedsFromTwist, StraightDrivesBothWheelsEqually) {
  WheelSpeeds w = wheelSpeedsFromTwist(0.5, 0.0, kSeparation);
  EXPECT_DOUBLE_EQ(w.left, 0.5);
  EXPECT_DOUBLE_EQ(w.right, 0.5);
}

TEST(WheelSpeedsFromTwist, PositiveTurnIsCounterClockwise) {
  // Turning left (CCW) means the right wheel goes faster
  WheelSpeeds w = wheelSpeedsFromTwist(0.0, 1.0, kSeparation);
  EXPECT_DOUBLE_EQ(w.left, -0.207);
  EXPECT_DOUBLE_EQ(w.right, 0.207);
}

TEST(WheelSpeedsFromTwist, TurnRateIsInRadiansPerSecond) {
  // Regression: the yaw term used to be divided by wheel radius, scaling
  // turns by about 17x on a 0.06 m wheel. Wheel speed difference must be
  // exactly angular * separation.
  WheelSpeeds w = wheelSpeedsFromTwist(0.3, 0.5, kSeparation);
  EXPECT_NEAR(w.right - w.left, 0.5 * kSeparation, 1e-12);
  EXPECT_NEAR((w.left + w.right) / 2.0, 0.3, 1e-12);
}

TEST(TwistFromWheelSpeeds, PureTurnReportsTurnNotSidewaysMotion) {
  // Regression: odometry treated the wheels as body x and y velocity and
  // reported turns with the wrong sign
  BodyVelocity b = twistFromWheelSpeeds(-0.207, 0.207, kSeparation);
  EXPECT_NEAR(b.linear, 0.0, 1e-12);
  EXPECT_NEAR(b.angular, 1.0, 1e-12);
}

TEST(TwistFromWheelSpeeds, InvertsWheelSpeedsFromTwist) {
  for (double linear : {-1.0, 0.0, 0.25, 1.5}) {
    for (double angular : {-2.0, -0.3, 0.0, 0.83, 1.67}) {
      WheelSpeeds w = wheelSpeedsFromTwist(linear, angular, kSeparation);
      BodyVelocity b = twistFromWheelSpeeds(w.left, w.right, kSeparation);
      EXPECT_NEAR(b.linear, linear, 1e-12);
      EXPECT_NEAR(b.angular, angular, 1e-12);
    }
  }
}

// --- Current filter

TEST(CurrentBufferSize, ZeroWindowDisablesFiltering) {
  EXPECT_EQ(currentBufferSize(0.0, 20.0), 0u);
}

TEST(CurrentBufferSize, TinyWindowKeepsAtLeastOneSample) {
  // Regression: a window shorter than one sample period truncated to 0,
  // which emptied the buffer and silently disabled over-current protection
  EXPECT_EQ(currentBufferSize(0.04, 20.0), 1u);
}

TEST(CurrentBufferSize, ScalesWithWindowAndRate) {
  EXPECT_EQ(currentBufferSize(1.0, 20.0), 20u);
  EXPECT_EQ(currentBufferSize(0.5, 50.0), 25u);
}

TEST(CurrentBufferSize, CappedAtTenSeconds) {
  EXPECT_EQ(currentBufferSize(30.0, 20.0), 200u);
}

// --- Over-current state machine

TEST(ProtectionState, NormalTripsOnOverLimit) {
  EXPECT_EQ(nextProtectionState(kNormal, true, 100.0, 5.0), kOverCurrentWarning);
  EXPECT_EQ(nextProtectionState(kNormal, false, 100.0, 5.0), kNormal);
}

TEST(ProtectionState, WarningWaitsForQuietCmdVel) {
  EXPECT_EQ(nextProtectionState(kOverCurrentWarning, false, 4.9, 5.0), kOverCurrentWarning);
  EXPECT_EQ(nextProtectionState(kOverCurrentWarning, false, 5.0, 5.0), kRecoveryWaiting);
}

TEST(ProtectionState, ZeroRecoveryTimeoutLatches) {
  EXPECT_EQ(nextProtectionState(kOverCurrentWarning, false, 1e6, 0.0), kOverCurrentWarning);
}

TEST(ProtectionState, RecoveryReturnsToNormalOnlyWhenCurrentIsLow) {
  EXPECT_EQ(nextProtectionState(kRecoveryWaiting, false, 10.0, 5.0), kNormal);
  EXPECT_EQ(nextProtectionState(kRecoveryWaiting, true, 10.0, 5.0), kOverCurrentWarning);
}

// --- Watchdog

TEST(Watchdog, StopsWhenCommandsStopWhileMoving) {
  EXPECT_TRUE(watchdogExpired(true, 0.6, 0.5));
  EXPECT_FALSE(watchdogExpired(true, 0.4, 0.5));
}

TEST(Watchdog, IgnoresStoppedRobot) {
  EXPECT_FALSE(watchdogExpired(false, 10.0, 0.5));
}

TEST(Watchdog, ZeroTimeoutDisables) {
  EXPECT_FALSE(watchdogExpired(true, 10.0, 0.0));
}
