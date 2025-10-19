#pragma once

#include "roboclaw.h"

class Cmd {
 public:
  void execute() {
    for (int retry = 0; retry < 3 /*### maxCommandRetries_*/; retry++) {
      try {
        std::lock_guard<std::mutex> lock(
            RoboClaw::buffered_command_mutex_);  // Lock the mutex
        send();
        roboclaw_.debug_log_.showLog();
      } catch (const RoboClaw::TRoboClawException& e) {
        roboclaw_.debug_log_.showLog();
        // Use WARN for retries (recoverable), ERROR for final failure
        if (retry < 2) {
          RCUTILS_LOG_WARN(
              "[%s] Communication error (retry %d/3): %s",
              name_, retry + 1, e.what());
        } else {
          RCUTILS_LOG_ERROR(
              "[%s] Communication error (final attempt): %s",
              name_, e.what());
        }
        // Track the error even if we're going to retry
        roboclaw_.recordError(e);
        continue;
      } catch (const std::exception& e) {
        roboclaw_.debug_log_.showLog();
        if (retry < 2) {
          RCUTILS_LOG_WARN(
              "[%s] Std exception (retry %d/3): %s",
              name_, retry + 1, e.what());
        } else {
          RCUTILS_LOG_ERROR(
              "[%s] Std exception (final attempt): %s",
              name_, e.what());
        }
        // Track as generic communication error
        roboclaw_.recordError(RoboClaw::CommunicationException("execute", "Std exception: %s", e.what()));
        continue;
      } catch (...) {
        roboclaw_.debug_log_.showLog();
        if (retry < 2) {
          RCUTILS_LOG_WARN("[%s] Unknown exception (retry %d/3)", name_, retry + 1);
        } else {
          RCUTILS_LOG_ERROR("[%s] Unknown exception (final attempt)", name_);
        }
        // Track as generic communication error
        roboclaw_.recordError(RoboClaw::CommunicationException("execute", "Unknown exception"));
        continue;
      }
      // Mutex released here, now safe to call process() which may send other commands
      process();
      roboclaw_.recordSuccessfulCommunication();
      return;
    }

    roboclaw_.debug_log_.showLog();
    auto final_exception = RoboClaw::TRoboClawException(
        "[RoboClaw::Cmd::execute] RETRY COUNT EXCEEDED for %s", name_);
    roboclaw_.recordFailedCommunication(final_exception);
    RCUTILS_LOG_ERROR("[RoboClaw::Cmd::execute] RETRY COUNT EXCEEDED for %s", name_);
    throw final_exception;
  }

  virtual void send() = 0;  // Declare send as a pure virtual function
  virtual void process() {}  // Optional post-processing after mutex released

 protected:
  Cmd(RoboClaw &roboclaw, const char *name, const RoboClaw::kMotor motor)
      : motor_(motor), roboclaw_(roboclaw) {
    strncpy(name_, name, sizeof(name_));
    name_[sizeof(name_) - 1] = '\0';  // Ensure null-termination
  }

  RoboClaw::kMotor motor_;
  RoboClaw &roboclaw_;
  char name_[32];

 private:
  Cmd() = delete;  // Disallow default constructor
};
