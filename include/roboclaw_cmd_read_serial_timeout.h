#pragma once

#include "roboclaw_cmd.h"

class CmdReadSerialTimeout : public Cmd {
 public:
  CmdReadSerialTimeout(RoboClaw &roboclaw, uint8_t &timeout_tenths)
      : Cmd(roboclaw, "ReadSerialTimeout", RoboClaw::kNone),
        timeout_tenths_(timeout_tenths) {}

  void send() override {
    try {
      roboclaw_.appendToWriteLog("ReadSerialTimeout: WROTE: ");
      
      uint16_t crc = 0;
      roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
      roboclaw_.updateCrc(crc, RoboClaw::GETSERIALTIMEOUT);
      
      roboclaw_.writeN2(false, 2, roboclaw_.portAddress_,
                        RoboClaw::GETSERIALTIMEOUT);
      
      // Read 1-byte timeout value (in tenths of seconds, 0-255)
      uint8_t datum = roboclaw_.readByteWithTimeout2();
      roboclaw_.updateCrc(crc, datum);

      // Verify CRC
      uint16_t responseCrc = 0;
      uint8_t crc_high = roboclaw_.readByteWithTimeout2();
      responseCrc = crc_high << 8;
      uint8_t crc_low = roboclaw_.readByteWithTimeout2();
      responseCrc |= crc_low;
      
      if (responseCrc != crc) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadSerialTimeout] invalid CRC expected: 0x%04X, "
            "got: 0x%04X",
            crc, responseCrc);
        datum = 0;
      }

      timeout_tenths_ = datum;
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadSerialTimeout] Uncaught exception !!!");
      timeout_tenths_ = 0;
    }

    roboclaw_.appendToReadLog(", RESULT: %u tenths of seconds", timeout_tenths_);
  }

 private:
  uint8_t &timeout_tenths_;
};
