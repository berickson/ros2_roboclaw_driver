#include "roboclaw.h"

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <rcutils/logging_macros.h>
#include <stdio.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <boost/assign.hpp>
#include <cstdint>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include "roboclaw_cmd_do_buffered_m1m2_drive_speed_accel_distance.h"
#include "roboclaw_cmd_do_m1m2_drive_speed_accel.h"
#include "roboclaw_cmd_read_encoder.h"
#include "roboclaw_cmd_read_encoder_speed.h"
#include "roboclaw_cmd_read_firmware_version.h"
#include "roboclaw_cmd_read_logic_battery_voltage.h"
#include "roboclaw_cmd_read_main_battery_voltage.h"
#include "roboclaw_cmd_read_motor_currents.h"
#include "roboclaw_cmd_read_motor_velocity_pidq.h"
#include "roboclaw_cmd_read_serial_timeout.h"
#include "roboclaw_cmd_read_status.h"
#include "roboclaw_cmd_read_temperature.h"
#include "roboclaw_cmd_set_encoder_value.h"
#include "roboclaw_cmd_set_pid.h"
#include "roboclaw_cmd_set_serial_timeout.h"
#include "ros2_roboclaw_driver/srv/reset_encoders.h"

const char *RoboClaw::motorNames_[] = {"M1", "M2", "NONE"};

// Initialize the static mutex
std::mutex RoboClaw::buffered_command_mutex_;

RoboClaw::RoboClaw(const TPIDQ m1Pid, const TPIDQ m2Pid, float m1MaxCurrent,
                   float m2MaxCurrent, std::string device_name,
                   uint8_t device_port, uint32_t baud_rate,
                   float serial_timeout, bool(do_debug),
                   bool do_low_level_debug)
    : do_debug_(do_debug),
      do_low_level_debug_(do_low_level_debug),
      baud_rate_(baud_rate),
      device_port_(device_port),
      maxCommandRetries_(3),
      maxM1Current_(m1MaxCurrent),
      maxM2Current_(m2MaxCurrent),
      device_name_(device_name),
      portAddress_(128),
      current_protection_state_(NORMAL),
      filter_window_seconds_(1.0),
      recovery_timeout_seconds_(5.0),
      sensor_update_rate_(20.0),
      m1_current_average_(0.0),
      m2_current_average_(0.0),
      max_buffer_size_(0),
      debug_log_(this) {
  // Initialize timestamps - set last_nonzero_cmd_vel_time_ far in the past
  // so recovery can start immediately after over-current
  last_nonzero_cmd_vel_time_ = std::chrono::steady_clock::now() - std::chrono::hours(1);
  
  openPort();
  RCUTILS_LOG_INFO("[RoboClaw::RoboClaw] RoboClaw software version: %s",
                   getVersion().c_str());
  try {
    setSerialTimeout(serial_timeout);
    RCUTILS_LOG_INFO("[RoboClaw::RoboClaw] Serial timeout set to: %f seconds",
                     serial_timeout);
  } catch (const std::exception& e) {
    RCUTILS_LOG_WARN(
        "[RoboClaw::RoboClaw] Failed to set serial timeout: %s. "
        "Continuing with default timeout.", e.what());
  }
  setM1PID(m1Pid.p, m1Pid.i, m1Pid.d, m1Pid.qpps);
  setM2PID(m2Pid.p, m2Pid.i, m2Pid.d, m2Pid.qpps);
  CmdSetEncoderValue m1(*this, kM1, 0);
  m1.execute();
  CmdSetEncoderValue m2(*this, kM2, 0);
  m2.execute();
  // ros2_roboclaw_driver::srv::ResetEncoders::Request resetRequest;
  // resetRequest.left = 0;
  // resetRequest.right = 0;
  // ros2_roboclaw_driver::srv::ResetEncoders::Response response;
  // resetEncoders(resetRequest, response);
  g_singleton = this;
  readSensorGroup();
}

RoboClaw::~RoboClaw() {}

void RoboClaw::doMixedSpeedAccelDist(uint32_t accel_quad_pulses_per_second,
                                     int32_t m1_quad_pulses_per_second,
                                     uint32_t m1_max_distance,
                                     int32_t m2_quad_pulses_per_second,
                                     uint32_t m2_max_distance) {
  CmdDoBufferedM1M2DriveSpeedAccelDistance command(
      *this, accel_quad_pulses_per_second, m1_quad_pulses_per_second,
      m1_max_distance, m2_quad_pulses_per_second, m2_max_distance);
  command.execute();
}

void RoboClaw::doMixedSpeedAccel(uint32_t accel_quad_pulses_per_second,
                                 int32_t m1_quad_pulses_per_second,
                                 int32_t m2_quad_pulses_per_second) {
  CmdDoM1M2DriveSpeedAccel command(
      *this, accel_quad_pulses_per_second, m1_quad_pulses_per_second,
      m2_quad_pulses_per_second);
  command.execute();
}

uint32_t RoboClaw::getErrorStatus() {
  return g_sensor_value_group_.error_status;
}

std::string RoboClaw::getErrorString() {
  return g_sensor_value_group_.error_string;
}

// 32-bit status decode per updated RoboClaw documentation.
// Lower 16 bits retain classic meanings. Upper bits (16-31) may include
// extended status such as position/speed error limits or other firmware
// specific flags; placeholders added for known documented bits. Adjust names
// to match exact manual revision you are using.
std::string RoboClaw::getErrorString(uint32_t status) {
  if (status == 0) return "normal";
  std::stringstream ss;
  // Bit 0-15 (classic)
  if (status & 0x00000001) { ss << "[M1 OverCurrent Warning] "; motorAlarms_ |= kM1_OVER_CURRENT_ALARM; } else { motorAlarms_ &= ~kM1_OVER_CURRENT_ALARM; }
  if (status & 0x00000002) { ss << "[M2 OverCurrent Warning] "; motorAlarms_ |= kM2_OVER_CURRENT_ALARM; } else { motorAlarms_ &= ~kM2_OVER_CURRENT_ALARM; }
  if (status & 0x00000004) ss << "[E-Stop] ";
  if (status & 0x00000008) ss << "[Temperature Error] ";
  if (status & 0x00000010) ss << "[Temperature2 Error] ";
  if (status & 0x00000020) ss << "[Main Battery High Error] ";
  if (status & 0x00000040) ss << "[Logic Battery High Error] ";
  if (status & 0x00000080) ss << "[Logic Battery Low Error] ";
  if (status & 0x00000100) ss << "[M2 Driver Fault] ";
  if (status & 0x00000200) ss << "[M1 Driver Fault] ";
  if (status & 0x00000400) ss << "[Main Battery High Warning] ";
  if (status & 0x00000800) ss << "[Main Battery Low Warning] ";
  if (status & 0x00001000) ss << "[Temperature Warning] ";
  if (status & 0x00002000) ss << "[Temperature2 Warning] ";
  if (status & 0x00004000) ss << "[M1 Home] ";
  if (status & 0x00008000) ss << "[M2 Home] ";
  // Upper 16 bits (example labels; verify with manual revision)
  if (status & 0x00010000) ss << "[Speed Error Limit Warning] ";
  if (status & 0x00020000) ss << "[Position Error Limit Warning] ";
  if (status & 0x00040000) ss << "[Reserved Bit 18] ";
  if (status & 0x00080000) ss << "[Reserved Bit 19] ";
  if (status & 0x00100000) ss << "[Reserved Bit 20] ";
  if (status & 0x00200000) ss << "[Reserved Bit 21] ";
  if (status & 0x00400000) ss << "[Reserved Bit 22] ";
  if (status & 0x00800000) ss << "[Reserved Bit 23] ";
  if (status & 0x01000000) ss << "[Extended Flag 24] ";
  if (status & 0x02000000) ss << "[Extended Flag 25] ";
  if (status & 0x04000000) ss << "[Extended Flag 26] ";
  if (status & 0x08000000) ss << "[Extended Flag 27] ";
  if (status & 0x10000000) ss << "[Extended Flag 28] ";
  if (status & 0x20000000) ss << "[Extended Flag 29] ";
  if (status & 0x40000000) ss << "[Extended Flag 30] ";
  if (status & 0x80000000) ss << "[Extended Flag 31] ";
  return ss.str();
}

float RoboClaw::getLogicBatteryLevel() {
  return g_sensor_value_group_.logic_battery_level;
}

float RoboClaw::getMainBatteryLevel() {
  return g_sensor_value_group_.main_battery_level;
}

// ### change result type to uint16_t
unsigned short RoboClaw::get2ByteCommandResult2(uint8_t command) {
  uint16_t crc = 0;
  updateCrc(crc, portAddress_);
  updateCrc(crc, command);

  writeN2(false, 2, portAddress_, command);
  unsigned short result = 0;
  uint8_t datum = readByteWithTimeout2();
  result |= datum << 8;
  updateCrc(crc, datum);

  datum = readByteWithTimeout2();
  result |= datum;
  updateCrc(crc, datum);

  uint16_t responseCrc = 0;
  datum = readByteWithTimeout2();
  responseCrc = datum << 8;
  datum = readByteWithTimeout2();
  responseCrc |= datum;
  if (responseCrc == crc) {
    return result;
  } else {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::get2ByteCommandResult2] invalid CRC expected: "
        "0x%02X, got: 0x%02X",
        crc, responseCrc);
    throw new TRoboClawException(
        "[RoboClaw::get2ByteCommandResult2 INVALID CRC");
    return 0;
  }
}

RoboClaw::TMotorCurrents RoboClaw::getMotorCurrents() {
  return g_sensor_value_group_.motor_currents;
}

RoboClaw::TPIDQ RoboClaw::getPIDQM1() { return g_sensor_value_group_.m1_pidq; }

RoboClaw::TPIDQ RoboClaw::getPIDQM2() { return g_sensor_value_group_.m2_pidq; }

float RoboClaw::getTemperature() { return g_sensor_value_group_.temperature; }

unsigned long RoboClaw::getUlongCommandResult2(uint8_t command) {
  uint16_t crc = 0;
  updateCrc(crc, portAddress_);
  updateCrc(crc, command);

  writeN2(false, 2, portAddress_, command);
  unsigned long result = 0;
  uint8_t datum = readByteWithTimeout2();
  result |= datum << 24;
  updateCrc(crc, datum);
  datum = readByteWithTimeout2();
  result |= datum << 16;
  updateCrc(crc, datum);
  datum = readByteWithTimeout2();
  result |= datum << 8;
  updateCrc(crc, datum);
  datum = readByteWithTimeout2();
  result |= datum;
  updateCrc(crc, datum);

  uint16_t responseCrc = 0;
  datum = readByteWithTimeout2();
  responseCrc = datum << 8;
  datum = readByteWithTimeout2();
  responseCrc |= datum;
  if (responseCrc == crc) {
    return result;
  }

  RCUTILS_LOG_ERROR(
      "[RoboClaw::getUlongCommandResult2] Expected CRC of: 0x%02X, but "
      "got: 0x%02X",
      int(crc), int(responseCrc));
  throw new TRoboClawException(
      "[RoboClaw::getUlongCommandResult2] INVALID CRC");
  return 0;
}

uint32_t RoboClaw::getULongCont2(uint16_t &crc) {
  uint32_t result = 0;
  uint8_t datum = readByteWithTimeout2();
  result |= datum << 24;
  updateCrc(crc, datum);
  datum = readByteWithTimeout2();
  result |= datum << 16;
  updateCrc(crc, datum);
  datum = readByteWithTimeout2();
  result |= datum << 8;
  updateCrc(crc, datum);
  datum = readByteWithTimeout2();
  result |= datum;
  updateCrc(crc, datum);
  return result;
}

int32_t RoboClaw::getVelocity(kMotor motor) {
  if (motor == kM1) {
    return g_sensor_value_group_.m1_velocity;
  } else {
    return g_sensor_value_group_.m2_velocity;
  }
}

int32_t RoboClaw::getVelocityResult(uint8_t command) {
  uint16_t crc = 0;
  updateCrc(crc, portAddress_);
  updateCrc(crc, command);

  writeN2(false, 2, portAddress_, command);
  int32_t result = 0;
  uint8_t datum = readByteWithTimeout2();
  result |= datum << 24;
  updateCrc(crc, datum);

  datum = readByteWithTimeout2();
  result |= datum << 16;
  updateCrc(crc, datum);

  datum = readByteWithTimeout2();
  result |= datum << 8;
  updateCrc(crc, datum);

  datum = readByteWithTimeout2();
  result |= datum;
  updateCrc(crc, datum);

  uint8_t direction = readByteWithTimeout2();
  updateCrc(crc, direction);
  if (direction != 0) result = -result;

  uint16_t responseCrc = 0;
  datum = readByteWithTimeout2();
  responseCrc = datum << 8;
  datum = readByteWithTimeout2();
  responseCrc |= datum;
  if (responseCrc == crc) {
    return result;
  }

  RCUTILS_LOG_ERROR(
      "[RoboClaw::getVelocityResult] Expected CRC of: 0x%02X, but got: "
      "0x%02X",
      int(crc), int(responseCrc));
  throw new TRoboClawException("[RoboClaw::getVelocityResult] INVALID CRC");
  return 0;
}

int32_t RoboClaw::getM1Encoder() {
  return g_sensor_value_group_.m1_encoder_command_result.value;
}

int8_t RoboClaw::getM1EncoderStatus() {
  return g_sensor_value_group_.m1_encoder_command_result.status;
}

int32_t RoboClaw::getM2Encoder() {
  return g_sensor_value_group_.m2_encoder_command_result.value;
}

int8_t RoboClaw::getM2EncoderStatus() {
  return g_sensor_value_group_.m2_encoder_command_result.status;
}

std::string RoboClaw::getVersion() {
  std::string version;
  CmdReadFirmwareVersion command(*this, version);
  command.execute();
  return version;
}

void RoboClaw::openPort() {
  RCUTILS_LOG_INFO("[RoboClaw::openPort] about to open port: %s",
                   device_name_.c_str());
  device_port_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY);
  if (device_port_ < 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to open USB port: %s, errno: (%d) "
        "%s",
        device_name_.c_str(), errno, strerror(errno));
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to open USB port");
  }

  // Fetch the current port settings.
  struct termios portOptions;
  int ret = 0;

  ret = tcgetattr(device_port_, &portOptions);
  if (ret < 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to get terminal options "
        "(tcgetattr), error: %d: %s",
        errno, strerror(errno));
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to get terminal options (tcgetattr)");
  }

  if (cfsetispeed(&portOptions, B38400) < 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to set terminal speed "
        "(cfsetispeed)");
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to set terminal speed "
        "(cfsetispeed)");
  }

  speed_t baud;
  switch (baud_rate_) {
    case 9600:
      baud = B9600;
      break;
    case 19200:
      baud = B19200;
      break;
    case 38400:
      baud = B38400;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    default:
      RCUTILS_LOG_ERROR("[RoboClaw::openPort] Unsupported baud rate: %u",
                        baud_rate_);
      throw new TRoboClawException(
          "[RoboClaw::openPort] Unsupported baud rate");
  }

  if (cfsetispeed(&portOptions, baud) < 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to set terminal input speed "
        "(cfsetispeed)");
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to set terminal input speed "
        "(cfsetispeed)");
  }
  if (cfsetospeed(&portOptions, B38400) < 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to set terminal speed "
        "(cfsetospeed)");
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to set terminal speed "
        "(cfsetospeed)");
  }

  // Configure other settings
  portOptions.c_cflag &= ~PARENB;   // Disable parity.
  portOptions.c_cflag &= ~CSTOPB;   // 1 stop bit
  portOptions.c_cflag &= ~CSIZE;    // Clear data size bits
  portOptions.c_cflag |= CS8;       // 8 data bits
  portOptions.c_cflag &= ~CRTSCTS;  // Disable hardware flow control
  portOptions.c_cflag |=
      CREAD | CLOCAL;  // Enable read and ignore control lines

  portOptions.c_lflag &= ~ICANON;  // Disable canonical mode
  portOptions.c_lflag &= ~ECHO;    // Disable echo
  portOptions.c_lflag &= ~ECHOE;   // Disable erasure
  portOptions.c_lflag &= ~ECHONL;  // Disable new-line echo
  portOptions.c_lflag &=
      ~ISIG;  // Disable interpretation of INTR, QUIT and SUSP

  portOptions.c_iflag &=
      ~(IXON | IXOFF | IXANY);  // Disable software flow control
  portOptions.c_iflag &=
      ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
        ICRNL);  // Disable special handling of received bytes

  portOptions.c_oflag &= ~OPOST;  // Disable output processing

  portOptions.c_cc[VMIN] = 0;   // Non-blocking read
  portOptions.c_cc[VTIME] = 5;  // Timeout of 0.5 seconds

  if (tcsetattr(device_port_, TCSANOW, &portOptions) != 0) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::openPort] Unable to set terminal options "
        "(tcsetattr)");
    throw new TRoboClawException(
        "[RoboClaw::openPort] Unable to set terminal options "
        "(tcsetattr)");
  }
}

uint8_t RoboClaw::readByteWithTimeout2() {
  struct pollfd ufd[1];
  ufd[0].fd = device_port_;
  ufd[0].events = POLLIN;

  int retval = poll(ufd, 1, 11);
  if (retval < 0) {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout2 Poll failed (%d) %s",
                      errno, strerror(errno));
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout2 Read error");
  } else if (retval == 0) {
    std::stringstream ev;
    ev << "[RoboClaw::readByteWithTimeout2 TIMEOUT revents: " << std::hex
       << ufd[0].revents;
    RCUTILS_LOG_ERROR(ev.str().c_str());
    throw new TRoboClawException("[RoboClaw::readByteWithTimeout2 TIMEOUT");
  } else if (ufd[0].revents & POLLERR) {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout2 Error on socket");
    restartPort();
    throw new TRoboClawException(
        "[RoboClaw::readByteWithTimeout2 Error on socket");
  } else if (ufd[0].revents & POLLIN) {
    unsigned char buffer[1];
    ssize_t bytesRead = ::read(device_port_, buffer, sizeof(buffer));
    if (bytesRead != 1) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::readByteWithTimeout2 Failed to read 1 byte, read: "
          "%d",
          (int)bytesRead);
      throw TRoboClawException(
          "[RoboClaw::readByteWithTimeout2 Failed to read 1 byte");
    }

    if (do_debug_ || do_low_level_debug_) {
      appendToReadLog("%02X ", buffer[0]);
      if (do_low_level_debug_) {
        RCUTILS_LOG_INFO("Read: %02X", buffer[0]);
      }
    }

    return buffer[0];
  } else {
    RCUTILS_LOG_ERROR("[RoboClaw::readByteWithTimeout2 Unhandled case");
    throw new TRoboClawException(
        "[RoboClaw::readByteWithTimeout2 Unhandled case");
  }

  return 0;
}

void RoboClaw::readSensorGroup() {
  if (singleton() != nullptr) {
    TPIDQ m1_read_velocity_pidq_result;
    TPIDQ m2_read_velocity_pidq_result;
    CmdReadMotorVelocityPIDQ cmd_m1_read_motor_velocity_pidq(
        *this, kM1, m1_read_velocity_pidq_result);
    cmd_m1_read_motor_velocity_pidq.execute();
    CmdReadMotorVelocityPIDQ cmd_m2_read_motor_velocity_pidq(
        *this, kM2, m2_read_velocity_pidq_result);
    cmd_m2_read_motor_velocity_pidq.execute();
    float logic_battery_level = 0.0;
    CmdReadLogicBatteryVoltage cmd_logic_battery(*this, logic_battery_level);
    cmd_logic_battery.execute();
    float main_battery_level = 0.0;
    CmdReadMainBatteryVoltage cmd_main_battery(*this, main_battery_level);
    cmd_main_battery.execute();
    EncodeResult m1_encoder_command_result{};
    EncodeResult m2_encoder_command_result{};
    CmdReadEncoder m1_read_encoder_cmd(*this, kM1, m1_encoder_command_result);
    m1_read_encoder_cmd.execute();
    CmdReadEncoder m2_read_encoder_cmd(*this, kM2, m2_encoder_command_result);
    m2_read_encoder_cmd.execute();
    TMotorCurrents motor_currents{};
    CmdReadMotorCurrents cmd_read_motor_currents(*this, motor_currents);
    cmd_read_motor_currents.execute();
    int32_t m1_encoder_speed = 0;
    int32_t m2_encoder_speed = 0;
    CmdReadEncoderSpeed cmd_m1_read_encoder_speed(*this, kM1, m1_encoder_speed);
    cmd_m1_read_encoder_speed.execute();
    CmdReadEncoderSpeed cmd_m2_read_encoder_speed(*this, kM2, m2_encoder_speed);
    cmd_m2_read_encoder_speed.execute();
    float temperature = 0.0;
    CmdReadTemperature cmd_read_temperature(*this, temperature);
    cmd_read_temperature.execute();
    uint32_t status = 0;
    CmdReadStatus cmd_read_status(*this, status);
    cmd_read_status.execute();
    g_sensor_value_group_.error_status = status;
    g_sensor_value_group_.error_string = singleton()->getErrorString(status);
    
    // Append current protection status if not normal
    if (current_protection_state_ != NORMAL) {
      const char* state_names[] = {"NORMAL", "OVER_CURRENT_WARNING", 
                                    "RECOVERY_WAITING", "RECOVERING"};
      if (g_sensor_value_group_.error_string.empty() || 
          g_sensor_value_group_.error_string == "normal") {
        g_sensor_value_group_.error_string = "[";
      } else {
        g_sensor_value_group_.error_string += "[";
      }
      g_sensor_value_group_.error_string += state_names[current_protection_state_];
      g_sensor_value_group_.error_string += "] ";
    }
    
    g_sensor_value_group_.logic_battery_level = logic_battery_level;
    g_sensor_value_group_.m1_encoder_command_result = m1_encoder_command_result;
    g_sensor_value_group_.m1_pidq = m1_read_velocity_pidq_result;
    g_sensor_value_group_.m1_velocity = m1_encoder_speed;
    g_sensor_value_group_.m2_encoder_command_result = m2_encoder_command_result;
    g_sensor_value_group_.m2_pidq = m2_read_velocity_pidq_result;
    g_sensor_value_group_.m2_velocity = m2_encoder_speed;
    g_sensor_value_group_.main_battery_level = main_battery_level;

    // Call getMotorCurrents before getMotorAlarms;
    g_sensor_value_group_.motor_currents = motor_currents;
    g_sensor_value_group_.motor_alarms = singleton()->getMotorAlarms();
    g_sensor_value_group_.temperature = temperature;
    g_sensor_value_group_.last_sensor_read_time_ = std::chrono::system_clock::now();
  }
}

bool RoboClaw::resetEncoders(
    ros2_roboclaw_driver::srv::ResetEncoders::Request &request,
    ros2_roboclaw_driver::srv::ResetEncoders::Response &response) {
  try {
    CmdSetEncoderValue m1(*this, kM1, request.left);
    CmdSetEncoderValue m2(*this, kM2, request.right);
    m1.execute();
    m2.execute();
    response.ok = true;
  } catch (...) {
    RCUTILS_LOG_ERROR("[RoboClaw::resetEncoders] uncaught exception");
  }
  return true;
}

void RoboClaw::restartPort() {
  close(device_port_);
  usleep(200000);
  openPort();
}

void RoboClaw::setM1PID(float p, float i, float d, uint32_t qpps) {
  CmdSetPid command(*this, kM1, p, i, d, qpps);
  command.execute();
}

void RoboClaw::setM2PID(float p, float i, float d, uint32_t qpps) {
  CmdSetPid command(*this, kM2, p, i, d, qpps);
  command.execute();
}

void RoboClaw::setSerialTimeout(float timeout_seconds) {
  // Convert seconds to tenths of seconds (0.1s units)
  // Clamp to valid range: 0-255 tenths = 0.0-25.5 seconds
  float timeout_tenths_float = timeout_seconds * 10.0f;
  uint8_t timeout_tenths;
  
  if (timeout_tenths_float < 0.0f) {
    timeout_tenths = 0;
  } else if (timeout_tenths_float > 255.0f) {
    timeout_tenths = 255;
  } else {
    timeout_tenths = static_cast<uint8_t>(timeout_tenths_float);
  }
  
  CmdSetSerialTimeout command(*this, timeout_tenths);
  command.execute();
}

float RoboClaw::getSerialTimeout() {
  uint8_t timeout_tenths = 0;
  CmdReadSerialTimeout command(*this, timeout_tenths);
  command.execute();
  // Convert tenths of seconds back to seconds
  return static_cast<float>(timeout_tenths) / 10.0f;
}

void RoboClaw::stop() {
  CmdDoM1M2DriveSpeedAccel stopCommand(*this, 0, 0, 0);
  stopCommand.execute();
}

void RoboClaw::updateCrc(uint16_t &crc, uint8_t data) {
  crc = crc ^ ((uint16_t)data << 8);
  for (int i = 0; i < 8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }
}

void RoboClaw::writeByte2(uint8_t byte) {
  ssize_t result;
  do {
    result = ::write(device_port_, &byte, 1);
    // RCUTILS_LOG_INFO("--> wrote: 0x%02X, result: %ld", byte, result);  //
    // ####
    if (do_debug_ || do_low_level_debug_) {
      if (result == 1) {
        appendToWriteLog("%02X ", byte);
        if (do_low_level_debug_) {
          RCUTILS_LOG_INFO("Write: %02X", byte);
        }
      } else {
        appendToWriteLog("~%02X ", byte);
        if (do_low_level_debug_) {
          RCUTILS_LOG_ERROR("Write fail: %02X", byte);
        }
      }
    }

  } while (result == -1 && errno == EAGAIN);

  if (result != 1) {
    RCUTILS_LOG_ERROR(
        "[RoboClaw::writeByte2to write one byte, result: %d, "
        "errno: %d)",
        (int)result, errno);
    restartPort();
    throw new TRoboClawException(
        "[RoboClaw::writeByte2 Unable to write one byte");
  }
}

void RoboClaw::writeN2(bool sendCRC, uint8_t cnt, ...) {
  uint16_t crc = 0;
  va_list marker;
  va_start(marker, cnt);

  for (uint8_t i = 0; i < cnt; i++) {
    uint8_t byte = va_arg(marker, int);
    updateCrc(crc, byte);
    writeByte2(byte);
  }

  va_end(marker);

  if (sendCRC) {
    writeByte2(crc >> 8);
    writeByte2(crc);

    uint8_t response = readByteWithTimeout2();
    if (response != 0xFF) {
      char msg[128];
      snprintf(
          msg, sizeof(msg),
          "[RoboClaw::writeN2] Invalid ACK response, expected 0xFF but got "
          "0x%02X",
          response);
      RCUTILS_LOG_ERROR("%s", msg);
      throw new TRoboClawException(msg);
    }
  }
}

RoboClaw *RoboClaw::singleton() { return g_singleton; }

RoboClaw *RoboClaw::g_singleton = nullptr;

// Current protection methods implementation

void RoboClaw::setCurrentProtectionParams(float filter_window_seconds,
                                           float recovery_timeout_seconds,
                                           float sensor_update_rate) {
  // Validate parameters
  filter_window_seconds_ = std::max(0.0f, std::min(10.0f, filter_window_seconds));
  recovery_timeout_seconds_ = std::max(0.0f, std::min(60.0f, recovery_timeout_seconds));
  sensor_update_rate_ = sensor_update_rate;
  
  updateBufferSize();
  
  RCUTILS_LOG_INFO("[RoboClaw::setCurrentProtectionParams] filter_window: %.2f, "
                   "recovery_timeout: %.2f, sensor_rate: %.2f, max_buffer_size: %zu, "
                   "maxM1Current: %.2f, maxM2Current: %.2f",
                   filter_window_seconds_, recovery_timeout_seconds_, 
                   sensor_update_rate_, max_buffer_size_,
                   maxM1Current_, maxM2Current_);
}

void RoboClaw::notifyCmdVel(bool is_zero) {
  if (!is_zero) {
    // Track when non-zero cmd_vel is received
    last_nonzero_cmd_vel_time_ = std::chrono::steady_clock::now();
    
    // If in recovery, abort and go back to warning
    if (current_protection_state_ == RECOVERY_WAITING) {
      transitionState(OVER_CURRENT_WARNING, "cmd_vel non-zero during recovery");
    }
  }
}

void RoboClaw::addCurrentSample(float m1_current, float m2_current) {
  if (filter_window_seconds_ > 0.0) {
    // Add to history buffers
    m1_current_history_.push_back(m1_current);
    m2_current_history_.push_back(m2_current);
    
    // Trim if exceeds max buffer size
    while (m1_current_history_.size() > max_buffer_size_) {
      m1_current_history_.pop_front();
    }
    while (m2_current_history_.size() > max_buffer_size_) {
      m2_current_history_.pop_front();
    }
    
    // Calculate averages
    m1_current_average_ = calculateAverage(m1_current_history_);
    m2_current_average_ = calculateAverage(m2_current_history_);
  } else {
    // Use instantaneous values (legacy mode)
    m1_current_average_ = m1_current;
    m2_current_average_ = m2_current;
  }
}

void RoboClaw::updateBufferSize() {
  if (filter_window_seconds_ > 0.0) {
    max_buffer_size_ = std::min(
        (size_t)(filter_window_seconds_ * sensor_update_rate_),
        (size_t)(10.0 * sensor_update_rate_)  // Hard cap at 10 seconds
    );
    
    // Trim buffers if they exceed new size
    while (m1_current_history_.size() > max_buffer_size_) {
      m1_current_history_.pop_front();
    }
    while (m2_current_history_.size() > max_buffer_size_) {
      m2_current_history_.pop_front();
    }
  } else {
    // Clear buffers when filtering is disabled
    m1_current_history_.clear();
    m2_current_history_.clear();
    max_buffer_size_ = 0;
  }
}

float RoboClaw::calculateAverage(const std::deque<float>& history) const {
  if (history.empty()) {
    return 0.0f;
  }
  
  float sum = 0.0f;
  for (float value : history) {
    sum += value;
  }
  return sum / history.size();
}

void RoboClaw::transitionState(OverCurrentState new_state, const char* reason) {
  if (new_state != current_protection_state_) {
    const char* state_names[] = {"NORMAL", "OVER_CURRENT_WARNING", 
                                  "RECOVERY_WAITING", "RECOVERING"};
    RCUTILS_LOG_INFO("[RoboClaw::CurrentProtection] State: %s -> %s (%s)",
                     state_names[current_protection_state_],
                     state_names[new_state], reason);
    current_protection_state_ = new_state;
    
    // Record timestamps for state transitions
    if (new_state == OVER_CURRENT_WARNING) {
      alarm_triggered_time_ = std::chrono::steady_clock::now();
    } else if (new_state == RECOVERY_WAITING) {
      recovery_start_time_ = std::chrono::steady_clock::now();
    } else if (new_state == NORMAL) {
      // Clear alarm flags when returning to normal
      motorAlarms_ &= ~(kM1_OVER_CURRENT_ALARM | kM2_OVER_CURRENT_ALARM);
    }
  }
}
