#include <rclcpp/rclcpp.hpp>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include "motor_driver.h"
#include "roboclaw.h"
#include "ros2_roboclaw_driver/msg/robo_claw_status.hpp"

// Diagnostic updater callback - single comprehensive status check
void diagnosticOverall(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  try {
    RoboClaw* roboclaw = RoboClaw::singleton();
    RoboClaw::ConnectionState conn_state = roboclaw->getConnectionState();
    
    // If disconnected, only report connection state - don't try to read device
    if (conn_state == RoboClaw::DISCONNECTED) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Disconnected");
      stat.add("Connection State", "DISCONNECTED");
      return;
    }
    
    // Device is connected - safe to read cached values
    uint32_t error_status = roboclaw->getErrorStatus();
    std::string error_string = roboclaw->getErrorString();
    RoboClaw::OverCurrentState protection_state = roboclaw->getCurrentProtectionState();
    
    // Determine overall status level
    if (error_status & 0x03FC) {  // Error bits 2-9 (E-stop, temp errors, driver faults, battery errors)
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error_string);
    } else if (error_status & 0x3C00) {  // Warning bits 10-13 (battery/temp warnings)
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, error_string);
    } else if (protection_state == RoboClaw::OVER_CURRENT_WARNING || 
               protection_state == RoboClaw::RECOVERY_WAITING) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Current protection active");
    } else if (error_status == 0 && protection_state == RoboClaw::NORMAL) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, error_string);
    }
    
    // Note: getVersion() sends a command, so we use cached version instead
    
    // Firmware version (cached at startup)
    stat.add("Firmware Version", roboclaw->getCachedFirmwareVersion());
    
    // Get current time for elapsed time calculations
    auto now = std::chrono::steady_clock::now();
    
    // Connection details
    stat.add("Connection State", "CONNECTED");
    stat.add("Consecutive Errors", static_cast<int>(roboclaw->getConsecutiveErrors()));
    stat.add("Total Messages", static_cast<int>(roboclaw->getTotalMessages()));
    stat.add("Total Errors", static_cast<int>(roboclaw->getTotalErrors()));
    
    // Error statistics breakdown
    const RoboClaw::ErrorStats& error_stats = roboclaw->getErrorStats();
    stat.add("Total Timeouts", static_cast<int>(error_stats.total_timeouts));
    stat.add("Total CRC Errors", static_cast<int>(error_stats.total_crc_errors));
    stat.add("Total ACK Errors", static_cast<int>(error_stats.total_ack_errors));
    stat.add("Total Device Not Responding", static_cast<int>(error_stats.total_device_not_responding));
    stat.add("Total Communication Errors", static_cast<int>(error_stats.total_communication_errors));
    
    // Last error details
    if (!error_stats.last_error_message.empty()) {
      stat.add("Last Error Message", error_stats.last_error_message);
      auto error_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - error_stats.last_error_time);
      stat.add("Last Error Time (ms ago)", static_cast<int>(error_elapsed.count()));
    }
    
    // Calculate time since last successful communication
    auto last_comm = roboclaw->getLastSuccessfulCommunication();
    auto comm_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_comm);
    stat.add("Last Successful Communication (ms ago)", static_cast<int>(comm_elapsed.count()));
    
    // Hardware status
    stat.add("Error Status", static_cast<int>(error_status));
    stat.add("Error String", error_string);
    
    // Battery voltages
    float main_battery = roboclaw->getMainBatteryLevel();
    float logic_battery = roboclaw->getLogicBatteryLevel();
    stat.add("Main Battery Voltage", main_battery);
    stat.add("Logic Battery Voltage", logic_battery);
    
    // Check battery warning/error flags from device
    if (error_status & 0x0020) stat.add("Main Battery High Error", "TRUE");
    if (error_status & 0x0040) stat.add("Logic Battery High Error", "TRUE");
    if (error_status & 0x0080) stat.add("Logic Battery Low Error", "TRUE");
    if (error_status & 0x0400) stat.add("Main Battery High Warning", "TRUE");
    if (error_status & 0x0800) stat.add("Main Battery Low Warning", "TRUE");
    
    // Motor currents (instantaneous from device + smoothed from driver)
    RoboClaw::TMotorCurrents currents = roboclaw->getMotorCurrents();
    stat.add("Motor 1 Current", currents.m1Current);
    stat.add("Motor 1 Current Smoothed", roboclaw->getM1CurrentSmoothed());
    stat.add("Motor 2 Current", currents.m2Current);
    stat.add("Motor 2 Current Smoothed", roboclaw->getM2CurrentSmoothed());
    
    // Motor encoder positions, velocities, and status
    stat.add("Motor 1 Encoder", static_cast<int>(roboclaw->getM1Encoder()));
    stat.add("Motor 1 Encoder Status", static_cast<int>(roboclaw->getM1EncoderStatus()));
    stat.add("Motor 1 Velocity", static_cast<int>(roboclaw->getVelocity(RoboClaw::kM1)));
    stat.add("Motor 2 Encoder", static_cast<int>(roboclaw->getM2Encoder()));
    stat.add("Motor 2 Encoder Status", static_cast<int>(roboclaw->getM2EncoderStatus()));
    stat.add("Motor 2 Velocity", static_cast<int>(roboclaw->getVelocity(RoboClaw::kM2)));
    
    // Check motor fault flags from device
    if (error_status & 0x0001) stat.add("Motor 1 Over-Current Warning", "TRUE");
    if (error_status & 0x0200) stat.add("Motor 1 Driver Fault", "TRUE");
    if (error_status & 0x0002) stat.add("Motor 2 Over-Current Warning", "TRUE");
    if (error_status & 0x0100) stat.add("Motor 2 Driver Fault", "TRUE");
    
    // Temperature
    float temperature = roboclaw->getTemperature();
    stat.add("Temperature", temperature);
    
    // Check temperature warning/error flags from device
    if (error_status & 0x0004) stat.add("E-Stop", "TRUE");
    if (error_status & 0x0008) stat.add("Temperature Error", "TRUE");
    if (error_status & 0x0010) stat.add("Temperature2 Error", "TRUE");
    if (error_status & 0x1000) stat.add("Temperature Warning", "TRUE");
    if (error_status & 0x2000) stat.add("Temperature2 Warning", "TRUE");
    
    // Current protection state (software state machine)
    const char* protection_state_names[] = {"NORMAL", "OVER_CURRENT_WARNING", "RECOVERY_WAITING", "RECOVERING"};
    stat.add("Current Protection State", protection_state_names[protection_state]);
    
    // Time since last non-zero cmd_vel (for recovery monitoring)
    auto last_cmd_vel = roboclaw->getLastNonzeroCmdVelTime();
    auto cmd_vel_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cmd_vel);
    stat.add("Time Since Nonzero Cmd_Vel (ms)", static_cast<int>(cmd_vel_elapsed.count()));
    
    // Performance: sensor update timing
    auto last_sensor_read = roboclaw->getLastSensorReadTime();
    auto sensor_now = std::chrono::system_clock::now();
    auto sensor_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(sensor_now - last_sensor_read);
    stat.add("Time Since Last Sensor Update (ms)", static_cast<int>(sensor_elapsed.count()));
    
    // Note: sensor_update_rate would require tracking time between readSensorGroup calls
    // This could be added in a future enhancement with additional state tracking
  } catch (const RoboClaw::TRoboClawException& e) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string("Exception: ") + e.what());
  } catch (const std::exception& e) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string("Std exception: ") + e.what());
  } catch (...) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unknown exception accessing device");
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared("roboclaw");
  MotorDriver &motorDriver = MotorDriver::singleton();
  motorDriver.onInit(node);

  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  qos.avoid_ros_namespace_conventions(false);

  std::string statusTopicName;
  node->get_parameter("roboclaw_status_topic", statusTopicName);
  RCUTILS_LOG_INFO("[motor_driver_node] roboclaw_status_topic: %s",
                   statusTopicName.c_str());

  rclcpp::Publisher<ros2_roboclaw_driver::msg::RoboClawStatus>::SharedPtr
      statusPublisher =
          node->create_publisher<ros2_roboclaw_driver::msg::RoboClawStatus>(
              statusTopicName, qos);

  // Create diagnostic updater
  diagnostic_updater::Updater diagnostic_updater(node);
  diagnostic_updater.setHardwareID("RoboClaw");
  diagnostic_updater.add("roboclaw", diagnosticOverall);

  ros2_roboclaw_driver::msg::RoboClawStatus roboClawStatus;
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {
    try {
      roboClawStatus.logic_battery_voltage =
          RoboClaw::singleton()->getLogicBatteryLevel();
      roboClawStatus.main_battery_voltage =
          RoboClaw::singleton()->getMainBatteryLevel();
      RoboClaw::TMotorCurrents motorCurrents =
          RoboClaw::singleton()->getMotorCurrents();
      roboClawStatus.m1_motor_current = motorCurrents.m1Current;
      roboClawStatus.m2_motor_current = motorCurrents.m2Current;

      RoboClaw::TPIDQ pidq = RoboClaw::singleton()->getPIDQM1();

      roboClawStatus.m1_p = pidq.p;
      roboClawStatus.m1_i = pidq.i;
      roboClawStatus.m1_d = pidq.d;
      roboClawStatus.m1_qpps = pidq.qpps;

      pidq = RoboClaw::singleton()->getPIDQM2();
      roboClawStatus.m2_p = pidq.p;
      roboClawStatus.m2_i = pidq.i;
      roboClawStatus.m2_d = pidq.d;
      roboClawStatus.m2_qpps = pidq.qpps;

      roboClawStatus.temperature = RoboClaw::singleton()->getTemperature();

      roboClawStatus.m1_encoder_value = RoboClaw::singleton()->getM1Encoder();
      roboClawStatus.m1_encoder_status =
          RoboClaw::singleton()->getM1EncoderStatus();
      roboClawStatus.m2_encoder_value = RoboClaw::singleton()->getM2Encoder();
      roboClawStatus.m2_encoder_status =
          RoboClaw::singleton()->getM2EncoderStatus();

      roboClawStatus.m1_current_speed =
          RoboClaw::singleton()->getVelocity(RoboClaw::kM1);
      roboClawStatus.m2_current_speed =
          RoboClaw::singleton()->getVelocity(RoboClaw::kM2);

      roboClawStatus.error_string = RoboClaw::singleton()->getErrorString();

      // Only publish status when connected (connection state is in diagnostics)
      if (RoboClaw::singleton()->getConnectionState() == RoboClaw::CONNECTED) {
        statusPublisher->publish(roboClawStatus);
      }
    } catch (RoboClaw::TRoboClawException *e) {
      RCUTILS_LOG_ERROR("[motor_driver_node] Exception: %s", e->what());
    } catch (...) {
      RCUTILS_LOG_ERROR("[motor_driver_node] Uncaught exception !!!");
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}