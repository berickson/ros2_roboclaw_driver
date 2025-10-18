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
    
    // Note: getVersion() sends a command, so we skip it in diagnostics to avoid extra communication
    stat.add("Error Status", static_cast<int>(error_status));
    stat.add("Error String", error_string);
    stat.add("Connection State", "CONNECTED");
    
    const char* protection_state_names[] = {"NORMAL", "OVER_CURRENT_WARNING", "RECOVERY_WAITING", "RECOVERING"};
    stat.add("Current Protection State", protection_state_names[protection_state]);
  } catch (RoboClaw::TRoboClawException* e) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string("Exception: ") + e->what());
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