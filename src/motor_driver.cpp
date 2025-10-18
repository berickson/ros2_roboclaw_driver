#include "motor_driver.h"

#include <math.h>
#include <rcutils/logging_macros.h>
#include <stdint.h>

#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "roboclaw.h"

MotorDriver::MotorDriver()
    : device_name_("foo_bar"),
      wheel_radius_(0.10169),
      wheel_separation_(0.345) {
}

void MotorDriver::declareParameters() {
  // 1. Device Connection (Read-Only, require restart)
  rcl_interfaces::msg::ParameterDescriptor device_name_desc;
  device_name_desc.description = "Serial device path for RoboClaw";
  device_name_desc.read_only = true;
  node_->declare_parameter<std::string>("device_name", "roboclaw", device_name_desc);
  
  rcl_interfaces::msg::ParameterDescriptor baud_rate_desc;
  baud_rate_desc.description = "Serial baud rate (must match RoboClaw config)";
  baud_rate_desc.read_only = true;
  baud_rate_desc.integer_range.resize(1);
  baud_rate_desc.integer_range[0].from_value = 9600;
  baud_rate_desc.integer_range[0].to_value = 460800;
  baud_rate_desc.integer_range[0].step = 0;
  node_->declare_parameter<int>("baud_rate", 38400, baud_rate_desc);
  
  rcl_interfaces::msg::ParameterDescriptor device_port_desc;
  device_port_desc.description = "RoboClaw device address on serial bus";
  device_port_desc.read_only = true;
  device_port_desc.integer_range.resize(1);
  device_port_desc.integer_range[0].from_value = 128;
  device_port_desc.integer_range[0].to_value = 135;
  device_port_desc.integer_range[0].step = 1;
  node_->declare_parameter<int>("device_port", 123, device_port_desc);
  
  rcl_interfaces::msg::ParameterDescriptor serial_timeout_desc;
  serial_timeout_desc.description = "Timeout for serial communication in seconds (0.0=disabled)";
  serial_timeout_desc.read_only = true;
  serial_timeout_desc.floating_point_range.resize(1);
  serial_timeout_desc.floating_point_range[0].from_value = 0.0;
  serial_timeout_desc.floating_point_range[0].to_value = 5.0;
  serial_timeout_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("serial_timeout", 0.5, serial_timeout_desc);
  
  // 2. Robot Geometry (Read-Only, require restart)
  rcl_interfaces::msg::ParameterDescriptor wheel_separation_desc;
  wheel_separation_desc.description = "Distance between left and right wheel contact points (meters)";
  wheel_separation_desc.read_only = true;
  wheel_separation_desc.floating_point_range.resize(1);
  wheel_separation_desc.floating_point_range[0].from_value = 0.1;
  wheel_separation_desc.floating_point_range[0].to_value = 2.0;
  wheel_separation_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("wheel_separation", 0.0, wheel_separation_desc);
  
  rcl_interfaces::msg::ParameterDescriptor meters_per_quad_pulse_desc;
  meters_per_quad_pulse_desc.description = "Linear distance traveled per encoder count (meters). Measure by driving robot a known distance and dividing by encoder counts. Set to 0.0 to use deprecated quad_pulses_per_meter instead.";
  meters_per_quad_pulse_desc.read_only = true;
  meters_per_quad_pulse_desc.floating_point_range.resize(1);
  meters_per_quad_pulse_desc.floating_point_range[0].from_value = 0.0;
  meters_per_quad_pulse_desc.floating_point_range[0].to_value = 0.01;
  meters_per_quad_pulse_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("meters_per_quad_pulse", 0.0, meters_per_quad_pulse_desc);
  
  rcl_interfaces::msg::ParameterDescriptor wheel_radius_desc;
  wheel_radius_desc.description = "DEPRECATED: Use meters_per_quad_pulse instead. Wheel radius in meters";
  wheel_radius_desc.read_only = true;
  wheel_radius_desc.floating_point_range.resize(1);
  wheel_radius_desc.floating_point_range[0].from_value = 0.01;
  wheel_radius_desc.floating_point_range[0].to_value = 0.5;
  wheel_radius_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("wheel_radius", 0.0, wheel_radius_desc);
  
  rcl_interfaces::msg::ParameterDescriptor quad_pulses_per_meter_desc;
  quad_pulses_per_meter_desc.description = "DEPRECATED: Use meters_per_quad_pulse instead. Encoder counts per meter";
  quad_pulses_per_meter_desc.read_only = true;
  quad_pulses_per_meter_desc.integer_range.resize(1);
  quad_pulses_per_meter_desc.integer_range[0].from_value = 100;
  quad_pulses_per_meter_desc.integer_range[0].to_value = 10000;
  quad_pulses_per_meter_desc.integer_range[0].step = 0;
  node_->declare_parameter<int>("quad_pulses_per_meter", 0, quad_pulses_per_meter_desc);
  
  rcl_interfaces::msg::ParameterDescriptor quad_pulses_per_revolution_desc;
  quad_pulses_per_revolution_desc.description = "DEPRECATED: Use meters_per_quad_pulse instead. Encoder counts per wheel revolution";
  quad_pulses_per_revolution_desc.read_only = true;
  quad_pulses_per_revolution_desc.floating_point_range.resize(1);
  quad_pulses_per_revolution_desc.floating_point_range[0].from_value = 100.0;
  quad_pulses_per_revolution_desc.floating_point_range[0].to_value = 10000.0;
  quad_pulses_per_revolution_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("quad_pulses_per_revolution", 0, quad_pulses_per_revolution_desc);
  
  // 3. Motor Control - PID Tuning (Runtime changeable)
  rcl_interfaces::msg::ParameterDescriptor m1_p_desc;
  m1_p_desc.description = "Motor 1 (left) PID proportional gain";
  m1_p_desc.floating_point_range.resize(1);
  m1_p_desc.floating_point_range[0].from_value = 0.0;
  m1_p_desc.floating_point_range[0].to_value = 100000.0;
  m1_p_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("m1_p", 0.0, m1_p_desc);
  
  rcl_interfaces::msg::ParameterDescriptor m1_i_desc;
  m1_i_desc.description = "Motor 1 (left) PID integral gain";
  m1_i_desc.floating_point_range.resize(1);
  m1_i_desc.floating_point_range[0].from_value = 0.0;
  m1_i_desc.floating_point_range[0].to_value = 100000.0;
  m1_i_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("m1_i", 0.0, m1_i_desc);
  
  rcl_interfaces::msg::ParameterDescriptor m1_d_desc;
  m1_d_desc.description = "Motor 1 (left) PID derivative gain";
  m1_d_desc.floating_point_range.resize(1);
  m1_d_desc.floating_point_range[0].from_value = 0.0;
  m1_d_desc.floating_point_range[0].to_value = 100000.0;
  m1_d_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("m1_d", 0.0, m1_d_desc);
  
  rcl_interfaces::msg::ParameterDescriptor m1_qpps_desc;
  m1_qpps_desc.description = "Motor 1 max speed in quad pulses/sec (for PID scaling, not a limit)";
  m1_qpps_desc.integer_range.resize(1);
  m1_qpps_desc.integer_range[0].from_value = 0;
  m1_qpps_desc.integer_range[0].to_value = 1000000;
  m1_qpps_desc.integer_range[0].step = 0;
  node_->declare_parameter<int>("m1_qpps", 0, m1_qpps_desc);
  
  rcl_interfaces::msg::ParameterDescriptor m2_p_desc;
  m2_p_desc.description = "Motor 2 (right) PID proportional gain";
  m2_p_desc.floating_point_range.resize(1);
  m2_p_desc.floating_point_range[0].from_value = 0.0;
  m2_p_desc.floating_point_range[0].to_value = 100000.0;
  m2_p_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("m2_p", 0.0, m2_p_desc);
  
  rcl_interfaces::msg::ParameterDescriptor m2_i_desc;
  m2_i_desc.description = "Motor 2 (right) PID integral gain";
  m2_i_desc.floating_point_range.resize(1);
  m2_i_desc.floating_point_range[0].from_value = 0.0;
  m2_i_desc.floating_point_range[0].to_value = 100000.0;
  m2_i_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("m2_i", 0.0, m2_i_desc);
  
  rcl_interfaces::msg::ParameterDescriptor m2_d_desc;
  m2_d_desc.description = "Motor 2 (right) PID derivative gain";
  m2_d_desc.floating_point_range.resize(1);
  m2_d_desc.floating_point_range[0].from_value = 0.0;
  m2_d_desc.floating_point_range[0].to_value = 100000.0;
  m2_d_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("m2_d", 0.0, m2_d_desc);
  
  rcl_interfaces::msg::ParameterDescriptor m2_qpps_desc;
  m2_qpps_desc.description = "Motor 2 max speed in quad pulses/sec (for PID scaling, not a limit)";
  m2_qpps_desc.integer_range.resize(1);
  m2_qpps_desc.integer_range[0].from_value = 0;
  m2_qpps_desc.integer_range[0].to_value = 1000000;
  m2_qpps_desc.integer_range[0].step = 0;
  node_->declare_parameter<int>("m2_qpps", 0, m2_qpps_desc);
  
  // 4. Motion Limits (Runtime changeable)
  rcl_interfaces::msg::ParameterDescriptor max_linear_velocity_desc;
  max_linear_velocity_desc.description = "Maximum forward/backward velocity (m/s)";
  max_linear_velocity_desc.floating_point_range.resize(1);
  max_linear_velocity_desc.floating_point_range[0].from_value = 0.0;
  max_linear_velocity_desc.floating_point_range[0].to_value = 10.0;
  max_linear_velocity_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("max_linear_velocity", 0.0, max_linear_velocity_desc);
  
  rcl_interfaces::msg::ParameterDescriptor max_angular_velocity_desc;
  max_angular_velocity_desc.description = "Maximum rotation velocity (rad/s)";
  max_angular_velocity_desc.floating_point_range.resize(1);
  max_angular_velocity_desc.floating_point_range[0].from_value = 0.0;
  max_angular_velocity_desc.floating_point_range[0].to_value = 10.0;
  max_angular_velocity_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("max_angular_velocity", 0.0, max_angular_velocity_desc);
  
  rcl_interfaces::msg::ParameterDescriptor max_linear_acceleration_desc;
  max_linear_acceleration_desc.description = "Maximum linear acceleration (m/s^2). Set to 0.0 to use deprecated accel_quad_pulses_per_second instead.";
  max_linear_acceleration_desc.floating_point_range.resize(1);
  max_linear_acceleration_desc.floating_point_range[0].from_value = 0.0;
  max_linear_acceleration_desc.floating_point_range[0].to_value = 20.0;
  max_linear_acceleration_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("max_linear_acceleration", 0.0, max_linear_acceleration_desc);
  
  rcl_interfaces::msg::ParameterDescriptor accel_quad_pulses_per_second_desc;
  accel_quad_pulses_per_second_desc.description = "DEPRECATED: Use max_linear_acceleration instead. Maximum acceleration in quad pulses/sec^2";
  accel_quad_pulses_per_second_desc.integer_range.resize(1);
  accel_quad_pulses_per_second_desc.integer_range[0].from_value = 100;
  accel_quad_pulses_per_second_desc.integer_range[0].to_value = 100000;
  accel_quad_pulses_per_second_desc.integer_range[0].step = 0;
  node_->declare_parameter<int>("accel_quad_pulses_per_second", 600, accel_quad_pulses_per_second_desc);
  
  rcl_interfaces::msg::ParameterDescriptor m1_max_current_desc;
  m1_max_current_desc.description = "Motor 1 current limit (trip threshold) in Amps";
  m1_max_current_desc.floating_point_range.resize(1);
  m1_max_current_desc.floating_point_range[0].from_value = 0.0;
  m1_max_current_desc.floating_point_range[0].to_value = 50.0;
  m1_max_current_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("m1_max_current", 0.0, m1_max_current_desc);
  
  rcl_interfaces::msg::ParameterDescriptor m2_max_current_desc;
  m2_max_current_desc.description = "Motor 2 current limit (trip threshold) in Amps";
  m2_max_current_desc.floating_point_range.resize(1);
  m2_max_current_desc.floating_point_range[0].from_value = 0.0;
  m2_max_current_desc.floating_point_range[0].to_value = 50.0;
  m2_max_current_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("m2_max_current", 0.0, m2_max_current_desc);
  
  // 5. Current Protection (Runtime changeable)
  rcl_interfaces::msg::ParameterDescriptor filter_desc;
  filter_desc.description = "Time window for averaging current readings in seconds. 0.0=instantaneous, >0.0=filtered";
  filter_desc.floating_point_range.resize(1);
  filter_desc.floating_point_range[0].from_value = 0.0;
  filter_desc.floating_point_range[0].to_value = 10.0;
  filter_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("current_filter_window_seconds", 1.0, filter_desc);
  
  rcl_interfaces::msg::ParameterDescriptor recovery_desc;
  recovery_desc.description = "Seconds of zero cmd_vel before auto-recovery from over-current. 0.0=no auto-recovery";
  recovery_desc.floating_point_range.resize(1);
  recovery_desc.floating_point_range[0].from_value = 0.0;
  recovery_desc.floating_point_range[0].to_value = 60.0;
  recovery_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("recovery_timeout_seconds", 5.0, recovery_desc);
  
  // 6. Safety (Runtime changeable)
  rcl_interfaces::msg::ParameterDescriptor watchdog_desc;
  watchdog_desc.description = "Watchdog timeout - stops motors if no cmd_vel received (seconds)";
  watchdog_desc.floating_point_range.resize(1);
  watchdog_desc.floating_point_range[0].from_value = 0.0;
  watchdog_desc.floating_point_range[0].to_value = 5.0;
  watchdog_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("max_seconds_uncommanded_travel", 0.0, watchdog_desc);
  
  // 7. Publishing & Topics (Read-Only, require restart)
  rcl_interfaces::msg::ParameterDescriptor publish_joint_states_desc;
  publish_joint_states_desc.description = "Publish wheel joint states for robot_state_publisher";
  publish_joint_states_desc.read_only = true;
  node_->declare_parameter<bool>("publish_joint_states", true, publish_joint_states_desc);
  
  rcl_interfaces::msg::ParameterDescriptor publish_odom_desc;
  publish_odom_desc.description = "Publish odometry on /odom topic";
  publish_odom_desc.read_only = true;
  node_->declare_parameter<bool>("publish_odom", true, publish_odom_desc);
  
  rcl_interfaces::msg::ParameterDescriptor roboclaw_status_topic_desc;
  roboclaw_status_topic_desc.description = "Topic name for RoboClaw status messages";
  roboclaw_status_topic_desc.read_only = true;
  node_->declare_parameter<std::string>("roboclaw_status_topic", "roboclaw_status", roboclaw_status_topic_desc);
  
  rcl_interfaces::msg::ParameterDescriptor sensor_update_rate_desc;
  sensor_update_rate_desc.description = "Rate for reading encoders and publishing status (Hz)";
  sensor_update_rate_desc.read_only = true;
  sensor_update_rate_desc.floating_point_range.resize(1);
  sensor_update_rate_desc.floating_point_range[0].from_value = 1.0;
  sensor_update_rate_desc.floating_point_range[0].to_value = 100.0;
  sensor_update_rate_desc.floating_point_range[0].step = 0.0;
  node_->declare_parameter<float>("sensor_update_rate", 20.0, sensor_update_rate_desc);
  
  // 8. Debug & Logging (Runtime changeable)
  rcl_interfaces::msg::ParameterDescriptor do_debug_desc;
  do_debug_desc.description = "Enable protocol-level debug logging (command packets)";
  node_->declare_parameter<bool>("do_debug", false, do_debug_desc);
  
  rcl_interfaces::msg::ParameterDescriptor do_low_level_debug_desc;
  do_low_level_debug_desc.description = "Enable byte-level serial debug logging (every read/write) - WARNING: massive output";
  node_->declare_parameter<bool>("do_low_level_debug", false, do_low_level_debug_desc);
}

void MotorDriver::initializeParameters() {
  node_->get_parameter("accel_quad_pulses_per_second",
                      accel_quad_pulses_per_second_);
  node_->get_parameter("baud_rate", baud_rate_);
  node_->get_parameter("device_name", device_name_);
  node_->get_parameter("device_port", device_port_);
  node_->get_parameter("do_debug", do_debug_);
  node_->get_parameter("do_low_level_debug", do_low_level_debug_);
  node_->get_parameter("m1_p", m1_p_);
  node_->get_parameter("m1_i", m1_i_);
  node_->get_parameter("m1_d", m1_d_);
  node_->get_parameter("m1_qpps", m1_qpps_);
  node_->get_parameter("m1_max_current", m1_max_current_);
  node_->get_parameter("m2_p", m2_p_);
  node_->get_parameter("m2_i", m2_i_);
  node_->get_parameter("m2_d", m2_d_);
  node_->get_parameter("m2_qpps", m2_qpps_);
  node_->get_parameter("m2_max_current", m2_max_current_);
  node_->get_parameter("max_angular_velocity", max_angular_velocity_);
  node_->get_parameter("max_linear_velocity", max_linear_velocity_);
  node_->get_parameter("max_linear_acceleration", max_linear_acceleration_);
  node_->get_parameter("max_seconds_uncommanded_travel",
                      max_seconds_uncommanded_travel_);
  node_->get_parameter("meters_per_quad_pulse", meters_per_quad_pulse_);
  node_->get_parameter("publish_joint_states", publish_joint_states_);
  node_->get_parameter("publish_odom", publish_odom_);
  node_->get_parameter("quad_pulses_per_meter", quad_pulses_per_meter_);
  node_->get_parameter("quad_pulses_per_revolution",
                      quad_pulses_per_revolution_);
  node_->get_parameter("sensor_update_rate", sensor_update_rate_);
  node_->get_parameter("serial_timeout", serial_timeout_);
  node_->get_parameter("wheel_radius", wheel_radius_);
  node_->get_parameter("wheel_separation", wheel_separation_);
  node_->get_parameter("current_filter_window_seconds", current_filter_window_seconds_);
  node_->get_parameter("recovery_timeout_seconds", recovery_timeout_seconds_);
  
  // Handle backward compatibility and compute derived values
  if (meters_per_quad_pulse_ == 0.0 && quad_pulses_per_meter_ > 0) {
    // Convert from deprecated quad_pulses_per_meter
    meters_per_quad_pulse_ = 1.0 / quad_pulses_per_meter_;
    RCUTILS_LOG_WARN("Using deprecated quad_pulses_per_meter parameter. Please use meters_per_quad_pulse instead.");
  }
  
  if (max_linear_acceleration_ == 0.0 && accel_quad_pulses_per_second_ > 0 && meters_per_quad_pulse_ > 0.0) {
    // Convert from deprecated accel_quad_pulses_per_second
    max_linear_acceleration_ = accel_quad_pulses_per_second_ * meters_per_quad_pulse_;
    RCUTILS_LOG_WARN("Using deprecated accel_quad_pulses_per_second parameter. Please use max_linear_acceleration instead.");
  }
  
  // Convert max_linear_acceleration back to quad pulses for RoboClaw commands
  if (max_linear_acceleration_ > 0.0 && meters_per_quad_pulse_ > 0.0) {
    accel_quad_pulses_per_second_ = static_cast<int>(max_linear_acceleration_ / meters_per_quad_pulse_);
  }

  logParameters();
}

void MotorDriver::logParameters() const {
  RCUTILS_LOG_INFO("accel_quad_pulses_per_second: %d",
                   accel_quad_pulses_per_second_);
  RCUTILS_LOG_INFO("baud_rate: %d", baud_rate_);
  RCUTILS_LOG_INFO("device_name: %s", device_name_.c_str());
  RCUTILS_LOG_INFO("device_port: %d", device_port_);
  RCUTILS_LOG_INFO("do_debug: %s", do_debug_ ? "True" : "False");
  RCUTILS_LOG_INFO("do_low_level_debug: %s",
                   do_low_level_debug_ ? "True" : "False");
  RCUTILS_LOG_INFO("m1_p: %f", m1_p_);
  RCUTILS_LOG_INFO("m1_i: %f", m1_i_);
  RCUTILS_LOG_INFO("m1_d: %f", m1_d_);
  RCUTILS_LOG_INFO("m1_qpps: %d", m1_qpps_);
  RCUTILS_LOG_INFO("m1_max_current: %f", m1_max_current_);
  RCUTILS_LOG_INFO("m2_p: %f", m2_p_);
  RCUTILS_LOG_INFO("m2_i: %f", m2_i_);
  RCUTILS_LOG_INFO("m2_d: %f", m2_d_);
  RCUTILS_LOG_INFO("m2_qpps: %d", m2_qpps_);
  RCUTILS_LOG_INFO("m2_max_current: %f", m2_max_current_);
  RCUTILS_LOG_INFO("max_angular_velocity: %f", max_angular_velocity_);
  RCUTILS_LOG_INFO("max_linear_velocity: %f", max_linear_velocity_);
  RCUTILS_LOG_INFO("max_seconds_uncommanded_travel: %f",
                   max_seconds_uncommanded_travel_);
  RCUTILS_LOG_INFO("publish_joint_states: %s",
                   publish_joint_states_ ? "True" : "False");
  RCUTILS_LOG_INFO("publish_odom: %s", publish_odom_ ? "True" : "False");
  RCUTILS_LOG_INFO("quad_pulses_per_meter: %d", quad_pulses_per_meter_);
  RCUTILS_LOG_INFO("quad_pulses_per_revolution: %3.4f",
                   quad_pulses_per_revolution_);
  RCUTILS_LOG_INFO("sensor_update_rate: %f", sensor_update_rate_);
  RCUTILS_LOG_INFO("serial_timeout: %f", serial_timeout_);
  RCUTILS_LOG_INFO("wheel_radius: %f", wheel_radius_);
  RCUTILS_LOG_INFO("wheel_separation: %f", wheel_separation_);
  RCUTILS_LOG_INFO("current_filter_window_seconds: %f", current_filter_window_seconds_);
  RCUTILS_LOG_INFO("recovery_timeout_seconds: %f", recovery_timeout_seconds_);
}

void MotorDriver::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (RoboClaw::singleton() != nullptr) {
    // Check connection state first
    if (RoboClaw::singleton()->getConnectionState() == RoboClaw::DISCONNECTED) {
      // Don't send commands when disconnected
      return;
    }
    
    double x_velocity =
        std::min(std::max((float)msg->linear.x, -max_linear_velocity_),
                 max_linear_velocity_);
    double yaw_velocity =
        std::min(std::max((float)msg->angular.z, -max_angular_velocity_),
                 max_angular_velocity_);
    
    bool is_zero = (msg->linear.x == 0) && (msg->angular.z == 0);
    RoboClaw::singleton()->notifyCmdVel(is_zero);
    
    // Block movement commands if in fault state
    auto state = RoboClaw::singleton()->getCurrentProtectionState();
    if (state == RoboClaw::OVER_CURRENT_WARNING || state == RoboClaw::RECOVERY_WAITING) {
      // Don't send movement commands while in fault/recovery state
      RoboClaw::singleton()->stop();
      return;
    }
    
    if (is_zero) {
      RoboClaw::singleton()->stop();
    } else if ((fabs(x_velocity) > 0.01) || (fabs(yaw_velocity) > 0.01)) {
      const double m1_desired_velocity =
          x_velocity - (yaw_velocity * wheel_separation_ / 2.0) / wheel_radius_;
      const double m2_desired_velocity =
          x_velocity + (yaw_velocity * wheel_separation_ / 2.0) / wheel_radius_;

      // Convert m/s to quad pulses per second
      const int32_t m1_quad_pulses_per_second =
          m1_desired_velocity / meters_per_quad_pulse_;
      const int32_t m2_quad_pulses_per_second =
          m2_desired_velocity / meters_per_quad_pulse_;
      
      // Use immediate speed+accel command (no distance buffering)
      RoboClaw::singleton()->doMixedSpeedAccel(
          accel_quad_pulses_per_second_, m1_quad_pulses_per_second,
          m2_quad_pulses_per_second);
    }
  }
}

void MotorDriver::onInit(rclcpp::Node::SharedPtr node) {
  node_ = node;
  declareParameters();
  initializeParameters();
  
  // Read current protection parameters from the shared node
  node_->get_parameter("current_filter_window_seconds", current_filter_window_seconds_);
  node_->get_parameter("recovery_timeout_seconds", recovery_timeout_seconds_);

  RoboClaw::TPIDQ m1Pid = {m1_p_, m1_i_, m1_d_, (uint32_t)m1_qpps_,
                           m1_max_current_};
  RoboClaw::TPIDQ m2Pid = {m2_p_, m2_i_, m2_d_, (uint32_t)m2_qpps_,
                           m2_max_current_};

  new RoboClaw(m1Pid, m2Pid, m1_max_current_, m2_max_current_,
               device_name_.c_str(), device_port_, baud_rate_, serial_timeout_,
               do_debug_, do_low_level_debug_);
  
  // Configure current protection parameters
  RoboClaw::singleton()->setCurrentProtectionParams(
      current_filter_window_seconds_, recovery_timeout_seconds_, sensor_update_rate_);
  
  RCUTILS_LOG_INFO("Main battery: %f",
                   RoboClaw::singleton()->getMainBatteryLevel());

  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(rclcpp::DurabilityPolicy::Volatile);
  qos.avoid_ros_namespace_conventions(false);

  cmdVelSub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos,
      std::bind(&MotorDriver::cmdVelCallback, this, std::placeholders::_1));

  // Set up parameter callback for runtime updates on the shared node
  auto param_callback = [this](const std::vector<rclcpp::Parameter> &parameters) {
    return this->parametersCallback(parameters);
  };
  param_callback_handle_ = node_->add_on_set_parameters_callback(param_callback);

  if (publish_joint_states_) {
    joint_state_publisher_ =
        node_->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                             qos);
  }

  if (publish_odom_) {
    odom_publisher_ =
        node_->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  }

  // Start the publisher thread if we are publishing joint states or odometry
  // messages. This thread will read the RoboClaw sensors and publish the
  // corresponding messages at the specified sensor update rate.
  // The thread will run until the node is shut down.
  if (publish_joint_states_ || publish_odom_) {
    this->publisher_thread_ = std::thread(&MotorDriver::publisherThread, this);
  }
}

void MotorDriver::eulerToQuaternion(float roll, float pitch, float yaw,
                                    float *q) {
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  q[0] = cy * cp * cr + sy * sp * sr;
  q[1] = cy * cp * sr - sy * sp * cr;
  q[2] = sy * cp * sr + cy * sp * cr;
  q[3] = sy * cp * cr - cy * sp * sr;
}

void MotorDriver::publisherThread() {
  static rclcpp::Clock::SharedPtr clock =
      std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::WallRate loop_rate(sensor_update_rate_);
  rclcpp::Time now = clock->now();
  rclcpp::Time last_time = now;
  bool logged_disconnection = false;  // Only log disconnection once

  while (rclcpp::ok()) {
    loop_rate.sleep();
    if (RoboClaw::singleton() != nullptr) {
      try {
        RoboClaw::singleton()->readSensorGroup();

        nav_msgs::msg::Odometry odometry_msg;
      sensor_msgs::msg::JointState joint_state_msg;

      odometry_msg.header.stamp = clock->now();
      odometry_msg.header.frame_id = "base_link";

      joint_state_msg.header.stamp = clock->now();
      joint_state_msg.header.frame_id = "base_link";

      if (g_singleton->publish_joint_states_) {
        float encoder_left = RoboClaw::singleton()->getM1Encoder() * 1.0;
        float encoder_right = RoboClaw::singleton()->getM2Encoder() * 1.0;
        double radians_left =
            ((encoder_left * 1.0) / g_singleton->quad_pulses_per_revolution_) *
            2.0 * M_PI;
        double radians_right =
            ((encoder_right * 1.0) / g_singleton->quad_pulses_per_revolution_) *
            2.0 * M_PI;
        joint_state_msg.name.push_back("front_left_wheel");
        joint_state_msg.name.push_back("front_right_wheel");
        joint_state_msg.position.push_back(radians_left);
        joint_state_msg.position.push_back(radians_right);
        g_singleton->joint_state_publisher_->publish(joint_state_msg);
      }

      if (g_singleton->publish_odom_) {
        now = clock->now();
        double dt = (now - last_time).seconds();
        last_time = now;

        // Get velocities in quad pulses per second and convert to m/s
        float linear_velocity_x =
            RoboClaw::singleton()->getVelocity(RoboClaw::kM1) * g_singleton->meters_per_quad_pulse_;
        float linear_velocity_y =
            RoboClaw::singleton()->getVelocity(RoboClaw::kM2) * g_singleton->meters_per_quad_pulse_;
        float angular_velocity_z = (linear_velocity_x - linear_velocity_y) /
                                   g_singleton->wheel_separation_;

        // Calculate the robot's position and orientation
        static float x_pos_(0.0);
        static float y_pos_(0.0);
        static float heading_(0.0);
        static bool first_time = true;
        if (first_time) {
          first_time = false;
          x_pos_ = 0.0;
          y_pos_ = 0.0;
          heading_ = 0.0;
        }
        float delta_heading = angular_velocity_z * dt;  // radians
        float cos_h = cos(heading_);
        float sin_h = sin(heading_);
        float delta_x =
            (linear_velocity_x * cos_h - linear_velocity_y * sin_h) * dt;  // m
        float delta_y =
            (linear_velocity_x * sin_h + linear_velocity_y * cos_h) * dt;  // m
        // calculate current position of the robot
        x_pos_ += delta_x;
        y_pos_ += delta_y;
        heading_ += delta_heading;
        // calculate robot's heading in quaternion angle
        // ROS has a function to calculate yaw in quaternion angle
        float q[4];
        eulerToQuaternion(0, 0, heading_, q);

        // robot's position in x,y, and z
        odometry_msg.pose.pose.position.x = x_pos_;
        odometry_msg.pose.pose.position.y = y_pos_;
        odometry_msg.pose.pose.position.z = 0.0;
        // robot's heading in quaternion
        odometry_msg.pose.pose.orientation.x = (double)q[1];
        odometry_msg.pose.pose.orientation.y = (double)q[2];
        odometry_msg.pose.pose.orientation.z = (double)q[3];
        odometry_msg.pose.pose.orientation.w = (double)q[0];
        odometry_msg.pose.covariance[0] = 0.001;
        odometry_msg.pose.covariance[7] = 0.001;
        odometry_msg.pose.covariance[35] = 0.001;

        odometry_msg.twist.twist.linear.x = linear_velocity_x;
        odometry_msg.twist.twist.linear.y = linear_velocity_y;
        odometry_msg.twist.twist.linear.z = 0.0;
        odometry_msg.twist.twist.angular.x = 0.0;
        odometry_msg.twist.twist.angular.y = 0.0;
        odometry_msg.twist.twist.angular.z = angular_velocity_z;
        g_singleton->odom_publisher_->publish(odometry_msg);
      }
      
      // If we successfully read sensors, reset the disconnection log flag
      logged_disconnection = false;
      
      } catch (RoboClaw::TRoboClawException *e) {
        // Only log once when we become disconnected, not on every attempt
        if (!logged_disconnection) {
          RCUTILS_LOG_WARN("[MotorDriver::publisherThread] Exception reading sensors: %s", e->what());
          logged_disconnection = true;
        }
        // Continue running - don't let exception terminate thread
      } catch (...) {
        if (!logged_disconnection) {
          RCUTILS_LOG_WARN("[MotorDriver::publisherThread] Uncaught exception reading sensors");
          logged_disconnection = true;
        }
        // Continue running - don't let exception terminate thread
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult MotorDriver::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto &param : parameters) {
    const std::string &name = param.get_name();
    
    // PID Parameters (8 params) - Runtime changeable
    if (name == "m1_p") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 100000.0) {
        m1_p_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setM1PID(m1_p_, m1_i_, m1_d_, m1_qpps_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated m1_p to %.2f", new_value);
      } else {
        result.successful = false;
        result.reason = "m1_p must be between 0.0 and 100000.0";
        return result;
      }
    } else if (name == "m1_i") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 100000.0) {
        m1_i_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setM1PID(m1_p_, m1_i_, m1_d_, m1_qpps_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated m1_i to %.2f", new_value);
      } else {
        result.successful = false;
        result.reason = "m1_i must be between 0.0 and 100000.0";
        return result;
      }
    } else if (name == "m1_d") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 100000.0) {
        m1_d_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setM1PID(m1_p_, m1_i_, m1_d_, m1_qpps_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated m1_d to %.2f", new_value);
      } else {
        result.successful = false;
        result.reason = "m1_d must be between 0.0 and 100000.0";
        return result;
      }
    } else if (name == "m1_qpps") {
      int new_value = param.as_int();
      if (new_value >= 0 && new_value <= 1000000) {
        m1_qpps_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setM1PID(m1_p_, m1_i_, m1_d_, m1_qpps_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated m1_qpps to %d", new_value);
      } else {
        result.successful = false;
        result.reason = "m1_qpps must be between 0 and 1000000";
        return result;
      }
    } else if (name == "m2_p") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 100000.0) {
        m2_p_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setM2PID(m2_p_, m2_i_, m2_d_, m2_qpps_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated m2_p to %.2f", new_value);
      } else {
        result.successful = false;
        result.reason = "m2_p must be between 0.0 and 100000.0";
        return result;
      }
    } else if (name == "m2_i") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 100000.0) {
        m2_i_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setM2PID(m2_p_, m2_i_, m2_d_, m2_qpps_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated m2_i to %.2f", new_value);
      } else {
        result.successful = false;
        result.reason = "m2_i must be between 0.0 and 100000.0";
        return result;
      }
    } else if (name == "m2_d") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 100000.0) {
        m2_d_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setM2PID(m2_p_, m2_i_, m2_d_, m2_qpps_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated m2_d to %.2f", new_value);
      } else {
        result.successful = false;
        result.reason = "m2_d must be between 0.0 and 100000.0";
        return result;
      }
    } else if (name == "m2_qpps") {
      int new_value = param.as_int();
      if (new_value >= 0 && new_value <= 1000000) {
        m2_qpps_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setM2PID(m2_p_, m2_i_, m2_d_, m2_qpps_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated m2_qpps to %d", new_value);
      } else {
        result.successful = false;
        result.reason = "m2_qpps must be between 0 and 1000000";
        return result;
      }
    }
    
    // Motion Limits (5 params) - Runtime changeable
    else if (name == "max_linear_velocity") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 10.0) {
        max_linear_velocity_ = new_value;
        RCLCPP_INFO(node_->get_logger(), "Updated max_linear_velocity to %.2f m/s", new_value);
      } else {
        result.successful = false;
        result.reason = "max_linear_velocity must be between 0.0 and 10.0";
        return result;
      }
    } else if (name == "max_angular_velocity") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 10.0) {
        max_angular_velocity_ = new_value;
        RCLCPP_INFO(node_->get_logger(), "Updated max_angular_velocity to %.2f rad/s", new_value);
      } else {
        result.successful = false;
        result.reason = "max_angular_velocity must be between 0.0 and 10.0";
        return result;
      }
    } else if (name == "max_linear_acceleration") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 20.0) {
        max_linear_acceleration_ = new_value;
        // Update the derived accel_quad_pulses_per_second value
        if (meters_per_quad_pulse_ > 0.0) {
          accel_quad_pulses_per_second_ = static_cast<int>(new_value / meters_per_quad_pulse_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated max_linear_acceleration to %.2f m/s^2 (%d qpps)", 
                    new_value, accel_quad_pulses_per_second_);
      } else {
        result.successful = false;
        result.reason = "max_linear_acceleration must be between 0.0 and 20.0";
        return result;
      }
    } else if (name == "m1_max_current") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 50.0) {
        m1_max_current_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setMaxCurrents(m1_max_current_, m2_max_current_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated m1_max_current to %.2f A", new_value);
      } else {
        result.successful = false;
        result.reason = "m1_max_current must be between 0.0 and 50.0";
        return result;
      }
    } else if (name == "m2_max_current") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 50.0) {
        m2_max_current_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setMaxCurrents(m1_max_current_, m2_max_current_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated m2_max_current to %.2f A", new_value);
      } else {
        result.successful = false;
        result.reason = "m2_max_current must be between 0.0 and 50.0";
        return result;
      }
    }
    
    // Current Protection (2 params) - Runtime changeable
    else if (name == "current_filter_window_seconds") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 10.0) {
        current_filter_window_seconds_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setCurrentProtectionParams(
              current_filter_window_seconds_, recovery_timeout_seconds_, sensor_update_rate_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated current_filter_window_seconds to %.2f s", new_value);
      } else {
        result.successful = false;
        result.reason = "current_filter_window_seconds must be between 0.0 and 10.0";
        return result;
      }
    } else if (name == "recovery_timeout_seconds") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 60.0) {
        recovery_timeout_seconds_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setCurrentProtectionParams(
              current_filter_window_seconds_, recovery_timeout_seconds_, sensor_update_rate_);
        }
        RCLCPP_INFO(node_->get_logger(), "Updated recovery_timeout_seconds to %.2f s", new_value);
      } else {
        result.successful = false;
        result.reason = "recovery_timeout_seconds must be between 0.0 and 60.0";
        return result;
      }
    }
    
    // Safety (1 param) - Runtime changeable
    else if (name == "max_seconds_uncommanded_travel") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 5.0) {
        max_seconds_uncommanded_travel_ = new_value;
        RCLCPP_INFO(node_->get_logger(), "Updated max_seconds_uncommanded_travel to %.2f s", new_value);
      } else {
        result.successful = false;
        result.reason = "max_seconds_uncommanded_travel must be between 0.0 and 5.0";
        return result;
      }
    }
    
    // Debug & Logging (2 params) - Runtime changeable
    else if (name == "do_debug") {
      do_debug_ = param.as_bool();
      if (RoboClaw::singleton() != nullptr) {
        RoboClaw::singleton()->setDoDebug(do_debug_);
      }
      RCLCPP_INFO(node_->get_logger(), "Updated do_debug to %s", do_debug_ ? "true" : "false");
    } else if (name == "do_low_level_debug") {
      do_low_level_debug_ = param.as_bool();
      if (RoboClaw::singleton() != nullptr) {
        RoboClaw::singleton()->setDoLowLevelDebug(do_low_level_debug_);
      }
      RCLCPP_INFO(node_->get_logger(), "Updated do_low_level_debug to %s", do_low_level_debug_ ? "true" : "false");
    }
    
    // Read-only parameters - Reject runtime changes
    else if (name == "device_name" || name == "baud_rate" || name == "device_port" || 
             name == "serial_timeout" || name == "wheel_separation" || 
             name == "meters_per_quad_pulse" || name == "wheel_radius" || 
             name == "quad_pulses_per_meter" || name == "quad_pulses_per_revolution" ||
             name == "publish_joint_states" || name == "publish_odom" || 
             name == "roboclaw_status_topic" || name == "sensor_update_rate") {
      result.successful = false;
      result.reason = "Parameter '" + name + "' is read-only and requires node restart to change";
      return result;
    }
    
    // Deprecated parameters that should not be changed at runtime
    else if (name == "accel_quad_pulses_per_second") {
      result.successful = false;
      result.reason = "Parameter 'accel_quad_pulses_per_second' is deprecated. Use 'max_linear_acceleration' instead";
      return result;
    }
  }
  
  return result;
}

MotorDriver &MotorDriver::singleton() {
  if (!g_singleton) {
    g_singleton = new MotorDriver();
  }

  return *g_singleton;
}

MotorDriver *MotorDriver::g_singleton = nullptr;