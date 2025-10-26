#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>

class MotorDriver {
 public:
  MotorDriver();
  static MotorDriver &singleton();

  void onInit(rclcpp::Node::SharedPtr node);

  // Getters for commanded velocities (for status publishing)
  float getM1CommandedVelocityMps() const { return m1_commanded_velocity_mps_; }
  float getM2CommandedVelocityMps() const { return m2_commanded_velocity_mps_; }
  
  // Getter for meters_per_quad_pulse (for status publishing)
  float getMetersPerQuadPulse() const { return meters_per_quad_pulse_; }

 private:
  void declareParameters();
  void eulerToQuaternion(float roll, float pitch, float yaw, float *q);
  void initializeParameters();
  void logParameters() const;
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void mainLoopThread();  // Main periodic loop: reads sensors, checks safety, publishes
  void watchdogTimerCallback();
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::thread main_loop_thread_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  int accel_quad_pulses_per_second_;
  int baud_rate_;
  std::string device_name_;
  int device_port_;
  bool do_debug_;
  bool do_low_level_debug_;
  float m1_pid_p_;
  float m1_pid_i_;
  float m1_pid_d_;
  int m1_pid_qpps_max_;
  float m1_max_current_;
  float m2_pid_p_;
  float m2_pid_i_;
  float m2_pid_d_;
  int m2_pid_qpps_max_;
  float m2_max_current_;
  float max_angular_velocity_;
  float max_linear_velocity_;
  float max_linear_acceleration_;  // NEW: SI unit for acceleration (m/s^2)
  float max_seconds_uncommanded_travel_;
  float meters_per_quad_pulse_;  // NEW: SI unit calibration (replaces wheel_radius + pulses_per_meter)
  bool publish_joint_states_;
  bool publish_odom_;
  int quad_pulses_per_meter_;
  float quad_pulses_per_revolution_;
  float sensor_update_rate_;  // Hz
  float serial_timeout_;  // seconds
  float wheel_radius_;
  float wheel_separation_;
  float current_filter_window_seconds_;  // For current averaging
  float recovery_timeout_seconds_;       // For auto-recovery from over-current

  // Track commanded velocities for debugging/monitoring
  float m1_commanded_velocity_mps_;  // meters per second
  float m2_commanded_velocity_mps_;  // meters per second

  // Coast timeout tracking
  float coast_timeout_seconds_;       // Time without cmd_vel before coasting (0.0=disabled)
  rclcpp::Time last_cmd_vel_time_;    // Time of last cmd_vel message received
  bool is_coasting_;                  // True when motors are in coast mode

  static MotorDriver *g_singleton;
};
