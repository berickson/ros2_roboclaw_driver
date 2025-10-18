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
  node_->declare_parameter<int>("accel_quad_pulses_per_second", 600);
  node_->declare_parameter<int>("baud_rate", 38400);
  node_->declare_parameter<std::string>("device_name", "roboclaw");
  node_->declare_parameter<int>("device_port", 123);
  node_->declare_parameter<bool>("do_debug", false);
  node_->declare_parameter<bool>("do_low_level_debug", false);
  node_->declare_parameter<float>("m1_p", 0.0);
  node_->declare_parameter<float>("m1_i", 0.0);
  node_->declare_parameter<float>("m1_d", 0.0);
  node_->declare_parameter<int>("m1_qpps", 0);
  node_->declare_parameter<float>("m1_max_current", 0.0);
  node_->declare_parameter<float>("m2_p", 0.0);
  node_->declare_parameter<float>("m2_i", 0.0);
  node_->declare_parameter<float>("m2_d", 0.0);
  node_->declare_parameter<int>("m2_qpps", 0);
  node_->declare_parameter<float>("max_angular_velocity", 0.0);
  node_->declare_parameter<float>("max_linear_velocity", 0.0);
  node_->declare_parameter<float>("m2_max_current", 0.0);
  node_->declare_parameter<float>("max_seconds_uncommanded_travel", 0.0);
  node_->declare_parameter<bool>("publish_joint_states", true);
  node_->declare_parameter<bool>("publish_odom", true);
  node_->declare_parameter<int>("quad_pulses_per_meter", 0);
  node_->declare_parameter<float>("quad_pulses_per_revolution", 0);
  node_->declare_parameter<float>("sensor_update_rate", 20.0);  // Hz
  node_->declare_parameter<float>("serial_timeout", 0.5);  // seconds
  node_->declare_parameter<float>("wheel_radius", 0.0);
  node_->declare_parameter<float>("wheel_separation", 0.0);
  node_->declare_parameter<std::string>("roboclaw_status_topic", "roboclaw_status");
  
  // Declare current protection parameters with descriptions
  rcl_interfaces::msg::ParameterDescriptor filter_desc;
  filter_desc.description = "Time window for averaging motor current readings. 0.0=instantaneous (legacy), 0.1-10.0=averaged";
  node_->declare_parameter<float>("current_filter_window_seconds", 1.0, filter_desc);
  
  rcl_interfaces::msg::ParameterDescriptor recovery_desc;
  recovery_desc.description = "Seconds of zero cmd_vel required before auto-recovery from over-current. 0.0=disabled";
  node_->declare_parameter<float>("recovery_timeout_seconds", 5.0, recovery_desc);
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
  node_->get_parameter("max_seconds_uncommanded_travel",
                      max_seconds_uncommanded_travel_);
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

      const int32_t m1_quad_pulses_per_second =
          m1_desired_velocity * quad_pulses_per_meter_;
      const int32_t m2_quad_pulses_per_second =
          m2_desired_velocity * quad_pulses_per_meter_;
      
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

        float linear_velocity_x =
            RoboClaw::singleton()->getVelocity(RoboClaw::kM1) * 1.0;
        float linear_velocity_y =
            RoboClaw::singleton()->getVelocity(RoboClaw::kM2) * 1.0;
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
    if (param.get_name() == "current_filter_window_seconds") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 10.0) {
        current_filter_window_seconds_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setCurrentProtectionParams(
              current_filter_window_seconds_, recovery_timeout_seconds_, sensor_update_rate_);
        }
        RCUTILS_LOG_INFO("[MotorDriver] Updated current_filter_window_seconds to %.2f", new_value);
      } else {
        result.successful = false;
        result.reason = "current_filter_window_seconds must be between 0.0 and 10.0";
        return result;
      }
    } else if (param.get_name() == "recovery_timeout_seconds") {
      float new_value = param.as_double();
      if (new_value >= 0.0 && new_value <= 60.0) {
        recovery_timeout_seconds_ = new_value;
        if (RoboClaw::singleton() != nullptr) {
          RoboClaw::singleton()->setCurrentProtectionParams(
              current_filter_window_seconds_, recovery_timeout_seconds_, sensor_update_rate_);
        }
        RCUTILS_LOG_INFO("[MotorDriver] Updated recovery_timeout_seconds to %.2f", new_value);
      } else {
        result.successful = false;
        result.reason = "recovery_timeout_seconds must be between 0.0 and 60.0";
        return result;
      }
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