# Connection State and Diagnostics Design

## Overview
This document describes how the ros2_roboclaw_driver handles connection state management and reports device health through the ROS2 diagnostics system.

## Design Principles

### Separation of Concerns
The driver follows standard ROS2 patterns by separating **sensor data** from **system health monitoring**:

- **Status Messages** (`RoboClawStatus` topic): Reports current hardware sensor readings
- **Diagnostics** (`/diagnostics` topic): Reports connection state and system health

## Connection State Behavior

### When Connected
- Driver publishes `RoboClawStatus` messages regularly at configured rate (default: 20 Hz)
- Messages contain current sensor readings from the RoboClaw device
- Diagnostics reports "OK" status with connection health metrics

### When Disconnected
- Driver **STOPS publishing** `RoboClawStatus` messages
  - Rationale: No valid sensor data available
  - Prevents downstream nodes from processing stale/invalid readings
  - Standard ROS2 pattern for hardware drivers
  
- Driver **CONTINUES publishing** diagnostics with ERROR level
  - Reports "DISCONNECTED" state
  - Includes last successful communication timestamp
  - Tracks consecutive error count
  - Provides actionable error information

### Connection Recovery
- When device reconnects, driver automatically resumes normal operation
- `RoboClawStatus` messages resume publishing
- Diagnostics transitions from ERROR to OK level
- No manual intervention required

## Client-Side Disconnect Detection

Clients have two standard options for detecting disconnection:

### Option 1: Monitor Message Timestamps (Most Common)
```cpp
// Example: Check if data is stale
auto now = this->get_clock()->now();
auto age = now - last_msg_time_;
if (age > rclcpp::Duration::from_seconds(timeout_threshold)) {
    // No recent data - device likely disconnected
}
```

**Pros:**
- Simple and reliable
- Works with any ROS2 topic
- Configurable timeout per application needs
- No additional dependencies

**Use when:** You need immediate detection of missing data

### Option 2: Subscribe to Diagnostics (Recommended for Health Monitoring)
```cpp
void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    for (const auto& status : msg->status) {
        if (status.name == "roboclaw: Communication") {
            if (status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
                // Device disconnected - check status.message for details
            }
        }
    }
}
```

**Pros:**
- Centralized health monitoring
- Detailed error information available
- Distinguishes different failure modes
- Integration with standard diagnostic tools

**Use when:** You need comprehensive system health monitoring or detailed error information

### Recommended Approach
Use **both** methods:
- **Timestamp monitoring** for immediate detection of missing data
- **Diagnostics subscription** for understanding *why* data stopped and overall system health

## Diagnostics System

### Diagnostic Topics
The driver publishes to the standard `/diagnostics` topic with these keys:

#### `roboclaw: Communication`
Reports serial communication health:
- **Level OK**: All communications successful, device responding
- **Level WARN**: Occasional communication errors
- **Level ERROR**: Persistent failures or disconnection
- **Level STALE**: No recent updates

**Key-Value Pairs:**
- `Connection State`: "CONNECTED" or "DISCONNECTED"
- `Consecutive Errors`: Count of sequential failed commands
- `Last Successful Communication`: Timestamp of last successful command
- `Error Threshold`: Maximum consecutive errors before disconnect

#### `roboclaw: Hardware Status` (Future)
Reports device hardware status from GETERROR command

#### `roboclaw: Battery` (Future)
Reports battery voltage levels and warnings

#### `roboclaw: Motor 1/2` (Future)
Reports individual motor status and currents

#### `roboclaw: Temperature` (Future)
Reports device temperature

#### `roboclaw: Current Protection` (Future)
Reports software current protection state machine

## RoboClawStatus Message

The `RoboClawStatus` message contains **only sensor readings**, not connection state:

```
float32  m1_p
float32  m1_i
float32  m1_d
uint32   m1_qpps
int32    m1_current_speed
float32  m1_motor_current
int32    m1_encoder_value
uint8    m1_encoder_status

float32  m2_p
float32  m2_i
float32  m2_d
uint32   m2_qpps
int32    m2_current_speed
float32  m2_motor_current
int32    m2_encoder_value
uint8    m2_encoder_status

float32  main_battery_voltage
float32  logic_battery_voltage
float32  temperature
string   error_string
```

**Note:** `connection_state` field was removed (Phase 1, Task 1a) because:
- Connection state is system health information, not sensor data
- Belongs in diagnostics, not status messages
- Prevents confusion about whether message represents current or stale data
- Follows standard ROS2 patterns

## Rationale

### Why Stop Publishing on Disconnect?
1. **Data Integrity**: Prevents processing of invalid/stale sensor readings
2. **Standard Pattern**: Matches behavior of most ROS2 hardware drivers
3. **Clear Semantics**: Empty topic = no data available (expected behavior)
4. **Tool Compatibility**: Works naturally with Foxglove, rqt, and other ROS2 tools

### Why NOT Include Connection State in Status Messages?
1. **Separation of Concerns**: Status = data, Diagnostics = health
2. **Prevents Ambiguity**: If disconnected, is the message data valid? Stale?
3. **Standard ROS2 Practice**: System health belongs in diagnostics
4. **Centralized Monitoring**: Tools like `rqt_robot_monitor` expect health info in diagnostics

### Why Continue Publishing Diagnostics?
1. **System Monitoring**: Health monitoring continues even when device offline
2. **Error Details**: Provides actionable information about why device disconnected
3. **Recovery Tracking**: Shows when connection restored
4. **Standard Pattern**: Diagnostics persist across device lifecycle

## Examples

### Foxglove Studio
- **Connected**: Status panel shows current values, diagnostics shows green
- **Disconnected**: Status panel shows "no messages", diagnostics shows red with error details

### rqt_robot_monitor
- **Connected**: Shows green checkmark for "roboclaw: Communication"
- **Disconnected**: Shows red X with error message describing issue

### Custom Node
```cpp
class RoboClawMonitor : public rclcpp::Node {
  rclcpp::Time last_status_time_;
  bool is_connected_ = false;
  
  void status_callback(const RoboClawStatus::SharedPtr msg) {
    last_status_time_ = this->now();
    // Process valid sensor data
  }
  
  void diagnostics_callback(const DiagnosticArray::SharedPtr msg) {
    for (const auto& status : msg->status) {
      if (status.name == "roboclaw: Communication") {
        is_connected_ = (status.level == DiagnosticStatus::OK);
      }
    }
  }
  
  void timer_callback() {
    auto age = this->now() - last_status_time_;
    if (age > rclcpp::Duration::from_seconds(1.0)) {
      // No recent data - check diagnostics for details
      if (!is_connected_) {
        RCLCPP_WARN(this->get_logger(), "RoboClaw disconnected");
      }
    }
  }
};
```

## Related Documents
- [Diagnostics Implementation Plan](../../roboclaw_diagnostics_plan.md)
- [ROS2 Diagnostics Documentation](https://github.com/ros/diagnostics)
- [Package README](README.md)
