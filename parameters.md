# ros2_roboclaw_driver Parameters Reference

## 1. Device Connection

Serial connection configuration. **Requires node restart.**

| Parameter | Type | Default | Range/Options | Description |
|-----------|------|---------|---------------|-------------|
| `device_name` | string | `/dev/roboclaw` | Valid device path | Serial device path for RoboClaw |
| `baud_rate` | int | 115200 | 9600-460800 | Serial baud rate (must match RoboClaw config) |
| `device_port` | int | 128 | 128-135 | RoboClaw device address on serial bus |
| `serial_timeout` | float | 0.5 | 0.0-5.0 s | Timeout for serial communication (0.0=disabled) |

---

## 2. Robot Geometry

Physical robot dimensions. **Requires node restart.**

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `wheel_separation` | float | required | 0.0-2.0 m | Distance between left and right wheel contact points |
| `meters_per_quad_pulse` | float | required | 0.0-0.01 | Linear distance traveled per encoder count |
| `quad_pulses_per_revolution` | float | 0.0 | 0.0-10000.0 | Encoder counts per wheel revolution. Required when `publish_joint_states` is true |

The node exits at startup with an error if a required value is missing.

**Deprecated:** `quad_pulses_per_meter` (use `meters_per_quad_pulse`). `wheel_radius` is unused and only accepted so old configs still load.

---

## 3. Motor Control - PID Tuning

RoboClaw internal PID controller parameters. **Runtime changeable.**

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `m1_pid_p` | float | 5000.0 | 0.0-100000.0 | Motor 1 (left) PID proportional gain |
| `m1_pid_i` | float | 0.0 | 0.0-100000.0 | Motor 1 (left) PID integral gain |
| `m1_pid_d` | float | 0.0 | 0.0-100000.0 | Motor 1 (left) PID derivative gain |
| `m1_pid_qpps_max` | int | 2437 | 0-1000000 | Motor 1 max speed in quad pulses/sec (for PID scaling) |
| `m2_pid_p` | float | 5000.0 | 0.0-100000.0 | Motor 2 (right) PID proportional gain |
| `m2_pid_i` | float | 0.0 | 0.0-100000.0 | Motor 2 (right) PID integral gain |
| `m2_pid_d` | float | 0.0 | 0.0-100000.0 | Motor 2 (right) PID derivative gain |
| `m2_pid_qpps_max` | int | 2437 | 0-1000000 | Motor 2 max speed in quad pulses/sec (for PID scaling) |

**Note:** `qpps_max` is a PID tuning parameter, not a velocity limit. Use `max_linear_velocity` to limit speed.

---

## Motion Limits

Robot motion constraints. **Runtime changeable.**

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `max_linear_velocity` | float | 2.0 | 0.0-10.0 m/s | Maximum forward/backward velocity |
| `max_angular_velocity` | float | 2.0 | 0.0-10.0 rad/s | Maximum rotation velocity |
| `max_linear_acceleration` | float | 2.0 | 0.1-20.0 m/s² | Maximum linear acceleration |
| `m1_max_current` | float | 10.0 | 0.0-50.0 A | Motor 1 current limit (trip threshold) |
| `m2_max_current` | float | 10.0 | 0.0-50.0 A | Motor 2 current limit (trip threshold) |

**Deprecated:** `accel_quad_pulses_per_second` (use `max_linear_acceleration`)

---

## Current Protection

Over-current protection configuration. **Runtime changeable.**

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `current_filter_window_seconds` | float | 1.0 | 0.0-10.0 s | Time window for averaging current readings (0.0=instantaneous) |
| `recovery_timeout_seconds` | float | 5.0 | 0.0-60.0 s | Seconds of zero cmd_vel before auto-recovery (0.0=no auto-recovery) |

A latched fault (or any active fault) can be cleared by hand:

```bash
ros2 service call /roboclaw/clear_fault std_srvs/srv/Trigger
```

---

## Safety

Watchdog timeout. **Runtime changeable.**

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `max_seconds_uncommanded_travel` | float | 0.5 | 0.0-5.0 s | Stops motors if they are moving and no cmd_vel arrives within this time (0.0=disabled). Keep it longer than your cmd_vel interval |
| `coast_timeout_seconds` | float | 0.5 | 0.0-60.0 s | Time without cmd_vel before releasing motor holding torque (0.0=disabled) |

---

## Publishing & Topics

ROS2 topic publishing configuration. **Requires node restart.**

| Parameter | Type | Default | Options | Description |
|-----------|------|---------|---------|-------------|
| `publish_joint_states` | bool | false | true/false | Publish wheel joint states |
| `publish_odom` | bool | true | true/false | Publish odometry on /odom topic |
| `roboclaw_status_topic` | string | `roboclaw_status` | Any valid topic | Topic name for RoboClaw status messages |
| `sensor_update_rate` | float | 20.0 | 1.0-100.0 Hz | Rate for reading encoders and publishing |

---

## Debug & Logging

Debug logging control. **Runtime changeable.**

| Parameter | Type | Default | Options | Description |
|-----------|------|---------|---------|-------------|
| `do_debug` | bool | false | true/false | Enable protocol-level debug logging |
| `do_low_level_debug` | bool | false | true/false | Enable byte-level serial debug logging |

**Warning:** `do_low_level_debug=true` produces massive log output.

---

## See Also

- [RoboClaw User Manual](https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf)
- [ROS2 Parameter Documentation](https://docs.ros.org/en/jazzy/Concepts/About-ROS-2-Parameters.html)
