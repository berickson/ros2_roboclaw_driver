# ros2_roboclaw_driver (berickson fork)

ROS2 driver for a RoboClaw driving two motors in a differential drive
(m1 = left, m2 = right). Forked from
[wimblerobotics/ros2_roboclaw_driver](https://github.com/wimblerobotics/ros2_roboclaw_driver)
and used on the [red-crash](https://github.com/berickson/red-crash) robot.

## Differences From Upstream

- **Survives USB drops.** Communication errors no longer kill the node. It
  marks the RoboClaw disconnected, retries every 5 s, and re-sends the serial
  timeout and PID settings when it comes back.
- **Speed commands.** `cmd_vel` uses speed + acceleration commands instead of
  buffered distance commands, with acceleration set in m/s²
  (`max_linear_acceleration`) and distance per encoder count
  (`meters_per_quad_pulse`).
- **Over-current protection.** Motor currents are averaged over a window and
  act as a slow fuse that stops the motors, then auto-recovers after a period
  of zero `cmd_vel`.
- **Coast mode.** Motors coast (no holding power) when `cmd_vel` stops for
  `coast_timeout_seconds`.
- **Diagnostics.** Connection state, error counts, batteries, currents and
  temperature are published on `/diagnostics`.
- **Runtime parameters.** PID, limits, current protection and debug settings
  can be changed with `ros2 param set`. See [parameters.md](parameters.md).
- **Fixes.** SetDutyCycle byte count, negative odometry readings.
- **Message change.** `connection_state` was removed from `RoboClawStatus`;
  it is in diagnostics instead.

## Build and Run

```bash
colcon build --symlink-install --packages-select ros2_roboclaw_driver
ros2 launch ros2_roboclaw_driver ros2_roboclaw_driver.launch.py
```

Settings live in `config/motor_driver.yaml`; every parameter is described in
[parameters.md](parameters.md). Publishes `/odom` and `/joint_states` at
`sensor_update_rate` (20 Hz). Encoders reset to zero each time the node starts.

Upstream's [README](https://github.com/wimblerobotics/ros2_roboclaw_driver#readme)
covers Raspberry Pi UART wiring; much of the rest no longer matches this fork.

## Remotes

`origin` is this fork. `upstream` is wimblerobotics, for pulling their fixes.
