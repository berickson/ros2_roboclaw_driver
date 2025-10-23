float32  m1_pid_p
float32  m1_pid_i
float32  m1_pid_d
uint32  m1_pid_qpps_max
int32   m1_current_speed_qpps  # Current speed in quadrature pulses per second
float32  m1_current_speed  # Current speed in m/s (PV - Process Value)
float32  m1_commanded_speed  # Commanded speed in m/s (SP - Setpoint)
float32  m1_motor_current
int32  m1_encoder_value
uint8  m1_encoder_status

float32  m2_pid_p
float32  m2_pid_i
float32  m2_pid_d
uint32  m2_pid_qpps_max
int32   m2_current_speed_qpps  # Current speed in quadrature pulses per second
float32  m2_current_speed  # Current speed in m/s (PV - Process Value)
float32  m2_commanded_speed  # Commanded speed in m/s (SP - Setpoint)
float32  m2_motor_current
int32  m2_encoder_value
uint8  m2_encoder_status

float32  main_battery_voltage
float32  logic_battery_voltage
float32  temperature
string  error_string