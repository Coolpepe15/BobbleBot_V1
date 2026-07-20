# Telemetry published by the balance controller on each control loop cycle.
# Field names are snake_case per ROS 2 message conventions (the original
# ROS 1 bobble_controllers package used CamelCase for these same fields).

int32 control_mode
float32 measured_tilt_dot
float32 measured_turn_rate
float32 filtered_tilt_dot
float32 filtered_turn_rate
float32 tilt
float32 tilt_rate
float32 heading
float32 turn_rate
float32 forward_velocity
float32 desired_velocity
float32 desired_tilt
float32 desired_turn_rate
float32 left_motor_position
float32 left_motor_velocity
float32 right_motor_position
float32 right_motor_velocity
float32 tilt_effort
float32 heading_effort
float32 left_motor_effort_cmd
float32 right_motor_effort_cmd
