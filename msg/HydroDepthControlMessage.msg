uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample

int32 depth_controller_mode
bool depth_controller_enabled
bool depth_controller_output_finite

float32 depth
float32 depth_rate
float32 depth_setpoint
float32 throttle_limited
float32 horizontal_thrust

#非光滑反馈
float32 nsf_e1
float32 nsf_e2
float32 nsf_s
float32 nsf_s_norm
float32 nsf_s_norm_power
float32 nsf_u
float32 nsf_u_limited

#高阶滑模
float32 hsmc_x1
float32 hsmc_x2
float32 hsmc_x3
float32 hsmc_surface
float32 hsmc_u


float32 vertical_thrust
float32 vertical_thrust_limited
