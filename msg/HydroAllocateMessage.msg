uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample

float32[4] thrust_xz_control	# 控制得到的四个推力，0-lx 1-lz 2-rx 3-rz
float32[4] thrust_xz_limited	# 限幅后的四个推力，顺序同上

float32[4] actuator_optimed	# 优化求解的执行器期望值，0-左翼 1-右翼 2-左推 3-右推
float32 allocator_delta_left_requested
float32 allocator_delta_right_requested
float32 allocator_thrust_left_requested
float32 allocator_thrust_right_requested
bool allocator_output_suppressed

float32 man_delta_motor
float32 man_delta_servo
float32 man_manual_motor
float32 man_manual_servo

bool allocator_valid
bool allocator_converged
bool allocator_finite
bool allocator_bounded
uint16 allocator_iterations
float32 allocator_residual
uint32 allocator_saturation_mask
uint8 allocator_solve_status
float32 jacobian_condition

float32 u_phi_effective_model
bool foil_position_feedback_available
bool u_effective_is_model_based
float32 g_left
float32 g_right
int8 logical_left_servo_index
int8 logical_right_servo_index
bool logical_mapping_matches_expected

bool foil_gate_enabled
bool aux_gate_enabled_raw
bool nav_gate_enabled_raw
bool aux_gate_data_valid
bool aux_gate_value_finite
uint8 gate_close_reason
int32 foil_aux_parameter
float32 foil_aux_value
float32 foil_threshold_parameter
int8 foil_aux_rc_channel
bool foil_aux_mapping_valid
uint64 manual_control_timestamp
uint64 manual_control_timestamp_sample
uint32 manual_control_age_us
uint8 manual_control_data_source
bool manual_control_valid
bool rc_signal_lost
bool rc_failsafe
int32 allocate_mode_parameter
float32 wing_max_angle_parameter

# Stateless allocator query context and current virtual command.
float32 lift_coefficient
float32 speed_squared
bool speed_context_cached
bool speed_context_stale
uint32 speed_context_age_us
float32 body_pitch
float32 context_max_hydrofoil_angle
float32 max_thruster_thrust
float32 available_thruster_thrust
float32[12] virtual_control_matrix
float32 command_force_x
float32 command_force_z
float32 command_u_phi
float32 command_u_theta
float32 command_u_psi
