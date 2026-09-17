uint64 timestamp

float32 roll_deg
float32 p_deg_s
float32 pitch_deg
float32 yaw_deg

float32 solve_us

float32 d_base_a
float32 d_base_b
float32 d_raw
float32 d_corr
float32 d_gate_corr_a
float32 d_gate_corr_b

float32 active_corr_a
float32 active_corr_b
float32 active_candidate_a
float32 active_candidate_b

float32 allocator_applied_corr_a
float32 allocator_applied_corr_b

float32 fmpc_act_lim
float32 fmpc_cand_max
float32 fmpc_fcand_max

uint8 fmpc_mode
uint8 effective_mode # Raw fmpc_mode=6 can be inactive outside STAB
uint8 nav_state
bool mode6_eligible
uint8 fmpc_act_en
uint8 fmpc_sign_ok

uint8 diag_ok
uint8 sensor_ok
uint8 base_ok
uint8 direction_ok
uint8 attitude_ok
uint8 angular_velocity_ok
uint8 base_command_ok
uint8 torque_setpoint_ok
uint8 solver_ok

uint8 active_requested
uint8 active_allowed
uint8 active_applied
uint8 active_fallback
uint8 fallback_reason

uint8 gate_pass
uint8 candidate_limit_flag
uint8 bounded_requested
uint8 bounded_allowed
uint8 bounded_applied
uint8 full_requested
uint8 full_allowed
uint8 full_applied
uint8 full_trim_en
float32 full_duration_s
float32 full_trim_a
float32 full_trim_b
float32 full_mpc_corr_a
float32 full_target_a
float32 full_target_b
uint8 servo_saturation_flag

# TOPICS fixed_mpc_shadow_status
