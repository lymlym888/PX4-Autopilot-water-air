uint64 timestamp
uint64 source_timestamp
uint64 mode_entry_timestamp
uint64 selector_epoch # Same-command producer acknowledgement, not a latest-value join
uint64 nav_state_timestamp # Allocator flight-mode generation; MPC source must match it
uint8 nav_state
uint8 configured_mode # Raw FMPC_MODE; requested_mode is the flight-mode-qualified mode

uint32 source_sequence
uint32 mode_session_id

uint8 requested_mode
uint8 source_mode
int8 left_index
int8 right_index

bool base_ok
bool command_valid
bool fallback_valid
uint8 base_src

float32 srv_l_b
float32 srv_r_b
float32 d_base_a
float32 d_base_b

float32 app_ca
float32 app_cb

float32 srv_l_o
float32 srv_r_o

float32 requested_target_a
float32 source_implemented_target_a
bool source_target_clamped

bool act_ap
bool fb
uint8 fb_r

# TOPICS fixed_mpc_allocator_status
