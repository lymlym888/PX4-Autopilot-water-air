uint64 timestamp
uint64 source_timestamp
uint64 source_mode_entry_timestamp
uint64 selector_epoch # Current local Mode 6 entry request; zero outside Mode 6
uint64 source_selector_epoch # Acknowledgement carried by the consumed source command
uint64 last_event_timestamp

uint32 source_sequence
uint32 source_session_id
uint32 authority_transition_count
uint32 fallback_event_count

uint8 OWNER_LEGACY = 0
uint8 OWNER_MPC = 1
uint8 OWNER_FALLBACK_BASE = 2
uint8 OWNER_FALLBACK_ORDINARY = 3
uint8 OWNER_INHIBITED = 4
uint8 OWNER_INVALID = 5

uint8 requested_mode
uint8 selected_owner
uint8 selector_reason
uint8 upstream_fallback_reason
uint8 last_event_reason
uint8 last_upstream_fallback_reason

int8 left_index
int8 right_index

bool source_fresh
bool source_session_valid
bool source_command_valid
bool source_fallback_valid
bool target_clamped
bool channels_enabled

float32 ordinary_left
float32 ordinary_right
float32 hydro_left
float32 hydro_right
float32 final_left
float32 final_right

float32 requested_target_a
float32 source_implemented_target_a
float32 final_implemented_diff_a
float32 takeover_jump_a

# TOPICS fixed_mpc_final_output_status
