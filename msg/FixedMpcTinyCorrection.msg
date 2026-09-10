uint64 timestamp
uint64 mode_entry_timestamp
uint64 selector_epoch # Echo of final selector request, zero until acknowledged

uint32 command_sequence
uint32 mode_session_id

uint8 mode
bool act_req
bool act_allow
bool act_apply
bool fb
uint8 fb_r

float32 corr_a
float32 corr_b
float32 cand_a
float32 cand_b
float32 base_a
float32 base_b
float32 requested_target_a
float32 requested_target_b

bool diag_ok
bool sen_ok
bool base_ok
bool dir_ok

# TOPICS fixed_mpc_tiny_correction
