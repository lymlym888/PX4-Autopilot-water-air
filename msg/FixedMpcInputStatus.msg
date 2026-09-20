uint64 timestamp
uint64 checked_at
uint64 base_checked_at
uint64 feedback_checked_at
uint64 attitude_timestamp
uint64 rate_timestamp
uint64 base_timestamp
uint64 torque_timestamp
uint64 feedback_timestamp
uint64 last_reject_timestamp

uint32 rejection_mask
uint32 rejection_count
uint32 last_rejection_mask

float32 attitude_age_ms
float32 rate_age_ms
float32 base_age_ms
float32 torque_age_ms
float32 feedback_age_ms
float32 model_previous
float32 model_proposal
float32 model_implemented
float32 requested_correction
float32 implemented_target
float32 model_trim
float32 model_correction
float32 unconstrained_correction
float32 full_slew_rate # Effective Mode 6 differential rate, normalized units/s
float32 full_error_gain # Effective angle-error feedback multiplier, not a Q weight
float32 full_max_step # Effective normal differential step at the fixed 50-Hz update
bool tuning_sanitized # Invalid/out-of-range new tuning replaced by defaults/bounds
float32 trim_candidate_a
float32 trim_bound_a
float32 trim_reserve_a

uint8 FULL_NOMINAL_TRIM = 2
uint8 mode
uint8 configured_mode # Raw parameter, while mode is flight-mode-qualified
uint8 nav_state
uint64 nav_state_timestamp
bool flight_status_valid
bool mode6_eligible
uint8 full_src
uint8 full_model_variant
int8 model_sign
bool feedback_used
bool rate_recovery
bool trim_projected
bool slew_limited
bool constraint_conflict

uint8 TRIM_DISABLED = 0
uint8 TRIM_UNCONFIRMED = 1 # Reserved for older firmware; no confirmation parameter remains
uint8 TRIM_INVALID_CONFIG = 2
uint8 TRIM_WAITING_BASE = 3
uint8 TRIM_OUTSIDE_BOUNDS = 4
uint8 TRIM_READY = 5
uint8 TRIM_READY_LIMITED = 6
uint8 trim_status
bool trim_valid

uint64 reference_timestamp
uint64 reference_sample
uint64 reference_checked_at
float32 reference_age_ms
float32 roll_stick
float32 roll_ref_deg
float32 roll_error_deg
uint8 REF_UNUSED = 0
uint8 REF_READY = 1
uint8 REF_MISSING = 2
uint8 REF_INVALID = 3
uint8 REF_ZERO_TIME = 4
uint8 REF_FUTURE = 5
uint8 REF_STALE = 6
uint8 REF_NONFINITE = 7
uint8 REF_OUT_OF_RANGE = 8
uint8 REF_INVALID_CONFIG = 9
uint8 REF_TIME_ORDER = 10
uint8 REF_INVALID_SOURCE = 11
uint8 reference_status
uint8 reference_source
bool reference_valid

# rejection_mask bits match InputRejection in FullMpcInput.hpp.
# model_previous/proposal/implemented/trim: Servo4 - Servo5 total differential.
# model_correction: model_implemented - model_trim, used by variant 2 prediction.
# correction/target use runtime HY_SV0_IDX - HY_SV1_IDX differential.
# Variant 2 uses trim-relative nominal kinematics and analytic continuous MPC.
# unconstrained_correction is before amplitude/rate/physical limiting.
# rate_recovery means final physical/soft projection exceeded the normal rate step.
# trim_candidate_a is the entry-captured FMPC_TRIM_A, never live CADRC.
# trim_bound_a is the physical/FCAND bound when accepted.
# trim_reserve_a is min(correction limit, available two-sided headroom) when accepted.
# trim_projected is retained for old readers; static trim is never projected.
# trim_status=6 is valid but has reduced correction headroom; ordinary clamps still apply.
# trim_valid describes this session's accepted static trim, not measured angle validity.
# model_proposal is now the joint-constrained implemented model input.
# slew_limited denotes a joint-interval rate constraint or recovery limiting the preferred target.
# constraint_conflict means the correction band and physical range no longer intersect.
# Mode 6: roll * HY_ROLL_MAX is clipped to +/-15 deg, as in all STAB reference paths.
# Prediction uses (actual roll minus reference) * full_error_gain; rate reference stays zero.
# roll_error_deg remains the unscaled physical error. Only Mode 6 uses these tuning fields.
# Physical roll remains in fixed_mpc_shadow_status.roll_deg, never replaced by error.
# reference_sample is the original manual input timestamp, not its republish time.
# Both reference timestamps must be younger than COM_RC_LOSS_T; valid must be true.
# reference_source uses standard manual_control_setpoint data_source values.
# Rejection bit 17 denotes an invalid manual reference; reference_status gives detail.
# reference_valid does not imply MPC applied; all existing hard-fault checks remain.
# mode is effective mode; configured_mode is the raw FMPC_MODE parameter.
# Mode 6 requires a fresh STAB vehicle_status; a new nav_state_timestamp starts a new session.
# TOPICS fixed_mpc_input_status
