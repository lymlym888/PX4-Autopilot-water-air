#pragma once

#include <stdint.h>

namespace fixed_mpc_shadow {

enum class FmpcMode : uint8_t {
    OFF = 0,
    SHADOW_ONLY = 1,
    GATED_SHADOW = 2,
    ACTIVE_TINY = 3,
    FALLBACK = 4,
    ACTIVE_BOUNDED_MPC = 5,
    FULL_ROLL_MPC_TEST = 6
};

enum class FmpcBaseSource : uint8_t {
    NONE = 0,
    HYDRO_SERVOS = 1,
    HYDRO_TORQUE_SETPOINT_ONLY = 2,
    STALE_OR_INVALID = 3,
    ALLOCATOR_STATUS = 4
};

enum class FmpcGateReason : uint8_t {
    NONE = 0,
    MODE_OFF = 1,
    SHADOW_ONLY = 2,
    GATED_SHADOW_ONLY = 3,
    BASE_BELOW_MIN = 4,
    DIRECTION_COMPATIBLE = 5,
    DIRECTION_CONFLICT = 6,
    PARAM_ACTIVE_DISABLED = 7,
    SIGN_NOT_CONFIRMED = 8,
    SENSOR_INVALID = 9,
    BASE_COMMAND_INVALID = 10,
    ROLL_LIMIT = 11,
    RATE_LIMIT = 12,
    CANDIDATE_LIMIT = 13,
    ACTIVE_LIMIT_CLAMPED = 14,
    USER_FALLBACK_MODE = 15,
    SATURATION_NO_HEADROOM = 17,
    BOUNDED_ACTIVE_DISABLED = 18,
    BOUNDED_LIMIT_CLAMPED = 19,
    BOUNDED_ACTIVE_APPLIED = 20,
    FULL_ACTIVE_DISABLED = 21,
    FULL_LIMIT_CLAMPED = 22,
    FULL_DURATION_TIMEOUT = 23,
    FULL_ACTIVE_APPLIED = 24,
    FULL_SOURCE_INVALID = 25,
    UNKNOWN = 26
};

enum class FallbackReason : uint8_t {
    NONE = 0,
    SIGN_NOT_CONFIRMED = 1,
    MODE_OFF = 2,
    SHADOW_ONLY = 3,
    GATED_SHADOW_ONLY = 4,
    PARAM_ACTIVE_DISABLED = 5,
    SENSOR_INVALID = 6,
    STALE_SENSOR_DATA = 7,
    BASE_COMMAND_INVALID = 8,
    DIAGNOSTIC_ERROR = 9,
    DIRECTION_CONFLICT = 10,
    ROLL_LIMIT = 11,
    RATE_LIMIT = 12,
    CANDIDATE_LIMIT = 13,
    ACTIVE_LIMIT_CLAMPED = 14,
    USER_FALLBACK_MODE = 15,
    SATURATION_NO_HEADROOM = 17,
    BOUNDED_ACTIVE_DISABLED = 18,
    BOUNDED_LIMIT_CLAMPED = 19,
    BOUNDED_ACTIVE_APPLIED = 20,
    FULL_ACTIVE_DISABLED = 21,
    FULL_LIMIT_CLAMPED = 22,
    FULL_DURATION_TIMEOUT = 23,
    FULL_ACTIVE_APPLIED = 24,
    FULL_SOURCE_INVALID = 25,
    UNKNOWN = 26
};

static constexpr float kFmpcActLimHardMax = 0.02f;
static constexpr float kFmpcBndLimHardMax = 0.05f;
// Full-mode limits are normalized left-minus-right differences, not single-servo travel.
// Each servo remains bounded by [-1, 1], with the existing margin/headroom applied later.
static constexpr float kFmpcFullLimHardMax = 2.0f;
static constexpr float kFmpcFullCandMaxHardMin = 0.10f;
static constexpr float kFmpcFullCandMaxHardMax = 2.0f;
static constexpr float kFmpcFullDurHardMaxS = 5.0f;
static constexpr float kAllocatorCorrectionTopicMaxAgeMs = 200.f;
static constexpr float kAllocatorStatusMaxAgeMs = 200.f;

constexpr bool full_duration_timed_out(float duration_limit_s, float elapsed_s)
{
    return duration_limit_s > 0.f && elapsed_s > duration_limit_s;
}

constexpr float constrain_full_candidate(float requested_diff, float candidate_limit)
{
    return requested_diff < -candidate_limit ? -candidate_limit :
           (requested_diff > candidate_limit ? candidate_limit : requested_diff);
}

} // namespace fixed_mpc_shadow
