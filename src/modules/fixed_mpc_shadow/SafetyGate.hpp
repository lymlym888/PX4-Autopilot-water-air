#pragma once
#include <stdint.h>

namespace fixed_mpc_shadow {

enum class FallbackReason : uint8_t {
    NONE = 0,
    SIGN_NOT_CONFIRMED,
    RC_DISABLED,
    NOT_ARMED_OR_INVALID_ARM_STATE,
    ROLL_LIMIT,
    ROLL_RATE_LIMIT,
    DELTA_LIMIT,
    DELTA_RATE_LIMIT,
	LOW_BATTERY,
	SOLVER_FAIL,
	NAN_DETECTED,
	SOLVE_TIME_EXCEEDED,
	MISSING_SENSOR_DATA,
	STALE_SENSOR_DATA,
	MISSING_BASE_COMMAND,
	STALE_BASE_COMMAND,
	UNKNOWN
};

struct GateInput {
    bool actuator_sign_confirmed;
    bool rc_enable;
    bool armed;
    bool solver_success;
    bool finite_input;
    float roll_abs;
    float roll_rate_abs;
    float delta_abs;
    float delta_rate_abs;
    float battery_voltage;
    float solve_time_us;
};

struct GateOutput {
    bool gate_pass;
    FallbackReason fallback_reason;
};

class SafetyGate {
public:
    GateOutput evaluate(const GateInput &input) const;
};

}  // namespace fixed_mpc_shadow
