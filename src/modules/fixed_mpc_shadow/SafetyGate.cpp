#include "SafetyGate.hpp"
#include "FixedMPCParams.hpp"
#include <math.h>

namespace fixed_mpc_shadow {

GateOutput SafetyGate::evaluate(const GateInput &input) const {
    GateOutput out{false, FallbackReason::UNKNOWN};
    if (!input.finite_input || isnan(input.roll_abs) || isnan(input.roll_rate_abs) || isnan(input.delta_abs)) {
        out.fallback_reason = FallbackReason::NAN_DETECTED; return out;
    }
    if (!input.actuator_sign_confirmed) { out.fallback_reason = FallbackReason::SIGN_NOT_CONFIRMED; return out; }
    if (!input.rc_enable) { out.fallback_reason = FallbackReason::RC_DISABLED; return out; }
    if (!input.armed) { out.fallback_reason = FallbackReason::NOT_ARMED_OR_INVALID_ARM_STATE; return out; }
    if (!input.solver_success) { out.fallback_reason = FallbackReason::SOLVER_FAIL; return out; }
    if (input.roll_abs > kRollAbsMaxRad) { out.fallback_reason = FallbackReason::ROLL_LIMIT; return out; }
    if (input.roll_rate_abs > kRollRateAbsMaxRadS) { out.fallback_reason = FallbackReason::ROLL_RATE_LIMIT; return out; }
    if (input.delta_abs > kDeltaMax) { out.fallback_reason = FallbackReason::DELTA_LIMIT; return out; }
    if (input.delta_rate_abs > kDDeltaMax) { out.fallback_reason = FallbackReason::DELTA_RATE_LIMIT; return out; }
    if (input.battery_voltage < kBatteryMinV) { out.fallback_reason = FallbackReason::LOW_BATTERY; return out; }
    if (input.solve_time_us > static_cast<float>(kSolveTimeMaxUs)) { out.fallback_reason = FallbackReason::SOLVE_TIME_EXCEEDED; return out; }
    out.gate_pass = true;
    out.fallback_reason = FallbackReason::NONE;
    return out;
}

}  // namespace fixed_mpc_shadow
