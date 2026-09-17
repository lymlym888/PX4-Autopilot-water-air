#pragma once

#include "FullMpcInput.hpp"

namespace fixed_mpc_shadow
{

// Values are published as fixed_mpc_input_status.trim_status.
enum class StaticTrimStatus : uint8_t {
	Disabled = 0,
	Unconfirmed = 1, // Reserved for old logs; no commissioning confirmation gate remains.
	InvalidConfig = 2,
	WaitingForBase = 3,
	OutsideBounds = 4,
	Ready = 5,
	ReadyLimited = 6,
};

struct StaticFullTrim {
	float candidate{0.f};
	float trim{0.f};
	float bound{0.f};
	float reserve{0.f};
	bool valid{false};
	bool limited{false};
	StaticTrimStatus status{StaticTrimStatus::Disabled};

	void reset() { *this = {}; }

	void begin(float configured_trim)
	{
		reset();
		candidate = configured_trim;
	}

	void update(bool enabled, bool base_valid, float correction_limit, float candidate_limit,
		    float mean_servo, float servo_limit)
	{
		if (!enabled) { status = StaticTrimStatus::Disabled; return; }
		if (valid) { status = limited ? StaticTrimStatus::ReadyLimited : StaticTrimStatus::Ready; return; }
		if (!__builtin_isfinite(candidate) || fabsf(candidate) > 2.f) {
			status = StaticTrimStatus::InvalidConfig;
			return;
		}
		if (!base_valid) { status = StaticTrimStatus::WaitingForBase; return; }
		const FullMpcInput input{candidate, correction_limit, candidate_limit, mean_servo, servo_limit, 1};
		if (!input.valid()) { status = StaticTrimStatus::InvalidConfig; return; }
		bound = fminf(candidate_limit, full_differential_headroom(mean_servo, servo_limit));
		// Validate the fixed trim against the configured envelope, not transient
		// depth demand. Execution may temporarily clip this unchanged trim to zero.
		if (fabsf(candidate) > fminf(candidate_limit, 2.f * servo_limit)) {
			status = StaticTrimStatus::OutsideBounds;
			return;
		}

		// Never move the configured operating point to make a larger correction fit.
		// Reduced correction headroom is diagnostic-only; normal clamps remain downstream.
		reserve = fminf(correction_limit, fmaxf(bound - fabsf(candidate), 0.f));
		limited = reserve + 1.e-6f < correction_limit;
		trim = candidate;
		valid = true;
		status = limited ? StaticTrimStatus::ReadyLimited : StaticTrimStatus::Ready;
	}
};

} // namespace fixed_mpc_shadow
