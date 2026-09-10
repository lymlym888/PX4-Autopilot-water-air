#pragma once

#include "FullMpcInput.hpp"

namespace fixed_mpc_shadow
{

// Values are published as fixed_mpc_input_status.trim_status.
enum class StaticTrimStatus : uint8_t {
	Disabled = 0,
	Unconfirmed = 1,
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
	bool confirmed{false};
	bool valid{false};
	bool limited{false};
	StaticTrimStatus status{StaticTrimStatus::Disabled};

	void reset() { *this = {}; }

	void begin(float configured_trim, bool reviewed)
	{
		reset();
		candidate = configured_trim;
		confirmed = reviewed;
	}

	void update(bool enabled, bool base_valid, float correction_limit, float candidate_limit,
		    float mean_servo, float servo_limit)
	{
		if (!enabled) { status = StaticTrimStatus::Disabled; return; }
		if (valid) { status = limited ? StaticTrimStatus::ReadyLimited : StaticTrimStatus::Ready; return; }
		if (!confirmed) { status = StaticTrimStatus::Unconfirmed; return; }
		if (!__builtin_isfinite(candidate) || fabsf(candidate) > 2.f) {
			status = StaticTrimStatus::InvalidConfig;
			return;
		}
		if (!base_valid) { status = StaticTrimStatus::WaitingForBase; return; }
		const FullMpcInput input{candidate, correction_limit, candidate_limit, mean_servo, servo_limit, 1};
		if (!input.valid()) { status = StaticTrimStatus::InvalidConfig; return; }
		bound = fminf(candidate_limit, 2.f * (servo_limit - fabsf(mean_servo)));
		if (fabsf(candidate) > bound) {
			status = StaticTrimStatus::OutsideBounds;
			return;
		}

		// Never move a reviewed operating point to make a larger correction fit.
		// Reduced correction headroom is diagnostic-only; normal clamps remain downstream.
		reserve = fminf(correction_limit, fmaxf(bound - fabsf(candidate), 0.f));
		limited = reserve + 1.e-6f < correction_limit;
		trim = candidate;
		valid = true;
		status = limited ? StaticTrimStatus::ReadyLimited : StaticTrimStatus::Ready;
	}
};

} // namespace fixed_mpc_shadow
