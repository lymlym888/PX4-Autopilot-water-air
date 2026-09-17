#pragma once

#include "FixedMPCParams.hpp"
#include "FullMpcInput.hpp"

namespace fixed_mpc_shadow {

struct FullMpcTuning {
	float slew_rate{3.f};
	float error_gain{1.f};

	bool valid() const
	{
		return __builtin_isfinite(slew_rate) && slew_rate >= 1.5f && slew_rate <= 6.f &&
		       __builtin_isfinite(error_gain) && error_gain >= 0.5f && error_gain <= 2.f;
	}

	bool sanitize()
	{
		const bool changed = !valid();
		slew_rate = __builtin_isfinite(slew_rate) ? input_clamp(slew_rate, 1.5f, 6.f) : 3.f;
		error_gain = __builtin_isfinite(error_gain) ? input_clamp(error_gain, 0.5f, 2.f) : 1.f;
		return changed;
	}

	// Preserve the original step exactly at the default 50-Hz tuning.
	float maximum_step() const { return kDDeltaMax * (slew_rate / 3.f); }
};

} // namespace fixed_mpc_shadow
