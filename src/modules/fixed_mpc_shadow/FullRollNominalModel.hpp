#pragma once

#include <stdint.h>
#include "FixedMPCParams.hpp"

namespace fixed_mpc_shadow
{

// Mode 6 deviations about latched trim, not the offline identified A/c.
// Keep the signed Servo4-Servo5 input gain. With left=4/right=3, positive
// runtime differential accelerates positive roll: angle and rate need damping.
inline void full_roll_nominal_step(float phi, float p, float model_correction,
		float &next_phi, float &next_p)
{
	next_phi = phi + kTs * p + kB0 * model_correction;
	next_p = p + kB1 * model_correction;
}

// Generalized DARE for [phi, p, previous correction], next-state Q, R and delta-R.
// Derived from the nominal model above, not tuned to force a direction test.
static constexpr float kFullRollTerminal[3][3] = {
	{40.77620795434636f, 4.534753430904014f, -0.1731105667408956f},
	{4.534753430904014f, 3.628226446862853f, -0.1446380607052449f},
	{-0.1731105667408956f, -0.1446380607052449f, 0.02003273168264703f}
};

constexpr bool full_roll_coefficient_matches(float value, float expected)
{
	return value <= expected && value >= expected;
}

static_assert(full_roll_coefficient_matches(kTs, 0.02f) && kHorizon == 10 &&
	      full_roll_coefficient_matches(kQPhi, 1.f) && full_roll_coefficient_matches(kQP, 0.5f) &&
	      full_roll_coefficient_matches(kRDelta, 0.005f) && full_roll_coefficient_matches(kRDDelta, 0.05f) &&
	      full_roll_coefficient_matches(kB0, -0.0004309727952737038f) &&
	      full_roll_coefficient_matches(kB1, -0.021548639763685186f),
	      "Regenerate the Mode 6 terminal value for changed model/cost coefficients");

inline float full_roll_terminal_cost(float phi, float p, float model_correction)
{
	const float z[3] = {phi, p, model_correction};
	float cost = 0.f;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			cost += z[i] * kFullRollTerminal[i][j] * z[j];
		}
	}
	return cost;
}

} // namespace fixed_mpc_shadow
