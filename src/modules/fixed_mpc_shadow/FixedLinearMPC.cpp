#include "FixedLinearMPC.hpp"
#include "FixedMPCParams.hpp"
#include "FullRollNominalModel.hpp"
#include <math.h>

namespace fixed_mpc_shadow {

void FixedLinearMPC::init() {}

float FixedLinearMPC::clamp_delta(float delta) const {
    if (delta > kDeltaMax) return kDeltaMax;
    if (delta < -kDeltaMax) return -kDeltaMax;
    return delta;
}

float FixedLinearMPC::clamp_delta_rate(float delta, float previous_delta) const {
    const float high = previous_delta + kDDeltaMax;
    const float low = previous_delta - kDDeltaMax;
    if (delta > high) return high;
    if (delta < low) return low;
    return delta;
}

void FixedLinearMPC::one_step(float phi, float p, float delta, float &phi_next, float &p_next) const {
    phi_next = kA00 * phi + kA01 * p + kB0 * delta + kC0;
    p_next = kA10 * phi + kA11 * p + kB1 * delta + kC1;
}

float FixedLinearMPC::rollout_cost(float delta, const MpcState &state) const {
    float phi = state.phi_rad;
    float p = state.p_rad_s;
    float last = state.previous_delta;
    float cost = 0.0f;
    for (int i = 0; i < kHorizon; ++i) {
        float phi_next = 0.0f;
        float p_next = 0.0f;
        one_step(phi, p, delta, phi_next, p_next);
        cost += kQPhi * phi_next * phi_next + kQP * p_next * p_next;
        cost += kRDelta * delta * delta + kRDDelta * (delta - last) * (delta - last);
        phi = phi_next;
        p = p_next;
        last = delta;
    }
    return cost;
}

MpcResult FixedLinearMPC::solve_lqr_clipping(const MpcState &state) {
	MpcResult result{};
	result.solver_success = true;
	result.solve_time_us = 0u;  // Filled by caller perf counter in PX4.

	float best_delta_raw = 0.0f;
	float best_delta_amplitude_limited = clamp_delta(best_delta_raw);
	float best_delta_rate_limited = clamp_delta_rate(best_delta_amplitude_limited, state.previous_delta);
	float best_cost = rollout_cost(best_delta_rate_limited, state);

	for (int i = -12; i <= 12; ++i) {
		const float candidate_raw = (kDeltaMax * static_cast<float>(i)) / 4.0f;
		const float candidate_amplitude_limited = clamp_delta(candidate_raw);
		const float candidate_rate_limited = clamp_delta_rate(candidate_amplitude_limited, state.previous_delta);
		const float cost = rollout_cost(candidate_rate_limited, state);

		if (cost < best_cost) {
			best_cost = cost;
			best_delta_raw = candidate_raw;
			best_delta_amplitude_limited = candidate_amplitude_limited;
			best_delta_rate_limited = candidate_rate_limited;
		}
	}

	result.delta_raw = best_delta_raw;
	result.delta_amplitude_limited = best_delta_amplitude_limited;
	result.delta_rate_limited = best_delta_rate_limited;
	result.delta_clipped = best_delta_amplitude_limited;
	result.delta = best_delta_rate_limited;
	return result;
}

FullMpcResult FixedLinearMPC::solve_full_roll(const MpcState &state, const FullMpcInput &input,
		const FullMpcTuning &tuning)
{
	FullMpcResult result{};
	if (!input.valid() || !tuning.valid() || !__builtin_isfinite(state.phi_rad) || !__builtin_isfinite(state.p_rad_s) ||
	    !__builtin_isfinite(state.previous_delta)) {
		return result;
	}

	result.model_trim = input.model_sign * input.trim;
	const float previous_correction = state.previous_delta - result.model_trim;
	// Scale only the initial angle error feedback, not rate damping, trim or the whole output.
	float phi_free = state.phi_rad * tuning.error_gain;
	float phi_sensitivity = 0.f;
	float p_sensitivity = 0.f;
	float curvature = kRDDelta;
	float linear = -kRDDelta * previous_correction;

	// Exact scalar quadratic for a constant correction over the existing horizon.
	// No discrete-candidate deadband or assumed affine/self-restoring motion.
	for (int step = 0; step < kHorizon; ++step) {
		phi_free += kTs * state.p_rad_s;
		phi_sensitivity += kTs * p_sensitivity + kB0;
		p_sensitivity += kB1;
		curvature += kQPhi * phi_sensitivity * phi_sensitivity +
			     kQP * p_sensitivity * p_sensitivity + kRDelta;
		linear += kQPhi * phi_sensitivity * phi_free + kQP * p_sensitivity * state.p_rad_s;
	}

	const float free_terminal[3] = {phi_free, state.p_rad_s, 0.f};
	const float sensitivity[3] = {phi_sensitivity, p_sensitivity, 1.f};
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			curvature += sensitivity[i] * kFullRollTerminal[i][j] * sensitivity[j];
			linear += sensitivity[i] * kFullRollTerminal[i][j] * free_terminal[j];
		}
	}

	if (!__builtin_isfinite(curvature) || curvature <= 0.f || !__builtin_isfinite(linear)) {
		return result;
	}

	const float raw_correction = -linear / curvature;
	if (!__builtin_isfinite(raw_correction)) {
		return result;
	}

	// Mode 6 uses its configured correction envelope, not the legacy shadow cap.
	// The shared command projection still enforces total differential and headroom.
	const float amplitude = input_clamp(raw_correction, -input.correction_limit, input.correction_limit);
	result.command = implement_full_constrained_candidate(result.model_trim + amplitude,
			 state.previous_delta, tuning.maximum_step(), input);
	result.unconstrained_correction = input.model_sign * raw_correction;
	result.proposal.delta_raw = result.model_trim + raw_correction;
	result.proposal.delta_amplitude_limited = result.model_trim + amplitude;
	result.proposal.delta_rate_limited = result.command.model_input;
	result.proposal.delta_clipped = result.proposal.delta_amplitude_limited;
	result.proposal.delta = result.command.model_input;
	result.rate_recovery = fabsf(result.command.model_input - state.previous_delta) > tuning.maximum_step() + 1.e-6f;
	result.proposal.solver_success = __builtin_isfinite(result.command.model_input) &&
					__builtin_isfinite(result.proposal.delta_raw);
	return result;
}

MpcResult FixedLinearMPC::solve_projected_gradient_qp(const MpcState &state) {
    // Optional experiment: fixed-iteration projected gradient on a scalar first move.
    // Disabled by default in the module main; LQR/clipping is the safer baseline.
	MpcResult result{};
	result.solver_success = true;
	float u_raw = 0.0f;
	constexpr float step = 0.04f;

	for (int iter = 0; iter < 8; ++iter) {
		const float eps = 1.0e-3f;
		const float u_plus = clamp_delta_rate(clamp_delta(u_raw + eps), state.previous_delta);
		const float u_minus = clamp_delta_rate(clamp_delta(u_raw - eps), state.previous_delta);
		const float g = (rollout_cost(u_plus, state) - rollout_cost(u_minus, state)) / (2.0f * eps);
		u_raw -= step * g;
	}

	const float u_amplitude_limited = clamp_delta(u_raw);
	const float u_rate_limited = clamp_delta_rate(u_amplitude_limited, state.previous_delta);
	result.delta_raw = u_raw;
	result.delta_amplitude_limited = u_amplitude_limited;
	result.delta_rate_limited = u_rate_limited;
	result.delta_clipped = u_amplitude_limited;
	result.delta = u_rate_limited;
	result.solve_time_us = 0u;
	return result;
}

}  // namespace fixed_mpc_shadow
