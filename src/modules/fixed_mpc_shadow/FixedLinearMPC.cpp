#include "FixedLinearMPC.hpp"
#include "FixedMPCParams.hpp"
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
