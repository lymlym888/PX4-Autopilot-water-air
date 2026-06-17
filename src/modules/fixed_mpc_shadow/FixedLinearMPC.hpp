#pragma once
#include <stdint.h>

namespace fixed_mpc_shadow {

struct MpcState {
    float phi_rad;
    float p_rad_s;
    float previous_delta;
};

struct MpcResult {
	float delta;
	float delta_raw;
	float delta_amplitude_limited;
	float delta_rate_limited;
	float delta_clipped;
	bool solver_success;
	uint32_t solve_time_us;
};

class FixedLinearMPC {
public:
    void init();
    MpcResult solve_lqr_clipping(const MpcState &state);
    MpcResult solve_projected_gradient_qp(const MpcState &state);
    void one_step(float phi, float p, float delta, float &phi_next, float &p_next) const;

private:
    float clamp_delta(float delta) const;
    float clamp_delta_rate(float delta, float previous_delta) const;
    float rollout_cost(float delta, const MpcState &state) const;
};

}  // namespace fixed_mpc_shadow
