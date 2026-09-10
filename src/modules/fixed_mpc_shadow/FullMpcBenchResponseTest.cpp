#include "FixedLinearMPC.hpp"
#include "FullRollNominalModel.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

using namespace fixed_mpc_shadow;

// Acceptance probe for the requested quasi-static bench response, not a plant
// stability proof. Failure must not be "fixed" by reversing an actuator sign.
static FullMpcResult held_attitude(float roll_deg, const FullMpcInput &input)
{
	FixedLinearMPC solver;
	MpcState state{roll_deg * 0.01745329252f, 0.f, input.model_sign * input.trim};
	FullMpcResult result{};

	for (int step = 0; step < 200; ++step) {
		result = solver.solve_full_roll(state, input);

		if (!result.proposal.solver_success) {
			return result;
		}

		state.previous_delta = result.command.model_input;
	}

	return result;
}

int main()
{
	const float trims[] = {0.f, -0.22805431f, 0.22805431f};
	const float limits[] = {0.05f, 0.35f};
	const float rolls[] = {-7.f, -3.f, 0.f, 3.f, 7.f};
	unsigned failures = 0;
	unsigned cases = 0;
	std::puts("trim,limit,roll_deg,correction,target,solver_ok,quasistatic_acceptance");

	for (float trim : trims) {
		for (float limit : limits) {
			FullMpcInput input{};
			input.trim = trim;
			input.correction_limit = limit;
			input.candidate_limit = 0.5f;
			input.servo_limit = 0.98f;
			input.model_sign = -1;

			for (float roll : rolls) {
				const auto result = held_attitude(roll, input);
				// Log853's zero-reference CADRC trend is negative runtime
				// correction for positive roll. This is a requested benchmark,
				// not an independently measured hydrodynamic sign calibration.
				const bool response_ok = roll == 0.f ? std::fabs(result.command.correction) < 1.e-5f :
							result.command.correction * roll < -1.e-5f;
				const bool passed = result.proposal.solver_success && response_ok;
				failures += passed ? 0 : 1;
				++cases;
				std::printf("%.8f,%.2f,%.1f,%.8f,%.8f,%d,%s\n", trim, limit, roll,
					    result.command.correction, result.command.target,
					    result.proposal.solver_success ? 1 : 0, passed ? "PASS" : "FAIL");
			}
		}
	}

	std::fprintf(stderr, "Quasi-static acceptance: %u / %u failed. No hardware was commanded.\n", failures, cases);
	if (failures != 0) {
		return 1;
	}

	FixedLinearMPC solver;
	FullMpcInput input{};
	input.trim = -0.22805431f;
	input.correction_limit = .35f;
	input.candidate_limit = .5f;
	input.servo_limit = .98f;
	input.model_sign = -1;
	const float model_trim = input.model_sign * input.trim;
	for (float rate : {-0.1f, 0.1f}) {
		const auto r = solver.solve_full_roll({0.f, rate, model_trim}, input);
		assert(r.proposal.solver_success && r.command.correction * rate < 0.f);
	}
	const auto braking = solver.solve_full_roll({.12f, -.4f, model_trim}, input);
	const auto slow_return = solver.solve_full_roll({.12f, -.01f, model_trim}, input);
	assert(braking.command.correction > 0.f && slow_return.command.correction < 0.f);
	std::puts("rate_damping_and_return_braking,PASS");

	for (float limit : limits) {
		input.correction_limit = limit;
		for (float angle : {.0001f, .001f, .01f}) {
			const auto positive = solver.solve_full_roll({angle, 0.f, model_trim}, input);
			const auto negative = solver.solve_full_roll({-angle, 0.f, model_trim}, input);
			assert(positive.command.correction < 0.f && negative.command.correction > 0.f);
			assert(std::fabs(positive.command.correction + negative.command.correction) < 1.e-6f);
			assert(std::fabs(positive.command.model_input - model_trim) <= kDDeltaMax + 1.e-6f);
		}
	}
	std::puts("near_zero_continuous_symmetric_correction,PASS");

	// A clamp is a real limitation; do not demand equal absolute endpoints about
	// nonzero trim or make status falsely claim the requested target was applied.
	input.correction_limit = .35f;
	const auto clipped = held_attitude(7.f, input);
	assert(clipped.command.candidate_clamped && std::fabs(clipped.command.target + .5f) < 1.e-6f);
	assert(std::fabs(clipped.command.target - input.trim - clipped.command.correction) > .01f);
	input.mean_servo = .97f;
	const auto headroom = solver.solve_full_roll({.12f, 0.f, model_trim}, input);
	assert(headroom.command.headroom_clamped && headroom.rate_recovery);
	assert(std::fabs(headroom.command.target) <= .020001f);
	std::puts("physical_clamp_and_rate_recovery_reporting,PASS");
	input.mean_servo = 0.f;

	unsigned simulated = 0;
	for (float trim : trims) {
		input.trim = trim;
		for (float limit : limits) {
			input.correction_limit = limit;
			for (float roll : {-7.f, 7.f}) {
				MpcState state{roll * .01745329252f, 0.f, input.model_sign * trim};
				for (int step = 0; step < 1000; ++step) {
					const auto r = solver.solve_full_roll(state, input);
					assert(r.proposal.solver_success);
					assert(std::fabs(r.command.target) <= .500001f);
					assert(!r.rate_recovery);
					const float correction = r.command.model_input - input.model_sign * trim;
					float phi, p;
					full_roll_nominal_step(state.phi_rad, state.p_rad_s, correction, phi, p);
					state = {phi, p, r.command.model_input};
				}
				assert(std::fabs(state.phi_rad) < .001f && std::fabs(state.p_rad_s) < .001f);
				++simulated;
			}
		}
	}
	std::printf("nominal_closed_loop,PASS,%u synthetic 20-second runs; not physical validation\n", simulated);
	return 0;
}
