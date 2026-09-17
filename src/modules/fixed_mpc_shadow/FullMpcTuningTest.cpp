#include "FixedLinearMPC.hpp"
#include "FullRollNominalModel.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

using namespace fixed_mpc_shadow;

static bool near(float a, float b) { return std::fabs(a - b) < 1.e-6f; }

int main()
{
	FixedLinearMPC solver;
	FullMpcInput input{0.f, 2.f, 2.f, 0.f, .98f, -1};
	FullMpcTuning tuning{};
	assert(tuning.valid() && !tuning.sanitize() && tuning.maximum_step() == kDDeltaMax);
	for (float bad : {NAN, INFINITY, -INFINITY, -1.f, 0.f, 100.f}) {
		tuning = {bad, bad};
		assert(!solver.solve_full_roll({.01f, 0.f, 0.f}, input, tuning).proposal.solver_success);
		assert(tuning.sanitize() && tuning.valid());
	}
	std::puts("invalid_tuning_and_runtime_sanitization,PASS");

	for (float gain : {.5f, 1.f, 1.2f, 2.f}) {
		tuning = {3.f, gain};
		const MpcState state{.001f, 0.f, 0.f};
		const auto normal = solver.solve_full_roll(state, input);
		const auto positive = solver.solve_full_roll(state, input, tuning);
		const auto negative = solver.solve_full_roll({-.001f, 0.f, 0.f}, input, tuning);
		assert(positive.proposal.solver_success && !positive.command.slew_limited);
		assert(near(positive.command.correction, normal.command.correction * gain));
		assert(near(positive.command.correction, -negative.command.correction));
		assert(positive.command.correction < 0.f);
		const auto rate = solver.solve_full_roll({0.f, .01f, 0.f}, input, tuning);
		const auto original_rate = solver.solve_full_roll({0.f, .01f, 0.f}, input);
		assert(rate.command.correction == original_rate.command.correction);
		assert(solver.solve_full_roll({0.f, 0.f, 0.f}, input, tuning).command.correction == 0.f);
	}
	std::puts("angle_gain_symmetry_zero_and_unscaled_rate_feedback,PASS");

	for (float rate : {1.5f, 3.f, 3.6f, 6.f}) {
		tuning = {rate, 1.f};
		const auto r = solver.solve_full_roll({1.f, 0.f, 0.f}, input, tuning);
		assert(r.proposal.solver_success && r.command.slew_limited);
		assert(near(r.command.model_input, tuning.maximum_step()));
	}
	std::puts("configured_normal_step,PASS");

	unsigned runs = 0;
	unsigned failures = 0;
	for (float gain : {.5f, 1.f, 1.2f, 2.f}) {
		for (float rate : {1.5f, 3.f, 3.6f, 6.f}) {
			for (float trim : {-.2f, 0.f, .2f}) {
				for (float angle : {-.08f, .08f}) {
					tuning = {rate, gain};
					input.trim = trim;
					MpcState state{angle, 0.f, input.model_sign * trim};
					for (int step = 0; step < 1500; ++step) {
						const auto r = solver.solve_full_roll(state, input, tuning);
						assert(r.proposal.solver_success && !r.rate_recovery);
						assert(std::fabs(r.command.model_input - state.previous_delta) <= tuning.maximum_step() + 1.e-6f);
						assert(std::fabs(r.command.target) <= 1.960001f);
						float phi, p;
						full_roll_nominal_step(state.phi_rad, state.p_rad_s,
								r.command.model_input - input.model_sign * trim, phi, p);
						state = {phi, p, r.command.model_input};
					}
					if (std::fabs(state.phi_rad) >= .001f || std::fabs(state.p_rad_s) >= .001f) {
						std::fprintf(stderr, "nonconverged gain=%g rate=%g trim=%g angle=%g final_phi=%g final_p=%g\n",
							gain, rate, trim, angle, state.phi_rad, state.p_rad_s);
						++failures;
					}
					++runs;
				}
			}
		}
	}
	assert(failures == 0);
	std::printf("nominal_synthetic_closed_loop,PASS,%u runs; NOT hardware validation\n", runs);

	input.trim = 0.f;
	input.correction_limit = .05f;
	tuning = {6.f, 2.f};
	assert(std::fabs(solver.solve_full_roll({1.f, 0.f, 0.f}, input, tuning).command.correction) <= .050001f);
	input.correction_limit = 2.f;
	input.candidate_limit = .1f;
	const auto candidate = solver.solve_full_roll({1.f, 0.f, 0.f}, input, tuning);
	assert(candidate.command.candidate_clamped && std::fabs(candidate.command.target) <= .100001f);
	input.mean_servo = .97f;
	const auto physical = solver.solve_full_roll({1.f, 0.f, 1.f}, input, tuning);
	assert(physical.command.headroom_clamped && physical.rate_recovery);
	assert(std::fabs(physical.command.target) <= .020001f);
	input.mean_servo = NAN;
	assert(!solver.solve_full_roll({0.f, 0.f, 0.f}, input, tuning).proposal.solver_success);
	std::puts("amplitude_candidate_physical_limits_and_invalid_input,PASS");

	input = {0.f, 2.f, 2.f, 0.f, .98f, -1};
	MpcState state{1.f, 0.f, 0.f};
	for (const auto live : {FullMpcTuning{6.f, 2.f}, FullMpcTuning{1.5f, .5f}, FullMpcTuning{3.f, 1.f}}) {
		const auto r = solver.solve_full_roll(state, input, live);
		assert(r.proposal.solver_success);
		assert(std::fabs(r.command.model_input - state.previous_delta) <= live.maximum_step() + 1.e-6f);
		state.previous_delta = r.command.model_input;
	}
	std::puts("live_tuning_changes_remain_rate_bounded,PASS");
}
