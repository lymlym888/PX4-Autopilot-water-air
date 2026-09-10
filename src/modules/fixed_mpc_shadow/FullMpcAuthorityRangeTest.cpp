#include "FixedLinearMPC.hpp"
#include "FixedMpcInterface.hpp"
#include "../control_allocator/RollFinOutputSelector.hpp"

#ifdef STAGE5A_BASELINE_HEADER
#define fixed_mpc_shadow before_shadow
#include STAGE5A_BASELINE_HEADER
#undef fixed_mpc_shadow
#endif

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

using namespace fixed_mpc_shadow;

static FullMpcResult run(float roll, FullMpcInput input)
{
	FixedLinearMPC solver;
	MpcState state{roll * .01745329252f, 0.f, input.model_sign * input.trim};
	roll_fin_output::State selector_state{};
	roll_fin_output::Input final{};
	final.requested_mode = 6;
	final.now = 1000000;
	final.left_index = input.model_sign == -1 ? 4 : 3;
	final.right_index = input.model_sign == -1 ? 3 : 4;
	final.servo_count = 8;
	FullMpcResult result{};
	for (unsigned step = 0; step < 1000; ++step) {
		result = solver.solve_full_roll(state, input);
		assert(result.proposal.solver_success && !result.command.constraint_conflict);
		assert(result.rate_recovery || std::fabs(result.command.model_input - state.previous_delta) <= .060001f);
		assert(std::fabs(result.command.target - input.trim) <= input.correction_limit + 1.e-6f);
		assert(std::fabs(result.command.target) <= input.candidate_limit + 1.e-6f);
		auto &s = final.source;
		s.status_timestamp = final.now;
		s.source_timestamp = final.now;
		s.mode_entry_timestamp = 1000000;
		s.selector_epoch = 1000000;
		s.session_id = 1;
		s.sequence = step + 1;
		s.requested_mode = 6;
		s.source_mode = 6;
		s.left_index = final.left_index;
		s.right_index = final.right_index;
		s.command_valid = true;
		s.command_left = input.mean_servo + .5f * result.command.target;
		s.command_right = input.mean_servo - .5f * result.command.target;
		s.requested_diff = result.command.requested_target;
		s.implemented_diff = result.command.target;
		assert(std::fabs(s.command_left) <= input.servo_limit + 1.e-6f);
		assert(std::fabs(s.command_right) <= input.servo_limit + 1.e-6f);
		final.ordinary_left = step % 2 ? .8f : -.8f;
		final.ordinary_right = -final.ordinary_left;
		const auto selected = roll_fin_output::select(final, selector_state);
		assert(selected.owner == roll_fin_output::Owner::Mpc);
		assert(std::fabs(selected.final_diff - result.command.target) < 1.e-6f);
		assert(std::fabs(.5f * (selected.final_left + selected.final_right) - input.mean_servo) < 1.e-6f);
		// With identical pair commands and zero ordinary input, both owners must
		// publish identical servo commands. No Mode-6-only actuator scaling.
		auto legacy = final;
		legacy.requested_mode = 1;
		legacy.ordinary_left = legacy.ordinary_right = 0.f;
		legacy.legacy_hydro_left = s.command_left;
		legacy.legacy_hydro_right = s.command_right;
		roll_fin_output::State legacy_state{};
		const auto baseline = roll_fin_output::select(legacy, legacy_state);
		assert(std::fabs(baseline.final_left - selected.final_left) < 1.e-6f);
		assert(std::fabs(baseline.final_right - selected.final_right) < 1.e-6f);
		state.previous_delta = input.model_sign * selected.final_diff;
		final.now += 20000;
	}
	return result;
}

int main()
{
	assert(kFmpcFullLimHardMax == 2.f && kFmpcFullCandMaxHardMax == 2.f);
	unsigned runs = 0;
	std::puts("limit,common,sign,roll,trim,target");
	for (float limit : {.05f, .35f, .5f, 1.f, 2.f}) {
		for (float common : {-.3f, 0.f, .3f}) {
			for (int sign : {-1, 1}) {
				for (float roll : {-20.f, 0.f, 20.f}) {
					FullMpcInput input{};
					input.trim = -.27577f;
					input.correction_limit = limit;
					input.candidate_limit = kFmpcFullCandMaxHardMax;
					input.mean_servo = common;
					input.servo_limit = .98f;
					input.model_sign = sign;
					input.trim = project_full_trim(input).trim;
					const auto r = run(roll, input);
					const float correction = r.command.target - input.trim;
					assert(roll == 0.f ? std::fabs(correction) < 1.e-5f : correction * roll * sign > 0.f);
					if (limit >= .5f && roll != 0.f) { assert(std::fabs(correction) > .35f); }
					std::printf("%.2f,%.2f,%d,%.1f,%.5f,%.5f\n", limit, common, sign, roll, input.trim, r.command.target);
					++runs;
				}
			}
		}
	}
	std::printf("expanded_envelope,PASS,%u synthetic 20-second solver-to-selector runs\n", runs);
	FullMpcInput input{};
	input.correction_limit = 2.f;
	input.candidate_limit = .5f;
	input.servo_limit = .98f;
	input.model_sign = -1;
	const auto candidate = run(20.f, input);
	assert(candidate.command.candidate_clamped && std::fabs(candidate.command.target + .5f) < 1.e-6f);
	input.candidate_limit = 2.f;
	input.mean_servo = .9f;
	const auto headroom = run(20.f, input);
	assert(headroom.command.headroom_clamped && std::fabs(headroom.command.target + .16f) < 1.e-6f);
	std::puts("fcand_and_physical_margin,PASS,configured and physical limits remain independent");

#ifdef STAGE5A_BASELINE_HEADER
	FixedLinearMPC solver;
	before_shadow::FixedLinearMPC before;
	unsigned compared = 0;
	float worst = 0.f;
	for (float limit : {0.f, .01f, .05f, .2f, .35f}) {
		for (int index = 0; index < 4096; ++index) {
			input.correction_limit = limit;
			input.candidate_limit = .5f;
			input.mean_servo = (index % 19 - 9) * .05f;
			input.trim = (index % 17 - 8) * .075f;
			input.model_sign = index % 2 ? -1 : 1;
			input.trim = project_full_trim(input).trim;
			const MpcState state{(index % 71 - 35) * .01f, (index % 53 - 26) * .025f,
				input.model_sign * (input.trim + (index % 7 - 3) * .01f)};
			before_shadow::FullMpcInput old_input{};
			old_input.trim = input.trim;
			old_input.correction_limit = input.correction_limit;
			old_input.candidate_limit = input.candidate_limit;
			old_input.mean_servo = input.mean_servo;
			old_input.servo_limit = input.servo_limit;
			old_input.model_sign = input.model_sign;
			const auto old = before.solve_full_roll({state.phi_rad, state.p_rad_s, state.previous_delta}, old_input);
			const auto now = solver.solve_full_roll(state, input);
			assert(now.proposal.solver_success == old.proposal.solver_success);
			assert(now.unconstrained_correction == old.unconstrained_correction);
			const float error = std::fabs(now.command.target - old.command.target);
			worst = std::fmax(worst, error);
			assert(error < 1.e-6f);
			assert(std::fabs(now.command.correction - old.command.correction) < 1.e-6f);
			assert(now.command.constraint_conflict == old.command.constraint_conflict);
			++compared;
		}
	}
	std::printf("existing_envelopes,PASS,%u comparisons to pre-edit actual solver; max target error %.9g\n", compared, worst);
#endif
}
