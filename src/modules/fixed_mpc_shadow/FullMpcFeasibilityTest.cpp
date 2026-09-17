#include "FixedLinearMPC.hpp"
#include "../control_allocator/RollFinOutputSelector.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace fixed_mpc_shadow;

int main()
{
	FixedLinearMPC solver;
	const float candidates[] {-.75317f, .75317f};
	const float limits[] {.05f, .35f};
	const int signs[] {-1, 1};
	const float angles[] {-3.f, 0.f, 3.f};
	unsigned runs = 0;
	std::puts("raw_trim,limit,sign,roll_deg,latched_trim,final_target,correction");
	for (float raw : candidates) {
		for (float limit : limits) {
			for (int sign : signs) {
				FullMpcInput input{};
				input.trim = raw;
				input.correction_limit = limit;
				input.candidate_limit = .5f;
				input.servo_limit = .98f;
				input.model_sign = sign;
				const auto latch = project_full_trim(input);
				assert(latch.valid && latch.projected);
				assert(std::fabs(std::fabs(latch.trim) - (.5f-limit)) < 1.e-6f);
				assert(std::fabs(latch.trim) + latch.reserve <= .500001f);
				input.trim = latch.trim;
				for (float angle : angles) {
					// Feedback begins at the observed physical boundary, not fictitious raw trim.
					MpcState state{angle*.01745329252f, 0.f, sign*std::copysign(.5f, raw)};
					roll_fin_output::State selector_state{};
					roll_fin_output::Input final{};
					final.requested_mode = 6; final.now = 1000000;
					final.left_index = sign == -1 ? 4 : 3;
					final.right_index = sign == -1 ? 3 : 4; final.servo_count = 8;
					FullMpcResult r{};
					for (unsigned step = 0; step < 1000; ++step) {
						r = solver.solve_full_roll(state, input);
						assert(r.proposal.solver_success && !r.command.constraint_conflict);
						assert(std::fabs(r.command.target) <= .500001f);
						assert(std::fabs(r.command.target-input.trim) <= limit+1.e-6f);
						assert(r.rate_recovery || std::fabs(r.command.model_input-state.previous_delta) <= .060001f);
						auto &s = final.source;
						s.status_timestamp = final.now; s.source_timestamp = final.now;
						s.mode_entry_timestamp = 1000000; s.selector_epoch = 1000000;
						s.session_id = 1; s.sequence = step+1; s.requested_mode = 6; s.source_mode = 6;
						s.left_index = final.left_index; s.right_index = final.right_index; s.command_valid = true;
						s.command_left = .5f*r.command.target; s.command_right = -.5f*r.command.target;
						s.requested_diff = r.command.requested_target; s.implemented_diff = r.command.target;
						final.ordinary_left = step%2 ? .8f : -.8f;
						final.ordinary_right = -final.ordinary_left;
						const auto selected = roll_fin_output::select(final, selector_state);
						assert(selected.owner == roll_fin_output::Owner::Mpc);
						assert(std::fabs(selected.final_diff-r.command.target) < 1.e-6f);
						state.previous_delta = sign*selected.final_diff;
						final.now += 20000;
					}
					const float correction = r.command.target-input.trim;
					assert(angle == 0.f ? std::fabs(correction) < 1.e-5f : correction*angle*sign > .001f);
					std::printf("%.5f,%.2f,%d,%.1f,%.5f,%.5f,%.5f\n", raw,limit,sign,angle,input.trim,r.command.target,correction);
					++runs;
				}
			}
		}
	}
	std::printf("projected_trim_bidirectional,PASS,%u 20-second runs using actual solver and final selector\n",runs);

	FullMpcInput input{};
	input.trim = -.75317f; input.correction_limit = .05f; input.candidate_limit = .5f;
	input.servo_limit = .98f; input.model_sign = -1;
	const auto impossible = solver.solve_full_roll({.015967f, 0.f, .5f}, input);
	assert(impossible.command.constraint_conflict && std::fabs(impossible.command.target+.5f) < 1.e-6f);
	input.trim = project_full_trim(input).trim;
	input.mean_servo = .97f;
	const auto shrunk = solver.solve_full_roll({0.f,0.f,.45f},input);
	assert(shrunk.command.constraint_conflict && shrunk.command.headroom_clamped && shrunk.rate_recovery);
	assert(std::fabs(shrunk.command.target) <= .020001f);
	assert(input.trim == -.45f);
	input.mean_servo = 0.f;
	const auto recovery = solver.solve_full_roll({0.f,0.f,shrunk.command.model_input},input);
	assert(!recovery.command.constraint_conflict && recovery.rate_recovery);
	assert(std::fabs(recovery.command.target+.4f) < 1.e-6f);
	std::puts("constraint_conflict_and_recovery,PASS,no fake correction authority or trim relatching when headroom shrinks");

	input.trim = -.2f;
	assert(!project_full_trim(input).projected);
	input.mean_servo = .98f;
	assert(project_full_trim(input).trim == 0.f && project_full_trim(input).reserve == 0.f);
	input.mean_servo = 1.f;
	assert(project_full_trim(input).valid && project_full_trim(input).bound == 0.f);
	input.mean_servo = 1.01f;
	assert(!project_full_trim(input).valid);
	input.mean_servo = 0.f; input.trim = std::numeric_limits<float>::quiet_NaN();
	assert(!project_full_trim(input).valid);
	input.trim = 0.f; input.correction_limit = -1.f;
	assert(!project_full_trim(input).valid);
	std::puts("trim_invalid_and_zero_headroom,PASS,invalid input rejected and zero physical authority not invented");
}
