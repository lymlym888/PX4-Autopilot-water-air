#include "FixedLinearMPC.hpp"
#include "StaticFullTrim.hpp"
#include "../control_allocator/RollFinOutputSelector.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>

#define fixed_mpc_shadow previous_input
#include DEPTH_PRIORITY_BASELINE_INPUT
#undef fixed_mpc_shadow

using namespace fixed_mpc_shadow;
namespace rf = roll_fin_output;

static bool near(float a, float b) { return std::fabs(a - b) < 2.e-6f; }

int main()
{
	previous_input::FullMpcInput old{0.f, 2.f, 2.f, 1.f, .98f, -1};
	assert(!old.valid());
	std::puts("original_common_one_rejection,REPRODUCED");

	unsigned regression = 0;
	for (float mean : {-.98f, -.97f, -.5f, 0.f, .5f, .97f, .98f}) {
		for (float trim : {-.4f, 0.f, .4f}) {
			for (float proposal : {-2.f, -.2f, 0.f, .2f, 2.f}) {
				for (float previous : {-1.f, 0.f, 1.f}) {
					FullMpcInput in{trim, .5f, 2.f, mean, .98f, -1};
					old = {trim, .5f, 2.f, mean, .98f, -1};
					const auto a = implement_full_constrained_candidate(proposal, previous, .06f, in);
					const auto b = previous_input::implement_full_constrained_candidate(proposal, previous, .06f, old);
					assert(a.target == b.target && a.correction == b.correction && a.model_input == b.model_input);
					assert(a.requested_target == b.requested_target && a.candidate_clamped == b.candidate_clamped);
					assert(a.headroom_clamped == b.headroom_clamped && a.slew_limited == b.slew_limited);
					assert(a.constraint_conflict == b.constraint_conflict);
					++regression;
				}
			}
		}
	}
	std::printf("previous_feasible_projection_bitwise,PASS,%u\n", regression);

	FixedLinearMPC solver;
	unsigned samples = 0;
	for (int sign : {-1, 1}) {
		for (float trim : {-.2f, 0.f, .2f}) {
			for (float start_common : {-1.f, 1.f}) {
				StaticFullTrim latch;
				latch.begin(trim);
				latch.update(true, true, .05f, 2.f, start_common, .98f);
				assert(latch.valid && latch.trim == trim && latch.reserve == 0.f);
				assert(latch.status == StaticTrimStatus::ReadyLimited);
				FullMpcInput in{trim, .05f, 2.f, start_common, .98f, sign};
				MpcState state{.15f, 0.f, sign * trim};
				rf::State selector{};
				rf::Input final{};
				final.now = 1000000; final.requested_mode = 6;
				final.left_index = sign == -1 ? 4 : 3;
				final.right_index = sign == -1 ? 3 : 4; final.servo_count = 8;
				// Enter saturated, regain room, exhaust it again, then recover.
				for (unsigned step = 0; step < 3000; ++step) {
					const float commons[] {start_common, .97f, 0.f, -start_common, -.99f, 0.f};
					in.mean_servo = commons[step / 500];
					state.phi_rad = step % 200 < 100 ? .15f : -.15f;
					assert(in.valid());
					const auto r = solver.solve_full_roll(state, in);
					assert(r.proposal.solver_success);
					const float room = full_differential_headroom(in.mean_servo, in.servo_limit);
					assert(std::fabs(r.command.target) <= room + 1.e-6f);
					if (std::fabs(in.mean_servo) > .98f) {
						assert(r.command.target == 0.f && r.command.headroom_clamped);
					}
					// Same residual-headroom projection used by HydroAllocator for a newer base.
					const float target = input_clamp(r.command.target, -room, room);
					auto &s = final.source;
					s.status_timestamp = final.now; s.source_timestamp = final.now;
					s.mode_entry_timestamp = 1000000; s.selector_epoch = 1000000;
					s.session_id = 1; s.sequence = step + 1;
					s.requested_mode = 6; s.source_mode = 6;
					s.left_index = final.left_index; s.right_index = final.right_index;
					s.command_valid = true;
					s.command_left = in.mean_servo + target / 2.f;
					s.command_right = in.mean_servo - target / 2.f;
					s.requested_diff = r.command.requested_target; s.implemented_diff = target;
					final.ordinary_left = step % 2 ? .9f : -.9f;
					final.ordinary_right = -final.ordinary_left;
					const auto out = rf::select(final, selector);
					assert(out.owner == rf::Owner::Mpc && out.reason == rf::Reason::None);
					assert(near((out.final_left + out.final_right) / 2.f, in.mean_servo));
					assert(near(out.final_diff, target));
					assert(std::fabs(out.final_left) <= 1.f && std::fabs(out.final_right) <= 1.f);
					assert(selector.fallback_event_count == 0 && latch.trim == trim);
					if (step % 100 > 50 && in.mean_servo == 0.f) {
						assert(std::fabs(out.final_diff) > .01f);
					}
					state.previous_delta = sign * out.final_diff;
					final.now += 20000;
					++samples;
				}
				final.now += rf::kSourceMaxAgeUs + 1;
				assert(rf::select(final, selector).owner != rf::Owner::Mpc);
				assert(selector.fallback_event_count == 1);
			}
		}
	}
	std::printf("depth_priority_solver_selector,PASS,%u samples in 12 synthetic 60-second runs; real board NOT TESTED\n", samples);
	FullMpcInput in{0.f, 2.f, 2.f, 0.f, .98f, -1};
	for (float bad : {NAN, INFINITY, -INFINITY, 1.01f, -1.01f}) {
		in.mean_servo = bad;
		assert(!in.valid() && !solver.solve_full_roll({.1f, 0.f, 0.f}, in).proposal.solver_success);
	}
	in.mean_servo = 0.f;
	assert(!solver.solve_full_roll({NAN, 0.f, 0.f}, in).proposal.solver_success);
	StaticFullTrim invalid;
	invalid.begin(.6f);
	invalid.update(true, true, .05f, .5f, 1.f, .98f);
	assert(!invalid.valid && invalid.status == StaticTrimStatus::OutsideBounds);
	std::puts("nonfinite_physical_bounds_stale_and_invalid_trim,PASS");
}
