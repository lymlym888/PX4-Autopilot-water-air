#include "StaticFullTrim.hpp"
#include "FixedLinearMPC.hpp"
#include "../control_allocator/RollFinOutputSelector.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>

using namespace fixed_mpc_shadow;

static bool near(float a, float b) { return std::fabs(a-b) < 1.e-6f; }

int main()
{
	StaticFullTrim latch;
	latch.begin(0.f);
	latch.update(true, true, .5f, 1.f, 0.f, .98f);
	assert(latch.valid && latch.status == StaticTrimStatus::Ready);
	for (float invalid : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), 2.01f}) {
		latch.begin(invalid);
		latch.update(true, true, .5f, 1.f, 0.f, .98f);
		assert(!latch.valid && latch.status == StaticTrimStatus::InvalidConfig);
	}
	latch.begin(-.78757f);
	latch.update(true, true, .05f, .5f, 0.f, .98f);
	assert(!latch.valid && latch.status == StaticTrimStatus::OutsideBounds && latch.candidate == -.78757f);
	latch.begin(.1f);
	latch.update(true, false, .5f, 1.f, 0.f, .98f);
	assert(!latch.valid && latch.status == StaticTrimStatus::WaitingForBase);
	latch.update(true, true, .5f, 1.f, 0.f, .98f);
	assert(latch.valid && latch.trim == .1f && latch.status == StaticTrimStatus::Ready);
	latch.update(true, false, .5f, 1.f, .97f, .98f);
	assert(latch.trim == .1f); // Data faults are rejected by prepare_full_input, not by moving trim.
	latch.reset();
	assert(!latch.valid && latch.status == StaticTrimStatus::Disabled);
	latch.begin(0.f);
	latch.update(false, true, .5f, 1.f, 0.f, .98f);
	assert(!latch.valid && latch.status == StaticTrimStatus::Disabled);
	latch.update(true, true, .5f, 1.f, 0.f, .98f);
	assert(latch.valid && latch.status == StaticTrimStatus::Ready);
	// A genuinely reviewed nonzero trim must not be projected to another center.
	latch.begin(-.78757f);
	latch.update(true, true, .5f, 1.f, 0.f, .98f);
	assert(latch.valid && latch.trim == -.78757f && latch.status == StaticTrimStatus::ReadyLimited);
	assert(near(latch.reserve, .21243f));
	std::puts("static_trim_validation,PASS,no automatic zero or CADRC fallback trim; no silent projection");

	FixedLinearMPC solver;
	unsigned runs = 0;
	std::puts("configured_trim,limit,sign,roll,implemented_correction,final_diff");
	for (float configured : {0.f, -.1f, .1f}) {
		for (float limit : {.05f, .35f, .5f}) {
			for (int sign : {-1, 1}) {
				for (float roll : {-10.f, 0.f, 10.f}) {
					latch.begin(configured);
					latch.update(true, true, limit, 1.f, .02f, .98f);
					assert(latch.valid && latch.trim == configured);
					FullMpcInput input{latch.trim, limit, 1.f, .02f, .98f, sign};
					MpcState state{roll*.01745329252f, 0.f, sign*latch.trim};
					roll_fin_output::Input final{};
					roll_fin_output::State selector{};
					final.now = 1000000;
					final.requested_mode = 6;
					final.left_index = sign == -1 ? 4 : 3;
					final.right_index = sign == -1 ? 3 : 4;
					final.servo_count = 8;
					FullMpcResult r{};
					for (unsigned step = 0; step < 1000; ++step) {
						r = solver.solve_full_roll(state, input);
						assert(r.proposal.solver_success && !r.command.constraint_conflict);
						assert(std::fabs(r.command.model_input-state.previous_delta) <= .060001f);
						auto &s = final.source;
						s.status_timestamp = final.now; s.source_timestamp = final.now;
						s.mode_entry_timestamp = 1000000; s.selector_epoch = 1000000;
						s.session_id = 1; s.sequence = step+1; s.requested_mode = 6; s.source_mode = 6;
						s.left_index = final.left_index; s.right_index = final.right_index; s.command_valid = true;
						s.command_left = input.mean_servo + .5f*r.command.target;
						s.command_right = input.mean_servo - .5f*r.command.target;
						s.requested_diff = r.command.requested_target; s.implemented_diff = r.command.target;
						final.ordinary_left = step%2 ? -.9f : .9f;
						final.ordinary_right = -final.ordinary_left;
						const auto selected = roll_fin_output::select(final, selector);
						assert(selected.owner == roll_fin_output::Owner::Mpc);
						assert(near(selected.final_diff, r.command.target));
						assert(std::fabs(selected.final_left) <= .98f && std::fabs(selected.final_right) <= .98f);
						state.previous_delta = sign*selected.final_diff;
						final.now += 20000;
					}
					const float correction = r.command.target-configured;
					assert(roll == 0.f ? near(correction, 0.f) : correction*roll*sign > 0.f);
					if (configured == 0.f && roll != 0.f) { assert(near(std::fabs(r.command.target),limit)); }
					std::printf("%.2f,%.2f,%d,%.1f,%.5f,%.5f\n",configured,limit,sign,roll,correction,r.command.target);
					++runs;
				}
			}
		}
	}
	std::printf("reviewed_static_trim_pipeline,PASS,%u synthetic 20-second solver/selector runs; not mechanical calibration\n",runs);
}
