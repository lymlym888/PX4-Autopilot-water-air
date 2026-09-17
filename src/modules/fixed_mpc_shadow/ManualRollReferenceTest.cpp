#include "ManualRollReference.hpp"
#include "StaticFullTrim.hpp"
#include "FullRollNominalModel.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

using namespace fixed_mpc_shadow;

int main()
{
	const uint64_t now = 10000000;
	const ManualRollSample good{now - 1000, now - 2000, .2f, 1, true, true};
	auto ref = manual_roll_reference(now, good, 30.f, .5f);
	assert(ref.valid() && fabsf(ref.roll_rad - .104719755f) < 1.e-7f);
	auto expect = [&](ManualRollSample sample, RollReferenceStatus status) {
		const auto r = manual_roll_reference(now, sample, 30.f, .5f);
		assert(r.status == status && !r.valid());
		MpcState state{.1f, .2f, .3f};
		const MpcState before = state;
		assert(!apply_roll_reference(6, r, state));
		assert(memcmp(&before, &state, sizeof(state)) == 0);
	};
	auto bad = good; bad.copied = false; expect(bad, RollReferenceStatus::Missing);
	bad = good; bad.valid = false; expect(bad, RollReferenceStatus::Invalid);
	bad = good; bad.timestamp = 0; expect(bad, RollReferenceStatus::ZeroTime);
	bad = good; bad.timestamp_sample = 0; expect(bad, RollReferenceStatus::ZeroTime);
	bad = good; bad.timestamp = now + 1; expect(bad, RollReferenceStatus::Future);
	bad = good; bad.timestamp_sample = now + 1; expect(bad, RollReferenceStatus::Future);
	bad = good; bad.timestamp_sample = now - 500; expect(bad, RollReferenceStatus::TimeOrder);
	bad = good; bad.timestamp_sample = now - 500000; expect(bad, RollReferenceStatus::Stale);
	bad = good; bad.timestamp = now; bad.timestamp_sample = now - 1000000; expect(bad, RollReferenceStatus::Stale);
	bad = good; bad.roll = NAN; expect(bad, RollReferenceStatus::Nonfinite);
	bad = good; bad.roll = INFINITY; expect(bad, RollReferenceStatus::Nonfinite);
	bad = good; bad.roll = 1.01f; expect(bad, RollReferenceStatus::OutOfRange);
	bad = good; bad.roll = -1.01f; expect(bad, RollReferenceStatus::OutOfRange);
	bad = good; bad.source = 0; expect(bad, RollReferenceStatus::InvalidSource);
	bad = good; bad.source = 8; expect(bad, RollReferenceStatus::InvalidSource);
	for (float bound : {-1.f, 80.1f, NAN, INFINITY}) {
		assert(manual_roll_reference(now, good, bound, .5f).status == RollReferenceStatus::InvalidConfig);
	}
	for (float timeout : {0.f, -.1f, 36.f, NAN, INFINITY}) {
		assert(manual_roll_reference(now, good, 30.f, timeout).status == RollReferenceStatus::InvalidConfig);
	}
	bad = good; bad.timestamp_sample = now - 499999;
	assert(manual_roll_reference(now, bad, 30.f, .5f).valid());
	for (int source = 1; source <= 7; ++source) {
		bad = good; bad.source = source;
		assert(manual_roll_reference(now, bad, 30.f, .5f).valid());
	}
	// Cached messages remain valid between updates, but a stalled source expires.
	for (uint64_t age = 2000; age < 500000; age += 20000) {
		assert(manual_roll_reference(good.timestamp_sample + age, good, 30.f, .5f).valid());
	}
	assert(!manual_roll_reference(good.timestamp_sample + 500000, good, 30.f, .5f).valid());

	FixedLinearMPC solver;
	int legacy = 0, zero = 0, translated = 0;
	for (int mode = 0; mode <= 5; ++mode) {
		for (float phi : {-.5f, 0.f, .5f}) {
			for (float p : {-.2f, 0.f, .2f}) {
				for (const auto &r : {ref, RollReference{}}) {
					MpcState state{phi, p, .1f};
					const auto before = state;
					assert(apply_roll_reference(mode, r, state));
					assert(memcmp(&state, &before, sizeof(state)) == 0);
					const auto a = solver.solve_lqr_clipping(before);
					const auto b = solver.solve_lqr_clipping(state);
					assert(a.delta_raw == b.delta_raw && a.delta == b.delta);
					++legacy;
				}
			}
		}
	}
	for (float limit : {.05f, .35f, 1.5f}) {
		for (int sign : {-1, 1}) {
			for (float trim : {-.1f, 0.f, .1f}) {
				const FullMpcInput input{trim, limit, 1.5f, 0.f, .98f, sign};
				for (float phi : {-.2f, 0.f, .2f}) {
					for (float p : {-.1f, 0.f, .1f}) {
						auto sample = good; sample.roll = 0.f;
						const auto r = manual_roll_reference(now, sample, 30.f, .5f);
						MpcState state{phi, p, sign * trim};
						const auto old = solver.solve_full_roll(state, input);
						assert(apply_roll_reference(6, r, state));
						const auto current = solver.solve_full_roll(state, input);
						assert(old.command.target == current.command.target && old.proposal.delta_raw == current.proposal.delta_raw);
						++zero;
						for (float stick : {-1.f, -.2f, .2f, 1.f}) {
							sample.roll = stick;
							const auto tracking = manual_roll_reference(now, sample, 30.f, .5f);
							MpcState shifted{phi, p, sign * trim};
							MpcState expected{phi - tracking.roll_rad, p, sign * trim};
							assert(apply_roll_reference(6, tracking, shifted));
							const auto a = solver.solve_full_roll(shifted, input);
							const auto b = solver.solve_full_roll(expected, input);
							assert(a.command.target == b.command.target && a.proposal.delta_raw == b.proposal.delta_raw);
							assert(shifted.p_rad_s == p && shifted.previous_delta == sign * trim);
							++translated;
						}
					}
				}
			}
		}
	}
	// Persistent static-trim plus solver loop: stick sign, equilibrium and limits.
	StaticFullTrim trim;
	trim.begin(0.f); trim.update(true, true, 1.5f, 1.5f, 0.f, .98f);
	const FullMpcInput input{trim.trim, 1.5f, 1.5f, 0.f, .98f, -1};
	for (float stick : {-.2f, 0.f, .2f}) {
		auto sample = good; sample.roll = stick;
		const auto r = manual_roll_reference(now, sample, 30.f, .5f);
		float previous = 0.f;
		for (int i = 0; i < 1500; ++i) {
			MpcState state{0.f, 0.f, previous};
			assert(apply_roll_reference(6, r, state));
			const auto response = solver.solve_full_roll(state, input);
			assert(response.proposal.solver_success);
			assert(fabsf(response.command.target + previous) <= .060001f);
			assert(fabsf(response.command.target) <= 1.5f);
			previous = response.command.model_input;
		}
		assert(stick == 0.f ? previous == 0.f : previous * stick < 0.f);
		MpcState at_reference{r.roll_rad, 0.f, 0.f};
		assert(apply_roll_reference(6, r, at_reference));
		assert(solver.solve_full_roll(at_reference, input).command.target == 0.f);
		assert(trim.trim == 0.f && trim.valid);
	}
	float phi = 0.f, p = 0.f, previous = 0.f;
	for (float stick : {.2f, -.2f, 0.f}) {
		auto sample = good; sample.roll = stick;
		const auto r = manual_roll_reference(now, sample, 30.f, .5f);
		for (int i = 0; i < 1500; ++i) {
			MpcState state{phi, p, previous};
			assert(apply_roll_reference(6, r, state));
			const auto response = solver.solve_full_roll(state, input);
			assert(response.proposal.solver_success);
			previous = response.command.model_input;
			float next_phi, next_p;
			full_roll_nominal_step(phi, p, previous, next_phi, next_p);
			phi = next_phi; p = next_p;
		}
		assert(fabsf(phi - r.roll_rad) < 1.e-4f && fabsf(p) < 1.e-4f);
	}
	std::cout << "manual_reference_validation,PASS\nlegacy_mode_cases," << legacy
		<< ",PASS\nzero_reference_cases," << zero << ",PASS\ntranslated_reference_cases,"
		<< translated << ",PASS\npositive_negative_zero_30s_cases,3,PASS\n"
		<< "nominal_closed_loop_reference_steps,3,PASS (not physical validation)\n";
}
