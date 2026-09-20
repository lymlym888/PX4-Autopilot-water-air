#include <lib/hydro_control/StabilizedRollReference.hpp>
#include "../fixed_mpc_shadow/ManualRollReference.hpp"
#include "../fixed_mpc_shadow/Mode6FlightMode.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

int main()
{
	constexpr float rad = 0.017453292519943295f;
	unsigned cases = 0;
	for (unsigned nav = 0; nav < 32; ++nav) {
		for (unsigned fmpc = 0; fmpc <= 6; ++fmpc) {
			for (float maximum : {10.f, 15.f, 30.f, 45.f, 80.f}) {
				for (int step = -1000; step <= 1000; ++step) {
					const float stick = step / 1000.f;
					const float raw = stick * (maximum * rad);
					const bool stab = nav == fixed_mpc_shadow::kStabilizedNavState;
					const float actual = hydro_control::limit_stabilized_roll_reference(raw, stab);
					const float expected = stab ? fmaxf(-15.f * rad, fminf(raw, 15.f * rad)) : raw;
					assert(actual == expected);
					if (!stab || fabsf(raw) <= 15.f * rad) { assert(actual == raw); }
					// The controller selector still excludes non-STAB Mode 6.
					const auto flight = fixed_mpc_shadow::select_flight_mode(fmpc, nav, 1000000, 900000, 1000000);
					assert(flight.mode6_eligible == (fmpc == 6 && stab));
					++cases;
				}
			}
		}
	}
	for (float degrees : {-40.f, -15.001f, -15.f, -14.999f, -10.f, 0.f, 10.f, 14.999f, 15.f, 15.001f, 40.f}) {
		const float expected_deg = fmaxf(-15.f, fminf(degrees, 15.f));
		const float out = hydro_control::limit_stabilized_roll_reference(degrees * rad, true);
		assert(fabsf(out / rad - expected_deg) < 2.e-6f);
		std::printf("requested_deg,%.3f,STAB_reference_deg,%.3f\n", static_cast<double>(degrees), static_cast<double>(out / rad));
	}
	for (bool stab : {false, true}) {
		assert(std::isnan(hydro_control::limit_stabilized_roll_reference(NAN, stab)));
		assert(hydro_control::limit_stabilized_roll_reference(INFINITY, stab) == INFINITY);
		assert(hydro_control::limit_stabilized_roll_reference(-INFINITY, stab) == -INFINITY);
	}
	const uint64_t now = 10000000;
	fixed_mpc_shadow::ManualRollSample sample{now - 1000, now - 2000, 0.f, 1, true, true};
	for (float maximum : {10.f, 15.f, 30.f, 45.f, 80.f}) {
		for (float stick : {-1.f, -.5f, -.25f, 0.f, .25f, .5f, 1.f}) {
			sample.roll = stick;
			const auto mpc = fixed_mpc_shadow::manual_roll_reference(now, sample, maximum, .5f);
			const float controller_reference = hydro_control::limit_stabilized_roll_reference(stick * (maximum * rad), true);
			assert(mpc.valid() && mpc.roll_rad == controller_reference);
		}
	}
	// Smaller requests are not rescaled: 25% of the existing 30-degree mapping is 7.5, not 3.75 degrees.
	sample.roll = .25f;
	assert(fabsf(fixed_mpc_shadow::manual_roll_reference(now, sample, 30.f, .5f).roll_rad / rad - 7.5f) < 1.e-6f);
	sample.roll = 1.01f;
	assert(!fixed_mpc_shadow::manual_roll_reference(now, sample, 30.f, .5f).valid());
	sample.roll = NAN;
	assert(!fixed_mpc_shadow::manual_roll_reference(now, sample, 30.f, .5f).valid());
	sample.roll = 1.f; sample.timestamp_sample = now - 500000;
	assert(!fixed_mpc_shadow::manual_roll_reference(now, sample, 30.f, .5f).valid());
	std::printf("STAB_only_reference_cap,PASS,%u helper cases; actual MPC reference agrees; physical test pending\n", cases);
}
