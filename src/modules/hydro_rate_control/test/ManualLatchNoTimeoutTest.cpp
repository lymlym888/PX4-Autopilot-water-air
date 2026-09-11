// No hardware: verify the log886-style publication gap cannot expire the latch.
#include "../HydroRollController.hpp"
#include "../RollReferenceGenerator.hpp"
#include "../LADRC.hpp"
#include "../MCEO_ADRC.hpp"
#include "../AMCEO_ADRC.hpp"
#include "../ClassicalNADRC.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

template<typename Controller>
void runGapSequence(Controller &controller, bool private_td)
{
	HydroRollControllerInput in{};
	in.phi = 0.02f;
	in.command_roll = in.phi;
	in.phi_ref = in.phi;
	in.dt_measured = in.dt_used = 0.0025f;
	in.u_max = 0.5f;
	controller.reset(in);
	const auto resets = controller.resetCounter();
	ManualRollCommandLatch latch;
	latch.reset(in.phi);
	RollReferenceGenerator reference;
	reference.reset(in.phi, in.phi);
	HydroRollLimiter limiter;
	limiter.reset();
	float previous = 0.f;
	for (unsigned k = 0; k < 1000; ++k) {
		const uint64_t now = 1000000 + k * 2500;
		const bool fresh = k % 92 == 0; // 230 ms gaps cross the old 200 ms threshold.
		latch.update(true, fresh, true, 0.2f, 1, now, now, 0.5f, in.phi);
		assert(latch.valid()); // No local expiry -> no age-triggered fallback/reset.
		assert(std::fabs(latch.commandRoll() - 0.1f) < 1e-6f);
		in.timestamp_sample = now;
		in.command_roll = latch.commandRoll();
		if (!private_td) {
			assert(reference.update(in.command_roll, 2.f, 1.f, in.dt_used));
			in.phi_ref = reference.phiRef();
			in.phi_ref_dot = reference.phiRefDot();
			in.phi_ref_ddot = reference.phiRefDdot();
		} else {
			in.phi_ref = in.command_roll;
			in.phi_ref_dot = in.phi_ref_ddot = 0.f;
		}
		in.u_applied_prev = limiter.appliedPrevious();
		const auto request = controller.update(in);
		assert(request.valid && controller.resetCounter() == resets);
		assert(request.reset_event == (k == 0));
		const auto limited = limiter.update(request.u_requested, 0.f, in.dt_used, 0.5f, 10.f);
		assert(limited.valid && std::fabs(limited.u_applied) <= 0.5f);
		assert(std::fabs(limited.u_applied - previous) <= 10.f * in.dt_used + 1e-6f);
		previous = limited.u_applied;
	}
}

int main()
{
	ManualRollCommandLatch latch;
	latch.reset(0.f);
	latch.update(true, false, false, 0.f, 0, 0, 1000000, 0.5f, 0.f);
	assert(!latch.valid()); // No invented reference before first valid input.
	latch.update(true, true, true, 0.4f, 1, 1000000, 1000000, 0.5f, 0.f);
	for (uint64_t gap : {199000ULL, 200000ULL, 202000ULL, 230000ULL, 500000ULL, 2000000ULL, 60000000ULL}) {
		latch.update(true, false, false, NAN, 0, 0, 1000000 + gap, 0.5f, -0.1f);
		assert(latch.valid() && std::fabs(latch.commandRoll()-0.2f) < 1e-6f);
		assert(std::fabs(latch.ageSeconds()-gap*1e-6f) < 1e-5f);
	}
	// PX4 explicit invalid input still invalidates; local hold cannot mask it.
	latch.update(true, true, false, 0.4f, 1, 62000000, 62000000, 0.5f, -0.1f);
	assert(!latch.valid() && latch.commandRoll() == -0.1f);
	latch.update(true, true, true, NAN, 1, 63000000, 63000000, 0.5f, 0.f);
	assert(!latch.valid());
	latch.update(true, true, true, 0.4f, 1, 64000001, 64000000, 0.5f, 0.f);
	assert(!latch.valid());
	latch.update(true, true, true, 0.4f, 1, 65000000, 65000000, 0.5f, 0.f);
	assert(latch.valid());
	latch.update(false, false, false, 0.f, 0, 0, 65000001, 0.5f, 0.f);
	assert(!latch.valid()); // Leaving applicable mode still clears the latch.

	LADRC ladrc;
	MCEOADRC mceo;
	AMCEOADRC amceo;
	ClassicalNADRC nadrc;
	assert(ladrc.setParameters({10.1746f, -0.5f, 6.f, 3.f, 5.f}));
	assert(mceo.setParameters({10.1746f, -0.5f, 6.f, 3.f, 5.f}));
	assert(amceo.setParameters({{10.1746f, -0.5f, 6.f, 3.f, 18.f},
		18.f, 46.f, 0.16f, 0.7f, 0.01f, 0.055f, 0.0523598776f, 0.698131701f}));
	assert(nadrc.setParameters({10.1746f, 6.f, 3.f, 20.f, 20.f, 2.f,
		0.5f, 0.25f, 0.01f, 0.8f, 1.2f, 0.01f}));
	runGapSequence(ladrc, false);
	runGapSequence(mceo, false);
	runGapSequence(amceo, false);
	runGapSequence(nadrc, true);
	std::puts("ManualLatchNoTimeoutTest: PASS (IDs 2-5 gap sequences, age diagnostics, explicit-invalid/mode protection)");
}
