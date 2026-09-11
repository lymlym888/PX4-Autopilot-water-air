/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "../AMCEO_ADRC.hpp"
#include "../HydroRollController.hpp"
#include "../LADRC.hpp"
#include "../MCEO_ADRC.hpp"
#include "../RollReferenceGenerator.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace
{
bool nearlyEqual(float lhs, float rhs, float tolerance = 1e-6f)
{
	return std::fabs(lhs - rhs) <= tolerance;
}
}

int main()
{
	// A valid sample updates the latch in radians. Publication gaps retain it.
	ManualRollCommandLatch latch;
	latch.reset(0.05f);
	latch.update(true, true, true, 0.2f, 1, 900000, 1000000, 0.5f, 0.05f);
	assert(latch.valid());
	assert(nearlyEqual(latch.commandRoll(), 0.1f));
	assert(nearlyEqual(latch.ageSeconds(), 0.1f));
	latch.update(true, false, false, 0.f, 0, 0, 1200000, 0.5f, 0.06f);
	assert(latch.valid());
	assert(nearlyEqual(latch.commandRoll(), 0.1f));

	// User-requested removal of local age expiry: hold across the old timeout.
	latch.update(true, false, false, 0.f, 0, 0, 1500001, 0.5f, 0.07f);
	assert(latch.valid());
	assert(nearlyEqual(latch.commandRoll(), 0.1f));
	assert(latch.ageSeconds() > 0.5f);

	// An explicit invalid sample also invalidates immediately.
	latch.update(true, true, false, 0.8f, 1, 1600000, 1600000, 0.5f, -0.02f);
	assert(!latch.valid());
	assert(nearlyEqual(latch.commandRoll(), -0.02f));

	RollReferenceGenerator reference;
	reference.reset(0.3f, -0.1f);
	assert(reference.valid());
	assert(nearlyEqual(reference.phiRef(), -0.1f));
	assert(nearlyEqual(reference.phiRefDot(), 0.f));
	assert(reference.update(0.3f, 2.f, 1.f, 0.01f));
	assert(std::isfinite(reference.phiRef()));
	assert(std::isfinite(reference.phiRefDot()));
	assert(std::isfinite(reference.phiRefDdot()));

	// Invalid reference input is rejected without producing a valid state.
	assert(!reference.update(NAN, 2.f, 1.f, 0.01f));
	assert(!reference.valid());

	HydroRollLimiter limiter;
	limiter.reset(0.f);
	float previous = 0.f;

	for (int i = 0; i < 100; ++i) {
		const HydroRollLimiterResult result = limiter.update(1.f, 0.f, 0.01f, 0.4f, 2.f);
		assert(result.valid);
		assert(std::fabs(result.u_applied) <= 0.400001f);
		assert(std::fabs(result.u_applied - previous) <= 0.020001f);
		previous = result.u_applied;
	}

	assert(nearlyEqual(previous, 0.4f));
	const HydroRollLimiterResult invalid_limiter = limiter.update(NAN, 0.f, 0.01f, 0.4f, 2.f);
	assert(!invalid_limiter.valid);
	assert(nearlyEqual(limiter.appliedPrevious(), 0.f));

	// Controller histories are independent; only the selected object is updated.
	HydroRollControllerInput input{};
	input.phi = 0.1f;
	input.p = -0.2f;
	input.command_roll = 0.3f;
	input.phi_ref = 0.1f;
	input.phi_ref_dot = 0.f;
	input.phi_ref_ddot = 0.f;
	input.dt_measured = 0.0025f;
	input.dt_used = 0.0025f;
	input.u_applied_prev = 0.f;
	input.timestamp_sample = 123456;

	LADRC ladrc;
	MCEOADRC mceo;
	AMCEOADRC amceo;
	const HydroRollControllerConfig ladrc_config{10.1746f, -0.5f, 26.f, 9.8f, 25.f};
	const HydroRollControllerConfig mceo_config{10.1746f, -0.5f, 26.f, 9.8f, 22.f};
	assert(ladrc.setParameters(ladrc_config));
	assert(mceo.setParameters(mceo_config));
	ladrc.reset(input);
	mceo.reset(input);
	amceo.reset(input);
	const HydroRollControllerOutput ladrc_output = ladrc.update(input);
	assert(ladrc_output.valid);
	assert(std::isfinite(ladrc_output.u_requested));
	assert(ladrc_output.reset_event);
	assert(ladrc.updateCounter() == 1);
	assert(mceo.updateCounter() == 0);
	assert(amceo.updateCounter() == 0);
	assert(ladrc.isValid() && mceo.isValid() && !amceo.isValid());
	// An unconfigured controller remains safely invalid; Stage 4 AMCEO behavior
	// is exercised with a valid atomic configuration in AmceoGoldenTest.
	const HydroRollControllerOutput unconfigured = amceo.update(input);
	assert(!unconfigured.valid);
	assert(std::isnan(unconfigured.u_requested));

	std::puts("HydroRollInfrastructureTest: PASS");
	return 0;
}
