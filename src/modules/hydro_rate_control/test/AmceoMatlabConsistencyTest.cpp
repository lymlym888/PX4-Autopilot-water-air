/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "../AMCEO_ADRC.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace
{
bool nearlyEqual(float actual, float expected, float tolerance = 8e-5f)
{
	return std::fabs(actual - expected) <= tolerance * fmaxf(1.f, std::fabs(expected));
}
}

int main()
{
	// The common model values, 1 ms step and 0.34 actuator limit below are used
	// only to audit one deterministic calculation from the supplied MATLAB file.
	// They are not firmware defaults; flight code uses the Stage 3 parameter
	// snapshot, measured PX4 dt and HY_RLL_U_MAX.
	const AMCEOControllerConfig matlab_case {
		{9.4f, -1.05f, 26.f, 9.8f, 18.f},
		18.f, 46.f, 0.16f, 0.70f, 0.01f, 0.055f,
		0.0523598776f, 0.698131701f
	};

	AMCEOADRC controller;
	assert(controller.setParameters(matlab_case));
	HydroRollControllerInput input{};
	input.phi = 0.04f;
	input.p = -0.07f;
	input.phi_ref = 0.02f;
	input.phi_ref_dot = 0.01f;
	input.phi_ref_ddot = -0.03f;
	input.dt_measured = 0.001f;
	input.dt_used = 0.001f;
	input.u_applied_prev = 0.11f;
	input.u_max = 0.34f;
	input.timestamp_sample = 1000;
	controller.reset(input);

	// Current measurement differs from the reset state, making every observer
	// derivative nonzero. Expected constants were evaluated independently from
	// the MATLAB continuous equations and approved scheduler equations.
	input.phi = 0.055f;
	const HydroRollControllerOutput output = controller.update(input);
	assert(output.valid);
	assert(nearlyEqual(controller.sigmaE(), 0.4965634048f));
	assert(nearlyEqual(controller.sigmaU(), 0.3235293925f));
	assert(nearlyEqual(controller.rhoStar(), 0.5043590665f));
	assert(nearlyEqual(controller.rho(), 0.1662610769f));
	assert(nearlyEqual(controller.observerBandwidth(), 22.65530968f));

	// The inferred derivatives match MATLAB's continuous observer definition:
	// [z2+l1*eo, a*z2+b0*u+z3+l2*eo, z4+l3*eo, l4*eo].
	assert(nearlyEqual((controller.z1() - 0.04f) / input.dt_used, 1.289318562f));
	assert(nearlyEqual((controller.z2() + 0.07f) / input.dt_used, 47.30117416f));
	// Continuous MATLAB z3_dot is unchanged, but the user-authorized state
	// cap now activates here: 697.688... * 0.001 > 0.25. Do not claim the
	// clipped state difference is the unconstrained continuous derivative.
	assert(nearlyEqual(controller.z3(), 0.25f));
	assert(nearlyEqual(controller.z4() / input.dt_used, 3951.584717f));

	// Firmware intentionally computes control from the updated observer state,
	// the current-observer convention inherited from Stage 3.
	const float expected_unbounded_z3 = 697.6880493f * input.dt_used;
	const float expected_bounded_u = -0.1047442257f + (expected_unbounded_z3 - 0.25f) / 9.4f;
	assert(nearlyEqual(output.u_requested, expected_bounded_u));

	std::puts("AmceoMatlabConsistencyTest: PASS");
	return 0;
}
