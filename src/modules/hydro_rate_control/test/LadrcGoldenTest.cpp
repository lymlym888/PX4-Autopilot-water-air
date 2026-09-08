/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "../HydroRollController.hpp"
#include "../LADRC.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace
{
bool nearlyEqual(float actual, float expected, float tolerance = 3e-5f)
{
	return std::fabs(actual - expected) <= tolerance * fmaxf(1.f, std::fabs(expected));
}

HydroRollControllerInput inputTemplate()
{
	HydroRollControllerInput input{};
	input.phi = 0.f;
	input.p = 0.f;
	input.command_roll = 0.f;
	input.phi_ref = 0.f;
	input.phi_ref_dot = 0.f;
	input.phi_ref_ddot = 0.f;
	input.dt_measured = 0.004f;
	input.dt_used = 0.004f;
	input.u_applied_prev = 0.f;
	input.timestamp_sample = 1000;
	return input;
}
}

int main()
{
	const HydroRollControllerConfig config{10.1746f, -0.5f, 26.f, 9.8f, 25.f};
	LADRC controller;
	assert(controller.setParameters(config));

	// Reset uses measured p only for initialization; normal feedback uses z2.
	HydroRollControllerInput input = inputTemplate();
	input.phi = 0.12f;
	input.p = -0.07f;
	controller.reset(input);
	assert(controller.isValid());
	assert(nearlyEqual(controller.z1(), input.phi));
	assert(nearlyEqual(controller.z2(), input.p));
	assert(nearlyEqual(controller.z3(), 0.f));

	// Zero equilibrium must remain finite and produce zero request.
	LADRC equilibrium;
	assert(equilibrium.setParameters(config));
	input = inputTemplate();
	equilibrium.reset(input);

	for (int i = 0; i < 50; ++i) {
		const HydroRollControllerOutput output = equilibrium.update(input);
		assert(output.valid);
		assert(std::isfinite(equilibrium.z1()) && std::isfinite(equilibrium.z2()) && std::isfinite(equilibrium.z3()));
		assert(nearlyEqual(output.u_requested, 0.f));
	}

	// Saved float32 golden vector for the specified fixed reference/measurement sequence.
	struct Golden {
		float z1;
		float z2;
		float z3;
		float e1;
		float e2;
		float u;
	};
	static constexpr Golden golden[] {
		{0.0493200012f, -0.0382558778f, -0.12500003f, -0.0706799924f, -0.0532558784f, 0.241246596f},
		{0.053270977f, 0.0616057143f, 0.730000019f, -0.0667290241f, 0.0466057137f, 0.0509325825f},
		{0.0579361096f, 0.173772469f, 1.65056419f, -0.0620638877f, 0.158772469f, -0.159502417f},
		{0.0661503673f, 0.368150413f, 3.21705723f, -0.0538496301f, 0.353150427f, -0.521675706f},
		{0.0741778612f, 0.545704842f, 4.58265924f, -0.0458221361f, 0.530704856f, -0.847423136f},
		{0.0850073248f, 0.782032967f, 6.38404274f, -0.0349926725f, 0.767032981f, -1.27977085f},
		{0.0950332582f, 0.982863128f, 7.82108498f, -0.0249667391f, 0.967863142f, -1.64006531f},
		{0.107354738f, 1.22776437f, 9.56900692f, -0.0126452595f, 1.21276438f, -2.07922888f},
	};

	controller.reset(HydroRollControllerInput{0.05f, -0.02f, 0.12f, 0.12f, 0.015f, -0.03f,
			0.004f, 0.004f, 0.f, 2000, 0.4f});

	for (unsigned i = 0; i < sizeof(golden) / sizeof(golden[0]); ++i) {
		input = inputTemplate();
		input.phi = 0.05f + 0.01f * static_cast<float>(i) + ((i % 2) ? 0.003f : -0.002f);
		input.command_roll = 0.12f;
		input.phi_ref = 0.12f;
		input.phi_ref_dot = 0.015f;
		input.phi_ref_ddot = -0.03f;
		input.u_applied_prev = -0.08f + 0.025f * static_cast<float>(i);
		const HydroRollControllerOutput output = controller.update(input);
		assert(output.valid);
		assert(nearlyEqual(controller.z1(), golden[i].z1));
		assert(nearlyEqual(controller.z2(), golden[i].z2));
		assert(nearlyEqual(controller.z3(), golden[i].z3));
		assert(nearlyEqual(controller.e1(), golden[i].e1));
		assert(nearlyEqual(controller.e2(), golden[i].e2));
		assert(nearlyEqual(output.u_requested, golden[i].u));
	}

	// Release-blocking applied-command test: z2 propagation uses the supplied
	// previous limited command, while the newly computed request is different.
	input = inputTemplate();
	input.phi = 0.1f;
	input.p = 0.2f;
	input.u_applied_prev = 0.3f;
	controller.reset(input);
	const HydroRollControllerOutput applied_feedback = controller.update(input);
	const float expected_z2 = 0.2f + input.dt_used * config.b0 * input.u_applied_prev;
	assert(nearlyEqual(controller.z2(), expected_z2));
	assert(!nearlyEqual(applied_feedback.u_requested, input.u_applied_prev));

	// Controller output is intentionally not internally clamped; only the
	// Stage 2B common limiter enforces actuator-path magnitude and slew bounds.
	input.phi_ref = 100.f;
	const HydroRollControllerOutput large_request = controller.update(input);
	assert(large_request.valid && std::fabs(large_request.u_requested) > 1.f);
	HydroRollLimiter limiter;
	limiter.reset(0.f);
	const HydroRollLimiterResult limited = limiter.update(large_request.u_requested, 0.f, input.dt_used, 0.4f, 2.f);
	assert(limited.valid && std::fabs(limited.u_applied) <= 0.400001f);
	assert(std::fabs(limited.u_applied) <= 2.f * input.dt_used + 1e-6f);

	// Nonfinite measurement/reference/dt and invalid b0 must never look valid.
	for (int fault = 0; fault < 4; ++fault) {
		LADRC invalid;
		assert(invalid.setParameters(config));
		input = inputTemplate();
		invalid.reset(input);

		if (fault == 0) { input.phi = NAN; }
		if (fault == 1) { input.p = INFINITY; }
		if (fault == 2) { input.phi_ref = NAN; }
		if (fault == 3) { input.dt_used = INFINITY; }

		const HydroRollControllerOutput output = invalid.update(input);
		assert(!output.valid && std::isnan(output.u_requested));
	}

	LADRC invalid_b0;
	HydroRollControllerConfig bad_config = config;
	bad_config.b0 = NAN;
	assert(!invalid_b0.setParameters(bad_config));
	input = inputTemplate();
	invalid_b0.reset(input);
	assert(!invalid_b0.update(input).valid);

	std::puts("LadrcGoldenTest: PASS");
	return 0;
}
