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
	// Independent oracle: GenerateBoundedGoldenVectors.py; z3 clipped to +/-0.25
	// after simultaneous Euler and before feedback. Original vectors are in git history.
	static constexpr Golden golden[] {
		{0.04932000000f, -0.03825587200f, -0.1250000000f, -0.07068000000f, -0.05325587200f, 0.2412465891f},
		{0.05327097651f, 0.06160571600f, 0.2500000000f, -0.06672902349f, 0.04660571600f, 0.09810887837f},
		{0.05793610642f, 0.1718524402f, 0.2500000000f, -0.06206389358f, 0.1568524402f, -0.02000006689f},
		{0.06614268426f, 0.3606281500f, 0.2500000000f, -0.05385731574f, 0.3456281500f, -0.2227965385f},
		{0.07414239158f, 0.5263719861f, 0.2500000000f, -0.04585760842f, 0.5113719861f, -0.4028804714f},
		{0.08490516205f, 0.7456354772f, 0.2500000000f, -0.03509483795f, 0.7306354772f, -0.6415743017f},
		{0.09481615534f, 0.9226956499f, 0.2500000000f, -0.02518384466f, 0.9076956499f, -0.8374420034f},
		{0.1069620913f, 1.138940833f, 0.2500000000f, -0.01303790866f, 1.123940833f, -1.076763169f},
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
