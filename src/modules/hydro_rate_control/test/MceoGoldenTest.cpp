/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "../HydroRollController.hpp"
#include "../LADRC.hpp"
#include "../MCEO_ADRC.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace
{
bool nearlyEqual(float actual, float expected, float tolerance = 4e-5f)
{
	return std::fabs(actual - expected) <= tolerance * fmaxf(1.f, std::fabs(expected));
}

HydroRollControllerInput inputTemplate()
{
	return {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.004f, 0.004f, 0.f, 1000, 0.4f};
}
}

int main()
{
	const HydroRollControllerConfig config{10.1746f, -0.5f, 26.f, 9.8f, 22.f};
	MCEOADRC controller;
	assert(controller.setParameters(config));
	HydroRollControllerInput input = inputTemplate();
	input.phi = -0.08f;
	input.p = 0.11f;
	controller.reset(input);
	assert(controller.isValid());
	assert(nearlyEqual(controller.z1(), input.phi));
	assert(nearlyEqual(controller.z2(), input.p));
	assert(nearlyEqual(controller.z3(), 0.f));
	assert(nearlyEqual(controller.z4(), 0.f));

	MCEOADRC equilibrium;
	assert(equilibrium.setParameters(config));
	input = inputTemplate();
	equilibrium.reset(input);

	for (int i = 0; i < 50; ++i) {
		const HydroRollControllerOutput output = equilibrium.update(input);
		assert(output.valid && nearlyEqual(output.u_requested, 0.f));
		assert(std::isfinite(equilibrium.z1()) && std::isfinite(equilibrium.z2())
		       && std::isfinite(equilibrium.z3()) && std::isfinite(equilibrium.z4()));
	}

	struct Golden {
		float z1;
		float z2;
		float z3;
		float z4;
		float e1;
		float e2;
		float u;
	};
	static constexpr Golden golden[] {
		{0.0492160022f, -0.0464478768f, -0.340736061f, -1.87404847f, -0.0707839951f, -0.0614478774f, 0.26832363f},
		{0.0538821779f, 0.110158607f, 2.00012016f, 11.0418901f, -0.0661178231f, 0.0951586068f, -0.116813622f},
		{0.0592922866f, 0.280710489f, 4.44951344f, 24.2706318f, -0.0607077107f, 0.265710503f, -0.527265966f},
		{0.068760246f, 0.573132396f, 8.58563137f, 46.4853287f, -0.0512397513f, 0.55813241f, -1.22526002f},
		{0.0778251663f, 0.830631614f, 12.0494118f, 64.513443f, -0.042174831f, 0.815631628f, -1.82422328f},
		{0.0900092348f, 1.17143035f, 16.5964527f, 88.1028671f, -0.0299907625f, 1.15643036f, -2.61376309f},
		{0.101027705f, 1.44730294f, 20.013916f, 104.960648f, -0.0189722925f, 1.43230295f, -3.22996068f},
		{0.114551164f, 1.78356051f, 24.1771355f, 125.549217f, -0.00544883311f, 1.76856053f, -3.98104882f},
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
		assert(nearlyEqual(controller.z4(), golden[i].z4));
		assert(nearlyEqual(controller.e1(), golden[i].e1));
		assert(nearlyEqual(controller.e2(), golden[i].e2));
		assert(nearlyEqual(output.u_requested, golden[i].u));
	}

	// Release-blocking applied-command and structural-difference check.
	input = inputTemplate();
	input.phi = 0.1f;
	input.p = 0.2f;
	input.u_applied_prev = 0.3f;
	controller.reset(input);
	const HydroRollControllerOutput applied_feedback = controller.update(input);
	const float expected_mceo_z2 = 0.2f + input.dt_used * (config.a_phi * 0.2f + config.b0 * input.u_applied_prev);
	assert(nearlyEqual(controller.z2(), expected_mceo_z2));
	assert(!nearlyEqual(applied_feedback.u_requested, input.u_applied_prev));

	LADRC ladrc;
	HydroRollControllerConfig ladrc_config = config;
	ladrc_config.observer_bandwidth = 22.f;
	assert(ladrc.setParameters(ladrc_config));
	ladrc.reset(input);
	assert(ladrc.update(input).valid);
	const float expected_ladrc_z2 = 0.2f + input.dt_used * config.b0 * input.u_applied_prev;
	assert(nearlyEqual(ladrc.z2(), expected_ladrc_z2));
	assert(nearlyEqual(controller.z2() - ladrc.z2(), input.dt_used * config.a_phi * 0.2f));

	// Nonzero innovation drives MCEO's distinct fourth observer state.
	input.phi = 0.3f;
	assert(controller.update(input).valid);
	assert(std::fabs(controller.z4()) > 1e-6f);

	input.phi_ref = 100.f;
	const HydroRollControllerOutput large_request = controller.update(input);
	assert(large_request.valid && std::fabs(large_request.u_requested) > 1.f);
	HydroRollLimiter limiter;
	limiter.reset(0.f);
	const HydroRollLimiterResult limited = limiter.update(large_request.u_requested, 0.f, input.dt_used, 0.4f, 2.f);
	assert(limited.valid && std::fabs(limited.u_applied) <= 0.400001f);
	assert(std::fabs(limited.u_applied) <= 2.f * input.dt_used + 1e-6f);

	for (int fault = 0; fault < 4; ++fault) {
		MCEOADRC invalid;
		assert(invalid.setParameters(config));
		input = inputTemplate();
		invalid.reset(input);

		if (fault == 0) { input.phi = NAN; }
		if (fault == 1) { input.p = INFINITY; }
		if (fault == 2) { input.phi_ref_dot = NAN; }
		if (fault == 3) { input.dt_used = INFINITY; }

		const HydroRollControllerOutput output = invalid.update(input);
		assert(!output.valid && std::isnan(output.u_requested));
	}

	MCEOADRC invalid_b0;
	HydroRollControllerConfig bad_config = config;
	bad_config.b0 = 0.f;
	assert(!invalid_b0.setParameters(bad_config));
	input = inputTemplate();
	invalid_b0.reset(input);
	assert(!invalid_b0.update(input).valid);

	std::puts("MceoGoldenTest: PASS");
	return 0;
}
