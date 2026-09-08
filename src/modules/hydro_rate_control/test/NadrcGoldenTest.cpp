/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "../AMCEO_ADRC.hpp"
#include "../ClassicalNADRC.hpp"
#include "../HydroRollController.hpp"
#include "../LADRC.hpp"
#include "../MCEO_ADRC.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

namespace
{
bool nearlyEqual(float actual, float expected, float tolerance = 1.2e-4f)
{
	return std::fabs(actual - expected) <= tolerance * fmaxf(1.f, std::fabs(expected));
}

ClassicalNADRCConfig defaultConfig()
{
	return {10.1746f, 26.f, 9.8f, 20.f, 20.f, 2.f, 0.5f, 0.25f, 0.01f, 0.8f, 1.2f, 0.01f};
}

HydroRollControllerInput inputTemplate()
{
	HydroRollControllerInput input{};
	input.dt_measured = 0.004f;
	input.dt_used = 0.004f;
	input.timestamp_sample = 1000;
	input.u_max = 0.4f;
	return input;
}
}

int main()
{
	const ClassicalNADRCConfig config = defaultConfig();
	ClassicalNADRC controller;
	assert(controller.setParameters(config));
	HydroRollControllerInput input = inputTemplate();
	input.phi = 0.12f;
	input.p = -0.07f;
	input.command_roll = 0.2f;
	controller.reset(input);
	assert(controller.isValid());
	assert(nearlyEqual(controller.v1(), input.command_roll));
	assert(nearlyEqual(controller.v2(), 0.f));
	assert(nearlyEqual(controller.z1(), input.phi));
	assert(nearlyEqual(controller.z2(), input.p));
	assert(nearlyEqual(controller.z3(), 0.f));
	assert(nearlyEqual(controller.e1(), 0.f));
	assert(nearlyEqual(controller.e2(), 0.f));
	assert(std::isnan(controller.tdAcceleration()));
	assert(std::isnan(controller.u0()));

	// Nonfinite command falls back to measured phi at reset, as specified.
	input.command_roll = NAN;
	controller.reset(input);
	assert(controller.isValid() && nearlyEqual(controller.v1(), input.phi));

	// Exact zero equilibrium remains finite with zero requested command.
	ClassicalNADRC equilibrium;
	assert(equilibrium.setParameters(config));
	input = inputTemplate();
	equilibrium.reset(input);

	for (int i = 0; i < 100; ++i) {
		const HydroRollControllerOutput output = equilibrium.update(input);
		assert(output.valid && nearlyEqual(output.u_requested, 0.f));
		assert(std::isfinite(equilibrium.v1()) && std::isfinite(equilibrium.v2()));
		assert(std::isfinite(equilibrium.z1()) && std::isfinite(equilibrium.z2())
		       && std::isfinite(equilibrium.z3()));
	}

	// Independent saved float32 vector exercises TD, both fal blocks, NESO,
	// previous-applied-command feedback, and disturbance compensation.
	struct Golden {
		float v1;
		float v2;
		float td_acceleration;
		float z1;
		float z2;
		float z3;
		float e1;
		float e2;
		float u0;
		float u_requested;
	};
	static constexpr Golden golden[] {
		{0.f, 0.08000000566f, 20.f, 0.04944000021f, -0.1192559004f, -2.02385807f, -0.04944000021f, 0.1992559135f, -0.9312753677f, 0.1073833629f},
		{0.0003200000501f, 0.1600000113f, 20.f, 0.05221737549f, 0.4293578863f, 8.89594841f, -0.05189737678f, -0.2693578601f, -4.468901634f, -1.313550472f},
		{0.0009600001504f, 0.2400000095f, 20.f, 0.05772263929f, 1.066739559f, 20.23808289f, -0.05676263943f, -0.8267395496f, -10.41906166f, -3.013105631f},
		{0.001920000184f, 0.3200000226f, 20.f, 0.06805616617f, 1.910633445f, 32.99756241f, -0.06613616645f, -1.590633392f, -20.06471825f, -5.215171337f},
		{0.003200000385f, 0.4000000358f, 20.f, 0.08048522472f, 2.721306324f, 45.02303696f, -0.07728522271f, -2.321306229f, -30.27498817f, -7.400588512f},
		{0.004800000694f, 0.4800000489f, 20.f, 0.09677399695f, 3.623466253f, 57.41861725f, -0.09197399765f, -3.143466234f, -42.59011459f, -9.82925415f},
		{0.006720000878f, 0.560000062f, 20.f, 0.1139621064f, 4.364563465f, 67.83474731f, -0.1072421074f, -3.804563522f, -53.06484222f, -11.88249111f},
		{0.008960001171f, 0.6400000453f, 20.f, 0.1335894614f, 5.073587894f, 76.98045349f, -0.1246294603f, -4.433588028f, -63.43819427f, -13.80090046f},
	};

	input = inputTemplate();
	input.phi = 0.05f;
	input.p = -0.02f;
	input.command_roll = 0.f;
	controller.reset(input);

	for (unsigned i = 0; i < sizeof(golden) / sizeof(golden[0]); ++i) {
		input = inputTemplate();
		input.phi = 0.05f + 0.01f * static_cast<float>(i) + ((i % 2) ? 0.003f : -0.002f);
		input.command_roll = 0.12f;
		input.u_applied_prev = -0.08f + 0.025f * static_cast<float>(i);
		const HydroRollControllerOutput output = controller.update(input);
		assert(output.valid);
		assert(nearlyEqual(controller.v1(), golden[i].v1));
		assert(nearlyEqual(controller.v2(), golden[i].v2));
		assert(nearlyEqual(controller.tdAcceleration(), golden[i].td_acceleration));
		assert(nearlyEqual(controller.z1(), golden[i].z1));
		assert(nearlyEqual(controller.z2(), golden[i].z2));
		assert(nearlyEqual(controller.z3(), golden[i].z3));
		assert(nearlyEqual(controller.e1(), golden[i].e1));
		assert(nearlyEqual(controller.e2(), golden[i].e2));
		assert(nearlyEqual(controller.u0(), golden[i].u0));
		assert(nearlyEqual(output.u_requested, golden[i].u_requested));
		assert(nearlyEqual(output.u_requested, (controller.u0() - controller.z3()) / config.b0));
	}

	// Release-blocking applied-command check: zero innovation isolates b0*u[k-1].
	input = inputTemplate();
	input.phi = 0.1f;
	input.p = 0.2f;
	input.command_roll = 0.1f;
	input.u_applied_prev = 0.3f;
	controller.reset(input);
	const HydroRollControllerOutput applied_feedback = controller.update(input);
	assert(applied_feedback.valid);
	assert(nearlyEqual(controller.z2(), 0.2f + input.dt_used * config.b0 * input.u_applied_prev));
	assert(!nearlyEqual(applied_feedback.u_requested, input.u_applied_prev));

	// Positive z3 is subtracted from u0 with exactly the specified sign.
	input = inputTemplate();
	controller.reset(input);
	input.phi = 0.1f;
	const HydroRollControllerOutput disturbed = controller.update(input);
	assert(disturbed.valid && controller.z3() > 0.f);
	assert(nearlyEqual(disturbed.u_requested, (controller.u0() - controller.z3()) / config.b0));
	assert(disturbed.u_requested < controller.u0() / config.b0);

	// No controller-internal limiting: a large latched step eventually produces
	// an over-range request, which the external common limiter contains.
	input = inputTemplate();
	controller.reset(input);
	input.command_roll = 100.f;
	HydroRollControllerOutput large_request{};

	for (int i = 0; i < 50 && std::fabs(large_request.u_requested) <= 1.f; ++i) {
		large_request = controller.update(input);
		assert(large_request.valid);
	}

	assert(std::fabs(large_request.u_requested) > 1.f);
	HydroRollLimiter limiter;
	limiter.reset(0.f);
	const HydroRollLimiterResult limited = limiter.update(large_request.u_requested, 0.f,
			input.dt_used, input.u_max, 2.f);
	assert(limited.valid && std::fabs(limited.u_applied) <= input.u_max + 1e-6f);
	assert(std::fabs(limited.u_applied) <= 2.f * input.dt_used + 1e-6f);

	// Shared timing guard accepts 9.9/10 ms and freezes the selected controller
	// at 10.1/40 ms or nonfinite dt. No TD/NESO propagation is attempted.
	for (const float accepted_dt : {0.0099f, 0.010f}) {
		input = inputTemplate();
		input.dt_used = accepted_dt;
		input.dt_measured = accepted_dt;
		controller.reset(input);
		const uint32_t updates_before = controller.updateCounter();
		assert(HydroRollTimingGuard::isValid(input.dt_used, 0.010f));
		assert(controller.update(input).valid);
		assert(controller.updateCounter() == updates_before + 1);
	}

	for (const float rejected_dt : {0.0101f, 0.040f, NAN}) {
		input = inputTemplate();
		controller.reset(input);
		const uint32_t updates_before = controller.updateCounter();
		const float v1_before = controller.v1();
		const float z1_before = controller.z1();
		input.dt_used = rejected_dt;

		if (HydroRollTimingGuard::isValid(input.dt_used, 0.010f)) {
			(void)controller.update(input);
		}

		assert(controller.updateCounter() == updates_before);
		assert(nearlyEqual(controller.v1(), v1_before));
		assert(nearlyEqual(controller.z1(), z1_before));
	}

	// Invalid inputs/configuration never return a plausible normalized request.
	for (int fault = 0; fault < 6; ++fault) {
		ClassicalNADRC invalid;
		assert(invalid.setParameters(config));
		input = inputTemplate();
		invalid.reset(input);
		if (fault == 0) { input.phi = NAN; }
		if (fault == 1) { input.p = INFINITY; }
		if (fault == 2) { input.command_roll = NAN; }
		if (fault == 3) { input.dt_measured = NAN; }
		if (fault == 4) { input.dt_used = INFINITY; }
		if (fault == 5) { input.u_applied_prev = NAN; }
		const HydroRollControllerOutput output = invalid.update(input);
		assert(!output.valid && std::isnan(output.u_requested));
	}

	for (int fault = 0; fault < 12; ++fault) {
		ClassicalNADRCConfig bad = config;
		if (fault == 0) { bad.b0 = 0.f; }
		if (fault == 1) { bad.k_phi1 = NAN; }
		if (fault == 2) { bad.k_phi2 = 0.f; }
		if (fault == 3) { bad.observer_bandwidth = INFINITY; }
		if (fault == 4) { bad.td_r = 0.f; }
		if (fault == 5) { bad.td_n0 = 0.9f; }
		if (fault == 6) { bad.observer_alpha1 = 0.f; }
		if (fault == 7) { bad.observer_alpha2 = NAN; }
		if (fault == 8) { bad.observer_delta = 0.f; }
		if (fault == 9) { bad.controller_alpha1 = -1.f; }
		if (fault == 10) { bad.controller_alpha2 = INFINITY; }
		if (fault == 11) { bad.controller_delta = 0.f; }
		ClassicalNADRC invalid;
		assert(!invalid.setParameters(bad));
	}

	// Inactive ID 2-4 state histories do not update while the NADRC object runs.
	LADRC ladrc;
	MCEOADRC mceo;
	AMCEOADRC amceo;
	assert(ladrc.setParameters({10.1746f, -0.5f, 26.f, 9.8f, 25.f}));
	assert(mceo.setParameters({10.1746f, -0.5f, 26.f, 9.8f, 22.f}));
	assert(amceo.setParameters({{10.1746f, -0.5f, 26.f, 9.8f, 18.f},
		18.f, 46.f, 0.16f, 0.70f, 0.01f, 0.055f, 0.0523598776f, 0.698131701f}));
	input = inputTemplate();
	ladrc.reset(input);
	mceo.reset(input);
	amceo.reset(input);
	controller.reset(input);
	assert(controller.update(input).valid); // ID 5 controller is active and valid.
	assert(ladrc.updateCounter() == 0 && mceo.updateCounter() == 0 && amceo.updateCounter() == 0);

	std::puts("NadrcGoldenTest: PASS");
	return 0;
}
