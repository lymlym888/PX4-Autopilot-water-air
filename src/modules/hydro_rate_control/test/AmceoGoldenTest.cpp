/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "../AMCEO_ADRC.hpp"
#include "../HydroRollController.hpp"
#include "../MCEO_ADRC.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace
{
bool nearlyEqual(float actual, float expected, float tolerance = 8e-5f)
{
	return std::fabs(actual - expected) <= tolerance * fmaxf(1.f, std::fabs(expected));
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

AMCEOControllerConfig defaultConfig()
{
	return {
		{10.1746f, -0.5f, 26.f, 9.8f, 18.f},
		18.f, 46.f, 0.16f, 0.70f, 0.01f, 0.055f,
		0.0523598776f, 0.698131701f
	};
}
}

int main()
{
	const AMCEOControllerConfig config = defaultConfig();
	AMCEOADRC controller;
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
	assert(nearlyEqual(controller.rho(), config.rho0));
	assert(nearlyEqual(controller.observerBandwidth(),
			   config.omega_min + (config.omega_max - config.omega_min) * config.rho0));
	assert(std::isnan(controller.sigmaE()) && std::isnan(controller.sigmaU()));

	// Zero equilibrium remains finite and leaves the scheduler at rho0.
	AMCEOADRC equilibrium;
	assert(equilibrium.setParameters(config));
	input = inputTemplate();
	equilibrium.reset(input);

	for (int i = 0; i < 50; ++i) {
		const HydroRollControllerOutput output = equilibrium.update(input);
		assert(output.valid && nearlyEqual(output.u_requested, 0.f));
		assert(nearlyEqual(equilibrium.rho(), config.rho0));
		assert(equilibrium.rho() >= 0.f && equilibrium.rho() <= 1.f);
		assert(equilibrium.observerBandwidth() >= config.omega_min
		       && equilibrium.observerBandwidth() <= config.omega_max);
	}

	// Exact scheduler calculation uses PRE-STEP tracking errors and u[k-1].
	AMCEOADRC scheduler;
	assert(scheduler.setParameters(config));
	input = inputTemplate();
	input.phi = 0.05f;
	input.p = -0.02f;
	input.phi_ref = 0.12f;
	input.phi_ref_dot = 0.015f;
	input.u_applied_prev = -0.08f;
	scheduler.reset(input);
	const float expected_sigma_e = fminf(1.f, std::fabs(input.phi - input.phi_ref) / config.e1_bar
				       + std::fabs(input.p - input.phi_ref_dot) / config.e2_bar);
	const float expected_sigma_u = std::fabs(input.u_applied_prev) / input.u_max;
	const float expected_rho_star = fminf(1.f, fmaxf(0.f,
					config.rho0 + config.ce * expected_sigma_e - config.cu * expected_sigma_u));
	const float expected_rho = fminf(1.f, fmaxf(0.f, config.rho0
				 + input.dt_used / config.tau_rho * (-config.rho0 + expected_rho_star)));
	assert(scheduler.update(input).valid);
	assert(nearlyEqual(scheduler.sigmaE(), expected_sigma_e));
	assert(nearlyEqual(scheduler.sigmaU(), expected_sigma_u));
	assert(nearlyEqual(scheduler.rhoStar(), expected_rho_star));
	assert(nearlyEqual(scheduler.rho(), expected_rho));

	// Saved independent float32 golden vector for the approved scheduling and
	// Stage 3 current-observer execution convention.
	struct Golden {
		float z1;
		float z2;
		float z3;
		float z4;
		float rho;
		float rho_star;
		float sigma_e;
		float sigma_u;
		float omega_o;
		float e1;
		float e2;
		float u;
	};
	static constexpr Golden golden[] {
		{0.04915515706f, -0.05063712597f, -0.4369373024f, -2.610851288f, 0.2107636333f, 0.8580000401f, 1.f, 0.1999999881f, 23.90138245f, -0.07084484398f, -0.06563712656f, 0.2817632556f},
		{0.05453942716f, 0.1568329036f, 3.106293201f, 19.79565239f, 0.2578808367f, 0.8586249948f, 1.f, 0.137500003f, 25.22066307f, -0.06546057016f, 0.141832903f, -0.2698744535f},
		{0.06086225808f, 0.3936519325f, 7.168635845f, 46.12958527f, 0.3016167879f, 0.8592500091f, 1.f, 0.07499999553f, 26.44527054f, -0.05913773924f, 0.3786519468f, -0.9017571807f},
		{0.07220654935f, 0.82553792f, 14.78564835f, 97.38050842f, 0.3422173858f, 0.8598750234f, 1.f, 0.01249999925f, 27.58208656f, -0.04779344797f, 0.8105379343f, -2.074137688f},
		{0.0827447474f, 1.194654584f, 21.10865211f, 139.8574982f, 0.3798379302f, 0.8594999909f, 1.f, 0.04999999702f, 28.63546181f, -0.03725524992f, 1.179654598f, -3.059904575f},
		{0.0971198082f, 1.704770923f, 30.08232307f, 202.1459351f, 0.4146769941f, 0.8588750362f, 1.f, 0.1125000045f, 29.61095619f, -0.02288018912f, 1.689770937f, -4.444873333f},
		{0.1092509031f, 2.067677498f, 35.83702087f, 239.877655f, 0.4469368458f, 0.8582500219f, 1.f, 0.174999997f, 30.51423264f, -0.01074909419f, 2.052677393f, -5.373178959f},
		{0.124418281f, 2.535078049f, 43.57496643f, 293.0045776f, 0.4768050909f, 0.8576250076f, 1.f, 0.237499997f, 31.35054207f, 0.004418283701f, 2.520077944f, -6.599676132f},
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
		assert(nearlyEqual(controller.rho(), golden[i].rho));
		assert(nearlyEqual(controller.rhoStar(), golden[i].rho_star));
		assert(nearlyEqual(controller.sigmaE(), golden[i].sigma_e));
		assert(nearlyEqual(controller.sigmaU(), golden[i].sigma_u));
		assert(nearlyEqual(controller.observerBandwidth(), golden[i].omega_o));
		assert(nearlyEqual(controller.e1(), golden[i].e1));
		assert(nearlyEqual(controller.e2(), golden[i].e2));
		assert(nearlyEqual(output.u_requested, golden[i].u));
	}

	// Scheduler bounds, saturation, tracking-error monotonicity and actuator penalty.
	AMCEOControllerConfig monotonic_config = config;
	monotonic_config.ce = 0.5f;
	monotonic_config.cu = 0.5f;
	AMCEOADRC low_error;
	AMCEOADRC high_error;
	AMCEOADRC high_actuator;
	assert(low_error.setParameters(monotonic_config));
	assert(high_error.setParameters(monotonic_config));
	assert(high_actuator.setParameters(monotonic_config));
	input = inputTemplate();
	low_error.reset(input);
	high_error.reset(input);
	high_actuator.reset(input);
	assert(low_error.update(input).valid);
	input.phi_ref = 10.f;
	assert(high_error.update(input).valid);
	input.phi_ref = 0.f;
	input.u_applied_prev = input.u_max;
	assert(high_actuator.update(input).valid);
	assert(high_error.rhoStar() >= low_error.rhoStar());
	assert(high_actuator.rhoStar() <= low_error.rhoStar());
	assert(high_error.sigmaE() >= 0.f && high_error.sigmaE() <= 1.f);
	assert(high_actuator.sigmaU() >= 0.f && high_actuator.sigmaU() <= 1.f);

	// Applied-command feedback is release-blocking: observer z2 uses u[k-1],
	// while the returned request is independently computed after propagation.
	input = inputTemplate();
	input.phi = 0.1f;
	input.p = 0.2f;
	input.phi_ref = 0.1f;
	input.u_applied_prev = 0.3f;
	controller.reset(input);
	const HydroRollControllerOutput applied_feedback = controller.update(input);
	const float expected_z2 = 0.2f + input.dt_used
				  * (config.common.a_phi * 0.2f + config.common.b0 * input.u_applied_prev);
	assert(nearlyEqual(controller.z2(), expected_z2));
	assert(!nearlyEqual(applied_feedback.u_requested, input.u_applied_prev));
	const float expected_v = input.phi_ref_ddot - config.common.k_phi1 * controller.e1()
			       - config.common.k_phi2 * controller.e2();
	const float expected_u = (expected_v - config.common.a_phi * controller.z2() - controller.z3())
			       / config.common.b0;
	assert(nearlyEqual(applied_feedback.u_requested, expected_u));

	// Release-blocking limiting case: with a constant scheduler bandwidth,
	// AMCEO and fixed MCEO must be the same four-state observer/controller.
	const float fixed_omega = 22.f;
	AMCEOControllerConfig fixed_config = config;
	fixed_config.rho0 = (fixed_omega - fixed_config.omega_min)
			    / (fixed_config.omega_max - fixed_config.omega_min);
	fixed_config.ce = 0.f;
	fixed_config.cu = 0.f;
	AMCEOADRC fixed_amceo;
	MCEOADRC fixed_mceo;
	const HydroRollControllerConfig mceo_config{config.common.b0, config.common.a_phi,
		config.common.k_phi1, config.common.k_phi2, fixed_omega};
	assert(fixed_amceo.setParameters(fixed_config));
	assert(fixed_mceo.setParameters(mceo_config));
	input = inputTemplate();
	input.phi = 0.03f;
	input.p = -0.04f;
	fixed_amceo.reset(input);
	fixed_mceo.reset(input);

	for (int i = 0; i < 40; ++i) {
		input.phi = 0.03f + 0.002f * static_cast<float>(i) + ((i % 3) ? 0.001f : -0.0015f);
		input.phi_ref = 0.08f - 0.0005f * static_cast<float>(i);
		input.phi_ref_dot = -0.006f;
		input.phi_ref_ddot = 0.002f;
		input.u_applied_prev = -0.12f + 0.005f * static_cast<float>(i);
		const HydroRollControllerOutput am_output = fixed_amceo.update(input);
		const HydroRollControllerOutput mc_output = fixed_mceo.update(input);
		assert(am_output.valid && mc_output.valid);
		assert(nearlyEqual(fixed_amceo.observerBandwidth(), fixed_omega));
		assert(nearlyEqual(fixed_amceo.z1(), fixed_mceo.z1()));
		assert(nearlyEqual(fixed_amceo.z2(), fixed_mceo.z2()));
		assert(nearlyEqual(fixed_amceo.z3(), fixed_mceo.z3()));
		assert(nearlyEqual(fixed_amceo.z4(), fixed_mceo.z4()));
		assert(nearlyEqual(am_output.u_requested, mc_output.u_requested));
	}

	// New-controller timing boundary is inclusive at 10 ms and rejects stalls.
	assert(HydroRollTimingGuard::isValid(0.0099f, 0.010f));
	assert(HydroRollTimingGuard::isValid(0.0100f, 0.010f));
	assert(!HydroRollTimingGuard::isValid(0.0101f, 0.010f));
	assert(!HydroRollTimingGuard::isValid(0.0400f, 0.010f));
	assert(!HydroRollTimingGuard::isValid(NAN, 0.010f));
	AMCEOADRC timing_guarded;
	assert(timing_guarded.setParameters(config));
	input = inputTemplate();
	timing_guarded.reset(input);
	const uint32_t updates_before_stall = timing_guarded.updateCounter();
	input.dt_used = 0.0400f;
	if (HydroRollTimingGuard::isValid(input.dt_used, 0.010f)) {
		(void)timing_guarded.update(input);
	}
	assert(timing_guarded.updateCounter() == updates_before_stall);

	// The controller never clamps u_requested; the external common limiter owns
	// both magnitude and slew constraints.
	controller.reset(inputTemplate());
	input = inputTemplate();
	input.phi_ref = 100.f;
	const HydroRollControllerOutput large_request = controller.update(input);
	assert(large_request.valid && std::fabs(large_request.u_requested) > 1.f);
	HydroRollLimiter limiter;
	limiter.reset(0.f);
	const HydroRollLimiterResult limited = limiter.update(large_request.u_requested, 0.f,
			input.dt_used, input.u_max, 2.f);
	assert(limited.valid && std::fabs(limited.u_applied) <= input.u_max + 1e-6f);
	assert(std::fabs(limited.u_applied) <= 2.f * input.dt_used + 1e-6f);

	// NaN/Inf inputs and invalid parameters never produce plausible output.
	for (int fault = 0; fault < 10; ++fault) {
		AMCEOADRC invalid;
		assert(invalid.setParameters(config));
		input = inputTemplate();
		invalid.reset(input);

		if (fault == 0) { input.phi = NAN; }
		if (fault == 1) { input.p = INFINITY; }
		if (fault == 2) { input.phi_ref = NAN; }
		if (fault == 3) { input.phi_ref_dot = INFINITY; }
		if (fault == 4) { input.dt_used = NAN; }
		if (fault == 5) { input.u_applied_prev = INFINITY; }
		if (fault == 6) { input.u_max = 0.f; }
		if (fault == 7) { input.command_roll = NAN; }
		if (fault == 8) { input.phi_ref_ddot = INFINITY; }
		if (fault == 9) { input.dt_measured = NAN; }

		const HydroRollControllerOutput output = invalid.update(input);
		assert(!output.valid && std::isnan(output.u_requested));
	}

	for (int fault = 0; fault < 9; ++fault) {
		AMCEOControllerConfig bad = config;
		if (fault == 0) { bad.common.b0 = NAN; }
		if (fault == 1) { bad.omega_min = 0.f; }
		if (fault == 2) { bad.omega_max = bad.omega_min; }
		if (fault == 3) { bad.rho0 = 1.1f; }
		if (fault == 4) { bad.ce = -0.1f; }
		if (fault == 5) { bad.cu = NAN; }
		if (fault == 6) { bad.tau_rho = 0.f; }
		if (fault == 7) { bad.e1_bar = -1.f; }
		if (fault == 8) { bad.e2_bar = INFINITY; }
		AMCEOADRC invalid;
		assert(!invalid.setParameters(bad));
		input = inputTemplate();
		invalid.reset(input);
		assert(!invalid.update(input).valid);
	}

	std::puts("AmceoGoldenTest: PASS");
	return 0;
}
