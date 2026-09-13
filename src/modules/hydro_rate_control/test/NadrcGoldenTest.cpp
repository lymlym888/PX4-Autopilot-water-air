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
	// Independent oracle: GenerateBoundedGoldenVectors.py; z3 clipped to +/-0.25
	// after simultaneous Euler and before feedback. Original vectors are in git history.
	static constexpr Golden golden[] {
		{0.000000000f, 0.08000000000f, 20.00000000f, 0.04944000000f, -0.1192558720f, -0.2500000000f, -0.04944000000f, 0.1992558720f, -0.9312756786f, -0.06695847292f},
		{0.0003200000000f, 0.1600000000f, 20.00000000f, 0.05221737651f, 0.4364532985f, 0.2500000000f, -0.05189737651f, -0.2764532985f, -4.533257200f, -0.4701174690f},
		{0.0009600000000f, 0.2400000000f, 20.00000000f, 0.05775101934f, 1.039251123f, 0.2500000000f, -0.05679101934f, -0.7992511233f, -10.10995992f, -1.018217907f},
		{0.001920000000f, 0.3200000000f, 20.00000000f, 0.06796777919f, 1.802764167f, 0.2500000000f, -0.06604777919f, -1.482764167f, -18.67923168f, -1.860439888f},
		{0.003200000000f, 0.4000000000f, 20.00000000f, 0.07998656886f, 2.483947230f, 0.2500000000f, -0.07678656886f, -2.083947230f, -26.98902461f, -2.677159260f},
		{0.004800000000f, 0.4800000000f, 20.00000000f, 0.09544558125f, 3.214947220f, 0.2500000000f, -0.09064558125f, -2.734947220f, -36.58601706f, -3.620389702f},
		{0.006720000000f, 0.5600000000f, 20.00000000f, 0.1113184306f, 3.756619320f, 0.2500000000f, -0.1045984306f, -3.196619320f, -43.79510884f, -4.328927804f},
		{0.008960000000f, 0.6400000000f, 20.00000000f, 0.1291484846f, 4.280275952f, 0.2500000000f, -0.1201884846f, -3.640275952f, -50.96797159f, -5.033905175f},
	};

	input = inputTemplate();
	input.phi = 0.05f;
	input.p = -0.02f;
	input.command_roll = 0.f;
	controller.reset(input);

	for (unsigned i = 0; i < sizeof(golden) / sizeof(golden[0]); ++i) {
		const Golden &expected = golden[i];
		input = inputTemplate();
		input.phi = 0.05f + 0.01f * static_cast<float>(i) + ((i % 2) ? 0.003f : -0.002f);
		input.command_roll = 0.12f;
		input.u_applied_prev = -0.08f + 0.025f * static_cast<float>(i);
		const HydroRollControllerOutput output = controller.update(input);
		assert(output.valid);
		assert(nearlyEqual(controller.v1(), expected.v1));
		assert(nearlyEqual(controller.v2(), expected.v2));
		assert(nearlyEqual(controller.tdAcceleration(), expected.td_acceleration));
		assert(nearlyEqual(controller.z1(), expected.z1));
		assert(nearlyEqual(controller.z2(), expected.z2));
		assert(nearlyEqual(controller.z3(), expected.z3));
		assert(nearlyEqual(controller.e1(), expected.e1));
		assert(nearlyEqual(controller.e2(), expected.e2));
		assert(nearlyEqual(controller.u0(), expected.u0));
		assert(nearlyEqual(output.u_requested, expected.u_requested));
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
