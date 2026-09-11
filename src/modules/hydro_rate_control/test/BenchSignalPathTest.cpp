// Offline diagnostics for the reported dry-bench delay/oscillation.
// Uses actual controller/reference/limiter implementations. It does NOT model
// the allocator, PWM, mechanics, sensor noise, uORB scheduling or hydrodynamics.
#include "../AMCEO_ADRC.hpp"
#include "../ClassicalNADRC.hpp"
#include "../HydroRollController.hpp"
#include "../LADRC.hpp"
#include "../MCEO_ADRC.hpp"
#include "../RollReferenceGenerator.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace
{
constexpr float dt = 0.0025f; // Diagnostic fixture, NOT a firmware dt change.

template<typename Controller>
void stationaryCheck(const char *name, Controller &controller)
{
	HydroRollControllerInput input{};
	input.dt_measured = dt;
	input.dt_used = dt;
	input.u_max = 1.f;
	input.timestamp_sample = 1000000;
	controller.reset(input);
	HydroRollLimiter limiter;
	limiter.reset();

	for (int k = 0; k < 24000; ++k) {
		input.u_applied_prev = limiter.appliedPrevious();
		input.timestamp_sample += 2500;
		const auto output = controller.update(input);
		assert(output.valid && std::isfinite(output.u_requested));
		assert(std::fabs(output.u_requested) < 1e-6f);
		assert(std::fabs(controller.z3()) < 1e-6f);
		const auto limited = limiter.update(output.u_requested, 0.f, dt, 1.f, 1.f);
		assert(limited.valid && std::fabs(limited.u_applied) < 1e-6f);
	}

	std::printf("%s: ideal zero sensor/reference/trim, 60 s: zero request/applied/z3 PASS\n", name);
}
}

int main()
{
	// Measure reference dynamics independently of all observers (no z3 exists).
	RollReferenceGenerator reference;
	reference.reset(0.f, 0.f);
	constexpr float command = 0.1745329252f; // Arbitrary 10-degree diagnostic step.
	float reference_t90 = NAN;
	float reference_t95 = NAN;
	for (int k = 1; k <= 2000; ++k) {
		assert(reference.update(command, 2.f, 1.f, dt));
		if (reference.phiRef() >= 0.9f * command && !std::isfinite(reference_t90)) {
			reference_t90 = k * dt;
		}
		if (reference.phiRef() >= 0.95f * command && !std::isfinite(reference_t95)) {
			reference_t95 = k * dt;
		}
	}
	assert(reference_t90 > 1.90f && reference_t90 < 1.98f);
	assert(reference_t95 > 2.32f && reference_t95 < 2.42f);
	std::printf("Reference only, W=2 ZETA=1: t90=%.4f s, t95=%.4f s (no observer)\n",
		    static_cast<double>(reference_t90), static_cast<double>(reference_t95));

	// Verify immediate slew response, not a two-second dead time. No observer.
	HydroRollLimiter limiter;
	limiter.reset(-1.f);
	float previous = -1.f;
	float reversal_time = NAN;
	for (int k = 1; k <= 805; ++k) {
		const auto output = limiter.update(1.f, 0.f, dt, 1.f, 1.f);
		assert(output.valid);
		assert(std::fabs(output.u_applied) <= 1.f);
		assert(std::fabs(output.u_applied - previous) <= dt + 1e-6f);
		if (k == 1) {
			assert(output.u_applied > previous && output.rate_limited);
		}
		previous = output.u_applied;
		if (output.u_applied >= 1.f - 1e-5f && !std::isfinite(reversal_time)) {
			reversal_time = k * dt;
		}
	}
	assert(reversal_time >= 1.99f && reversal_time <= 2.01f);
	std::printf("Limiter only, DU_MAX=1 U_MAX=1: -1 to +1 = %.4f s (no observer)\n",
		    static_cast<double>(reversal_time));

	LADRC ladrc;
	MCEOADRC mceo;
	AMCEOADRC amceo;
	ClassicalNADRC nadrc;
	assert(ladrc.setParameters({10.1746f, -0.5f, 26.f, 9.8f, 25.f}));
	assert(mceo.setParameters({10.1746f, -0.5f, 26.f, 9.8f, 22.f}));
	assert(amceo.setParameters({{10.1746f, -0.5f, 26.f, 9.8f, 18.f},
		18.f, 46.f, 0.16f, 0.70f, 0.01f, 0.055f, 0.0523598776f, 0.698131701f}));
	assert(nadrc.setParameters({10.1746f, 26.f, 9.8f, 20.f, 20.f, 2.f,
		0.5f, 0.25f, 0.01f, 0.8f, 1.2f, 0.01f}));
	stationaryCheck("LADRC", ladrc);
	stationaryCheck("MCEO", mceo);
	stationaryCheck("AMCEO", amceo);
	stationaryCheck("NADRC", nadrc);
	std::puts("BenchSignalPathTest: PASS (software fixtures only; not hardware root-cause clearance)");
}
