// User-requested +/-0.25 rad/s^2 disturbance-state bound for IDs 2-5.
// This is a state bound, not an output bound or a hardware stability guarantee.
#include "../LADRC.hpp"
#include "../MCEO_ADRC.hpp"
#include "../AMCEO_ADRC.hpp"
#include "../ClassicalNADRC.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>

namespace
{
HydroRollControllerInput inputTemplate()
{
	HydroRollControllerInput input{};
	input.dt_used = 0.0025f;
	input.dt_measured = input.dt_used;
	input.timestamp_sample = 1000000;
	input.u_max = 1.f;
	return input;
}

template<typename Controller>
void checkLimit(Controller &controller, float expected_limit = 0.25f)
{
	for (float sign : {-1.f, 1.f}) {
		auto input = inputTemplate();
		controller.reset(input);
		input.phi = sign * 100.f; // Deliberately nonphysical mathematical stress input.
		const auto output = controller.update(input);
		assert(output.valid);
		assert(controller.z3() == sign * expected_limit);
		assert(std::fabs(output.u_requested) > 1.f); // State cap is NOT output cap.
		HydroRollLimiter limiter;
		limiter.reset();
		const auto applied = limiter.update(output.u_requested, 0.f, input.dt_used, 1.f, 1.f);
		assert(applied.valid && std::fabs(applied.u_applied) <= input.dt_used + 1e-6f);
		controller.reset(input);
		assert(controller.z3() == 0.f);
	}

	for (float invalid : {NAN, INFINITY, -INFINITY, std::numeric_limits<float>::max()}) {
		auto input = inputTemplate();
		controller.reset(input);
		input.phi = invalid;
		const auto output = controller.update(input);
		assert(!output.valid && !controller.isValid() && std::isnan(output.u_requested));
	}
}

void near(float a, float b)
{
	assert(std::fabs(a-b) <= 1e-4f * fmaxf(1.f, fabsf(b)));
}
}

int main()
{
	LADRC ladrc;
	MCEOADRC mceo;
	AMCEOADRC amceo;
	ClassicalNADRC nadrc;
	assert(ladrc.setParameters({10.1746f, -0.5f, 26.f, 9.8f, 25.f}));
	assert(mceo.setParameters({10.1746f, -0.5f, 26.f, 9.8f, 22.f}));
	const AMCEOControllerConfig config{{10.1746f, -0.5f, 26.f, 9.8f, 18.f},
		18.f, 46.f, 0.16f, 0.7f, 0.01f, 0.055f, 0.0523598776f, 0.698131701f};
	assert(amceo.setParameters(config));
	assert(nadrc.setParameters({10.1746f, 26.f, 9.8f, 20.f, 20.f, 2.f,
		0.5f, 0.25f, 0.01f, 0.8f, 1.2f, 0.01f}));
	checkLimit(ladrc);
	checkLimit(mceo);
	checkLimit(amceo);
	checkLimit(nadrc);

	// Fixed-bandwidth equivalence still holds WHEN THE NEW CAP ACTIVATES.
	auto fixed = config;
	fixed.ce = fixed.cu = 0.f;
	fixed.rho0 = (22.f - 18.f) / (46.f - 18.f);
	assert(amceo.setParameters(fixed));
	auto input = inputTemplate();
	mceo.reset(input);
	amceo.reset(input);
	bool hit_limit = false;
	for (int k = 0; k < 100; ++k) {
		input.phi = k % 2 ? -100.f : 100.f;
		input.u_applied_prev = k % 2 ? -0.2f : 0.2f;
		const auto m = mceo.update(input);
		const auto a = amceo.update(input);
		assert(m.valid && a.valid);
		near(mceo.z1(), amceo.z1());
		near(mceo.z2(), amceo.z2());
		near(mceo.z3(), amceo.z3());
		near(mceo.z4(), amceo.z4());
		near(m.u_requested, a.u_requested);
		assert(std::fabs(mceo.z3()) <= 0.25f);
		hit_limit |= std::fabs(mceo.z3()) == 0.25f;
	}
	assert(hit_limit);
	std::puts("Z3LimitTest: PASS (bounds, reset, nonfinite/overflow rejection, external limiter, saturated MCEO equivalence)");
}
