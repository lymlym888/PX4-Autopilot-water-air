/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "../ClassicalNADRC.hpp"
#include "../HydroRollController.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

namespace
{
bool nearlyEqual(float actual, float expected, float tolerance = 2e-5f)
{
	return std::fabs(actual - expected) <= tolerance * fmaxf(1.f, std::fabs(expected));
}
}

int main()
{
	// Canonical fal branches, equality convention, and odd symmetry.
	assert(nearlyEqual(ClassicalNADRC::fal(0.f, 0.5f, 0.01f), 0.f));
	assert(nearlyEqual(ClassicalNADRC::fal(0.005f, 0.5f, 0.01f), 0.05f));
	assert(nearlyEqual(ClassicalNADRC::fal(0.01f, 0.5f, 0.01f), 0.1f));
	assert(nearlyEqual(ClassicalNADRC::fal(0.04f, 0.5f, 0.01f), 0.2f));
	assert(nearlyEqual(ClassicalNADRC::fal(-0.005f, 0.5f, 0.01f), -0.05f));
	assert(nearlyEqual(ClassicalNADRC::fal(-0.04f, 0.5f, 0.01f), -0.2f));
	assert(std::isnan(ClassicalNADRC::fal(0.1f, 0.f, 0.01f)));
	assert(std::isnan(ClassicalNADRC::fal(0.1f, 0.5f, 0.f)));
	assert(std::isnan(ClassicalNADRC::fal(NAN, 0.5f, 0.01f)));
	assert(std::isnan(ClassicalNADRC::fal(INFINITY, 0.5f, 0.01f)));

	// Canonical fhan zero, step direction, symmetry and nonlinear inner branch.
	assert(nearlyEqual(ClassicalNADRC::fhan(0.f, 0.f, 20.f, 0.008f), 0.f));
	assert(nearlyEqual(ClassicalNADRC::fhan(0.1f, 0.f, 20.f, 0.008f), -20.f));
	assert(nearlyEqual(ClassicalNADRC::fhan(-0.1f, 0.f, 20.f, 0.008f), 20.f));
	assert(nearlyEqual(ClassicalNADRC::fhan(0.0001f, 0.01f, 20.f, 0.008f), -4.0625f));
	assert(nearlyEqual(ClassicalNADRC::fhan(0.001f, 0.f, 20.f, 0.008f), -15.625f));

	// A finite positive and negative TD step converges symmetrically.
	for (const float command : {1.f, -1.f}) {
		float v1 = 0.f;
		float v2 = 0.f;
		constexpr float dt = 0.002f;

		for (int i = 0; i < 2000; ++i) {
			const float acceleration = ClassicalNADRC::fhan(v1 - command, v2, 20.f, 2.f * dt);
			assert(std::isfinite(acceleration));
			const float v1_dot = v2;
			v1 += dt * v1_dot;
			v2 += dt * acceleration;
		}

		assert(nearlyEqual(v1, command, 5e-4f));
		assert(nearlyEqual(v2, 0.f, 5e-4f));
	}

	// Timing boundary belongs to the shared new-controller guard. Very small d
	// and all invalid/nonfinite fhan arguments are explicitly rejected.
	assert(HydroRollTimingGuard::isValid(0.010f, 0.010f));
	assert(!HydroRollTimingGuard::isValid(0.0101f, 0.010f));
	assert(std::isfinite(ClassicalNADRC::fhan(0.1f, 0.f, 0.001f, 0.002f)));
	assert(std::isnan(ClassicalNADRC::fhan(0.1f, 0.f, 20.f, 1e-20f)));
	assert(std::isnan(ClassicalNADRC::fhan(NAN, 0.f, 20.f, 0.008f)));
	assert(std::isnan(ClassicalNADRC::fhan(0.f, INFINITY, 20.f, 0.008f)));
	assert(std::isnan(ClassicalNADRC::fhan(0.f, 0.f, 0.f, 0.008f)));
	assert(std::isnan(ClassicalNADRC::fhan(0.f, 0.f, 20.f, 0.f)));

	std::puts("NadrcMathTest: PASS");
	return 0;
}
