/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "../CADRC.h"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace
{
bool nearlyEqual(float actual, float expected)
{
	const float scale = fmaxf(1.f, fabsf(expected));
	return fabsf(actual - expected) <= 2e-6f * scale;
}
}

int main()
{
	ADRCController controller;
	controller.setJ0(10.1746f);
	controller.setGains(500.f, 15.f, 1000.f, 15.f);
	controller.setAlpha(0.f);
	controller.reset();

	static constexpr float expected_uc[] = {
		0.222797468f, -37.0937157f, -19.8513355f, -89.1247864f,
		781.995117f, -621.643555f, 1556.96313f, -1906.97449f,
		3515.57837f, -5058.61523f, 8408.91113f, -12841.3232f
	};
	static constexpr float expected_rate_sp[] = {
		0.f, -0.34937498f, -1.06648815f, -2.13029194f,
		4.09169817f, 9.5756464f, 14.3611317f, 18.4858036f,
		21.9854641f, 24.8941708f, 27.2442989f, 29.0666237f
	};

	for (int i = 0; i < 12; ++i) {
		const float phi = 0.01f * static_cast<float>(i);
		const float phi_ref = (i < 4) ? 0.f : 0.2f;
		const float p = -0.03f + 0.005f * static_cast<float>(i);
		controller.update(phi, phi_ref, p, 0.0025f);
		assert(nearlyEqual(controller.getUc(), expected_uc[i]));
		assert(nearlyEqual(controller.getRateSp(), expected_rate_sp[i]));
	}

	assert(nearlyEqual(controller._Z1(0, 0), 0.0573354699f));
	assert(nearlyEqual(controller._Z1(1, 0), 0.398889035f));
	assert(nearlyEqual(controller._Z2(0, 0), -130.673553f));
	assert(nearlyEqual(controller._Z2(1, 0), 18.2380619f));

	// The unchanged HydroRateControl CADRC path scales raw Uc, clamps, adds
	// trim, and clamps again. This checks the golden final roll contribution.
	const float scaled = controller.getUc() * 0.001f;
	const float first_clamp = fmaxf(-1.f, fminf(scaled, 1.f));
	const float applied = fmaxf(-1.f, fminf(first_clamp + 0.1f, 1.f));
	assert(nearlyEqual(scaled, -12.8413232f));
	assert(nearlyEqual(applied, -0.9f));

	std::puts("CadrcGoldenTest: PASS");
	return 0;
}
