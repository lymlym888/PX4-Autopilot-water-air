/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cmath>

#include "HydroRollController.hpp"

struct ClassicalNADRCConfig {
	float b0;                 ///< normalized-command-to-roll-acceleration effectiveness
	float k_phi1;             ///< nonlinear roll-angle feedback coefficient
	float k_phi2;             ///< nonlinear roll-rate feedback coefficient
	float observer_bandwidth; ///< fixed nonlinear ESO bandwidth [rad/s]
	float td_r;               ///< Han fhan tracking-speed factor
	float td_n0;              ///< TD integration-step multiplier, >= 1
	float observer_alpha1;    ///< first nonlinear ESO fal exponent
	float observer_alpha2;    ///< second nonlinear ESO fal exponent
	float observer_delta;     ///< nonlinear ESO fal threshold [rad]
	float controller_alpha1;  ///< roll-angle feedback fal exponent
	float controller_alpha2;  ///< roll-rate feedback fal exponent
	float controller_delta;   ///< shared numeric feedback fal threshold
};

/** Classical Han-style nonlinear ADRC: fhan TD + nonlinear ESO + NLSEF. */
class ClassicalNADRC
{
public:
	bool setParameters(const ClassicalNADRCConfig &config);
	void reset(const HydroRollControllerInput &input);
	HydroRollControllerOutput update(const HydroRollControllerInput &input);
	bool isValid() const { return _valid; }

	/** Canonical piecewise power function. Invalid arguments return NaN. */
	static float fal(float error, float alpha, float delta);

	/** Canonical Han fastest-control synthesis function. Invalid arguments return NaN. */
	static float fhan(float x1, float x2, float r_td, float h0);

	float v1() const { return _v1; }
	float v2() const { return _v2; }
	float tdAcceleration() const { return _td_acceleration; }
	float z1() const { return _z1; }
	float z2() const { return _z2; }
	float z3() const { return _z3; }
	float e1() const { return _e1; }
	float e2() const { return _e2; }
	float u0() const { return _u0; }
	float observerBandwidth() const { return _config.observer_bandwidth; }
	uint32_t resetCounter() const { return _reset_counter; }
	uint32_t updateCounter() const { return _update_counter; }

private:
	static float sign(float value);
	bool parametersValid() const;
	bool inputValid(const HydroRollControllerInput &input) const;

	ClassicalNADRCConfig _config{};
	float _v1{0.f};
	float _v2{0.f};
	float _td_acceleration{NAN};
	float _z1{0.f};
	float _z2{0.f};
	float _z3{0.f};
	float _e1{0.f};
	float _e2{0.f};
	float _u0{NAN};
	uint32_t _reset_counter{0};
	uint32_t _update_counter{0};
	bool _reset_event_pending{false};
	bool _valid{false};
};
