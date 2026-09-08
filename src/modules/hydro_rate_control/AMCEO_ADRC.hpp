/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cmath>

#include "HydroRollController.hpp"

struct AMCEOControllerConfig {
	HydroRollControllerConfig common;
	float omega_min; ///< minimum observer bandwidth [rad/s]
	float omega_max; ///< maximum observer bandwidth [rad/s]
	float rho0;      ///< baseline/initial scheduling factor [0, 1]
	float ce;        ///< nonnegative tracking-error scheduling weight
	float cu;        ///< nonnegative applied-command penalty weight
	float tau_rho;   ///< scheduler time constant [s]
	float e1_bar;    ///< roll-angle normalization scale [rad]
	float e2_bar;    ///< roll-rate normalization scale [rad/s]
};

class AMCEOADRC
{
public:
	bool setParameters(const AMCEOControllerConfig &config);
	void reset(const HydroRollControllerInput &input);
	HydroRollControllerOutput update(const HydroRollControllerInput &input);
	bool isValid() const { return _valid; }
	float z1() const { return _z1; }
	float z2() const { return _z2; }
	float z3() const { return _z3; }
	float z4() const { return _z4; }
	float e1() const { return _e1; }
	float e2() const { return _e2; }
	float rho() const { return _rho; }
	float rhoStar() const { return _rho_star; }
	float sigmaE() const { return _sigma_e; }
	float sigmaU() const { return _sigma_u; }
	float observerBandwidth() const { return _observer_bandwidth; }
	uint32_t resetCounter() const { return _reset_counter; }
	uint32_t updateCounter() const { return _update_counter; }

private:
	bool parametersValid() const;
	bool inputValid(const HydroRollControllerInput &input) const;
	static float saturate01(float value);

	AMCEOControllerConfig _config{};
	float _z1{0.f};
	float _z2{0.f};
	float _z3{0.f};
	float _z4{0.f};
	float _e1{0.f};
	float _e2{0.f};
	float _rho{0.f};
	float _rho_star{NAN};
	float _sigma_e{NAN};
	float _sigma_u{NAN};
	float _observer_bandwidth{NAN};
	uint32_t _reset_counter{0};
	uint32_t _update_counter{0};
	bool _reset_event_pending{false};
	bool _valid{false};
};
