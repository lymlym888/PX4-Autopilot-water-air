/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "RollReferenceGenerator.hpp"

#include <cmath>

void RollReferenceGenerator::reset(float command_roll, float current_roll)
{
	if (std::isfinite(current_roll)) {
		_phi_ref = current_roll;

	} else if (std::isfinite(command_roll)) {
		_phi_ref = command_roll;

	} else {
		_phi_ref = 0.f;
	}

	_phi_ref_dot = 0.f;
	_phi_ref_ddot = 0.f;
	_valid = true;
}

bool RollReferenceGenerator::update(float command_roll, float omega_r, float zeta_r, float dt)
{
	if (!_valid || !std::isfinite(command_roll) || !std::isfinite(omega_r) || !std::isfinite(zeta_r)
	    || !std::isfinite(dt) || omega_r <= 0.f || zeta_r < 0.f || dt <= 0.f) {
		_valid = false;
		return false;
	}

	const float acceleration = omega_r * omega_r * (command_roll - _phi_ref)
				   - 2.f * zeta_r * omega_r * _phi_ref_dot;

	// Explicit forward Euler: both state increments use the pre-step state/derivative.
	_phi_ref += _phi_ref_dot * dt;
	_phi_ref_dot += acceleration * dt;
	_phi_ref_ddot = omega_r * omega_r * (command_roll - _phi_ref)
				- 2.f * zeta_r * omega_r * _phi_ref_dot;

	_valid = std::isfinite(_phi_ref) && std::isfinite(_phi_ref_dot) && std::isfinite(_phi_ref_ddot);
	return _valid;
}

