/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "LADRC.hpp"

#include <cmath>

bool LADRC::parametersValid() const
{
	return std::isfinite(_config.b0) && _config.b0 > 0.f
	       && std::isfinite(_config.a_phi)
	       && std::isfinite(_config.k_phi1) && _config.k_phi1 > 0.f
	       && std::isfinite(_config.k_phi2) && _config.k_phi2 > 0.f
	       && std::isfinite(_config.observer_bandwidth) && _config.observer_bandwidth > 0.f;
}

bool LADRC::inputValid(const HydroRollControllerInput &input) const
{
	return std::isfinite(input.phi) && std::isfinite(input.p)
	       && std::isfinite(input.command_roll)
	       && std::isfinite(input.phi_ref) && std::isfinite(input.phi_ref_dot)
	       && std::isfinite(input.phi_ref_ddot)
	       && std::isfinite(input.dt_measured) && std::isfinite(input.dt_used) && input.dt_used > 0.f
	       && std::isfinite(input.u_applied_prev);
}

bool LADRC::setParameters(const HydroRollControllerConfig &config)
{
	_config = config;

	if (!parametersValid()) {
		_valid = false;
		return false;
	}

	return true;
}

void LADRC::reset(const HydroRollControllerInput &input)
{
	++_reset_counter;
	_update_counter = 0;
	_reset_event_pending = true;
	_valid = parametersValid() && std::isfinite(input.phi) && std::isfinite(input.p);

	if (_valid) {
		_z1 = input.phi;
		_z2 = input.p;
		_z3 = 0.f;
		_e1 = 0.f;
		_e2 = 0.f;

	} else {
		_z1 = NAN;
		_z2 = NAN;
		_z3 = NAN;
		_e1 = NAN;
		_e2 = NAN;
	}
}

HydroRollControllerOutput LADRC::update(const HydroRollControllerInput &input)
{
	++_update_counter;
	const bool reset_event = _reset_event_pending;
	_reset_event_pending = false;

	if (!_valid || !parametersValid() || !inputValid(input)) {
		_valid = false;
		return {NAN, false, reset_event};
	}

	const float omega_o = _config.observer_bandwidth;
	const float l1 = 3.f * omega_o;
	const float l2 = 3.f * omega_o * omega_o;
	const float l3 = omega_o * omega_o * omega_o;
	const float innovation = input.phi - _z1;

	// Causal sample-k order: propagate the ESO with measured phi[k] and the
	// previous common-limited u_applied[k-1], then control from updated states.
	const float z1_dot = _z2 + l1 * innovation;
	const float z2_dot = _config.b0 * input.u_applied_prev + _z3 + l2 * innovation;
	const float z3_dot = l3 * innovation;
	_z1 += input.dt_used * z1_dot;
	_z2 += input.dt_used * z2_dot;
	_z3 += input.dt_used * z3_dot;

	_e1 = _z1 - input.phi_ref;
	_e2 = _z2 - input.phi_ref_dot;
	const float v_phi = input.phi_ref_ddot - _config.k_phi1 * _e1 - _config.k_phi2 * _e2;
	const float u_requested = (v_phi - _z3) / _config.b0;

	_valid = std::isfinite(_z1) && std::isfinite(_z2) && std::isfinite(_z3)
		 && std::isfinite(_e1) && std::isfinite(_e2) && std::isfinite(u_requested);
	return {_valid ? u_requested : NAN, _valid, reset_event};
}
