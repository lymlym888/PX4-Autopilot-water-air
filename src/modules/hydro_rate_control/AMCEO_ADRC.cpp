/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "AMCEO_ADRC.hpp"

#include <cmath>

float AMCEOADRC::saturate01(float value)
{
	return fmaxf(0.f, fminf(value, 1.f));
}

bool AMCEOADRC::parametersValid() const
{
	const HydroRollControllerConfig &common = _config.common;
	return std::isfinite(common.b0) && common.b0 > 0.f
	       && std::isfinite(common.a_phi)
	       && std::isfinite(common.k_phi1) && common.k_phi1 > 0.f
	       && std::isfinite(common.k_phi2) && common.k_phi2 > 0.f
	       && std::isfinite(_config.omega_min) && _config.omega_min > 0.f
	       && std::isfinite(_config.omega_max) && _config.omega_max > _config.omega_min
	       && std::isfinite(_config.rho0) && _config.rho0 >= 0.f && _config.rho0 <= 1.f
	       && std::isfinite(_config.ce) && _config.ce >= 0.f
	       && std::isfinite(_config.cu) && _config.cu >= 0.f
	       && std::isfinite(_config.tau_rho) && _config.tau_rho > 0.f
	       && std::isfinite(_config.e1_bar) && _config.e1_bar > 0.f
	       && std::isfinite(_config.e2_bar) && _config.e2_bar > 0.f;
}

bool AMCEOADRC::inputValid(const HydroRollControllerInput &input) const
{
	return std::isfinite(input.phi) && std::isfinite(input.p)
	       && std::isfinite(input.command_roll)
	       && std::isfinite(input.phi_ref) && std::isfinite(input.phi_ref_dot)
	       && std::isfinite(input.phi_ref_ddot)
	       && std::isfinite(input.dt_measured) && std::isfinite(input.dt_used) && input.dt_used > 0.f
	       && std::isfinite(input.u_applied_prev)
	       && std::isfinite(input.u_max) && input.u_max > 0.f;
}

bool AMCEOADRC::setParameters(const AMCEOControllerConfig &config)
{
	_config = config;

	if (!parametersValid()) {
		_valid = false;
		return false;
	}

	return true;
}

void AMCEOADRC::reset(const HydroRollControllerInput &input)
{
	++_reset_counter;
	_update_counter = 0;
	_reset_event_pending = true;
	_valid = parametersValid() && std::isfinite(input.phi) && std::isfinite(input.p);

	if (_valid) {
		_z1 = input.phi;
		_z2 = input.p;
		_z3 = 0.f;
		_z4 = 0.f;
		_e1 = 0.f;
		_e2 = 0.f;
		_rho = _config.rho0;
		_rho_star = NAN;
		_sigma_e = NAN;
		_sigma_u = NAN;
		_observer_bandwidth = _config.omega_min + (_config.omega_max - _config.omega_min) * _rho;
		_valid = std::isfinite(_observer_bandwidth);

	} else {
		_z1 = NAN;
		_z2 = NAN;
		_z3 = NAN;
		_z4 = NAN;
		_e1 = NAN;
		_e2 = NAN;
		_rho = NAN;
		_rho_star = NAN;
		_sigma_e = NAN;
		_sigma_u = NAN;
		_observer_bandwidth = NAN;
	}
}

HydroRollControllerOutput AMCEOADRC::update(const HydroRollControllerInput &input)
{
	++_update_counter;
	const bool reset_event = _reset_event_pending;
	_reset_event_pending = false;

	if (!_valid || !parametersValid() || !inputValid(input)) {
		_valid = false;
		return {NAN, false, reset_event};
	}

	// Scheduler uses PRE-STEP observer tracking errors and the previous final
	// common-limited command. There is intentionally no innovation-based term.
	const float e1_sched = _z1 - input.phi_ref;
	const float e2_sched = _z2 - input.phi_ref_dot;
	_sigma_e = saturate01(std::fabs(e1_sched) / _config.e1_bar + std::fabs(e2_sched) / _config.e2_bar);
	_sigma_u = saturate01(std::fabs(input.u_applied_prev) / input.u_max);
	_rho_star = saturate01(_config.rho0 + _config.ce * _sigma_e - _config.cu * _sigma_u);
	_rho = saturate01(_rho + input.dt_used / _config.tau_rho * (-_rho + _rho_star));
	_observer_bandwidth = _config.omega_min + (_config.omega_max - _config.omega_min) * _rho;

	const float omega_o2 = _observer_bandwidth * _observer_bandwidth;
	const float l1 = 4.f * _observer_bandwidth;
	const float l2 = 6.f * omega_o2;
	const float l3 = 4.f * omega_o2 * _observer_bandwidth;
	const float l4 = omega_o2 * omega_o2;
	const float innovation = input.phi - _z1;

	// Causal firmware convention inherited from Stage 3: update the observer
	// with phi[k] and u_applied[k-1], then control from UPDATED observer states.
	const float z1_dot = _z2 + l1 * innovation;
	const float z2_dot = _config.common.a_phi * _z2 + _config.common.b0 * input.u_applied_prev
			     + _z3 + l2 * innovation;
	const float z3_dot = _z4 + l3 * innovation;
	const float z4_dot = l4 * innovation;
	_z1 += input.dt_used * z1_dot;
	_z2 += input.dt_used * z2_dot;
	_z3 += input.dt_used * z3_dot;
	_z4 += input.dt_used * z4_dot;

	// User-requested bench bound on disturbance state [rad/s^2], not output.
	// Reject overflow before clipping so nonfinite state cannot appear valid.
	if (!std::isfinite(_z3)) {
		_valid = false;
		return {NAN, false, reset_event};
	}

	if (_z3 > 50.f) {
		_z3 = 50.f;

	} else if (_z3 < -50.f) {
		_z3 = -50.f;
	}

	_e1 = _z1 - input.phi_ref;
	_e2 = _z2 - input.phi_ref_dot;
	const float v_phi = input.phi_ref_ddot - _config.common.k_phi1 * _e1 - _config.common.k_phi2 * _e2;
	const float u_requested = (v_phi - _config.common.a_phi * _z2 - _z3) / _config.common.b0;

	_valid = std::isfinite(_z1) && std::isfinite(_z2) && std::isfinite(_z3) && std::isfinite(_z4)
		 && std::isfinite(_e1) && std::isfinite(_e2)
		 && std::isfinite(_rho) && std::isfinite(_rho_star)
		 && std::isfinite(_sigma_e) && std::isfinite(_sigma_u)
		 && std::isfinite(_observer_bandwidth) && std::isfinite(u_requested);
	return {_valid ? u_requested : NAN, _valid, reset_event};
}
