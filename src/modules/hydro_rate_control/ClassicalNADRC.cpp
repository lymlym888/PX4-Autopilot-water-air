/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "ClassicalNADRC.hpp"

#include <cmath>
#include <float.h>

float ClassicalNADRC::sign(float value)
{
	return (value > 0.f) ? 1.f : ((value < 0.f) ? -1.f : 0.f);
}

float ClassicalNADRC::fal(float error, float alpha, float delta)
{
	if (!std::isfinite(error) || !std::isfinite(alpha) || !std::isfinite(delta)
	    || alpha <= 0.f || delta <= 0.f) {
		return NAN;
	}

	const float absolute_error = std::fabs(error);
	float result = NAN;

	if (absolute_error <= delta) {
		const float denominator = std::pow(delta, 1.f - alpha);

		if (!std::isfinite(denominator) || denominator <= 0.f) {
			return NAN;
		}

		result = error / denominator;

	} else {
		result = std::pow(absolute_error, alpha) * sign(error);
	}

	return std::isfinite(result) ? result : NAN;
}

float ClassicalNADRC::fhan(float x1, float x2, float r_td, float h0)
{
	if (!std::isfinite(x1) || !std::isfinite(x2) || !std::isfinite(r_td) || !std::isfinite(h0)
	    || r_td <= 0.f || h0 <= 0.f) {
		return NAN;
	}

	const float d = r_td * h0 * h0;

	// The canonical expression divides by d. Treat an under-resolved value as
	// invalid rather than allowing an overflow or a numerically plausible spike.
	if (!std::isfinite(d) || d <= FLT_MIN) {
		return NAN;
	}

	const float a0 = h0 * x2;
	const float y = x1 + a0;
	const float radicand = d * (d + 8.f * std::fabs(y));

	if (!std::isfinite(a0) || !std::isfinite(y) || !std::isfinite(radicand) || radicand < 0.f) {
		return NAN;
	}

	const float a1 = std::sqrt(radicand);
	const float a2 = a0 + sign(y) * (a1 - d) * 0.5f;
	const float sy = (sign(y + d) - sign(y - d)) * 0.5f;
	// Canonical Han form. The trailing term is +a2.
	const float a = (a0 + y - a2) * sy + a2;
	const float sa = (sign(a + d) - sign(a - d)) * 0.5f;
	const float fh = -r_td * (a / d - sign(a)) * sa - r_td * sign(a);
	return std::isfinite(fh) ? fh : NAN;
}

bool ClassicalNADRC::parametersValid() const
{
	return std::isfinite(_config.b0) && _config.b0 > 0.f
	       && std::isfinite(_config.k_phi1) && _config.k_phi1 > 0.f
	       && std::isfinite(_config.k_phi2) && _config.k_phi2 > 0.f
	       && std::isfinite(_config.observer_bandwidth) && _config.observer_bandwidth > 0.f
	       && std::isfinite(_config.td_r) && _config.td_r > 0.f
	       && std::isfinite(_config.td_n0) && _config.td_n0 >= 1.f
	       && std::isfinite(_config.observer_alpha1) && _config.observer_alpha1 > 0.f
	       && std::isfinite(_config.observer_alpha2) && _config.observer_alpha2 > 0.f
	       && std::isfinite(_config.observer_delta) && _config.observer_delta > 0.f
	       && std::isfinite(_config.controller_alpha1) && _config.controller_alpha1 > 0.f
	       && std::isfinite(_config.controller_alpha2) && _config.controller_alpha2 > 0.f
	       && std::isfinite(_config.controller_delta) && _config.controller_delta > 0.f;
}

bool ClassicalNADRC::inputValid(const HydroRollControllerInput &input) const
{
	return std::isfinite(input.phi) && std::isfinite(input.p)
	       && std::isfinite(input.command_roll)
	       && std::isfinite(input.dt_measured) && std::isfinite(input.dt_used) && input.dt_used > 0.f
	       && std::isfinite(input.u_applied_prev);
}

bool ClassicalNADRC::setParameters(const ClassicalNADRCConfig &config)
{
	_config = config;

	if (!parametersValid()) {
		_valid = false;
		return false;
	}

	return true;
}

void ClassicalNADRC::reset(const HydroRollControllerInput &input)
{
	++_reset_counter;
	_update_counter = 0;
	_reset_event_pending = true;
	_valid = parametersValid() && std::isfinite(input.phi) && std::isfinite(input.p);

	if (_valid) {
		_v1 = std::isfinite(input.command_roll) ? input.command_roll : input.phi;
		_v2 = 0.f;
		_td_acceleration = NAN;
		_z1 = input.phi;
		_z2 = input.p;
		_z3 = 0.f;
		_e1 = 0.f;
		_e2 = 0.f;
		_u0 = NAN;

	} else {
		_v1 = NAN;
		_v2 = NAN;
		_td_acceleration = NAN;
		_z1 = NAN;
		_z2 = NAN;
		_z3 = NAN;
		_e1 = NAN;
		_e2 = NAN;
		_u0 = NAN;
	}
}

HydroRollControllerOutput ClassicalNADRC::update(const HydroRollControllerInput &input)
{
	++_update_counter;
	const bool reset_event = _reset_event_pending;
	_reset_event_pending = false;

	if (!_valid || !parametersValid() || !inputValid(input)) {
		_valid = false;
		return {NAN, false, reset_event};
	}

	const float h0 = _config.td_n0 * input.dt_used;
	const float v1_dot = _v2;
	_td_acceleration = fhan(_v1 - input.command_roll, _v2, _config.td_r, h0);

	const float omega_o2 = _config.observer_bandwidth * _config.observer_bandwidth;
	const float beta01 = 3.f * _config.observer_bandwidth;
	const float beta02 = 3.f * omega_o2;
	const float beta03 = omega_o2 * _config.observer_bandwidth;
	const float observer_error = _z1 - input.phi; // estimated minus measured
	const float fal_o1 = fal(observer_error, _config.observer_alpha1, _config.observer_delta);
	const float fal_o2 = fal(observer_error, _config.observer_alpha2, _config.observer_delta);
	const float z1_dot = _z2 - beta01 * observer_error;
	const float z2_dot = _z3 - beta02 * fal_o1 + _config.b0 * input.u_applied_prev;
	const float z3_dot = -beta03 * fal_o2;

	if (!std::isfinite(h0) || !std::isfinite(v1_dot) || !std::isfinite(_td_acceleration)
	    || !std::isfinite(z1_dot) || !std::isfinite(z2_dot) || !std::isfinite(z3_dot)) {
		_valid = false;
		return {NAN, false, reset_event};
	}

	// Simultaneous forward-Euler propagation. The NESO uses u_applied[k-1].
	_v1 += input.dt_used * v1_dot;
	_v2 += input.dt_used * _td_acceleration;
	_z1 += input.dt_used * z1_dot;
	_z2 += input.dt_used * z2_dot;
	_z3 += input.dt_used * z3_dot;

	_e1 = _v1 - _z1;
	_e2 = _v2 - _z2;
	const float fal_c1 = fal(_e1, _config.controller_alpha1, _config.controller_delta);
	const float fal_c2 = fal(_e2, _config.controller_alpha2, _config.controller_delta);
	_u0 = _config.k_phi1 * fal_c1 + _config.k_phi2 * fal_c2;
	const float u_requested = (_u0 - _z3) / _config.b0;

	_valid = std::isfinite(_v1) && std::isfinite(_v2) && std::isfinite(_td_acceleration)
		 && std::isfinite(_z1) && std::isfinite(_z2) && std::isfinite(_z3)
		 && std::isfinite(_e1) && std::isfinite(_e2) && std::isfinite(_u0)
		 && std::isfinite(u_requested);
	return {_valid ? u_requested : NAN, _valid, reset_event};
}
