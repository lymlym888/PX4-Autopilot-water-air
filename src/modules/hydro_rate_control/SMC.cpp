#include "SMC.h"

#include <cmath>
#include <mathlib/math/Limits.hpp>

namespace
{
float sign(float value)
{
	return value > 0.f ? 1.f : (value < 0.f ? -1.f : 0.f);
}

// Han's fastest tracking function, as used by the reference controller.
float fhan(float position_error, float velocity, float rate, float filter)
{
	const float d = rate * filter;
	const float d0 = d * filter;
	const float y = position_error + filter * velocity;
	const float a0 = std::sqrt(d * d + 8.f * std::fabs(y));
	const float a = (std::fabs(y) > d0) ? velocity + 0.5f * (a0 - d) * sign(y) : velocity + y / filter;

	return (std::fabs(a) > d) ? -rate * sign(a) : -rate * a / d;
}
} // namespace

void SMC::setParameters(float control_gain, float td_rate, float td_filter, float observer_bandwidth,
			float smc_c, float smc_k, float smc_eta, float smc_boundary, float output_scale, float output_limit)
{
	constexpr float min_positive = 1e-4f;
	_control_gain = math::max(control_gain, min_positive);
	_td_rate = math::max(td_rate, min_positive);
	_td_filter = math::max(td_filter, min_positive);
	_observer_bandwidth = math::max(observer_bandwidth, min_positive);
	_smc_c = math::max(smc_c, 0.f);
	_smc_k = math::max(smc_k, 0.f);
	_smc_eta = math::max(smc_eta, 0.f);
	_smc_boundary = math::max(smc_boundary, min_positive);
	_output_scale = math::max(output_scale, min_positive);
	_output_limit = std::isfinite(output_limit) ? math::constrain(output_limit, min_positive, 1.f) : min_positive;
}

void SMC::reset()
{
	_td_position = 0.f;
	_td_velocity = 0.f;
	_z1 = 0.f;
	_z2 = 0.f;
	_z3 = 0.f;
	_previous_control = 0.f;
	_initialized = false;
}

float SMC::update(float angle, float angle_setpoint, float rate, float dt, Mode mode)
{
	if (!std::isfinite(angle) || !std::isfinite(angle_setpoint) || !std::isfinite(rate) || !std::isfinite(dt)) {
		reset();
		return 0.f;
	}

	if (!_initialized) {
		_td_position = angle_setpoint;
		_z1 = angle;
		_z2 = rate;
		_initialized = true;
	}

	const float time_step = math::constrain(dt, 0.0005f, 0.04f);
	const float td_acceleration = fhan(_td_position - angle_setpoint, _td_velocity, _td_rate, _td_filter);

	// The observer uses the previous cycle's scaled, limited control.
	const float observer_error = _z1 - angle;
	const float l1 = 3.f * _observer_bandwidth;
	const float l2 = 3.f * _observer_bandwidth * _observer_bandwidth;
	const float l3 = _observer_bandwidth * _observer_bandwidth * _observer_bandwidth;
	const float z1_derivative = _z2 - l1 * observer_error;
	const float z2_derivative = _z3 - l2 * observer_error + _control_gain * _previous_control;
	const float z3_derivative = -l3 * observer_error;

	const float position_error = _td_position - _z1;
	const float rate_error = _td_velocity - _z2;
	const float sliding_surface = _smc_c * position_error + rate_error;
	float virtual_control = _smc_c * rate_error + _smc_k * sliding_surface;

	if (mode == Mode::Full || mode == Mode::NoCompensation) {
		virtual_control += _smc_eta * math::constrain(sliding_surface / _smc_boundary, -1.f, 1.f);
	}

	if (mode == Mode::Full || mode == Mode::NoSwitching) {
		virtual_control -= _z3;
	}

	const float raw_control = virtual_control / _control_gain;
	const float control = math::constrain(raw_control * _output_scale, -_output_limit, _output_limit);

	// Compute this sample's torque before advancing all observer and TD states by Euler integration.
	_td_position += time_step * _td_velocity;
	_td_velocity += time_step * td_acceleration;
	_z1 += time_step * z1_derivative;
	_z2 += time_step * z2_derivative;
	_z3 += time_step * z3_derivative;
	_previous_control = control;

	if (!std::isfinite(_td_position) || !std::isfinite(_td_velocity) || !std::isfinite(_z1)
	    || !std::isfinite(_z2) || !std::isfinite(_z3) || !std::isfinite(control)) {
		reset();
		return 0.f;
	}

	return control;
}
