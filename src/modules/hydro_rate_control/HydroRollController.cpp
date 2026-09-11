/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "HydroRollController.hpp"

#include <cmath>

namespace
{
float constrainFloat(float value, float lower, float upper)
{
	return fmaxf(lower, fminf(value, upper));
}
}

void HydroRollLimiter::reset(float u_applied)
{
	_u_applied_prev = std::isfinite(u_applied) ? u_applied : 0.f;
}

HydroRollLimiterResult HydroRollLimiter::update(float u_requested, float trim, float dt_used, float u_max,
		float du_max)
{
	HydroRollLimiterResult result{};

	if (!std::isfinite(u_requested) || !std::isfinite(trim) || !std::isfinite(dt_used)
	    || !std::isfinite(u_max) || !std::isfinite(du_max) || dt_used <= 0.f || u_max < 0.f || du_max < 0.f) {
		reset();
		return result;
	}

	const float requested_with_trim = u_requested + trim;

	if (!std::isfinite(requested_with_trim)) {
		reset();
		return result;
	}

	const float max_step = du_max * dt_used;
	const float requested_step = requested_with_trim - _u_applied_prev;
	const float limited_step = constrainFloat(requested_step, -max_step, max_step);
	const float rate_limited_command = _u_applied_prev + limited_step;
	const float applied = constrainFloat(rate_limited_command, -u_max, u_max);

	result.u_applied = applied;
	result.valid = std::isfinite(applied);
	result.rate_limited = std::fabs(limited_step - requested_step) > 1e-6f;
	result.magnitude_limited = std::fabs(applied - rate_limited_command) > 1e-6f;

	if (result.valid) {
		_u_applied_prev = applied;

	} else {
		reset();
	}

	return result;
}

bool HydroRollTimingGuard::isValid(float dt_used, float dt_max)
{
	return std::isfinite(dt_used) && std::isfinite(dt_max) && dt_used > 0.f && dt_max > 0.f && dt_used <= dt_max;
}

void ManualRollCommandLatch::reset(float current_roll)
{
	_command_roll = std::isfinite(current_roll) ? current_roll : 0.f;
	_manual_roll_raw = NAN;
	_age_s = NAN;
	_sample_timestamp = 0;
	_data_source = 0;
	_valid = false;
	_applicable = false;
}

void ManualRollCommandLatch::update(bool applicable, bool new_sample, bool sample_valid, float manual_roll,
		uint8_t data_source, uint64_t sample_timestamp, uint64_t now,
		float roll_scale, float current_roll)
{
	const float safe_current_roll = std::isfinite(current_roll) ? current_roll : 0.f;

	if (!applicable) {
		reset(safe_current_roll);
		return;
	}

	if (!_applicable) {
		reset(safe_current_roll);
		_applicable = true;
	}

	if (new_sample) {
		_manual_roll_raw = manual_roll;
		_data_source = data_source;
		_sample_timestamp = sample_timestamp;
		const bool timestamp_valid = sample_timestamp != 0 && sample_timestamp <= now;
		_age_s = timestamp_valid ? static_cast<float>(now - sample_timestamp) * 1e-6f
			 : NAN;
		_valid = sample_valid && std::isfinite(manual_roll) && std::isfinite(roll_scale)
			 && timestamp_valid;

		if (_valid) {
			_command_roll = manual_roll * roll_scale;

		} else {
			_command_roll = safe_current_roll;
		}

	} else if (_valid) {
		if (_sample_timestamp == 0 || _sample_timestamp > now) {
			_valid = false;
			_age_s = NAN;
			_command_roll = safe_current_roll;

		} else {
			_age_s = static_cast<float>(now - _sample_timestamp) * 1e-6f;
			// Diagnostic age only. Missing publication is not an invalid command.
		}
	}
}
