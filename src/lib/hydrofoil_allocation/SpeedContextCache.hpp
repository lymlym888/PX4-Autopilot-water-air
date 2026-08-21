/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cmath>
#include <cstdint>

namespace hydrofoil
{

class SpeedContextCache
{
public:
	static constexpr uint64_t StaleTimeoutUs{500'000};

	struct Result {
		float speed_squared{0.f};
		uint32_t age_us{0};
		bool cached{false};
		bool stale{false};
		bool valid{false};
	};

	Result update(bool use_measured, float alternative_speed, uint64_t now,
		      bool new_sample, uint64_t sample_timestamp, float measured_speed_squared)
	{
		const float alternative_squared = alternative_speed * alternative_speed;

		if (!std::isfinite(alternative_squared) || alternative_squared < 0.f) {
			return {};
		}

		if (!use_measured) {
			return {alternative_squared, 0, false, false, true};
		}

		if (new_sample && sample_timestamp != 0 && now >= sample_timestamp
		    && std::isfinite(measured_speed_squared) && measured_speed_squared >= 0.f) {
			_value = measured_speed_squared < 1.f ? 1.f : measured_speed_squared;
			_timestamp = sample_timestamp;
			_have_sample = true;
		}

		if (!_have_sample) {
			return {alternative_squared, UINT32_MAX, true, true, true};
		}

		const uint64_t age = now >= _timestamp ? now - _timestamp : UINT64_MAX;
		const uint32_t age_us = age > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(age);
		const bool stale = age > StaleTimeoutUs;
		return {stale ? alternative_squared : _value, age_us, !new_sample, stale, true};
	}

	void reset()
	{
		_value = 0.f;
		_timestamp = 0;
		_have_sample = false;
	}

private:
	float _value{0.f};
	uint64_t _timestamp{0};
	bool _have_sample{false};
};

} // namespace hydrofoil
