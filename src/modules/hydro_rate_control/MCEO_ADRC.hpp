/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "HydroRollController.hpp"

class MCEOADRC
{
public:
	bool setParameters(const HydroRollControllerConfig &config);
	void reset(const HydroRollControllerInput &input);
	HydroRollControllerOutput update(const HydroRollControllerInput &input);
	bool isValid() const { return _valid; }
	float z1() const { return _z1; }
	float z2() const { return _z2; }
	float z3() const { return _z3; }
	float z4() const { return _z4; }
	float e1() const { return _e1; }
	float e2() const { return _e2; }
	float observerBandwidth() const { return _config.observer_bandwidth; }
	uint32_t resetCounter() const { return _reset_counter; }
	uint32_t updateCounter() const { return _update_counter; }

private:
	bool parametersValid() const;
	bool inputValid(const HydroRollControllerInput &input) const;

	HydroRollControllerConfig _config{};
	float _z1{0.f};
	float _z2{0.f};
	float _z3{0.f};
	float _z4{0.f};
	float _e1{0.f};
	float _e2{0.f};
	uint32_t _reset_counter{0};
	uint32_t _update_counter{0};
	bool _reset_event_pending{false};
	bool _valid{false};
};
