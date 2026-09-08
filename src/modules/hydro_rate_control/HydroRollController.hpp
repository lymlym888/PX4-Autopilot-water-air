/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

enum class HydroRollControllerId : uint8_t {
	Legacy = 0,
	Cadrc = 1,
	Ladrc = 2,
	MceoAdrc = 3,
	AmceoAdrc = 4,
	Nadrc = 5,
};

struct HydroRollControllerInput {
	float phi;                 ///< measured roll angle [rad]
	float p;                   ///< measured body roll rate [rad/s]
	float command_roll;        ///< raw latched roll-angle command [rad]
	float phi_ref;             ///< generated roll reference [rad]
	float phi_ref_dot;         ///< generated roll-reference rate [rad/s]
	float phi_ref_ddot;        ///< generated roll-reference acceleration [rad/s^2]
	float dt_measured;         ///< measured gyro/control interval [s]
	float dt_used;             ///< validated interval used by the new controller [s]
	float u_applied_prev;      ///< previous common-limited roll command [normalized]
	uint64_t timestamp_sample; ///< gyro/control sample time [us]
	float u_max;               ///< active common magnitude limit [normalized], for actuator-aware scheduling
};

struct HydroRollControllerOutput {
	float u_requested; ///< normalized roll command before the common limiter
	bool valid;        ///< true only when u_requested is finite and usable
	bool reset_event;  ///< true on the first update following controller reset
};

/** Shared physical/control parameters for normalized-command roll controllers. */
struct HydroRollControllerConfig {
	float b0;                 ///< nominal normalized-command-to-roll-acceleration effectiveness [(rad/s^2)/normalized]
	float a_phi;              ///< nominal roll damping coefficient [1/s]
	float k_phi1;             ///< roll-angle feedback coefficient [1/s^2]
	float k_phi2;             ///< roll-rate feedback coefficient [1/s]
	float observer_bandwidth; ///< controller-specific observer bandwidth [rad/s]
};

struct HydroRollLimiterResult {
	float u_applied;       ///< final common-limited roll command [normalized]
	bool valid;
	bool magnitude_limited;
	bool rate_limited;
};

class HydroRollLimiter
{
public:
	void reset(float u_applied = 0.f);

	/**
	 * Apply trim, then the discrete rate limit, then the magnitude limit.
	 *
	 * @param u_requested normalized controller request before trim/limiting
	 * @param trim normalized roll trim included in the final applied command
	 * @param dt_used validated controller interval [s]
	 * @param u_max absolute normalized-command limit
	 * @param du_max normalized-command slew limit [1/s]
	 */
	HydroRollLimiterResult update(float u_requested, float trim, float dt_used, float u_max, float du_max);

	float appliedPrevious() const { return _u_applied_prev; }

private:
	float _u_applied_prev{0.f};
};

class HydroRollTimingGuard
{
public:
	/** True only for a finite positive interval not exceeding dt_max [s]. */
	static bool isValid(float dt_used, float dt_max);
};

class ManualRollCommandLatch
{
public:
	void reset(float current_roll);

	/**
	 * Update the new-controller-only manual roll latch.
	 *
	 * Publication gaps retain the last command. An explicit invalid sample,
	 * a non-finite value, a zero/future timestamp, or timeout invalidates the
	 * latch and changes the held command to current_roll.
	 */
	void update(bool applicable, bool new_sample, bool sample_valid, float manual_roll,
		uint8_t data_source, uint64_t sample_timestamp, uint64_t now,
		float roll_scale, float timeout_s, float current_roll);

	float commandRoll() const { return _command_roll; }
	float manualRollRaw() const { return _manual_roll_raw; }
	float ageSeconds() const { return _age_s; }
	uint8_t dataSource() const { return _data_source; }
	bool valid() const { return _valid; }

private:
	float _command_roll{0.f};
	float _manual_roll_raw{0.f};
	float _age_s{0.f};
	uint64_t _sample_timestamp{0};
	uint8_t _data_source{0};
	bool _valid{false};
	bool _applicable{false};
};
