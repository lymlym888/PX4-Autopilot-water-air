#pragma once

/** Roll ADRC-SMC from HAAR_roll_ADRC_SMC: Han tracking differentiator, third-order LESO and sliding-mode feedback. */
class SMC
{
public:
	void setParameters(float control_gain, float td_rate, float td_filter, float observer_bandwidth,
			   float smc_c, float smc_k, float smc_eta, float smc_boundary, float output_scale, float output_limit);

	/** Seed the observer and reference from the next valid measurement and setpoint. */
	void reset();

	/** Inputs in rad, rad, rad/s and s. Returns normalized torque. */
	float update(float angle, float angle_setpoint, float rate, float dt);

private:
	float _control_gain{10.1746f};
	float _td_rate{10.f};
	float _td_filter{0.1f};
	float _observer_bandwidth{10.f};
	float _smc_c{3.f};
	float _smc_k{3.f};
	float _smc_eta{2.f};
	float _smc_boundary{0.05f};
	float _output_scale{1.f};
	float _output_limit{1.f};

	float _td_position{0.f};
	float _td_velocity{0.f};
	float _z1{0.f};
	float _z2{0.f};
	float _z3{0.f};
	float _previous_control{0.f};
	bool _initialized{false};
};
