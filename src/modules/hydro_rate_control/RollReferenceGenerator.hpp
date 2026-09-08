/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

class RollReferenceGenerator
{
public:
	/** Initialize phi_ref from measured roll [rad], with zero rate. */
	void reset(float command_roll, float current_roll);

	/**
	 * Advance a second-order reference model with explicit forward Euler.
	 * omega_r is natural frequency [rad/s], zeta_r is dimensionless, and dt
	 * is the explicitly supplied integration interval [s].
	 */
	bool update(float command_roll, float omega_r, float zeta_r, float dt);

	float phiRef() const { return _phi_ref; }             ///< [rad]
	float phiRefDot() const { return _phi_ref_dot; }       ///< [rad/s]
	float phiRefDdot() const { return _phi_ref_ddot; }     ///< [rad/s^2]
	bool valid() const { return _valid; }

private:
	float _phi_ref{0.f};
	float _phi_ref_dot{0.f};
	float _phi_ref_ddot{0.f};
	bool _valid{false};
};

