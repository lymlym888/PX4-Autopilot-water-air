#pragma once

namespace hydro_control
{

static constexpr float kStabilizedRollReferenceLimitRad = 15.f * 0.017453292519943295f;

inline float limit_stabilized_roll_reference(float requested_rad, bool stabilized)
{
	// Clip the already-mapped angle, not the stick gain. Leave invalid data for
	// the existing validity checks, and leave every non-STAB mode unchanged.
	if (!stabilized || !__builtin_isfinite(requested_rad)) { return requested_rad; }
	const float limit = kStabilizedRollReferenceLimitRad;
	return requested_rad < -limit ? -limit : (requested_rad > limit ? limit : requested_rad);
}

} // namespace hydro_control
