#pragma once

#include <stdint.h>

namespace fixed_mpc_shadow {

// Commander publishes vehicle_status every 500 ms and on mode changes.
static constexpr uint64_t kFlightStatusMaxAgeUs = 1500000;
static constexpr uint8_t kStabilizedNavState = 15;

struct FlightModeSelection {
	uint8_t configured_mode{0};
	uint8_t effective_mode{0};
	uint8_t nav_state{255};
	uint64_t nav_epoch{0};
	bool status_valid{false};
	bool mode6_eligible{false};
};

inline FlightModeSelection select_flight_mode(uint8_t configured_mode, uint8_t nav_state,
		uint64_t status_timestamp, uint64_t nav_epoch, uint64_t now)
{
	FlightModeSelection out{};
	out.configured_mode = configured_mode;
	out.effective_mode = configured_mode;
	out.nav_state = nav_state;
	out.nav_epoch = nav_epoch;
	out.status_valid = status_timestamp != 0 && status_timestamp <= now && nav_epoch != 0 &&
			nav_epoch <= status_timestamp && now - status_timestamp <= kFlightStatusMaxAgeUs;
	out.mode6_eligible = configured_mode == 6 && out.status_valid && nav_state == kStabilizedNavState;
	if (configured_mode == 6 && !out.mode6_eligible) {
		out.effective_mode = 1; // Preserve the existing non-MPC flight-mode output path.
	}
	return out;
}

inline bool flight_source_matches(const FlightModeSelection &local, uint8_t source_nav_state, uint64_t source_nav_epoch)
{
	return local.status_valid && local.nav_state == source_nav_state && local.nav_epoch == source_nav_epoch;
}

} // namespace fixed_mpc_shadow
