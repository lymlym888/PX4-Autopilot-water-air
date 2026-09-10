#pragma once

#include <stdint.h>

namespace fixed_mpc_shadow
{

// Bind a producer session to one explicit selector request, never to clock ordering
// between independently scheduled mode observers. A new epoch needs a new session.
struct Mode6SessionBinding {
	uint64_t producer_entry{0};
	uint64_t selector_epoch{0};

	uint64_t update(uint64_t entry, uint64_t now, uint8_t requested_mode,
			uint64_t request_timestamp, uint64_t request_epoch)
	{
		if (entry != producer_entry || entry == 0) {
			producer_entry = entry;
			selector_epoch = 0;
		}

		if (entry != 0 && selector_epoch == 0 && requested_mode == 6 && request_epoch != 0 &&
		    request_epoch <= request_timestamp && request_timestamp >= entry && request_timestamp <= now &&
		    now - request_timestamp <= 200000) {
			selector_epoch = request_epoch;
		}

		return selector_epoch;
	}
};

} // namespace fixed_mpc_shadow
