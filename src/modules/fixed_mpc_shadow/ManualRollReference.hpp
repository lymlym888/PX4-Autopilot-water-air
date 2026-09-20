#pragma once

#include "FixedLinearMPC.hpp"
#include <lib/hydro_control/StabilizedRollReference.hpp>
#include <math.h>
#include <stdint.h>

namespace fixed_mpc_shadow
{

enum class RollReferenceStatus : uint8_t {
	Unused = 0,
	Ready = 1,
	Missing = 2,
	Invalid = 3,
	ZeroTime = 4,
	Future = 5,
	Stale = 6,
	Nonfinite = 7,
	OutOfRange = 8,
	InvalidConfig = 9,
	TimeOrder = 10,
	InvalidSource = 11,
};

struct ManualRollSample {
	uint64_t timestamp{0};
	uint64_t timestamp_sample{0};
	float roll{0.f};
	uint8_t source{0};
	bool copied{false};
	bool valid{false};
};

struct RollReference {
	float roll_rad{0.f};
	RollReferenceStatus status{RollReferenceStatus::Unused};
	bool valid() const { return status == RollReferenceStatus::Ready; }
};

inline RollReference manual_roll_reference(uint64_t now, const ManualRollSample &sample,
		float maximum_roll_deg, float timeout_s)
{
	RollReference result{};
	if (!sample.copied) { result.status = RollReferenceStatus::Missing; }
	else if (!sample.valid) { result.status = RollReferenceStatus::Invalid; }
	else if (!__builtin_isfinite(maximum_roll_deg) || maximum_roll_deg < 0.f || maximum_roll_deg > 80.f ||
		 !__builtin_isfinite(timeout_s) || timeout_s <= 0.f || timeout_s > 35.f) {
		result.status = RollReferenceStatus::InvalidConfig;
	} else if (sample.timestamp == 0 || sample.timestamp_sample == 0) {
		result.status = RollReferenceStatus::ZeroTime;
	} else if (sample.timestamp > now || sample.timestamp_sample > now) {
		result.status = RollReferenceStatus::Future;
	} else if (sample.timestamp_sample > sample.timestamp) {
		result.status = RollReferenceStatus::TimeOrder;
	} else if (now - sample.timestamp >= static_cast<uint64_t>(timeout_s * 1.e6f) ||
		   now - sample.timestamp_sample >= static_cast<uint64_t>(timeout_s * 1.e6f)) {
		// Match COM_RC_LOSS_T's strict age check, including a republished old sample.
		result.status = RollReferenceStatus::Stale;
	} else if (!__builtin_isfinite(sample.roll)) {
		result.status = RollReferenceStatus::Nonfinite;
	} else if (fabsf(sample.roll) > 1.f) {
		result.status = RollReferenceStatus::OutOfRange;
	} else if (sample.source < 1 || sample.source > 7) {
		result.status = RollReferenceStatus::InvalidSource;
	} else {
		// Mode 6 is STAB-only. Use the same post-mapping cap as onboard CADRC.
		result.roll_rad = hydro_control::limit_stabilized_roll_reference(
			sample.roll * (maximum_roll_deg * 0.017453292519943295f), true);
		result.status = RollReferenceStatus::Ready;
	}
	return result;
}

inline bool apply_roll_reference(uint8_t mode, const RollReference &reference, MpcState &state)
{
	if (mode != 6) { return true; }
	if (!reference.valid()) { return false; }
	// Constant angle reference across this prediction horizon, p_ref = 0.
	// Do not differentiate the stick or modify physical-angle safety diagnostics.
	state.phi_rad -= reference.roll_rad;
	return true;
}

} // namespace fixed_mpc_shadow
