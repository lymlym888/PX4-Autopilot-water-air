#pragma once

#include <math.h>
#include <stdint.h>

namespace roll_fin_output
{

static constexpr uint8_t kModeFull = 6;
static constexpr uint64_t kSourceMaxAgeUs = 200000;
static constexpr float kTolerance = 1.0e-5f;

enum class Owner : uint8_t {
	Legacy = 0,
	Mpc = 1,
	FallbackBase = 2,
	FallbackOrdinary = 3,
	Inhibited = 4,
	Invalid = 5,
};

enum class Reason : uint8_t {
	None = 0,
	LegacyMode = 1,
	WaitingForCurrentSession = 2,
	SourceStale = 3,
	SourceFromFuture = 4,
	SourceModeMismatch = 5,
	SourceSessionInvalid = 6,
	SourceSequenceInvalid = 7,
	IndexInvalid = 8,
	IndexMismatch = 9,
	CommandInvalid = 10,
	CommandNonfinite = 11,
	CommandInconsistent = 12,
	BoundsInvalid = 13,
	NoFreshFallback = 14,
	TargetClamped = 15,
	ChannelsInhibited = 16,
	FlightModeMismatch = 17,
};

struct SourceCommand {
	uint64_t authority_epoch{0};
	uint64_t status_timestamp{0};
	uint64_t source_timestamp{0};
	uint64_t mode_entry_timestamp{0};
	uint64_t selector_epoch{0};
	uint32_t sequence{0};
	uint32_t session_id{0};
	uint8_t requested_mode{0};
	uint8_t source_mode{0};
	int8_t left_index{-1};
	int8_t right_index{-1};
	bool command_valid{false};
	bool fallback_valid{false};
	float command_left{0.f};
	float command_right{0.f};
	float fallback_left{0.f};
	float fallback_right{0.f};
	float requested_diff{0.f};
	float implemented_diff{0.f};
	uint8_t fallback_reason{0};
};

struct Input {
	uint64_t authority_epoch{0}; // nav_state_timestamp for a STAB-only session
	bool require_current_legacy{false}; // Never reuse an MPC cache outside the eligible flight mode.
	uint64_t now{0};
	uint8_t requested_mode{0};
	int left_index{-1};
	int right_index{-1};
	int servo_count{0};
	bool left_channel_enabled{true};
	bool right_channel_enabled{true};
	bool legacy_source_current{true};
	float ordinary_left{0.f};
	float ordinary_right{0.f};
	float legacy_hydro_left{0.f};
	float legacy_hydro_right{0.f};
	float left_min{-1.f};
	float left_max{1.f};
	float right_min{-1.f};
	float right_max{1.f};
	SourceCommand source{};
};

struct State {
	uint64_t previous_authority_epoch{0};
	bool initialized{false};
	uint8_t previous_mode{0};
	uint64_t mode_entry_timestamp{0};
	uint64_t mode_transition_timestamp{0};
	uint32_t accepted_session_id{0};
	uint64_t accepted_mode_entry_timestamp{0};
	uint64_t observed_producer_entry{0};
	uint64_t retired_producer_entry{0};
	uint32_t last_sequence{0};
	uint64_t last_source_timestamp{0};
	bool waiting_for_legacy_source{false};
	Owner last_owner{Owner::Invalid};
	bool fallback_active{false};
	uint32_t authority_transition_count{0};
	uint32_t fallback_event_count{0};
	uint64_t last_event_timestamp{0};
	uint8_t last_event_reason{0};
	uint8_t last_upstream_fallback_reason{0};
	float last_final_diff{0.f};
};

struct Result {
	Owner owner{Owner::Invalid};
	Reason reason{Reason::None};
	float final_left{0.f};
	float final_right{0.f};
	float final_diff{0.f};
	float takeover_jump{0.f};
	bool target_clamped{false};
	bool source_fresh{false};
	bool source_session_valid{false};
	bool source_command_valid{false};
	bool source_fallback_valid{false};
};

inline bool finite_value(float value)
{
	return __builtin_isfinite(value);
}

inline float clamp(float value, float minimum, float maximum)
{
	return value < minimum ? minimum : (value > maximum ? maximum : value);
}

inline float minimum(float lhs, float rhs)
{
	return lhs < rhs ? lhs : rhs;
}

inline float maximum(float lhs, float rhs)
{
	return lhs > rhs ? lhs : rhs;
}

inline bool valid_bounds(const Input &input)
{
	return finite_value(input.left_min) && finite_value(input.left_max) && finite_value(input.right_min) &&
	       finite_value(input.right_max) && input.left_min <= input.left_max && input.right_min <= input.right_max;
}

inline bool valid_indices(const Input &input)
{
	return input.left_index >= 0 && input.right_index >= 0 && input.left_index < input.servo_count &&
	       input.right_index < input.servo_count && input.left_index != input.right_index;
}

inline void update_events(const Input &input, State &state, Result &result, bool entered_mode6)
{
	result.final_diff = result.final_left - result.final_right;
	result.takeover_jump = entered_mode6 ? result.final_diff - state.last_final_diff : 0.f;

	if (result.owner != state.last_owner) {
		state.authority_transition_count++;
		state.last_event_timestamp = input.now;
		state.last_event_reason = static_cast<uint8_t>(result.reason);
	}

	const bool fallback_now = input.requested_mode == kModeFull && result.owner != Owner::Mpc;

	if (fallback_now && (!state.fallback_active ||
	    state.last_upstream_fallback_reason != input.source.fallback_reason)) {
		if (!state.fallback_active) {
			state.fallback_event_count++;
		}

		state.last_event_timestamp = input.now;
		state.last_event_reason = static_cast<uint8_t>(result.reason);
		state.last_upstream_fallback_reason = input.source.fallback_reason;
	}

	state.fallback_active = fallback_now;
	state.last_owner = result.owner;
	state.last_final_diff = result.final_diff;
}

inline Result select(const Input &input, State &state)
{
	Result result{};
	bool entered_mode6 = false;

	if (!state.initialized) {
		state.initialized = true;
		state.previous_mode = input.requested_mode;
		state.previous_authority_epoch = input.authority_epoch;
		state.mode_transition_timestamp = input.now;

		if (input.requested_mode == kModeFull) {
			state.mode_entry_timestamp = input.now;
			entered_mode6 = true;
		}

	} else if (input.requested_mode != state.previous_mode ||
		   (input.requested_mode == kModeFull && input.authority_epoch != state.previous_authority_epoch)) {
		const bool left_mode6 = state.previous_mode == kModeFull;

		if (left_mode6 && state.observed_producer_entry > state.retired_producer_entry) {
			state.retired_producer_entry = state.observed_producer_entry;
		}

		state.observed_producer_entry = 0;
		state.previous_mode = input.requested_mode;
		state.previous_authority_epoch = input.authority_epoch;
		state.mode_transition_timestamp = input.now;
		state.accepted_session_id = 0;
		state.accepted_mode_entry_timestamp = 0;
		state.last_sequence = 0;
		state.last_source_timestamp = 0;

		if (input.requested_mode == kModeFull) {
			state.mode_entry_timestamp = input.now;
			entered_mode6 = true;
			state.waiting_for_legacy_source = false;

		} else {
			state.mode_entry_timestamp = 0;
			state.waiting_for_legacy_source = left_mode6;
		}
	}

	if (!valid_indices(input) || !valid_bounds(input)) {
		result.owner = Owner::Invalid;
		result.reason = valid_indices(input) ? Reason::BoundsInvalid : Reason::IndexInvalid;
		update_events(input, state, result, entered_mode6);
		return result;
	}

	if (input.requested_mode != kModeFull) {
		if (state.waiting_for_legacy_source && input.legacy_source_current) {
			state.waiting_for_legacy_source = false;
		}

		const bool suppress_stale_hydro = state.waiting_for_legacy_source ||
			(input.require_current_legacy && !input.legacy_source_current);
		const float hydro_left = suppress_stale_hydro ? 0.f : input.legacy_hydro_left;
		const float hydro_right = suppress_stale_hydro ? 0.f : input.legacy_hydro_right;
		result.final_left = clamp(input.ordinary_left + hydro_left, input.left_min, input.left_max);
		result.final_right = clamp(input.ordinary_right + hydro_right, input.right_min, input.right_max);
		const bool channels_enabled = input.left_channel_enabled && input.right_channel_enabled;
		result.owner = channels_enabled ? Owner::Legacy : Owner::Inhibited;
		result.reason = channels_enabled ?
			(suppress_stale_hydro ? Reason::WaitingForCurrentSession : Reason::LegacyMode) :
			Reason::ChannelsInhibited;

		result.final_left = input.left_channel_enabled ? result.final_left : 0.f;
		result.final_right = input.right_channel_enabled ? result.final_right : 0.f;

		update_events(input, state, result, entered_mode6);
		return result;
	}

	const SourceCommand &source = input.source;
	const bool timestamps_nonzero = source.status_timestamp != 0 && source.source_timestamp != 0 &&
					 source.mode_entry_timestamp != 0;
	const bool timestamps_ordered = timestamps_nonzero && source.mode_entry_timestamp <= source.source_timestamp &&
					 source.source_timestamp <= source.status_timestamp;
	const bool source_not_future = timestamps_ordered && source.status_timestamp <= input.now;
	result.source_fresh = source_not_future &&
		(input.now - source.status_timestamp <= kSourceMaxAgeUs) &&
		(input.now - source.source_timestamp <= kSourceMaxAgeUs);

	// Retire even a pending, not-yet-acknowledged producer session when leaving Mode 6.
	if (result.source_fresh && source.requested_mode == kModeFull && source.source_mode == kModeFull &&
	    source.session_id != 0 && source.mode_entry_timestamp > state.observed_producer_entry) {
		state.observed_producer_entry = source.mode_entry_timestamp;
	}

	Reason envelope_reason = Reason::None;

	if (!timestamps_nonzero) {
		envelope_reason = Reason::SourceStale;

	} else if (!timestamps_ordered || !source_not_future) {
		envelope_reason = Reason::SourceFromFuture;

	} else if (!result.source_fresh) {
		envelope_reason = Reason::SourceStale;

	} else if (source.requested_mode != kModeFull || source.source_mode != kModeFull) {
		envelope_reason = Reason::SourceModeMismatch;

	} else if (source.authority_epoch != input.authority_epoch) {
		envelope_reason = Reason::FlightModeMismatch;

	} else if (source.selector_epoch == 0 || source.selector_epoch != state.mode_entry_timestamp ||
		   source.source_timestamp < state.mode_entry_timestamp) {
		// Only commands computed after acknowledging this selector entry may take over.
		envelope_reason = Reason::WaitingForCurrentSession;

	} else if (source.session_id == 0 || source.sequence == 0 ||
		   source.mode_entry_timestamp <= state.retired_producer_entry) {
		envelope_reason = Reason::SourceSessionInvalid;

	} else if (source.left_index != input.left_index || source.right_index != input.right_index) {
		envelope_reason = Reason::IndexMismatch;

	} else {
			if (state.accepted_session_id == 0) {
				state.accepted_session_id = source.session_id;
				state.accepted_mode_entry_timestamp = source.mode_entry_timestamp;
			}

			result.source_session_valid = source.session_id == state.accepted_session_id &&
						      source.mode_entry_timestamp == state.accepted_mode_entry_timestamp;

		if (!result.source_session_valid) {
			envelope_reason = Reason::SourceSessionInvalid;

		} else if (source.sequence < state.last_sequence || source.source_timestamp < state.last_source_timestamp) {
			envelope_reason = Reason::SourceSequenceInvalid;

		} else {
			state.last_sequence = source.sequence;
			state.last_source_timestamp = source.source_timestamp;
		}
	}

	if (envelope_reason == Reason::None && source.command_valid) {
		result.source_command_valid = finite_value(source.command_left) && finite_value(source.command_right) &&
			finite_value(source.requested_diff) && finite_value(source.implemented_diff);

		if (!result.source_command_valid) {
			envelope_reason = Reason::CommandNonfinite;

		} else if (fabsf((source.command_left - source.command_right) - source.implemented_diff) > kTolerance) {
			envelope_reason = Reason::CommandInconsistent;
		}

	} else if (envelope_reason == Reason::None) {
		envelope_reason = Reason::CommandInvalid;
	}

	if (envelope_reason == Reason::None) {
		const float common = 0.5f * (source.command_left + source.command_right);
		const float diff_min = maximum(2.f * (input.left_min - common),
						 2.f * (common - input.right_max));
		const float diff_max = minimum(2.f * (input.left_max - common),
						 2.f * (common - input.right_min));

		if (!finite_value(diff_min) || !finite_value(diff_max) || diff_min > diff_max) {
			envelope_reason = Reason::BoundsInvalid;

		} else {
			const float final_diff = clamp(source.implemented_diff, diff_min, diff_max);
			result.final_left = common + 0.5f * final_diff;
			result.final_right = common - 0.5f * final_diff;
			result.target_clamped = fabsf(final_diff - source.implemented_diff) > kTolerance;
			const bool channels_enabled = input.left_channel_enabled && input.right_channel_enabled;
			result.owner = channels_enabled ? Owner::Mpc : Owner::Inhibited;
			result.reason = channels_enabled ?
				(result.target_clamped ? Reason::TargetClamped : Reason::None) :
				Reason::ChannelsInhibited;

			result.final_left = input.left_channel_enabled ? result.final_left : 0.f;
			result.final_right = input.right_channel_enabled ? result.final_right : 0.f;

			update_events(input, state, result, entered_mode6);
			return result;
		}
	}

	const bool status_fresh = source.status_timestamp > 0 && source.status_timestamp <= input.now &&
		(input.now - source.status_timestamp <= kSourceMaxAgeUs);
	const bool fallback_indices_match = source.left_index == input.left_index && source.right_index == input.right_index;
	result.source_fallback_valid = status_fresh && fallback_indices_match &&
		source.authority_epoch == input.authority_epoch && source.fallback_valid &&
		finite_value(source.fallback_left) && finite_value(source.fallback_right) && finite_value(input.ordinary_left) &&
		finite_value(input.ordinary_right);

	if (result.source_fallback_valid) {
		result.final_left = clamp(input.ordinary_left + source.fallback_left, input.left_min, input.left_max);
		result.final_right = clamp(input.ordinary_right + source.fallback_right, input.right_min, input.right_max);
		result.owner = Owner::FallbackBase;
		result.reason = envelope_reason;

	} else {
		result.final_left = clamp(input.ordinary_left, input.left_min, input.left_max);
		result.final_right = clamp(input.ordinary_right, input.right_min, input.right_max);
		result.owner = Owner::FallbackOrdinary;
		result.reason = envelope_reason == Reason::None ? Reason::NoFreshFallback : envelope_reason;
	}

	if (!input.left_channel_enabled || !input.right_channel_enabled) {
		result.final_left = input.left_channel_enabled ? result.final_left : 0.f;
		result.final_right = input.right_channel_enabled ? result.final_right : 0.f;
		result.owner = Owner::Inhibited;
		result.reason = Reason::ChannelsInhibited;
	}

	update_events(input, state, result, entered_mode6);
	return result;
}

inline bool apply_selected_pair(const Input &input, const Result &result, float *controls, int control_count)
{
	if (controls == nullptr || control_count <= 0 || input.left_index < 0 || input.right_index < 0 ||
	    input.left_index >= control_count || input.right_index >= control_count || input.left_index == input.right_index) {
		return false;
	}

	controls[input.left_index] = result.final_left;
	controls[input.right_index] = result.final_right;
	return true;
}

} // namespace roll_fin_output
