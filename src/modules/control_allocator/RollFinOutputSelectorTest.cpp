#include "RollFinOutputSelector.hpp"
#include "../fixed_mpc_shadow/FixedMpcInterface.hpp"
#include "../fixed_mpc_shadow/Mode6Session.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

using roll_fin_output::Input;
using roll_fin_output::Owner;
using roll_fin_output::Reason;
using roll_fin_output::Result;
using roll_fin_output::State;

namespace
{

int failures = 0;

bool near(float lhs, float rhs, float tolerance = 1.0e-6f)
{
	return std::fabs(lhs - rhs) <= tolerance;
}

void check(bool condition, const char *test, const char *detail)
{
	std::printf("%s,%s,%s\n", test, condition ? "PASS" : "FAIL", detail);

	if (!condition) {
		++failures;
	}
}

Input base_input(uint64_t now, uint8_t mode)
{
	Input input{};
	input.now = now;
	input.requested_mode = mode;
	input.left_index = 2;
	input.right_index = 3;
	input.servo_count = 8;
	input.ordinary_left = 0.2f;
	input.ordinary_right = -0.1f;
	input.legacy_hydro_left = 0.15f;
	input.legacy_hydro_right = -0.05f;
	input.left_min = -1.f;
	input.left_max = 1.f;
	input.right_min = -1.f;
	input.right_max = 1.f;
	input.legacy_source_current = true;
	return input;
}

void set_healthy_source(Input &input, uint32_t session, uint32_t sequence, uint64_t entry,
			float common, float diff)
{
	input.source.status_timestamp = input.now;
	input.source.source_timestamp = input.now;
	input.source.mode_entry_timestamp = entry;
	input.source.selector_epoch = entry;
	input.source.sequence = sequence;
	input.source.session_id = session;
	input.source.requested_mode = roll_fin_output::kModeFull;
	input.source.source_mode = roll_fin_output::kModeFull;
	input.source.left_index = static_cast<int8_t>(input.left_index);
	input.source.right_index = static_cast<int8_t>(input.right_index);
	input.source.command_valid = true;
	input.source.fallback_valid = true;
	input.source.command_left = common + 0.5f * diff;
	input.source.command_right = common - 0.5f * diff;
	input.source.fallback_left = 0.12f;
	input.source.fallback_right = -0.08f;
	input.source.requested_diff = diff;
	input.source.implemented_diff = diff;
}

void test_legacy_modes()
{
	for (uint8_t mode = 1; mode <= 5; ++mode) {
		State state{};
		Input input = base_input(1000000 + mode, mode);
		const Result result = roll_fin_output::select(input, state);
		check(result.owner == Owner::Legacy && near(result.final_left, 0.35f) && near(result.final_right, -0.15f),
		      "legacy_modes", "Mode 1-5 retain ordinary plus hydro composition");
	}

	State boundary_state{};
	Input boundary = base_input(1100000, 1);
	boundary.ordinary_left = 0.9f;
	boundary.legacy_hydro_left = 0.4f;
	boundary.ordinary_right = -0.9f;
	boundary.legacy_hydro_right = -0.4f;
	const Result limited = roll_fin_output::select(boundary, boundary_state);
	check(near(limited.final_left, 1.f) && near(limited.final_right, -1.f),
	      "legacy_boundaries", "Legacy final clamp remains normalized [-1,1]");
}

void test_mode6_ownership_and_common_mode()
{
	State state{};
	Input input = base_input(2000000, 6);
	set_healthy_source(input, 11, 1, input.now, 0.2f, 0.3f);
	Result first = roll_fin_output::select(input, state);
	check(first.owner == Owner::Mpc && near(first.final_left, 0.35f) && near(first.final_right, 0.05f) &&
	      near(first.final_diff, 0.3f), "mode6_common", "MPC differential reconstructed around hydro common mode");

	input.now += 20000;
	input.source.status_timestamp = input.now;
	input.source.source_timestamp = input.now;
	input.source.sequence++;
	input.ordinary_left = -0.8f;
	input.ordinary_right = 0.8f;
	Result changed_ordinary = roll_fin_output::select(input, state);
	check(changed_ordinary.owner == Owner::Mpc && near(changed_ordinary.final_diff, first.final_diff) &&
	      near(changed_ordinary.final_left, first.final_left) && near(changed_ordinary.final_right, first.final_right),
	      "mode6_ownership", "Ordinary antisymmetric roll does not alter healthy Mode 6 output");
}

void test_signed_commands_and_nonroll_channels()
{
	State state{};
	Input input = base_input(3000000, 6);
	float controls[8] {0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f, 0.08f};
	const float untouched_0 = controls[0];
	const float untouched_7 = controls[7];
	controls[6] = std::numeric_limits<float>::quiet_NaN();
	const float commands[] {0.25f, -0.25f, 0.f, 0.1f, -0.1f};

	for (unsigned i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
		set_healthy_source(input, 21, i + 1, 3000000, 0.f, commands[i]);
		Result result = roll_fin_output::select(input, state);
		const bool applied = roll_fin_output::apply_selected_pair(input, result, controls, 8);
		check(applied && result.owner == Owner::Mpc && near(controls[2] - controls[3], commands[i]),
		      "signed_commands", "Positive, negative, zero, and cross-zero differential preserved");
		input.now += 20000;
	}

	check(near(controls[0], untouched_0) && near(controls[7], untouched_7) && std::isnan(controls[6]),
	      "nonroll_channels", "Applying selected pair preserves finite and unused-NaN unselected channels");
}

void test_asymmetric_limits()
{
	check(near(fixed_mpc_shadow::constrain_full_candidate(0.7f, 0.5f), 0.5f) &&
	      near(fixed_mpc_shadow::constrain_full_candidate(-0.7f, 0.5f), -0.5f) &&
	      near(fixed_mpc_shadow::constrain_full_candidate(0.2f, 0.5f), 0.2f),
	      "fcand_limit", "Shared firmware FCAND helper clamps positive/negative and preserves in-range targets");

	State state{};
	Input input = base_input(4000000, 6);
	input.left_min = -0.8f;
	input.left_max = 0.7f;
	input.right_min = -0.6f;
	input.right_max = 0.9f;
	set_healthy_source(input, 31, 1, input.now, 0.3f, 0.9f);
	input.source.requested_diff = 1.2f;
	Result positive = roll_fin_output::select(input, state);
	check(positive.owner == Owner::Mpc && positive.target_clamped && near(positive.final_diff, 0.8f) &&
	      near(positive.final_left, 0.7f) && near(positive.final_right, -0.1f),
	      "asymmetric_limit_positive", "Positive target constrained in feasible differential interval");

	input.now += 20000;
	set_healthy_source(input, 31, 2, 4000000, 0.3f, -1.5f);
	input.source.requested_diff = -1.8f;
	Result negative = roll_fin_output::select(input, state);
	check(negative.owner == Owner::Mpc && negative.target_clamped && near(negative.final_diff, -1.2f) &&
	      near(negative.final_left, -0.3f) && near(negative.final_right, 0.9f),
	      "asymmetric_limit_negative", "Negative target constrained in feasible differential interval");
}

void test_invalid_and_async_sources()
{
	{
		State state{};
		Input input = base_input(5000000, 6);
		set_healthy_source(input, 41, 1, input.now, 0.f, 0.2f);
		input.source.source_timestamp = 0;
		Result result = roll_fin_output::select(input, state);
		check(result.owner == Owner::FallbackBase && result.reason == Reason::SourceStale,
		      "missing_timestamp", "Missing MPC timestamp uses fresh same-frame base fallback");
	}

	{
		State state{};
		Input input = base_input(5100000, 6);
		set_healthy_source(input, 42, 1, input.now, 0.f, 0.2f);
		input.source.status_timestamp = input.now + 1;
		input.source.source_timestamp = input.now + 1;
		Result result = roll_fin_output::select(input, state);
		check(result.owner == Owner::FallbackOrdinary && result.reason == Reason::SourceFromFuture,
		      "future_timestamp", "Future status rejected and not treated as fresh fallback");
	}

	{
		State state{};
		Input input = base_input(5200000, 6);
		set_healthy_source(input, 43, 1, input.now, 0.f, 0.2f);
		input.source.source_timestamp = input.now - roll_fin_output::kSourceMaxAgeUs - 1;
		input.source.mode_entry_timestamp = input.source.source_timestamp;
		Result result = roll_fin_output::select(input, state);
		check(result.owner == Owner::FallbackBase && result.reason == Reason::SourceStale,
		      "stale_command", "Stale MPC command cannot be held; fresh base fallback remains available");
	}

	{
		State state{};
		Input input = base_input(5300000, 6);
		set_healthy_source(input, 44, 1, input.now, 0.f, 0.2f);
		input.source.command_left = std::numeric_limits<float>::quiet_NaN();
		Result result = roll_fin_output::select(input, state);
		check(result.owner == Owner::FallbackBase && result.reason == Reason::CommandNonfinite,
		      "nonfinite_command", "NaN Mode 6 command rejected with traceable base fallback");
	}

	{
		State state{};
		Input input = base_input(5400000, 6);
		set_healthy_source(input, 45, 2, input.now, 0.f, 0.2f);
		Result accepted = roll_fin_output::select(input, state);
		input.now += 20000;
		set_healthy_source(input, 46, 1, 5400000, 0.f, 0.2f);
		Result wrong_session = roll_fin_output::select(input, state);
		check(accepted.owner == Owner::Mpc && wrong_session.owner == Owner::FallbackBase &&
		      wrong_session.reason == Reason::SourceSessionInvalid,
		      "session_change", "Different producer session cannot replace active session silently");
	}

	{
		State state{};
		Input input = base_input(5500000, 6);
		set_healthy_source(input, 47, 3, input.now, 0.f, 0.2f);
		Result accepted = roll_fin_output::select(input, state);
		input.now += 20000;
		set_healthy_source(input, 47, 2, 5500000, 0.f, 0.2f);
		Result reordered = roll_fin_output::select(input, state);
		check(accepted.owner == Owner::Mpc && reordered.owner == Owner::FallbackBase &&
		      reordered.reason == Reason::SourceSequenceInvalid,
		      "sequence_reorder", "Out-of-order sequence rejected");
	}

	{
		State state{};
		Input input = base_input(5600000, 6);
		input.right_index = input.left_index;
		Result result = roll_fin_output::select(input, state);
		check(result.owner == Owner::Invalid && result.reason == Reason::IndexInvalid,
		      "index_conflict", "Conflicting roll-fin indexes reject ownership selection");
	}
}

void test_mode_transitions()
{
	State state{};
	Input input = base_input(6000000, 1);
	Result legacy = roll_fin_output::select(input, state);
	check(legacy.owner == Owner::Legacy, "transition_legacy", "Starts on legacy owner");

	input.now += 20000;
	input.requested_mode = 6;
	set_healthy_source(input, 51, 1, 5900000, 0.f, 0.3f);
	Result old_entry = roll_fin_output::select(input, state);
	check(old_entry.owner == Owner::FallbackBase && old_entry.reason == Reason::WaitingForCurrentSession,
	      "transition_1_to_6", "Pre-entry Mode 6 command cannot take ownership");

	input.now += 20000;
	set_healthy_source(input, 52, 1, 6020000, 0.f, 0.3f);
	Result current = roll_fin_output::select(input, state);
	check(current.owner == Owner::Mpc, "transition_current_session", "Current Mode 6 session takes ownership");

	input.now += 20000;
	input.requested_mode = 1;
	input.legacy_source_current = false;
	Result exit_stale = roll_fin_output::select(input, state);
	check(exit_stale.owner == Owner::Legacy && exit_stale.reason == Reason::WaitingForCurrentSession &&
	      near(exit_stale.final_left, input.ordinary_left),
	      "transition_6_to_1", "Stale Mode 6 hydro cache suppressed immediately on exit");

	input.now += 20000;
	input.legacy_source_current = true;
	Result exit_current = roll_fin_output::select(input, state);
	check(exit_current.owner == Owner::Legacy && near(exit_current.final_left,
	      input.ordinary_left + input.legacy_hydro_left),
	      "transition_legacy_restored", "Current legacy source restores original composition");

	input.now += 20000;
	input.requested_mode = 6;
	set_healthy_source(input, 52, 2, 6020000, 0.f, 0.3f);
	Result reused = roll_fin_output::select(input, state);
	check(reused.owner == Owner::FallbackBase && reused.reason == Reason::WaitingForCurrentSession,
	      "transition_6_1_6_old", "Previous Mode 6 session cannot be reused after rapid re-entry");

	input.now += 20000;
	set_healthy_source(input, 53, 1, 6100000, 0.f, 0.3f);
	Result reentered = roll_fin_output::select(input, state);
	check(reentered.owner == Owner::Mpc, "transition_6_1_6_new", "New session takes ownership after re-entry");
}

void test_inhibition_and_continuity()
{
	State state{};
	Input input = base_input(7000000, 6);
	set_healthy_source(input, 61, 1, input.now, 0.f, 0.2f);
	input.left_channel_enabled = false;
	Result inhibited = roll_fin_output::select(input, state);
	check(inhibited.owner == Owner::Inhibited && inhibited.reason == Reason::ChannelsInhibited &&
	      near(inhibited.final_left, 0.f), "channel_inhibit", "Existing channel inhibit overrides Mode 6 selector");

	bool unlimited_timeout = false;
	for (int i = 0; i <= 1000; ++i) {
		unlimited_timeout = unlimited_timeout ||
			fixed_mpc_shadow::full_duration_timed_out(0.f, 0.02f * static_cast<float>(i));
	}

	check(!unlimited_timeout, "duration_unlimited", "FULL_DUR=0 does not timeout over synthetic 20 s run");
	check(!fixed_mpc_shadow::full_duration_timed_out(3.f, 3.f) &&
	      fixed_mpc_shadow::full_duration_timed_out(3.f, 3.01f),
	      "duration_finite", "Positive FULL_DUR retains strict finite timeout behavior");

	State event_state{};
	Input healthy = base_input(8000000, 6);
	set_healthy_source(healthy, 71, 1, healthy.now, 0.f, 0.2f);
	Result applied = roll_fin_output::select(healthy, event_state);
	healthy.now += roll_fin_output::kSourceMaxAgeUs + 1;
	Result fault = roll_fin_output::select(healthy, event_state);
	check(applied.owner == Owner::Mpc && fault.owner != Owner::Mpc && event_state.fallback_event_count == 1 &&
	      event_state.last_event_timestamp == healthy.now,
	      "fault_event_latch", "A real stale-data interruption increments and latches fallback event diagnostics");
}

void test_session_handshake()
{
	const int offsets[] {-400, 400};
	for (int producer_offset : offsets) {
		State state{};
		fixed_mpc_shadow::Mode6SessionBinding binding{};
		Input input = base_input(166170000, 1);
		roll_fin_output::select(input, state);
		input.now = 166190890;
		input.requested_mode = 6;
		const uint64_t entry = input.now + producer_offset;
		roll_fin_output::select(input, state);
		const uint64_t epoch = state.mode_entry_timestamp;
		input.now += 20000;
		const uint64_t ack = binding.update(entry, input.now, 6, input.now - 1000, epoch);
		bool healthy = ack == epoch;

		for (int sample = 0; sample < 1001; ++sample) {
			set_healthy_source(input, 81, sample + 1, entry, 0.1f, -0.29846f);
			input.source.selector_epoch = ack;
			const Result result = roll_fin_output::select(input, state);
			healthy = healthy && result.owner == Owner::Mpc && result.source_session_valid &&
				  near(result.final_diff, -0.29846f);
			input.now += 20000;
		}

		check(healthy, producer_offset < 0 ? "log855_producer_first" : "consumer_first",
		      "Actual producer binding and selector stay MPC-owned for 20 seconds after handshake");
	}

	State state{};
	fixed_mpc_shadow::Mode6SessionBinding binding{};
	Input input = base_input(9000000, 6);
	set_healthy_source(input, 91, 1, 8999600, 0.f, 0.2f);
	input.source.selector_epoch = 0;
	Result pending = roll_fin_output::select(input, state);
	check(pending.owner == Owner::FallbackBase && pending.reason == Reason::WaitingForCurrentSession,
	      "handshake_pending", "Fresh command with no acknowledgement cannot take ownership");
	input.now += 20000;
	const uint64_t ack = binding.update(8999600, input.now, 6, 9000000, state.mode_entry_timestamp);
	set_healthy_source(input, 91, 2, 8999600, 0.f, 0.2f);
	input.source.selector_epoch = ack;
	Result accepted = roll_fin_output::select(input, state);
	input.now += 5000;
	Result cached = roll_fin_output::select(input, state);
	check(accepted.owner == Owner::Mpc && cached.owner == Owner::Mpc,
	      "handshake_async_cache", "A fresh acknowledged command works across unequal publication rates");
	input.now += 5000;
	input.source.selector_epoch = ack - 1;
	Result wrong = roll_fin_output::select(input, state);
	check(wrong.owner == Owner::FallbackBase && wrong.reason == Reason::WaitingForCurrentSession,
	      "handshake_wrong_epoch", "A fresh command from another selector entry is rejected");
	input.source.selector_epoch = ack;
	input.source.source_timestamp = ack - 1;
	Result pre_request = roll_fin_output::select(input, state);
	check(pre_request.owner != Owner::Mpc, "handshake_pre_request_command",
	      "Echo cannot authorize a command timestamp predating the request");

	input.now += 5000;
	input.requested_mode = 1;
	input.legacy_source_current = false;
	Result exit = roll_fin_output::select(input, state);
	check(exit.owner == Owner::Legacy && near(exit.final_left, input.ordinary_left),
	      "handshake_exit", "Exit suppresses cached MPC hydro without waiting for producer");
	input.now += 5000;
	input.requested_mode = 6;
	set_healthy_source(input, 91, 3, 8999600, 0.f, 0.2f);
	input.source.selector_epoch = ack;
	roll_fin_output::select(input, state);
	input.now += 20000;
	const uint64_t attempted_rebind = binding.update(8999600, input.now, 6, input.now, state.mode_entry_timestamp);
	input.source.selector_epoch = attempted_rebind;
	input.source.status_timestamp = input.now;
	input.source.source_timestamp = input.now;
	Result missed_exit = roll_fin_output::select(input, state);
	check(attempted_rebind == ack && missed_exit.owner == Owner::FallbackBase,
	      "handshake_missed_producer_exit", "A producer that missed the exit cannot relabel the old trim session");
	binding.update(0, input.now, 1, input.now, 0);
	input.now += 20000;
	const uint64_t new_entry = input.now;
	input.now += 20000;
	set_healthy_source(input, 92, 1, new_entry, 0.f, -0.2f);
	input.source.selector_epoch = binding.update(new_entry, input.now, 6, input.now, state.mode_entry_timestamp);
	Result reentered = roll_fin_output::select(input, state);
	check(reentered.owner == Owner::Mpc && near(reentered.final_diff, -0.2f),
	      "handshake_new_session", "New observed producer session can acknowledge the current selector entry");
	input.now += roll_fin_output::kSourceMaxAgeUs + 1;
	Result stopped = roll_fin_output::select(input, state);
	check(stopped.owner == Owner::FallbackOrdinary && !stopped.source_fallback_valid,
	      "handshake_source_stopped", "Acknowledgement does not bypass timeout or invent fresh base fallback");

	for (int failure = 0; failure < 6; ++failure) {
		fixed_mpc_shadow::Mode6SessionBinding invalid{};
		const uint64_t entry = failure == 0 ? 0 : 10000000;
		const uint64_t now = 10020000;
		const uint8_t mode = failure == 1 ? 1 : 6;
		const uint64_t request = failure == 2 ? now + 1 : (failure == 3 ? entry - 1 : now);
		const uint64_t epoch = failure == 4 ? 0 : (failure == 5 ? request + 1 : 10000000);
		check(invalid.update(entry, now, mode, request, epoch) == 0,
		      "invalid_handshake_request", "Missing entry/wrong mode/future/pre-entry/zero/misordered request rejected");
	}
	fixed_mpc_shadow::Mode6SessionBinding stale{};
	check(stale.update(10000000, 10300000, 6, 10000000, 10000000) == 0,
	      "stale_handshake_request", "Stale selector request is not acknowledged");

	State pending_state{};
	Input unbound = base_input(11000000, 6);
	set_healthy_source(unbound, 101, 1, 10999600, 0.f, 0.2f);
	unbound.source.selector_epoch = 0;
	roll_fin_output::select(unbound, pending_state);
	unbound.now += 5000;
	unbound.requested_mode = 1;
	roll_fin_output::select(unbound, pending_state);
	unbound.now += 5000;
	unbound.requested_mode = 6;
	roll_fin_output::select(unbound, pending_state);
	unbound.now += 20000;
	set_healthy_source(unbound, 101, 2, 10999600, 0.f, 0.2f);
	unbound.source.selector_epoch = pending_state.mode_entry_timestamp;
	Result retired = roll_fin_output::select(unbound, pending_state);
	check(retired.owner == Owner::FallbackBase && retired.reason == Reason::SourceSessionInvalid,
	      "handshake_retired_pending_session", "Even an unbound prior producer session cannot be relabeled on re-entry");
}

void test_scheduled_handshake()
{
	const uint64_t consumer_periods[] {4000, 20000, 50000};
	for (uint64_t period : consumer_periods) {
		bool all_healthy = true;
		unsigned healthy_samples = 0;
		for (uint64_t phase = 0; phase < 20000; phase += 1000) {
			for (int first = 0; first < 2; ++first) {
				State state{};
				fixed_mpc_shadow::Mode6SessionBinding binding{};
				Input input = base_input(10000000, 1);
				roll_fin_output::SourceCommand pending{};
				uint64_t deliver_at = 0, entry = 0, request_time = 0, request_epoch = 0;
				uint8_t previous_mode = 1, request_mode = 1;
				uint32_t session = 0, sequence = 0;
				for (uint64_t t = 0; t <= 32000000; t += 1000) {
					const uint64_t now = 10000000 + t;
					const uint8_t mode = (t < 1000000 || (t >= 24500000 && t < 25500000)) ? 1 : 6;
					if (deliver_at != 0 && now >= deliver_at) {
						input.source = pending;
						input.source.status_timestamp = now;
						deliver_at = 0;
					}
					for (int task = 0; task < 2; ++task) {
						if ((task + first) % 2 == 0 && t % 20000 == phase) {
							if (mode != previous_mode) {
								entry = mode == 6 ? now : 0;
								if (mode == 6) { ++session; }
								sequence = 0;
								previous_mode = mode;
							}
							const uint64_t ack = binding.update(entry, now, request_mode, request_time, request_epoch);
							Input proposal = base_input(now, mode);
							set_healthy_source(proposal, session, ++sequence, entry, 0.1f, -0.3f);
							proposal.source.requested_mode = mode;
							proposal.source.source_mode = mode;
							proposal.source.selector_epoch = ack;
							pending = proposal.source;
							deliver_at = now + 3000;
						} else if ((task + first) % 2 == 1 && t % period == 0) {
							input.now = now;
							input.requested_mode = mode;
							input.legacy_source_current = input.source.source_mode == mode;
							Result result = roll_fin_output::select(input, state);
							request_time = now;
							request_epoch = state.mode_entry_timestamp;
							request_mode = mode;
							if (mode == 6 && ((t >= 1500000 && t < 24500000) || t >= 26000000)) {
								++healthy_samples;
								all_healthy = all_healthy && result.owner == Owner::Mpc &&
									      near(result.final_diff, -0.3f);
							} else if (mode == 1) {
								all_healthy = all_healthy && result.owner == Owner::Legacy;
							}
						}
					}
				}
			}
		}
		check(all_healthy && healthy_samples > 0, "scheduled_handshake",
		      "20 producer phases and both task orders; 3ms transport; 1-6-1-6 and uninterrupted healthy ownership");
	}
}

} // namespace

int main()
{
	std::printf("test,status,detail\n");
	test_legacy_modes();
	test_mode6_ownership_and_common_mode();
	test_signed_commands_and_nonroll_channels();
	test_asymmetric_limits();
	test_invalid_and_async_sources();
	test_mode_transitions();
	test_inhibition_and_continuity();
	test_session_handshake();
	test_scheduled_handshake();
	return failures == 0 ? 0 : 1;
}
