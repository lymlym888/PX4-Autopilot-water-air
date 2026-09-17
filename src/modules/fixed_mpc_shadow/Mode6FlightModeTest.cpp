#include "Mode6FlightMode.hpp"
#include "Mode6Session.hpp"
#include "StaticFullTrim.hpp"
#include "FixedLinearMPC.hpp"
#include "../control_allocator/RollFinOutputSelector.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

using namespace fixed_mpc_shadow;
namespace rf = roll_fin_output;

static rf::Input input_for(const FlightModeSelection &flight, uint64_t now)
{
	rf::Input in{};
	in.now = now;
	in.requested_mode = flight.effective_mode;
	in.authority_epoch = flight.mode6_eligible ? flight.nav_epoch : 0;
	in.require_current_legacy = flight.configured_mode == 6;
	in.left_index = 4; in.right_index = 3; in.servo_count = 8;
	in.ordinary_left = .12f; in.ordinary_right = -.08f;
	in.legacy_hydro_left = .15f; in.legacy_hydro_right = -.05f;
	in.legacy_source_current = true;
	return in;
}

static void bind_source(rf::Input &in, rf::State &state, uint32_t session, uint64_t entry)
{
	auto &s = in.source;
	s.status_timestamp = in.now; s.source_timestamp = in.now;
	s.mode_entry_timestamp = entry; s.selector_epoch = state.mode_entry_timestamp;
	s.authority_epoch = in.authority_epoch;
	s.sequence = 1; s.session_id = session;
	s.requested_mode = 6; s.source_mode = 6;
	s.left_index = 4; s.right_index = 3;
	s.command_valid = true; s.fallback_valid = true;
	s.command_left = .2f; s.command_right = -.1f;
	s.fallback_left = .15f; s.fallback_right = -.05f;
	s.requested_diff = .3f; s.implemented_diff = .3f;
}

int main()
{
	for (uint8_t mode = 0; mode <= 6; ++mode) {
		for (uint8_t nav = 0; nav < 32; ++nav) {
			const auto f = select_flight_mode(mode, nav, 1000000, 900000, 1000000);
			assert(f.mode6_eligible == (mode == 6 && nav == kStabilizedNavState));
			assert(f.effective_mode == ((mode == 6 && nav != kStabilizedNavState) ? 1 : mode));
			if (f.effective_mode == 6) { continue; }
			auto in = input_for(f, 1000000);
			rf::State state{};
			const auto out = rf::select(in, state);
			assert(out.owner == rf::Owner::Legacy);
			assert(std::fabs(out.final_left - .27f) < 1.e-6f);
			assert(std::fabs(out.final_right + .13f) < 1.e-6f);
		}
	}
	for (const auto f : {
		select_flight_mode(6, 15, 0, 0, 1000000),
		select_flight_mode(6, 15, 1000001, 900000, 1000000),
		select_flight_mode(6, 15, 1000000, 1000001, 1000000),
		select_flight_mode(6, 15, 1000000, 900000, 2500001)}) {
		assert(!f.status_valid && !f.mode6_eligible && f.effective_mode == 1);
	}
	std::puts("all_nav_states_legacy_modes_and_invalid_status,PASS");

	uint64_t now = 2000000;
	auto flight = select_flight_mode(6, 15, now, now - 100, now);
	auto in = input_for(flight, now);
	rf::State selector{};
	assert(rf::select(in, selector).owner != rf::Owner::Mpc);
	bind_source(in, selector, 1, now);
	assert(rf::select(in, selector).owner == rf::Owner::Mpc);
	const auto old_source = in.source;

	// Consumer sees MANUAL before the producer/allocator: no stale hydro may be added.
	now += 10000;
	flight = select_flight_mode(6, 0, now, now, now);
	in = input_for(flight, now);
	in.legacy_source_current = false;
	in.legacy_hydro_left = old_source.command_left;
	in.legacy_hydro_right = old_source.command_right;
	in.source = old_source;
	auto out = rf::select(in, selector);
	assert(out.owner != rf::Owner::Mpc && out.reason == rf::Reason::WaitingForCurrentSession);
	assert(out.final_left == in.ordinary_left && out.final_right == in.ordinary_right);
	in.legacy_source_current = true;
	in.legacy_hydro_left = .15f; in.legacy_hydro_right = -.05f;
	out = rf::select(in, selector);
	assert(out.owner == rf::Owner::Legacy && std::fabs(out.final_left - .27f) < 1.e-6f);
	std::puts("stab_to_manual_suppresses_cached_mpc_then_restores_legacy,PASS");

	// First observation outside STAB must also reject cached MPC, not just explicit exits.
	rf::State fresh_selector{};
	in.legacy_source_current = false;
	assert(rf::select(in, fresh_selector).final_left == in.ordinary_left);

	now += 10000;
	flight = select_flight_mode(6, 15, now, now, now);
	in = input_for(flight, now);
	in.source = old_source;
	assert(rf::select(in, selector).owner != rf::Owner::Mpc);
	bind_source(in, selector, 2, now);
	assert(rf::select(in, selector).owner == rf::Owner::Mpc);
	const uint64_t previous_selector_epoch = selector.mode_entry_timestamp;
	const uint64_t previous_producer_entry = in.source.mode_entry_timestamp;

	// Both tasks miss a brief STAB -> HOLD -> STAB; nav generation still forces re-entry.
	now += 10000;
	flight = select_flight_mode(6, 15, now, now, now);
	in.now = now; in.authority_epoch = flight.nav_epoch;
	assert(rf::select(in, selector).owner != rf::Owner::Mpc);
	assert(selector.mode_entry_timestamp != previous_selector_epoch);
	// Forging a new handshake with the retired producer is still rejected.
	in.source.authority_epoch = flight.nav_epoch;
	in.source.selector_epoch = selector.mode_entry_timestamp;
	in.source.status_timestamp = now; in.source.source_timestamp = now;
	assert(in.source.mode_entry_timestamp == previous_producer_entry);
	assert(rf::select(in, selector).owner != rf::Owner::Mpc);
	bind_source(in, selector, 3, now);
	assert(rf::select(in, selector).owner == rf::Owner::Mpc);
	std::puts("reentry_and_missed_hold_transition_require_new_session,PASS");

	// Producer sees a new STAB generation first. A still-publishing old consumer must
	// not bind that new producer permanently to the previous selector handshake.
	Mode6SessionBinding binding{};
	const auto producer_flight = select_flight_mode(6, 15, 4000000, 3900000, 4000000);
	const bool old_request = flight_source_matches(producer_flight, 15, 3800000);
	binding.update(4000000, 4001000, old_request ? 6 : 1, 4000500, 3800100);
	assert(binding.selector_epoch == 0);
	const bool new_request = flight_source_matches(producer_flight, 15, 3900000);
	binding.update(4000000, 4002000, new_request ? 6 : 1, 4001500, 4001200);
	assert(binding.selector_epoch == 4001200);
	std::puts("producer_first_does_not_bind_old_flight_handshake,PASS");

	assert(!flight_source_matches(flight, 0, flight.nav_epoch));
	assert(!flight_source_matches(flight, 15, flight.nav_epoch - 1));
	assert(flight_source_matches(flight, 15, flight.nav_epoch));
	in.source.command_valid = false;
	out = rf::select(in, selector);
	assert(out.owner == rf::Owner::FallbackBase && selector.fallback_event_count > 0);
	in.source.authority_epoch--;
	out = rf::select(in, selector);
	assert(out.owner == rf::Owner::FallbackOrdinary && !out.source_fallback_valid);
	std::puts("hard_fault_fallback_cannot_use_previous_flight_epoch,PASS");

	// No confirmation flag exists: still reject invalid configured trim, not silently zero it.
	StaticFullTrim trim;
	trim.begin(.1f); trim.update(true, true, 2.f, 2.f, 0.f, .98f);
	assert(trim.valid && trim.trim == .1f);
	trim.begin(NAN); trim.update(true, true, 2.f, 2.f, 0.f, .98f);
	assert(!trim.valid);
	trim.begin(1.f); trim.update(true, true, .5f, .5f, 0.f, .98f);
	assert(!trim.valid);
	std::puts("automatic_static_trim_validation_without_enable_flag,PASS");

	// Sustained healthy STAB session: actual solver and selector, no commissioning timer.
	rf::State continuous{};
	FixedLinearMPC solver;
	FullMpcInput model{0.f, .5f, .5f, 0.f, .98f, -1};
	MpcState state{.01f, 0.f, 0.f};
	const uint64_t epoch = 5000000;
	for (unsigned step = 0; step < 3000; ++step) {
		now = epoch + step * 20000;
		flight = select_flight_mode(6, 15, epoch + (step / 25) * 500000, epoch, now);
		assert(flight.mode6_eligible);
		in = input_for(flight, now);
		if (step == 0) { rf::select(in, continuous); }
		bind_source(in, continuous, 1, epoch);
		const auto result = solver.solve_full_roll(state, model);
		assert(result.proposal.solver_success);
		in.source.sequence = step + 1;
		in.source.command_left = .5f * result.command.target;
		in.source.command_right = -.5f * result.command.target;
		in.source.requested_diff = result.command.requested_target;
		in.source.implemented_diff = result.command.target;
		assert(rf::select(in, continuous).owner == rf::Owner::Mpc);
		state.previous_delta = result.command.model_input;
	}
	in.left_channel_enabled = false;
	assert(rf::select(in, continuous).owner == rf::Owner::Inhibited);
	std::puts("60_second_synthetic_continuity_and_channel_inhibit,PASS; not board validation");
}
