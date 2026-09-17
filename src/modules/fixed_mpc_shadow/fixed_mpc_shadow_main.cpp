#include <drivers/drv_hrt.h>
#include <inttypes.h>
#include <lib/matrix/matrix/math.hpp>
#include <parameters/param.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <stdio.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/fixed_mpc_shadow_status.h>
#include <uORB/topics/fixed_mpc_tiny_correction.h>
#include <uORB/topics/fixed_mpc_allocator_status.h>
#include <uORB/topics/fixed_mpc_final_output_status.h>
#include <uORB/topics/fixed_mpc_input_status.h>

#include "FixedLinearMPC.hpp"
#include "FixedMPCDiagnostics.hpp"
#include "FixedMPCParams.hpp"
#include "FixedMpcInterface.hpp"
#include "Mode6Session.hpp"
#include "Mode6FlightMode.hpp"
#include "StaticFullTrim.hpp"
#include "ManualRollReference.hpp"

using namespace time_literals;

namespace fixed_mpc_shadow
{

static_assert(kStabilizedNavState == vehicle_status_s::NAVIGATION_STATE_STAB,
	      "Mode 6 eligibility must use the current STAB enum");

static constexpr uint32_t kIntervalMs = 20;
static constexpr uint32_t kIntervalUs = kIntervalMs * 1000;
static constexpr float kRadToDeg = 57.29577951308232f;
static constexpr float kDegToRad = 0.017453292519943295f;
static constexpr float kSensorAgeLimitMs = 200.f;
static constexpr float kBaseCommandAgeLimitMs = 200.f;
static constexpr float kDiagnosticTolerance = 1.0e-6f;
static constexpr const char *kWorkQueueName = "lp_default";

struct RuntimeConfig {
	uint8_t mode{static_cast<uint8_t>(FmpcMode::SHADOW_ONLY)};
	bool act_en{false};
	bool sign_ok{false};
	float act_lim{0.f};
	bool bnd_en{false};
	float bnd_lim{0.02f};
	float sat_margin{0.02f};
	float full_lim{0.05f};
	FullMpcTuning full_tuning{};
	bool tuning_sanitized{false};
	uint8_t full_src{2};
	bool full_trim_en{true};
	float full_trim_a{0.f};
	uint64_t nav_epoch{0};
	float g_basemin{0.10f};
	float g_lim{0.05f};
	float g_lowlim{0.02f};
	float roll_max_deg{8.f};
	float rate_max_deg_s{35.f};
	float cand_max{0.30f};
	float full_cand_max{0.50f};
	bool act_lim_clamped{false};
	bool bnd_lim_clamped{false};
	bool full_lim_clamped{false};
	float manual_roll_max_deg{NAN};
	float manual_timeout_s{NAN};
};

class FixedMpcShadow final : public ModuleBase<FixedMpcShadow>, public px4::ScheduledWorkItem
{
public:
	FixedMpcShadow() :
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
	{
		_mpc.init();

		for (int i = 0; i < 2; ++i) {
			char buffer[17];
			snprintf(buffer, sizeof(buffer), "HY_SV%u_IDX", i);
			_hy_sv_idx_param_handles[i] = param_find(buffer);
		}

		_fmpc_mode_handle = param_find("FMPC_MODE");
		_fmpc_act_en_handle = param_find("FMPC_ACT_EN");
		_fmpc_sign_ok_handle = param_find("FMPC_SIGN_OK");
		_fmpc_act_lim_handle = param_find("FMPC_ACT_LIM");
		_fmpc_bnd_en_handle = param_find("FMPC_BND_EN");
		_fmpc_bnd_lim_handle = param_find("FMPC_BND_LIM");
		_fmpc_sat_margin_handle = param_find("FMPC_SAT_MARGIN");
		_fmpc_full_lim_handle = param_find("FMPC_FULL_LIM");
		_fmpc_full_slew_handle = param_find("FMPC_FULL_SLEW");
		_fmpc_error_gain_handle = param_find("FMPC_ERR_GAIN");
		_fmpc_full_src_handle = param_find("FMPC_FULL_SRC");
		_fmpc_full_trim_en_handle = param_find("FMPC_FTRIM_EN");
		_fmpc_trim_a_handle = param_find("FMPC_TRIM_A");
		_fmpc_g_basemin_handle = param_find("FMPC_G_BASEMIN");
		_fmpc_g_lim_handle = param_find("FMPC_G_LIM");
		_fmpc_g_lowlim_handle = param_find("FMPC_G_LOWLIM");
		_fmpc_roll_max_handle = param_find("FMPC_ROLL_MAX");
		_fmpc_rate_max_handle = param_find("FMPC_RATE_MAX");
		_fmpc_cand_max_handle = param_find("FMPC_CAND_MAX");
		_fmpc_full_cand_max_handle = param_find("FMPC_FCAND_MAX");
		_hy_roll_max_handle = param_find("HY_ROLL_MAX");
		_com_rc_loss_t_handle = param_find("COM_RC_LOSS_T");

		_last_diag_snapshot = make_base_snapshot(hrt_absolute_time(), 0.f, RuntimeConfig{});
	}

	~FixedMpcShadow() override = default;

	static int task_spawn(int argc, char *argv[])
	{
		(void)argc;
		(void)argv;

		auto *instance = new FixedMpcShadow();

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return -1;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->start();
		return 0;
	}

	static int custom_command(int argc, char *argv[])
	{
		if (argc > 0 && !strcmp(argv[0], "help")) {
			return print_usage();
		}

		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason != nullptr) {
			PX4_INFO("%s", reason);
		}

		PX4_INFO_RAW("fixed_mpc_shadow commands:\n");
		PX4_INFO_RAW("  start\n");
		PX4_INFO_RAW("  stop\n");
		PX4_INFO_RAW("  status\n");
		PX4_INFO_RAW("  help\n");
		PX4_INFO_RAW("\n");
		PX4_INFO_RAW("modes: 0=OFF 1=SHADOW_ONLY 2=GATED_SHADOW 3=ACTIVE_TINY 4=FALLBACK 5=ACTIVE_BOUNDED_MPC 6=FULL_ROLL_MPC_TEST\n");
		PX4_INFO_RAW("default remains safe: FMPC_ACT_EN=0, FMPC_SIGN_OK=0\n");
		PX4_INFO_RAW("hydro_allocator stays the only hydro_servos publisher\n");
		return 0;
	}

	static bool running()
	{
		return is_running() && (get_instance() != nullptr);
	}

	static FixedMpcShadow *instance()
	{
		return get_instance();
	}

	void start()
	{
		ScheduleOnInterval(kIntervalUs);
		ScheduleNow();
	}

	int print_status() override
	{
		const Diagnostics s = read_snapshot();
		PX4_INFO("fixed_mpc_shadow: running");
		PX4_INFO("work_queue: %s", kWorkQueueName);
		PX4_INFO("interval_ms: %lu", (unsigned long)kIntervalMs);
		PX4_INFO("loop_count: %" PRIu64, s.loop_count);
		PX4_INFO("last_run_elapsed_ms: %.3f", (double)s.last_run_elapsed_ms);
		PX4_INFO("last_solve_time_us: %" PRIu32, s.solve_time_us);
		PX4_INFO("solver_success: %s", s.solver_success ? "true" : "false");
		PX4_INFO("roll_deg: %.3f", (double)s.roll_deg);
		PX4_INFO("roll_rate_deg_s: %.3f", (double)s.roll_rate_deg_s);
		PX4_INFO("base_command_valid: %s", s.base_command_valid ? "true" : "false");
		PX4_INFO("base_command_source: %s", base_command_source_string(s.base_command_source));
		PX4_INFO("delta_base_A: %.6f", (double)s.delta_base_A);
		PX4_INFO("delta_mpc_shadow_correction_A: %.6f", (double)s.delta_mpc_shadow_correction_A);
		PX4_INFO("delta_candidate_shadow_A_raw: %.6f", (double)s.delta_candidate_shadow_A_raw);
		PX4_INFO("fmpc_mode: %s", mode_string(s.fmpc_mode));
		PX4_INFO("fmpc_act_en: %s", s.fmpc_act_en ? "true" : "false");
		PX4_INFO("fmpc_sign_ok: %s", s.fmpc_sign_ok ? "true" : "false");
		PX4_INFO("fmpc_act_lim: %.6f", (double)s.fmpc_act_lim);
		PX4_INFO("gate_base_large: %s", s.gate_base_large ? "true" : "false");
		PX4_INFO("gate_direction_compatible: %s", s.gate_direction_compatible ? "true" : "false");
		PX4_INFO("gate_reason: %s", gate_reason_string(s.gate_reason));
		PX4_INFO("delta_gate_corr_A: %.6f", (double)s.delta_gate_corr_A);
		PX4_INFO("active_precheck_pass: %s", s.active_precheck_pass ? "true" : "false");
		PX4_INFO("active_corr_A: %.6f", (double)s.active_corr_A);
		PX4_INFO("active_candidate_A: %.6f", (double)s.active_candidate_A);
		PX4_INFO("active_applied: %s", s.active_applied ? "true" : "false");
		PX4_INFO("active_fallback: %s", s.active_fallback ? "true" : "false");
		PX4_INFO("active_fallback_reason: %s", fallback_reason_string(static_cast<FallbackReason>(s.active_fallback_reason)));
		PX4_INFO("allocator_applied_corr_A: %.6f", (double)s.allocator_applied_corr_A);
		PX4_INFO("allocator_applied_corr_B: %.6f", (double)s.allocator_applied_corr_B);
		PX4_INFO("active_output_allowed: %s", s.active_output_allowed ? "true" : "false");
		PX4_INFO("actuator_sign_confirmed: %s", s.actuator_sign_confirmed ? "true" : "false");
		PX4_INFO("safety_gate_pass: %s", s.safety_gate_pass ? "true" : "false");
		PX4_INFO("fallback_reason: %s", fallback_reason_string(static_cast<FallbackReason>(s.fallback_reason)));
		return 0;
	}

	void request_stop() override
	{
		ModuleBase<FixedMpcShadow>::request_stop();
		ScheduleNow();
	}

private:
	void Run() override
	{
		if (should_exit()) {
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		const hrt_abstime now = hrt_absolute_time();
		const float last_run_elapsed_ms = (_last_run_timestamp == 0) ? 0.f : (now - _last_run_timestamp) / 1000.f;
		_last_run_timestamp = now;
		_loop_count++;

		RuntimeConfig cfg = read_runtime_config();
		_vehicle_status_sub.update(&_vehicle_status);
		_flight_selection = select_flight_mode(cfg.mode, _vehicle_status.nav_state,
			_vehicle_status.timestamp, _vehicle_status.nav_state_timestamp, hrt_absolute_time());
		cfg.mode = _flight_selection.effective_mode;
		cfg.nav_epoch = _flight_selection.nav_epoch;
		_input_status = {};
		_input_status.timestamp = now;
		_input_status.mode = cfg.mode;
		_input_status.configured_mode = _flight_selection.configured_mode;
		_input_status.nav_state = _flight_selection.nav_state;
		_input_status.nav_state_timestamp = _flight_selection.nav_epoch;
		_input_status.flight_status_valid = _flight_selection.status_valid;
		_input_status.mode6_eligible = _flight_selection.mode6_eligible;
		_input_status.full_src = cfg.full_src;
		_input_status.full_slew_rate = cfg.full_tuning.slew_rate;
		_input_status.full_error_gain = cfg.full_tuning.error_gain;
		_input_status.full_max_step = cfg.full_tuning.maximum_step();
		_input_status.tuning_sanitized = cfg.tuning_sanitized;
		_input_status.full_model_variant = cfg.mode == static_cast<uint8_t>(FmpcMode::FULL_ROLL_MPC_TEST) ?
						  fixed_mpc_input_status_s::FULL_NOMINAL_TRIM : 0;
		_full_result = {};
		MpcState state{};
		Diagnostics snapshot = make_base_snapshot(now, last_run_elapsed_ms, cfg);
		update_base_command_snapshot(snapshot);
		update_full_trim_latch(snapshot, cfg);
		_input_status.trim_candidate_a = _static_trim.candidate;
		_input_status.trim_bound_a = _static_trim.bound;
		_input_status.trim_reserve_a = _static_trim.reserve;
		_input_status.trim_projected = false;
		_input_status.trim_status = static_cast<uint8_t>(_static_trim.status);
		_input_status.trim_valid = cfg.full_trim_en && _static_trim.valid;
		_final_output_sub.update(&_final_output);
		const bool same_flight_request = flight_source_matches(_flight_selection, _final_output.nav_state,
			_final_output.nav_state_timestamp);
		_mode6_binding.update(_mode6_entry_timestamp, now, same_flight_request ? _final_output.effective_mode : 1,
				     _final_output.timestamp, _final_output.selector_epoch);

		if (!update_inputs(state, snapshot)) {
			finalize_mode_logic(snapshot, cfg);
			evaluate_consistency(snapshot, cfg);
			publish_snapshot(snapshot);
			publish_shadow_diagnostics(snapshot);
			publish_tiny_correction(snapshot);
			return;
		}

		const hrt_abstime solve_start = hrt_absolute_time();
		const bool mode_full = cfg.mode == static_cast<uint8_t>(FmpcMode::FULL_ROLL_MPC_TEST);
		MpcResult result{};

		if (mode_full) {
			FullMpcInput input{};
			const bool reference_ok = update_roll_reference(cfg, state);
			if (reference_ok && prepare_full_input(cfg, snapshot, state, input)) {
				_full_result = _mpc.solve_full_roll(state, input, cfg.full_tuning);
				result = _full_result.proposal;
				if (!result.solver_success) { _input_status.rejection_mask |= SolverNonfinite; }
			}
			_input_status.model_previous = state.previous_delta;
			_input_status.model_proposal = result.delta_rate_limited;
			_input_status.model_implemented = _full_result.command.model_input;
			_input_status.requested_correction = _full_result.command.correction;
			_input_status.implemented_target = _full_result.command.target;
			_input_status.model_trim = _full_result.model_trim;
			_input_status.model_correction = _full_result.command.model_input - _full_result.model_trim;
			_input_status.unconstrained_correction = _full_result.unconstrained_correction;
			_input_status.rate_recovery = _full_result.rate_recovery;
			_input_status.slew_limited = _full_result.command.slew_limited;
			_input_status.constraint_conflict = _full_result.command.constraint_conflict;
		} else {
			result = _mpc.solve_lqr_clipping(state);
		}
		snapshot.solve_time_us = hrt_elapsed_time(&solve_start);
		snapshot.solver_success = result.solver_success;
		snapshot.delta_logged = state.previous_delta;
		snapshot.delta_mpc_raw = result.delta_raw;
		snapshot.delta_mpc_amplitude_limited = result.delta_amplitude_limited;
		const float amplitude_offset = mode_full && result.solver_success ? _full_result.model_trim : 0.f;
		const float amplitude_limit = amplitude_diagnostic_limit(cfg);
		snapshot.delta_mpc_amplitude_saturation_flag = fabsf(result.delta_raw - amplitude_offset) > (amplitude_limit + kDiagnosticTolerance);
		snapshot.delta_mpc_amplitude_saturation_ratio = (amplitude_limit > 0.f) ?
				math::min(fabsf(snapshot.delta_mpc_amplitude_limited - amplitude_offset) / amplitude_limit, 1.0f) : 0.f;
		snapshot.delta_mpc_rate_limit_reference = state.previous_delta;
		snapshot.delta_mpc_rate_limit_delta = result.delta_rate_limited - state.previous_delta;
		const float maximum_step = mode_full ? cfg.full_tuning.maximum_step() : kDDeltaMax;
		snapshot.delta_mpc_rate_limit_max_step = maximum_step;
		snapshot.delta_mpc_rate_limited = result.delta_rate_limited;
		snapshot.delta_mpc_rate_saturation_flag =
			fabsf(snapshot.delta_mpc_rate_limited - snapshot.delta_mpc_amplitude_limited) > kDiagnosticTolerance;
		snapshot.delta_mpc_rate_saturation_ratio = (maximum_step > 0.f) ?
				math::min(fabsf(snapshot.delta_mpc_amplitude_limited - snapshot.delta_mpc_rate_limit_reference) / maximum_step, 1.0f) : 0.f;
		snapshot.delta_mpc_clipped = snapshot.delta_mpc_amplitude_limited;
		snapshot.delta_mpc_saturation_flag = snapshot.delta_mpc_amplitude_saturation_flag;
		snapshot.delta_mpc_saturation_ratio = snapshot.delta_mpc_amplitude_saturation_ratio;
		snapshot.delta_mpc_pre_gate = mode_full ? _full_result.command.correction : snapshot.delta_mpc_rate_limited;
		snapshot.delta_mpc_shadow_correction =
			math::constrain(snapshot.delta_mpc_pre_gate, -snapshot.delta_mpc_shadow_correction_limit,
					snapshot.delta_mpc_shadow_correction_limit);
		snapshot.delta_mpc_shadow_correction_saturation_flag =
			fabsf(snapshot.delta_mpc_pre_gate) > (snapshot.delta_mpc_shadow_correction_limit + kDiagnosticTolerance);
		snapshot.delta_mpc_post_gate = 0.f;
		snapshot.delta_rate_mpc_pre_gate = snapshot.delta_mpc_rate_limit_delta;
		snapshot.delta_rate_mpc_post_gate = 0.f;
		snapshot.safety_gate_pass = false;

		update_candidate_shadow_snapshot(snapshot);
		compute_gated_shadow(snapshot, cfg);
		finalize_mode_logic(snapshot, cfg);
		evaluate_consistency(snapshot, cfg);
		publish_snapshot(snapshot);
		publish_shadow_diagnostics(snapshot);
		publish_tiny_correction(snapshot);

		if (!mode_full) {
			_previous_delta = snapshot.delta_mpc_pre_gate;
		}
	}

	RuntimeConfig read_runtime_config()
	{
		RuntimeConfig cfg{};
		int32_t mode = static_cast<int32_t>(FmpcMode::SHADOW_ONLY);
		int32_t act_en = 0;
		int32_t sign_ok = 0;
		float act_lim = 0.f;
		int32_t bnd_en = 0;
		float bnd_lim = 0.02f;
		float sat_margin = 0.02f;
		float full_lim = 0.05f;
		int32_t full_src = 2;
		int32_t full_trim_en = 1;
		float full_trim_a = 0.f;
		float g_basemin = 0.10f;
		float g_lim = 0.05f;
		float g_lowlim = 0.02f;
		float roll_max = 8.f;
		float rate_max = 35.f;
		float cand_max = 0.30f;
		float full_cand_max = 0.50f;

		if (_fmpc_mode_handle != PARAM_INVALID) { param_get(_fmpc_mode_handle, &mode); }
		if (_fmpc_act_en_handle != PARAM_INVALID) { param_get(_fmpc_act_en_handle, &act_en); }
		if (_fmpc_sign_ok_handle != PARAM_INVALID) { param_get(_fmpc_sign_ok_handle, &sign_ok); }
		if (_fmpc_act_lim_handle != PARAM_INVALID) { param_get(_fmpc_act_lim_handle, &act_lim); }
		if (_fmpc_bnd_en_handle != PARAM_INVALID) { param_get(_fmpc_bnd_en_handle, &bnd_en); }
		if (_fmpc_bnd_lim_handle != PARAM_INVALID) { param_get(_fmpc_bnd_lim_handle, &bnd_lim); }
		if (_fmpc_sat_margin_handle != PARAM_INVALID) { param_get(_fmpc_sat_margin_handle, &sat_margin); }
		if (_fmpc_full_lim_handle != PARAM_INVALID) { param_get(_fmpc_full_lim_handle, &full_lim); }
		if (_fmpc_full_slew_handle != PARAM_INVALID) { param_get(_fmpc_full_slew_handle, &cfg.full_tuning.slew_rate); }
		if (_fmpc_error_gain_handle != PARAM_INVALID) { param_get(_fmpc_error_gain_handle, &cfg.full_tuning.error_gain); }
		cfg.tuning_sanitized = cfg.full_tuning.sanitize();
		if (_fmpc_full_src_handle != PARAM_INVALID) { param_get(_fmpc_full_src_handle, &full_src); }
		if (_fmpc_full_trim_en_handle != PARAM_INVALID) { param_get(_fmpc_full_trim_en_handle, &full_trim_en); }
		if (_fmpc_trim_a_handle != PARAM_INVALID) { param_get(_fmpc_trim_a_handle, &full_trim_a); }
		if (_fmpc_g_basemin_handle != PARAM_INVALID) { param_get(_fmpc_g_basemin_handle, &g_basemin); }
		if (_fmpc_g_lim_handle != PARAM_INVALID) { param_get(_fmpc_g_lim_handle, &g_lim); }
		if (_fmpc_g_lowlim_handle != PARAM_INVALID) { param_get(_fmpc_g_lowlim_handle, &g_lowlim); }
		if (_fmpc_roll_max_handle != PARAM_INVALID) { param_get(_fmpc_roll_max_handle, &roll_max); }
		if (_fmpc_rate_max_handle != PARAM_INVALID) { param_get(_fmpc_rate_max_handle, &rate_max); }
		if (_fmpc_cand_max_handle != PARAM_INVALID) { param_get(_fmpc_cand_max_handle, &cand_max); }
		if (_fmpc_full_cand_max_handle != PARAM_INVALID) { param_get(_fmpc_full_cand_max_handle, &full_cand_max); }
		if (_hy_roll_max_handle != PARAM_INVALID) { param_get(_hy_roll_max_handle, &cfg.manual_roll_max_deg); }
		if (_com_rc_loss_t_handle != PARAM_INVALID) { param_get(_com_rc_loss_t_handle, &cfg.manual_timeout_s); }

		cfg.mode = static_cast<uint8_t>(math::constrain(mode, int32_t{0}, int32_t{6}));
		cfg.act_en = (act_en == 1);
		cfg.sign_ok = (sign_ok == 1);
		cfg.act_lim = math::max(act_lim, 0.f);
		if (cfg.act_lim > kFmpcActLimHardMax) {
			cfg.act_lim = kFmpcActLimHardMax;
			cfg.act_lim_clamped = true;
		}
		cfg.bnd_en = (bnd_en == 1);
		cfg.bnd_lim = math::max(bnd_lim, 0.f);
		if (cfg.bnd_lim > kFmpcBndLimHardMax) {
			cfg.bnd_lim = kFmpcBndLimHardMax;
			cfg.bnd_lim_clamped = true;
		}
		cfg.sat_margin = math::constrain(sat_margin, 0.f, 0.2f);
		cfg.full_lim = math::max(full_lim, 0.f);
		if (cfg.full_lim > kFmpcFullLimHardMax) {
			cfg.full_lim = kFmpcFullLimHardMax;
			cfg.full_lim_clamped = true;
		}
		cfg.full_src = static_cast<uint8_t>(math::constrain(full_src, int32_t{0}, int32_t{2}));
		cfg.full_trim_en = (full_trim_en == 1);
		// Reject invalid static trim at entry; do not silently change a reviewed value.
		cfg.full_trim_a = full_trim_a;
		cfg.g_basemin = math::max(g_basemin, 0.f);
		cfg.g_lim = math::constrain(g_lim, 0.f, kShadowCorrectionLimit);
		cfg.g_lowlim = math::constrain(g_lowlim, 0.f, cfg.g_lim);
		cfg.roll_max_deg = math::max(roll_max, 0.f);
		cfg.rate_max_deg_s = math::max(rate_max, 0.f);
		cfg.cand_max = math::constrain(cand_max, 0.f, kDeltaMax);
		cfg.full_cand_max = math::constrain(full_cand_max, kFmpcFullCandMaxHardMin, kFmpcFullCandMaxHardMax);
		return cfg;
	}

	void update_base_command_snapshot(Diagnostics &snapshot)
	{
		_hydro_servos_sub.update(&_hydro_servos);
		_hydro_torque_setpoint_sub.update(&_hydro_torque_setpoint);
		_fixed_mpc_allocator_status_sub.update(&_fixed_mpc_allocator_status);
		const hrt_abstime now = hrt_absolute_time();
		_input_status.base_checked_at = now;
		_input_status.torque_timestamp = _hydro_torque_setpoint.timestamp;

		snapshot.base_command_source = static_cast<uint8_t>(FmpcBaseSource::NONE);
		snapshot.base_command_age_ms = 1.0e9f;
		snapshot.hydro_torque_setpoint_age_ms = input_age_ms(now, _hydro_torque_setpoint.timestamp);

		if (PX4_ISFINITE(_hydro_torque_setpoint.xyz[0]) &&
		    input_fresh(now, _hydro_torque_setpoint.timestamp, static_cast<uint64_t>(kBaseCommandAgeLimitMs * 1000.f))) {
			snapshot.hydro_torque_setpoint_valid = true;
			snapshot.hydro_torque_roll_cmd = _hydro_torque_setpoint.xyz[0];
			snapshot.base_command_source = static_cast<uint8_t>(FmpcBaseSource::HYDRO_TORQUE_SETPOINT_ONLY);
		}

		if (_fixed_mpc_allocator_status.timestamp > 0) {
			const float age_ms = input_age_ms(now, _fixed_mpc_allocator_status.timestamp);
			if (input_fresh(now, _fixed_mpc_allocator_status.timestamp, static_cast<uint64_t>(kAllocatorStatusMaxAgeMs * 1000.f)) &&
			    _fixed_mpc_allocator_status.base_ok) {
				_input_status.base_timestamp = _fixed_mpc_allocator_status.timestamp;
				snapshot.base_command_age_ms = age_ms;
				snapshot.base_command_source = static_cast<uint8_t>(FmpcBaseSource::ALLOCATOR_STATUS);
				snapshot.left_servo_base = _fixed_mpc_allocator_status.srv_l_b;
				snapshot.right_servo_base = _fixed_mpc_allocator_status.srv_r_b;
				snapshot.delta_base_A = _fixed_mpc_allocator_status.d_base_a;
				snapshot.delta_base_B = _fixed_mpc_allocator_status.d_base_b;
				snapshot.allocator_applied_corr_A = _fixed_mpc_allocator_status.app_ca;
				snapshot.allocator_applied_corr_B = _fixed_mpc_allocator_status.app_cb;
				snapshot.base_command_valid =
					PX4_ISFINITE(snapshot.left_servo_base) &&
					PX4_ISFINITE(snapshot.right_servo_base) &&
					PX4_ISFINITE(snapshot.delta_base_A) &&
					PX4_ISFINITE(snapshot.delta_base_B);
				return;
			}
		}

		int left_servo_index = -1;
		int right_servo_index = -1;

		if (!resolve_hydro_servo_indices(left_servo_index, right_servo_index)) {
			if (_hydro_servos.timestamp > 0) {
				snapshot.base_command_source = static_cast<uint8_t>(FmpcBaseSource::STALE_OR_INVALID);
			}
			snapshot.base_command_valid = false;
			return;
		}

		if (_hydro_servos.timestamp == 0) {
			snapshot.base_command_valid = false;
			return;
		}

		_input_status.base_timestamp = _hydro_servos.timestamp;
		snapshot.base_command_age_ms = input_age_ms(now, _hydro_servos.timestamp);
		snapshot.base_command_source = static_cast<uint8_t>(FmpcBaseSource::STALE_OR_INVALID);
		snapshot.left_servo_base = _hydro_servos.control[left_servo_index];
		snapshot.right_servo_base = _hydro_servos.control[right_servo_index];

		if (!PX4_ISFINITE(snapshot.left_servo_base) || !PX4_ISFINITE(snapshot.right_servo_base)) {
			snapshot.base_command_valid = false;
			return;
		}

		snapshot.delta_base_A = snapshot.left_servo_base - snapshot.right_servo_base;
		snapshot.delta_base_B = snapshot.right_servo_base - snapshot.left_servo_base;

		if (!input_fresh(now, _hydro_servos.timestamp, static_cast<uint64_t>(kBaseCommandAgeLimitMs * 1000.f))) {
			snapshot.base_command_valid = false;
			return;
		}

		snapshot.base_command_source = static_cast<uint8_t>(FmpcBaseSource::HYDRO_SERVOS);
		snapshot.base_command_valid = true;
	}

	bool update_inputs(MpcState &state, Diagnostics &snapshot)
	{
		vehicle_attitude_s attitude{};
		vehicle_angular_velocity_s angular_velocity{};

		const bool attitude_copied = _vehicle_attitude_sub.copy(&attitude);
		const bool rate_copied = _vehicle_angular_velocity_sub.copy(&angular_velocity);
		const hrt_abstime now = hrt_absolute_time();
		_input_status.checked_at = now;
		_input_status.attitude_timestamp = attitude.timestamp;
		_input_status.rate_timestamp = angular_velocity.timestamp;
		snapshot.attitude_age_ms = input_age_ms(now, attitude.timestamp);
		snapshot.angular_velocity_age_ms = input_age_ms(now, angular_velocity.timestamp);
		uint32_t faults = 0;
		if (!attitude_copied) { faults |= AttitudeMissing; }
		if (!rate_copied) { faults |= RateMissing; }
		if (attitude.timestamp == 0) { faults |= AttitudeZeroTime; }
		if (angular_velocity.timestamp == 0) { faults |= RateZeroTime; }
		if (attitude.timestamp > now) { faults |= AttitudeFuture; }
		if (angular_velocity.timestamp > now) { faults |= RateFuture; }
		if (snapshot.attitude_age_ms > kSensorAgeLimitMs) { faults |= AttitudeStale; }
		if (snapshot.angular_velocity_age_ms > kSensorAgeLimitMs) { faults |= RateStale; }

		for (float q_component : attitude.q) {
			if (!PX4_ISFINITE(q_component)) {
				faults |= AttitudeNonfinite;
			}
		}

		if (!PX4_ISFINITE(angular_velocity.xyz[0])) {
			faults |= RateNonfinite;
		}

		_input_status.rejection_mask |= faults;
		snapshot.attitude_valid = !(faults & (AttitudeMissing | AttitudeZeroTime | AttitudeFuture | AttitudeStale | AttitudeNonfinite));
		snapshot.angular_velocity_valid = !(faults & (RateMissing | RateZeroTime | RateFuture | RateStale | RateNonfinite));
		if (faults != 0) {
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::SENSOR_INVALID);
			return false;
		}

		const matrix::Quatf q(attitude.q);
		const matrix::Eulerf euler(q);
		snapshot.phi_rad = euler.phi();
		snapshot.pitch_rad = euler.theta();
		snapshot.yaw_rad = euler.psi();
		snapshot.p_rad_s = angular_velocity.xyz[0];

		if (!PX4_ISFINITE(snapshot.phi_rad) || !PX4_ISFINITE(snapshot.pitch_rad) ||
		    !PX4_ISFINITE(snapshot.yaw_rad) || !PX4_ISFINITE(snapshot.p_rad_s)) {
			_input_status.rejection_mask |= EulerNonfinite;
			snapshot.attitude_valid = false;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::SENSOR_INVALID);
			return false;
		}

		snapshot.attitude_valid = true;
		snapshot.angular_velocity_valid = true;
		snapshot.roll_deg = snapshot.phi_rad * kRadToDeg;
		snapshot.roll_rate_deg_s = snapshot.p_rad_s * kRadToDeg;
		snapshot.pitch_deg = snapshot.pitch_rad * kRadToDeg;
		snapshot.yaw_deg = snapshot.yaw_rad * kRadToDeg;
		state.phi_rad = snapshot.phi_rad;
		state.p_rad_s = snapshot.p_rad_s;
		state.previous_delta = _previous_delta;
		return true;
	}

	bool update_roll_reference(const RuntimeConfig &cfg, MpcState &state)
	{
		manual_control_setpoint_s manual{};
		const bool copied = _manual_control_sub.copy(&manual);
		const hrt_abstime checked_at = hrt_absolute_time();
		const ManualRollSample sample{manual.timestamp, manual.timestamp_sample, manual.roll,
			manual.data_source, copied, manual.valid};
		const RollReference reference = manual_roll_reference(checked_at, sample,
			cfg.manual_roll_max_deg, cfg.manual_timeout_s);
		_input_status.reference_timestamp = manual.timestamp;
		_input_status.reference_sample = manual.timestamp_sample;
		_input_status.reference_checked_at = checked_at;
		_input_status.reference_age_ms = input_age_ms(checked_at, manual.timestamp_sample);
		_input_status.reference_source = manual.data_source;
		_input_status.roll_stick = manual.roll;
		_input_status.roll_ref_deg = reference.roll_rad * kRadToDeg;
		_input_status.reference_status = static_cast<uint8_t>(reference.status);
		_input_status.reference_valid = reference.valid();
		if (!apply_roll_reference(cfg.mode, reference, state)) {
			_input_status.rejection_mask |= ManualReferenceInvalid;
			return false;
		}
		_input_status.roll_error_deg = state.phi_rad * kRadToDeg;
		return true;
	}

	bool prepare_full_input(const RuntimeConfig &cfg, const Diagnostics &snapshot, MpcState &state, FullMpcInput &input)
	{
		int left = -1;
		int right = -1;
		if (!resolve_hydro_servo_indices(left, right) || model_input_sign(left, right) == 0) {
			_input_status.rejection_mask |= MappingInvalid;
			return false;
		}
		input.model_sign = model_input_sign(left, right);
		if (_full_input_entry != _mode6_entry_timestamp) {
			_full_input_entry = _mode6_entry_timestamp;
			_full_input_sign = input.model_sign;
		} else if (_full_input_sign != input.model_sign) {
			_input_status.rejection_mask |= MappingInvalid;
			return false;
		}
		_input_status.model_sign = input.model_sign;
		input.trim = cfg.full_trim_en ? _full_trim_latched_A : 0.f;
		input.correction_limit = cfg.full_src == 2 ? cfg.full_lim : math::min(cfg.full_lim, kShadowCorrectionLimit);
		input.candidate_limit = cfg.full_trim_en ? cfg.full_cand_max : cfg.cand_max;
		input.mean_servo = 0.5f * (snapshot.left_servo_base + snapshot.right_servo_base);
		input.servo_limit = math::max(1.f - cfg.sat_margin, 0.f);
		if (!snapshot.base_command_valid || !snapshot.hydro_torque_setpoint_valid ||
		    (cfg.full_trim_en && !_full_trim_latched_valid) || !input.valid()) {
			_input_status.rejection_mask |= CommandSpaceInvalid;
			return false;
		}

		_final_output_sub.update(&_final_output);
		const hrt_abstime now = hrt_absolute_time();
		_input_status.feedback_checked_at = now;
		_input_status.feedback_timestamp = _final_output.timestamp;
		_input_status.feedback_age_ms = input_age_ms(now, _final_output.timestamp);
		FullMpcFeedback feedback{};
		feedback.timestamp = _final_output.timestamp;
		feedback.entry_timestamp = _final_output.source_mode_entry_timestamp;
		feedback.session = _final_output.source_session_id;
		feedback.mode = _final_output.effective_mode;
		feedback.left_index = _final_output.left_index;
		feedback.right_index = _final_output.right_index;
		feedback.left = _final_output.final_left;
		feedback.right = _final_output.final_right;
		const auto history = full_previous_input(now, _mode6_entry_timestamp, _mode6_session_id,
				left, right, input.trim, feedback, state.previous_delta);
		_input_status.feedback_used = history == FullHistorySource::FinalOutput;
		if (history == FullHistorySource::Invalid) {
			_input_status.rejection_mask |= FeedbackInvalid;
			return false;
		}
		return true;
	}

	void update_candidate_shadow_snapshot(Diagnostics &snapshot) const
	{
		if (!snapshot.base_command_valid) {
			return;
		}

		snapshot.delta_mpc_shadow_correction_A = snapshot.delta_mpc_shadow_correction;
		snapshot.delta_mpc_shadow_correction_B = -snapshot.delta_mpc_shadow_correction_A;
		snapshot.delta_candidate_shadow_A_raw = snapshot.delta_base_A + snapshot.delta_mpc_shadow_correction_A;
		snapshot.delta_candidate_shadow_B_raw = snapshot.delta_base_B + snapshot.delta_mpc_shadow_correction_B;
		snapshot.delta_candidate_shadow_A_limited = math::constrain(snapshot.delta_candidate_shadow_A_raw, -kDeltaMax, kDeltaMax);
		snapshot.delta_candidate_shadow_B_limited = math::constrain(snapshot.delta_candidate_shadow_B_raw, -kDeltaMax, kDeltaMax);
		snapshot.delta_candidate_shadow_A_saturation_flag =
			fabsf(snapshot.delta_candidate_shadow_A_raw) > (kDeltaMax + kDiagnosticTolerance);
		snapshot.delta_candidate_shadow_B_saturation_flag =
			fabsf(snapshot.delta_candidate_shadow_B_raw) > (kDeltaMax + kDiagnosticTolerance);
		snapshot.delta_candidate_shadow_A_saturation_ratio = (kDeltaMax > 0.f) ?
				math::min(fabsf(snapshot.delta_candidate_shadow_A_limited) / kDeltaMax, 1.0f) : 0.f;
		snapshot.delta_candidate_shadow_B_saturation_ratio = (kDeltaMax > 0.f) ?
				math::min(fabsf(snapshot.delta_candidate_shadow_B_limited) / kDeltaMax, 1.0f) : 0.f;
	}

	void update_full_trim_latch(const Diagnostics &snapshot, const RuntimeConfig &cfg)
	{
		const uint8_t full_mode = static_cast<uint8_t>(FmpcMode::FULL_ROLL_MPC_TEST);
		const bool valid_base = snapshot.base_command_valid &&
			PX4_ISFINITE(snapshot.delta_base_A) && PX4_ISFINITE(snapshot.delta_base_B);

		if (cfg.mode == full_mode) {
			if (_previous_mode != full_mode || _mode6_nav_epoch != cfg.nav_epoch) {
				_mode6_nav_epoch = cfg.nav_epoch;
				_full_mode_start_timestamp = 0;
				_full_trim_latched_valid = false;
				_static_trim.begin(cfg.full_trim_a);
				_mode6_entry_timestamp = math::max(snapshot.timestamp_us, cfg.nav_epoch);
				_mode6_command_sequence = 0;
				_mode6_session_id++;

				if (_mode6_session_id == 0) {
					_mode6_session_id = 1;
				}
			}

			_static_trim.update(cfg.full_trim_en, valid_base, amplitude_diagnostic_limit(cfg),
				cfg.full_trim_en ? cfg.full_cand_max : cfg.cand_max,
				0.5f * (snapshot.left_servo_base + snapshot.right_servo_base), math::max(1.f - cfg.sat_margin, 0.f));
			_full_trim_latched_A = _static_trim.trim;
			_full_trim_latched_B = -_static_trim.trim;
			_full_trim_latched_valid = _static_trim.valid;

		} else {
			_full_trim_latched_valid = false;
			_full_trim_latched_A = 0.f;
			_full_trim_latched_B = 0.f;
			_static_trim.reset();
			_mode6_nav_epoch = 0;
			_mode6_entry_timestamp = 0;
			_mode6_command_sequence = 0;
		}

		_previous_mode = cfg.mode;
	}

	void compute_gated_shadow(Diagnostics &snapshot, const RuntimeConfig &cfg) const
	{
		snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::UNKNOWN);

		if (!snapshot.base_command_valid) {
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::BASE_COMMAND_INVALID);
			return;
		}

		const float raw_corr = math::constrain(snapshot.delta_mpc_shadow_correction_A, -cfg.g_lim, cfg.g_lim);
		snapshot.gate_base_large = fabsf(snapshot.delta_base_A) >= cfg.g_basemin;

		if (snapshot.gate_base_large) {
			if (fabsf(raw_corr) <= kDiagnosticTolerance || same_sign(raw_corr, snapshot.delta_base_A)) {
				snapshot.gate_direction_compatible = true;
				snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::DIRECTION_COMPATIBLE);
				snapshot.delta_gate_corr_A = raw_corr;
			} else {
				snapshot.gate_direction_compatible = false;
				snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::DIRECTION_CONFLICT);
				snapshot.delta_gate_corr_A = 0.f;
			}
		} else {
			snapshot.gate_direction_compatible = true;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::BASE_BELOW_MIN);
			const float low_limit = math::min(cfg.g_lowlim, cfg.g_lim);
			snapshot.delta_gate_corr_A = math::constrain(raw_corr, -low_limit, low_limit);
		}

		snapshot.delta_gate_corr_B = -snapshot.delta_gate_corr_A;
		snapshot.delta_gate_cand_A_raw = snapshot.delta_base_A + snapshot.delta_gate_corr_A;
		snapshot.delta_gate_cand_B_raw = snapshot.delta_base_B + snapshot.delta_gate_corr_B;
		snapshot.delta_gate_cand_A_limited = math::constrain(snapshot.delta_gate_cand_A_raw, -kDeltaMax, kDeltaMax);
		snapshot.delta_gate_cand_B_limited = math::constrain(snapshot.delta_gate_cand_B_raw, -kDeltaMax, kDeltaMax);
		snapshot.delta_gate_cand_A_sat = fabsf(snapshot.delta_gate_cand_A_raw) > (kDeltaMax + kDiagnosticTolerance);
		snapshot.delta_gate_cand_B_sat = fabsf(snapshot.delta_gate_cand_B_raw) > (kDeltaMax + kDiagnosticTolerance);
		snapshot.delta_gate_cand_A_ratio = math::min(fabsf(snapshot.delta_gate_cand_A_limited) / kDeltaMax, 1.0f);
		snapshot.delta_gate_cand_B_ratio = math::min(fabsf(snapshot.delta_gate_cand_B_limited) / kDeltaMax, 1.0f);
	}

	bool servo_headroom_ok(const Diagnostics &snapshot, const float target_diff_a, const float sat_margin) const
	{
		if (!snapshot.base_command_valid) {
			return false;
		}

		const float mean_servo = 0.5f * (snapshot.left_servo_base + snapshot.right_servo_base);
		const float servo_l_candidate = mean_servo + 0.5f * target_diff_a;
		const float servo_r_candidate = mean_servo - 0.5f * target_diff_a;
		const float abs_limit = math::max(1.f - sat_margin, 0.f);
		return fabsf(servo_l_candidate) <= (abs_limit + kDiagnosticTolerance) &&
		       fabsf(servo_r_candidate) <= (abs_limit + kDiagnosticTolerance);
	}

	void finalize_mode_logic(Diagnostics &snapshot, const RuntimeConfig &cfg)
	{
		const bool mode_tiny = cfg.mode == static_cast<uint8_t>(FmpcMode::ACTIVE_TINY);
		const bool mode_bounded = cfg.mode == static_cast<uint8_t>(FmpcMode::ACTIVE_BOUNDED_MPC);
		const bool mode_full = cfg.mode == static_cast<uint8_t>(FmpcMode::FULL_ROLL_MPC_TEST);

		snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::UNKNOWN);
		snapshot.active_fallback_reason = static_cast<uint8_t>(FallbackReason::UNKNOWN);
		snapshot.active_applied = false;
		snapshot.active_fallback = true;
		snapshot.active_output_allowed = false;
		snapshot.actuator_sign_confirmed = cfg.sign_ok;
		snapshot.bounded_requested = mode_bounded && cfg.act_en;
		snapshot.bounded_allowed = false;
		snapshot.bounded_applied = false;
		snapshot.full_requested = mode_full;
		snapshot.full_allowed = false;
		snapshot.full_applied = false;
		snapshot.full_duration_s = 0.f;
		snapshot.full_trim_a = (mode_full && cfg.full_trim_en && _full_trim_latched_valid) ? _full_trim_latched_A : 0.f;
		snapshot.full_trim_b = (mode_full && cfg.full_trim_en && _full_trim_latched_valid) ? _full_trim_latched_B : 0.f;
		snapshot.full_mpc_corr_a = 0.f;
		snapshot.full_target_a = 0.f;
		snapshot.full_target_b = 0.f;
		snapshot.full_candidate_clamped = false;
		snapshot.servo_saturation_flag = false;

		if (cfg.mode == static_cast<uint8_t>(FmpcMode::OFF)) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::MODE_OFF);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::MODE_OFF);
			return;
		}

		if (cfg.mode == static_cast<uint8_t>(FmpcMode::SHADOW_ONLY)) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::SHADOW_ONLY);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::SHADOW_ONLY);
			return;
		}

		if (cfg.mode == static_cast<uint8_t>(FmpcMode::GATED_SHADOW)) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::GATED_SHADOW_ONLY);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			if (snapshot.gate_reason == static_cast<uint8_t>(FmpcGateReason::UNKNOWN)) {
				snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::GATED_SHADOW_ONLY);
			}
			return;
		}

		if (cfg.mode == static_cast<uint8_t>(FmpcMode::FALLBACK)) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::USER_FALLBACK_MODE);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::USER_FALLBACK_MODE);
			return;
		}

		if (!mode_full && !cfg.act_en) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::PARAM_ACTIVE_DISABLED);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::PARAM_ACTIVE_DISABLED);
			return;
		}

		if (!mode_full && !cfg.sign_ok) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::SIGN_NOT_CONFIRMED);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::SIGN_NOT_CONFIRMED);
			return;
		}

		if (!snapshot.attitude_valid || !snapshot.angular_velocity_valid || !snapshot.solver_success) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::SENSOR_INVALID);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::SENSOR_INVALID);
			return;
		}

		if (!snapshot.base_command_valid || !snapshot.hydro_torque_setpoint_valid) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::BASE_COMMAND_INVALID);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::BASE_COMMAND_INVALID);
			return;
		}

		if (mode_full && cfg.full_trim_en && !_full_trim_latched_valid) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::BASE_COMMAND_INVALID);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::BASE_COMMAND_INVALID);
			return;
		}

		if (!mode_full && fabsf(snapshot.roll_deg) > cfg.roll_max_deg) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::ROLL_LIMIT);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::ROLL_LIMIT);
			return;
		}

		if (!mode_full && fabsf(snapshot.roll_rate_deg_s) > cfg.rate_max_deg_s) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::RATE_LIMIT);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::RATE_LIMIT);
			return;
		}

		if (!mode_full && !snapshot.gate_direction_compatible && snapshot.gate_base_large) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::DIRECTION_CONFLICT);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::DIRECTION_CONFLICT);
			return;
		}

		if (mode_bounded && !cfg.bnd_en) {
			_full_mode_start_timestamp = 0;
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::BOUNDED_ACTIVE_DISABLED);
			snapshot.active_fallback_reason = snapshot.fallback_reason;
			snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::BOUNDED_ACTIVE_DISABLED);
			return;
		}

		if (mode_tiny) {
			_full_mode_start_timestamp = 0;
			float act_limit = cfg.act_lim;
			if (cfg.act_lim_clamped) {
				snapshot.active_fallback_reason = static_cast<uint8_t>(FallbackReason::ACTIVE_LIMIT_CLAMPED);
				snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::ACTIVE_LIMIT_CLAMPED);
			}

			if (snapshot.gate_base_large) {
				snapshot.active_corr_A = math::constrain(snapshot.delta_gate_corr_A, -act_limit, act_limit);
			} else {
				const float low_limit = math::min(cfg.g_lowlim, act_limit);
				snapshot.active_corr_A = math::constrain(snapshot.delta_gate_corr_A, -low_limit, low_limit);
			}

			snapshot.active_corr_B = -snapshot.active_corr_A;
			snapshot.active_candidate_A = snapshot.delta_base_A + snapshot.active_corr_A;
			snapshot.active_candidate_B = snapshot.delta_base_B + snapshot.active_corr_B;
			snapshot.active_candidate_A_limited = math::constrain(snapshot.active_candidate_A, -kDeltaMax, kDeltaMax);
			snapshot.active_candidate_B_limited = math::constrain(snapshot.active_candidate_B, -kDeltaMax, kDeltaMax);

			if (fabsf(snapshot.active_candidate_A) > cfg.cand_max || fabsf(snapshot.active_candidate_B) > cfg.cand_max) {
				snapshot.active_corr_A = 0.f;
				snapshot.active_corr_B = 0.f;
				snapshot.active_candidate_A = snapshot.delta_base_A;
				snapshot.active_candidate_B = snapshot.delta_base_B;
				snapshot.active_candidate_A_limited = snapshot.active_candidate_A;
				snapshot.active_candidate_B_limited = snapshot.active_candidate_B;
				snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::CANDIDATE_LIMIT);
				snapshot.active_fallback_reason = snapshot.fallback_reason;
				snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::CANDIDATE_LIMIT);
				return;
			}
		}

		if (mode_bounded) {
			_full_mode_start_timestamp = 0;
			snapshot.bounded_allowed = true;
			float bounded_corr = math::constrain(snapshot.delta_gate_corr_A, -cfg.bnd_lim, cfg.bnd_lim);
			snapshot.active_corr_A = bounded_corr;
			snapshot.active_corr_B = -bounded_corr;
			snapshot.active_candidate_A = snapshot.delta_base_A + snapshot.active_corr_A;
			snapshot.active_candidate_B = snapshot.delta_base_B + snapshot.active_corr_B;
			snapshot.active_candidate_A_limited = math::constrain(snapshot.active_candidate_A, -kDeltaMax, kDeltaMax);
			snapshot.active_candidate_B_limited = math::constrain(snapshot.active_candidate_B, -kDeltaMax, kDeltaMax);

			if (fabsf(snapshot.active_candidate_A) > cfg.cand_max || fabsf(snapshot.active_candidate_B) > cfg.cand_max) {
				snapshot.active_corr_A = 0.f;
				snapshot.active_corr_B = 0.f;
				snapshot.active_candidate_A = snapshot.delta_base_A;
				snapshot.active_candidate_B = snapshot.delta_base_B;
				snapshot.active_candidate_A_limited = snapshot.active_candidate_A;
				snapshot.active_candidate_B_limited = snapshot.active_candidate_B;
				snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::CANDIDATE_LIMIT);
				snapshot.active_fallback_reason = snapshot.fallback_reason;
				snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::CANDIDATE_LIMIT);
				return;
			}

			if (!servo_headroom_ok(snapshot, snapshot.active_candidate_A, cfg.sat_margin)) {
				snapshot.servo_saturation_flag = true;
				snapshot.active_corr_A = 0.f;
				snapshot.active_corr_B = 0.f;
				snapshot.active_candidate_A = snapshot.delta_base_A;
				snapshot.active_candidate_B = snapshot.delta_base_B;
				snapshot.active_candidate_A_limited = snapshot.active_candidate_A;
				snapshot.active_candidate_B_limited = snapshot.active_candidate_B;
				snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::SATURATION_NO_HEADROOM);
				snapshot.active_fallback_reason = snapshot.fallback_reason;
				snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::SATURATION_NO_HEADROOM);
				return;
			}
		}

		if (mode_full) {
			snapshot.full_allowed = true;
			const FullMpcCommand &command = _full_result.command;
			if (!_full_result.proposal.solver_success || !PX4_ISFINITE(command.correction)) {
				_full_mode_start_timestamp = 0;
				snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::FULL_SOURCE_INVALID);
				snapshot.active_fallback_reason = snapshot.fallback_reason;
				snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::FULL_SOURCE_INVALID);
				return;
			}

			if (_full_mode_start_timestamp == 0) {
				_full_mode_start_timestamp = snapshot.timestamp_us;
			}

			snapshot.full_duration_s = (snapshot.timestamp_us - _full_mode_start_timestamp) / 1.0e6f;

			snapshot.full_mpc_corr_a = command.correction;
			const float requested_target_a = command.requested_target;
			const float requested_target_b = -command.requested_target;

			if (!PX4_ISFINITE(requested_target_a) || !PX4_ISFINITE(requested_target_b)) {
				_full_mode_start_timestamp = 0;
				snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::FULL_SOURCE_INVALID);
				snapshot.active_fallback_reason = snapshot.fallback_reason;
				snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::FULL_SOURCE_INVALID);
				return;
			}

			// Keep the requested target for diagnostics, then implement the finite target through hard bounds.
			snapshot.active_candidate_A = requested_target_a;
			snapshot.active_candidate_B = requested_target_b;
			snapshot.full_candidate_clamped = command.candidate_clamped;
			snapshot.servo_saturation_flag = command.headroom_clamped;
			// Publish the same implemented input whose cost was evaluated by the solver.
			snapshot.full_target_a = command.target;
			snapshot.full_target_b = -command.target;
			snapshot.active_candidate_A_limited = snapshot.full_target_a;
			snapshot.active_candidate_B_limited = snapshot.full_target_b;
			snapshot.active_corr_A = snapshot.full_target_a - snapshot.full_trim_a;
			snapshot.active_corr_B = snapshot.full_target_b - snapshot.full_trim_b;

			if (!PX4_ISFINITE(snapshot.full_target_a) || !PX4_ISFINITE(snapshot.full_target_b) ||
			    !PX4_ISFINITE(snapshot.active_corr_A) || !PX4_ISFINITE(snapshot.active_corr_B)) {
				_full_mode_start_timestamp = 0;
				snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::FULL_SOURCE_INVALID);
				snapshot.active_fallback_reason = snapshot.fallback_reason;
				snapshot.gate_reason = static_cast<uint8_t>(FmpcGateReason::FULL_SOURCE_INVALID);
				return;
			}
		} else {
			_full_mode_start_timestamp = 0;
		}

		snapshot.active_precheck_pass = true;
		snapshot.active_applied = true;
		snapshot.active_fallback = false;
		snapshot.active_output_allowed = true;
		snapshot.safety_gate_pass = true;
		snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::NONE);
		snapshot.active_fallback_reason = static_cast<uint8_t>(FallbackReason::NONE);
		snapshot.bounded_applied = mode_bounded;
		snapshot.full_applied = mode_full;
	}

	void publish_tiny_correction(const Diagnostics &diag)
	{
		fixed_mpc_tiny_correction_s msg{};
		msg.timestamp = diag.timestamp_us;
		const bool mode_full = diag.fmpc_mode == static_cast<uint8_t>(FmpcMode::FULL_ROLL_MPC_TEST);

		if (mode_full) {
			_mode6_command_sequence++;

			if (_mode6_command_sequence == 0) {
				_mode6_command_sequence = 1;
			}
		}

		msg.mode_entry_timestamp = mode_full ? _mode6_entry_timestamp : 0;
		msg.selector_epoch = mode_full ? _mode6_binding.selector_epoch : 0;
		msg.nav_state = _flight_selection.nav_state;
		msg.nav_state_timestamp = _flight_selection.nav_epoch;
		msg.command_sequence = mode_full ? _mode6_command_sequence : 0;
		msg.mode_session_id = mode_full ? _mode6_session_id : 0;
		msg.mode = diag.fmpc_mode;
		msg.act_req =
			(diag.fmpc_mode == static_cast<uint8_t>(FmpcMode::FULL_ROLL_MPC_TEST)) ||
			(((diag.fmpc_mode == static_cast<uint8_t>(FmpcMode::ACTIVE_TINY)) ||
			  (diag.fmpc_mode == static_cast<uint8_t>(FmpcMode::ACTIVE_BOUNDED_MPC))) && diag.fmpc_act_en);
		msg.act_allow = diag.active_precheck_pass;
		msg.act_apply = diag.active_applied;
		msg.fb = diag.active_fallback;
		msg.fb_r = diag.active_fallback_reason;
		msg.corr_a = diag.active_corr_A;
		msg.corr_b = diag.active_corr_B;
		msg.cand_a = mode_full ? diag.full_target_a : diag.active_candidate_A;
		msg.cand_b = mode_full ? diag.full_target_b : diag.active_candidate_B;
		msg.base_a = mode_full ? diag.full_trim_a : diag.delta_base_A;
		msg.base_b = mode_full ? diag.full_trim_b : diag.delta_base_B;
		msg.requested_target_a = diag.active_candidate_A;
		msg.requested_target_b = diag.active_candidate_B;
		msg.diag_ok = diag.diag_consistency_ok;
		msg.sen_ok = diag.attitude_valid && diag.angular_velocity_valid;
		msg.base_ok = diag.base_command_valid && diag.hydro_torque_setpoint_valid;
		msg.dir_ok = diag.gate_direction_compatible;
		_fixed_mpc_tiny_correction_pub.publish(msg);
	}

	void publish_shadow_diagnostics(const Diagnostics &diag)
	{
		fixed_mpc_shadow_status_s msg{};
		msg.timestamp = diag.timestamp_us;
		msg.roll_deg = diag.roll_deg;
		msg.p_deg_s = diag.roll_rate_deg_s;
		msg.pitch_deg = diag.pitch_deg;
		msg.yaw_deg = diag.yaw_deg;
		msg.solve_us = static_cast<float>(diag.solve_time_us);
		msg.d_base_a = diag.delta_base_A;
		msg.d_base_b = diag.delta_base_B;
		msg.d_raw = diag.delta_mpc_raw;
		msg.d_corr = diag.delta_mpc_shadow_correction;
		msg.d_gate_corr_a = diag.delta_gate_corr_A;
		msg.d_gate_corr_b = diag.delta_gate_corr_B;
		msg.active_corr_a = diag.active_corr_A;
		msg.active_corr_b = diag.active_corr_B;
		msg.active_candidate_a = diag.active_candidate_A;
		msg.active_candidate_b = diag.active_candidate_B;
		msg.allocator_applied_corr_a = diag.allocator_applied_corr_A;
		msg.allocator_applied_corr_b = diag.allocator_applied_corr_B;
		msg.fmpc_act_lim = diag.fmpc_act_lim;
		msg.fmpc_cand_max = diag.fmpc_cand_max;
		msg.fmpc_fcand_max = diag.fmpc_fcand_max;
		msg.fmpc_mode = _flight_selection.configured_mode;
		msg.effective_mode = diag.fmpc_mode;
		msg.nav_state = _flight_selection.nav_state;
		msg.mode6_eligible = _flight_selection.mode6_eligible;
		msg.fmpc_act_en = diag.fmpc_act_en ? 1 : 0;
		msg.fmpc_sign_ok = diag.fmpc_sign_ok ? 1 : 0;
		msg.diag_ok = diag.diag_consistency_ok ? 1 : 0;
		msg.sensor_ok = (diag.attitude_valid && diag.angular_velocity_valid) ? 1 : 0;
		msg.base_ok = (diag.base_command_valid && diag.hydro_torque_setpoint_valid) ? 1 : 0;
		msg.direction_ok = diag.gate_direction_compatible ? 1 : 0;
		msg.attitude_ok = diag.attitude_valid ? 1 : 0;
		msg.angular_velocity_ok = diag.angular_velocity_valid ? 1 : 0;
		msg.base_command_ok = diag.base_command_valid ? 1 : 0;
		msg.torque_setpoint_ok = diag.hydro_torque_setpoint_valid ? 1 : 0;
		msg.solver_ok = diag.solver_success ? 1 : 0;
		msg.active_requested =
			(diag.fmpc_mode == static_cast<uint8_t>(FmpcMode::ACTIVE_TINY) && diag.fmpc_act_en) ? 1 : 0;
		msg.active_allowed = diag.active_precheck_pass ? 1 : 0;
		msg.active_applied = diag.active_applied ? 1 : 0;
		msg.active_fallback = diag.active_fallback ? 1 : 0;
		msg.fallback_reason = diag.fallback_reason;
		msg.gate_pass = diag.safety_gate_pass ? 1 : 0;
		msg.candidate_limit_flag =
			(diag.full_candidate_clamped ||
			 diag.fallback_reason == static_cast<uint8_t>(FallbackReason::CANDIDATE_LIMIT)) ? 1 : 0;
		msg.bounded_requested = diag.bounded_requested ? 1 : 0;
		msg.bounded_allowed = diag.bounded_allowed ? 1 : 0;
		msg.bounded_applied = diag.bounded_applied ? 1 : 0;
		msg.full_requested = diag.full_requested ? 1 : 0;
		msg.full_allowed = diag.full_allowed ? 1 : 0;
		msg.full_applied = diag.full_applied ? 1 : 0;
		msg.full_trim_en = diag.fmpc_full_trim_en ? 1 : 0;
		msg.full_duration_s = diag.full_duration_s;
		msg.full_trim_a = diag.full_trim_a;
		msg.full_trim_b = diag.full_trim_b;
		msg.full_mpc_corr_a = diag.full_mpc_corr_a;
		msg.full_target_a = diag.full_target_a;
		msg.full_target_b = diag.full_target_b;
		msg.servo_saturation_flag = diag.servo_saturation_flag ? 1 : 0;
		_fixed_mpc_shadow_status_pub.publish(msg);

		if (!diag.base_command_valid) { _input_status.rejection_mask |= BaseInvalid; }
		if (!diag.hydro_torque_setpoint_valid) { _input_status.rejection_mask |= TorqueInvalid; }
		_input_status.attitude_age_ms = diag.attitude_age_ms;
		_input_status.rate_age_ms = diag.angular_velocity_age_ms;
		_input_status.base_age_ms = diag.base_command_age_ms;
		_input_status.torque_age_ms = diag.hydro_torque_setpoint_age_ms;
		if (_input_status.rejection_mask != 0 && _input_status.rejection_mask != _previous_rejection_mask) {
			_input_rejection_count++;
			_last_rejection_mask = _input_status.rejection_mask;
			_last_reject_timestamp = diag.timestamp_us;
		}
		_previous_rejection_mask = _input_status.rejection_mask;
		_input_status.rejection_count = _input_rejection_count;
		_input_status.last_rejection_mask = _last_rejection_mask;
		_input_status.last_reject_timestamp = _last_reject_timestamp;
		_input_status_pub.publish(_input_status);
	}

	Diagnostics make_base_snapshot(const hrt_abstime now, const float last_run_elapsed_ms, const RuntimeConfig &cfg) const
	{
		Diagnostics s{};
		s.timestamp_us = now;
		s.loop_count = _loop_count;
		s.last_run_elapsed_ms = last_run_elapsed_ms;
		s.attitude_age_ms = 1.0e9f;
		s.angular_velocity_age_ms = 1.0e9f;
		s.base_command_age_ms = 1.0e9f;
		s.hydro_torque_setpoint_age_ms = 1.0e9f;
		s.delta_mpc_shadow_correction_limit = kShadowCorrectionLimit;
		s.delta_mpc_rate_limit_max_step = cfg.mode == static_cast<uint8_t>(FmpcMode::FULL_ROLL_MPC_TEST) ?
						 cfg.full_tuning.maximum_step() : kDDeltaMax;
		s.diag_consistency_ok = true;
		s.fallback_reason = static_cast<uint8_t>(FallbackReason::UNKNOWN);
		s.active_fallback_reason = static_cast<uint8_t>(FallbackReason::UNKNOWN);
		s.active_output_allowed = false;
		s.actuator_sign_confirmed = cfg.sign_ok;
		s.safety_gate_pass = false;
		s.fmpc_mode = cfg.mode;
		s.fmpc_act_en = cfg.act_en;
		s.fmpc_sign_ok = cfg.sign_ok;
		s.fmpc_act_lim = cfg.act_lim;
		s.fmpc_bnd_en = cfg.bnd_en;
		s.fmpc_bnd_lim = cfg.bnd_lim;
		s.fmpc_sat_margin = cfg.sat_margin;
		s.fmpc_full_en = false; // Deprecated commissioning field, retained for old log readers.
		s.fmpc_full_lim = cfg.full_lim;
		s.fmpc_full_dur_s = 0.f; // Continuous while STAB/Mode 6 remains healthy.
		s.fmpc_full_src = cfg.full_src;
		s.fmpc_full_trim_en = cfg.full_trim_en;
		s.fmpc_g_basemin = cfg.g_basemin;
		s.fmpc_g_lim = cfg.g_lim;
		s.fmpc_g_lowlim = cfg.g_lowlim;
		s.fmpc_roll_max_deg = cfg.roll_max_deg;
		s.fmpc_rate_max_deg_s = cfg.rate_max_deg_s;
		s.fmpc_cand_max = cfg.cand_max;
		s.fmpc_fcand_max = cfg.full_cand_max;
		return s;
	}

	static float amplitude_diagnostic_limit(const RuntimeConfig &cfg)
	{
		// Match prepare_full_input/solve_full_roll; a valid configured envelope
		// must not be rejected downstream using the legacy shadow amplitude cap.
		return cfg.mode == static_cast<uint8_t>(FmpcMode::FULL_ROLL_MPC_TEST) ?
			(cfg.full_src == 2 ? cfg.full_lim : math::min(cfg.full_lim, kShadowCorrectionLimit)) : kDeltaMax;
	}

	void evaluate_consistency(Diagnostics &snapshot, const RuntimeConfig &cfg) const
	{
		const bool active_mode =
			cfg.mode == static_cast<uint8_t>(FmpcMode::ACTIVE_TINY) ||
			cfg.mode == static_cast<uint8_t>(FmpcMode::ACTIVE_BOUNDED_MPC) ||
			cfg.mode == static_cast<uint8_t>(FmpcMode::FULL_ROLL_MPC_TEST);
		const float amplitude_offset = cfg.mode == static_cast<uint8_t>(FmpcMode::FULL_ROLL_MPC_TEST) &&
					       snapshot.solver_success ? _full_result.model_trim : 0.f;
		const float amplitude_limit = amplitude_diagnostic_limit(cfg);
		const float expected_amplitude_limited = amplitude_offset + math::constrain(snapshot.delta_mpc_raw - amplitude_offset,
				-amplitude_limit, amplitude_limit);
		const float expected_amplitude_ratio = (amplitude_limit > 0.f) ? math::min(fabsf(expected_amplitude_limited - amplitude_offset) / amplitude_limit, 1.0f) : 0.f;
		const bool expected_amplitude_flag = fabsf(snapshot.delta_mpc_raw - amplitude_offset) > (amplitude_limit + kDiagnosticTolerance);
		const float expected_shadow_correction = math::constrain(
			snapshot.delta_mpc_pre_gate, -snapshot.delta_mpc_shadow_correction_limit,
			snapshot.delta_mpc_shadow_correction_limit);

		snapshot.diag_error_amplitude_limit =
			!nearly_equal(snapshot.delta_mpc_amplitude_limited, expected_amplitude_limited) ||
			!nearly_equal(snapshot.delta_mpc_amplitude_saturation_ratio, expected_amplitude_ratio) ||
			(snapshot.delta_mpc_amplitude_saturation_flag != expected_amplitude_flag);
		snapshot.diag_error_legacy_fields =
			!nearly_equal(snapshot.delta_mpc_clipped, snapshot.delta_mpc_amplitude_limited) ||
			(snapshot.delta_mpc_saturation_flag != snapshot.delta_mpc_amplitude_saturation_flag) ||
			!nearly_equal(snapshot.delta_mpc_saturation_ratio, snapshot.delta_mpc_amplitude_saturation_ratio);
		snapshot.diag_error_pre_gate_rate_limit =
			cfg.mode == static_cast<uint8_t>(FmpcMode::FULL_ROLL_MPC_TEST) ?
			!nearly_equal(snapshot.delta_mpc_pre_gate, _full_result.command.correction) :
			!nearly_equal(snapshot.delta_mpc_pre_gate, snapshot.delta_mpc_rate_limited);
		snapshot.diag_error_shadow_correction =
			!nearly_equal(snapshot.delta_mpc_shadow_correction, expected_shadow_correction);
		snapshot.diag_error_post_gate =
			(!active_mode &&
			 (!nearly_equal(snapshot.delta_mpc_post_gate, 0.f) ||
			  !nearly_equal(snapshot.delta_rate_mpc_post_gate, 0.f) ||
			  !nearly_equal(snapshot.delta_candidate_post_gate_A, 0.f) ||
			  !nearly_equal(snapshot.delta_candidate_post_gate_B, 0.f)));
		snapshot.diag_error_base_command = false;

		if (snapshot.base_command_valid) {
			snapshot.diag_error_base_command =
				!nearly_equal(snapshot.delta_base_A, snapshot.left_servo_base - snapshot.right_servo_base) ||
				!nearly_equal(snapshot.delta_base_B, snapshot.right_servo_base - snapshot.left_servo_base);
		}

		snapshot.diag_error_candidate_shadow = false;
		if (snapshot.base_command_valid) {
			snapshot.diag_error_candidate_shadow =
				!nearly_equal(snapshot.delta_mpc_shadow_correction_B, -snapshot.delta_mpc_shadow_correction_A) ||
				!nearly_equal(snapshot.delta_candidate_shadow_A_raw, snapshot.delta_base_A + snapshot.delta_mpc_shadow_correction_A) ||
				!nearly_equal(snapshot.delta_candidate_shadow_B_raw, snapshot.delta_base_B + snapshot.delta_mpc_shadow_correction_B);
		}

		snapshot.diag_error_candidate_post_gate = false;
		if (!active_mode) {
			snapshot.diag_error_candidate_post_gate =
				!nearly_equal(snapshot.active_corr_A, 0.f) ||
				!nearly_equal(snapshot.active_corr_B, 0.f) ||
				snapshot.active_applied;
		}

		snapshot.diag_consistency_error_count = 0;
		snapshot.diag_consistency_error_count += snapshot.diag_error_amplitude_limit ? 1u : 0u;
		snapshot.diag_consistency_error_count += snapshot.diag_error_legacy_fields ? 1u : 0u;
		snapshot.diag_consistency_error_count += snapshot.diag_error_pre_gate_rate_limit ? 1u : 0u;
		snapshot.diag_consistency_error_count += snapshot.diag_error_shadow_correction ? 1u : 0u;
		snapshot.diag_consistency_error_count += snapshot.diag_error_post_gate ? 1u : 0u;
		snapshot.diag_consistency_error_count += snapshot.diag_error_base_command ? 1u : 0u;
		snapshot.diag_consistency_error_count += snapshot.diag_error_candidate_shadow ? 1u : 0u;
		snapshot.diag_consistency_error_count += snapshot.diag_error_candidate_post_gate ? 1u : 0u;
		snapshot.diag_consistency_ok = (snapshot.diag_consistency_error_count == 0u);
	}

	void publish_snapshot(const Diagnostics &snapshot)
	{
		irqstate_t flags = px4_enter_critical_section();
		_last_diag_snapshot = snapshot;
		px4_leave_critical_section(flags);
	}

	Diagnostics read_snapshot() const
	{
		irqstate_t flags = px4_enter_critical_section();
		const Diagnostics snapshot = _last_diag_snapshot;
		px4_leave_critical_section(flags);
		return snapshot;
	}

	static bool nearly_equal(const float a, const float b)
	{
		return fabsf(a - b) <= kDiagnosticTolerance;
	}

	static bool same_sign(const float a, const float b)
	{
		if (fabsf(a) <= kDiagnosticTolerance || fabsf(b) <= kDiagnosticTolerance) {
			return true;
		}
		return (a > 0.f && b > 0.f) || (a < 0.f && b < 0.f);
	}

	bool resolve_hydro_servo_indices(int &left_servo_index, int &right_servo_index) const
	{
		int32_t left_servo_param = 0;
		int32_t right_servo_param = 0;

		if (_hy_sv_idx_param_handles[0] == PARAM_INVALID || _hy_sv_idx_param_handles[1] == PARAM_INVALID) {
			return false;
		}

		if (param_get(_hy_sv_idx_param_handles[0], &left_servo_param) != 0 ||
		    param_get(_hy_sv_idx_param_handles[1], &right_servo_param) != 0) {
			return false;
		}

		if (left_servo_param < 1 || left_servo_param > actuator_servos_s::NUM_CONTROLS ||
		    right_servo_param < 1 || right_servo_param > actuator_servos_s::NUM_CONTROLS ||
		    left_servo_param == right_servo_param) {
			return false;
		}

		left_servo_index = left_servo_param - 1;
		right_servo_index = right_servo_param - 1;
		return true;
	}

	static const char *mode_string(uint8_t mode)
	{
		switch (static_cast<FmpcMode>(mode)) {
		case FmpcMode::OFF: return "OFF";
		case FmpcMode::SHADOW_ONLY: return "SHADOW_ONLY";
		case FmpcMode::GATED_SHADOW: return "GATED_SHADOW";
		case FmpcMode::ACTIVE_TINY: return "ACTIVE_TINY";
		case FmpcMode::FALLBACK: return "FALLBACK";
		case FmpcMode::ACTIVE_BOUNDED_MPC: return "ACTIVE_BOUNDED_MPC";
		case FmpcMode::FULL_ROLL_MPC_TEST: return "FULL_ROLL_MPC_TEST";
		default: return "UNKNOWN";
		}
	}

	static const char *base_command_source_string(const uint8_t source)
	{
		switch (static_cast<FmpcBaseSource>(source)) {
		case FmpcBaseSource::NONE: return "NONE";
		case FmpcBaseSource::HYDRO_SERVOS: return "HYDRO_SERVOS";
		case FmpcBaseSource::HYDRO_TORQUE_SETPOINT_ONLY: return "HYDRO_TORQUE_SETPOINT_ONLY";
		case FmpcBaseSource::STALE_OR_INVALID: return "STALE_OR_INVALID";
		case FmpcBaseSource::ALLOCATOR_STATUS: return "ALLOCATOR_STATUS";
		default: return "UNKNOWN";
		}
	}

	static const char *gate_reason_string(const uint8_t reason)
	{
		switch (static_cast<FmpcGateReason>(reason)) {
		case FmpcGateReason::NONE: return "NONE";
		case FmpcGateReason::MODE_OFF: return "MODE_OFF";
		case FmpcGateReason::SHADOW_ONLY: return "SHADOW_ONLY";
		case FmpcGateReason::GATED_SHADOW_ONLY: return "GATED_SHADOW_ONLY";
		case FmpcGateReason::BASE_BELOW_MIN: return "BASE_BELOW_MIN";
		case FmpcGateReason::DIRECTION_COMPATIBLE: return "DIRECTION_COMPATIBLE";
		case FmpcGateReason::DIRECTION_CONFLICT: return "DIRECTION_CONFLICT";
		case FmpcGateReason::PARAM_ACTIVE_DISABLED: return "PARAM_ACTIVE_DISABLED";
		case FmpcGateReason::SIGN_NOT_CONFIRMED: return "SIGN_NOT_CONFIRMED";
		case FmpcGateReason::SENSOR_INVALID: return "SENSOR_INVALID";
		case FmpcGateReason::BASE_COMMAND_INVALID: return "BASE_COMMAND_INVALID";
		case FmpcGateReason::ROLL_LIMIT: return "ROLL_LIMIT";
		case FmpcGateReason::RATE_LIMIT: return "RATE_LIMIT";
		case FmpcGateReason::CANDIDATE_LIMIT: return "CANDIDATE_LIMIT";
		case FmpcGateReason::ACTIVE_LIMIT_CLAMPED: return "ACTIVE_LIMIT_CLAMPED";
		case FmpcGateReason::USER_FALLBACK_MODE: return "USER_FALLBACK_MODE";
		case FmpcGateReason::SATURATION_NO_HEADROOM: return "SATURATION_NO_HEADROOM";
		case FmpcGateReason::BOUNDED_ACTIVE_DISABLED: return "BOUNDED_ACTIVE_DISABLED";
		case FmpcGateReason::BOUNDED_LIMIT_CLAMPED: return "BOUNDED_LIMIT_CLAMPED";
		case FmpcGateReason::BOUNDED_ACTIVE_APPLIED: return "BOUNDED_ACTIVE_APPLIED";
		case FmpcGateReason::FULL_ACTIVE_DISABLED: return "FULL_ACTIVE_DISABLED";
		case FmpcGateReason::FULL_LIMIT_CLAMPED: return "FULL_LIMIT_CLAMPED";
		case FmpcGateReason::FULL_DURATION_TIMEOUT: return "FULL_DURATION_TIMEOUT";
		case FmpcGateReason::FULL_ACTIVE_APPLIED: return "FULL_ACTIVE_APPLIED";
		case FmpcGateReason::FULL_SOURCE_INVALID: return "FULL_SOURCE_INVALID";
		default: return "UNKNOWN";
		}
	}

	static const char *fallback_reason_string(FallbackReason reason)
	{
		switch (reason) {
		case FallbackReason::NONE: return "NONE";
		case FallbackReason::SIGN_NOT_CONFIRMED: return "SIGN_NOT_CONFIRMED";
		case FallbackReason::MODE_OFF: return "MODE_OFF";
		case FallbackReason::SHADOW_ONLY: return "SHADOW_ONLY";
		case FallbackReason::GATED_SHADOW_ONLY: return "GATED_SHADOW_ONLY";
		case FallbackReason::PARAM_ACTIVE_DISABLED: return "PARAM_ACTIVE_DISABLED";
		case FallbackReason::SENSOR_INVALID: return "SENSOR_INVALID";
		case FallbackReason::STALE_SENSOR_DATA: return "STALE_SENSOR_DATA";
		case FallbackReason::BASE_COMMAND_INVALID: return "BASE_COMMAND_INVALID";
		case FallbackReason::DIAGNOSTIC_ERROR: return "DIAGNOSTIC_ERROR";
		case FallbackReason::DIRECTION_CONFLICT: return "DIRECTION_CONFLICT";
		case FallbackReason::ROLL_LIMIT: return "ROLL_LIMIT";
		case FallbackReason::RATE_LIMIT: return "RATE_LIMIT";
		case FallbackReason::CANDIDATE_LIMIT: return "CANDIDATE_LIMIT";
		case FallbackReason::ACTIVE_LIMIT_CLAMPED: return "ACTIVE_LIMIT_CLAMPED";
		case FallbackReason::USER_FALLBACK_MODE: return "USER_FALLBACK_MODE";
		case FallbackReason::SATURATION_NO_HEADROOM: return "SATURATION_NO_HEADROOM";
		case FallbackReason::BOUNDED_ACTIVE_DISABLED: return "BOUNDED_ACTIVE_DISABLED";
		case FallbackReason::BOUNDED_LIMIT_CLAMPED: return "BOUNDED_LIMIT_CLAMPED";
		case FallbackReason::BOUNDED_ACTIVE_APPLIED: return "BOUNDED_ACTIVE_APPLIED";
		case FallbackReason::FULL_ACTIVE_DISABLED: return "FULL_ACTIVE_DISABLED";
		case FallbackReason::FULL_LIMIT_CLAMPED: return "FULL_LIMIT_CLAMPED";
		case FallbackReason::FULL_DURATION_TIMEOUT: return "FULL_DURATION_TIMEOUT";
		case FallbackReason::FULL_ACTIVE_APPLIED: return "FULL_ACTIVE_APPLIED";
		case FallbackReason::FULL_SOURCE_INVALID: return "FULL_SOURCE_INVALID";
		default: return "UNKNOWN";
		}
	}

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	vehicle_status_s _vehicle_status{};
	FlightModeSelection _flight_selection{};
	uint64_t _mode6_nav_epoch{0};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _hydro_servos_sub{ORB_ID(hydro_servos)};
	uORB::Subscription _hydro_torque_setpoint_sub{ORB_ID(hydro_torque_setpoint)};
	uORB::Subscription _fixed_mpc_allocator_status_sub{ORB_ID(fixed_mpc_allocator_status)};
	uORB::Subscription _final_output_sub{ORB_ID(fixed_mpc_final_output_status)};
	uORB::Publication<fixed_mpc_input_status_s> _input_status_pub{ORB_ID(fixed_mpc_input_status)};
	uORB::Publication<fixed_mpc_shadow_status_s> _fixed_mpc_shadow_status_pub{ORB_ID(fixed_mpc_shadow_status)};
	uORB::Publication<fixed_mpc_tiny_correction_s> _fixed_mpc_tiny_correction_pub{ORB_ID(fixed_mpc_tiny_correction)};

	FixedLinearMPC _mpc{};
	FullMpcResult _full_result{};
	hrt_abstime _full_input_entry{0};
	int _full_input_sign{0};
	fixed_mpc_final_output_status_s _final_output{};
	fixed_mpc_input_status_s _input_status{};
	uint32_t _previous_rejection_mask{0};
	uint32_t _input_rejection_count{0};
	uint32_t _last_rejection_mask{0};
	hrt_abstime _last_reject_timestamp{0};
	hrt_abstime _last_run_timestamp{0};
	uint64_t _loop_count{0};
	float _previous_delta{0.f};
	actuator_servos_s _hydro_servos{};
	vehicle_torque_setpoint_s _hydro_torque_setpoint{};
	fixed_mpc_allocator_status_s _fixed_mpc_allocator_status{};
	param_t _hy_sv_idx_param_handles[2]{PARAM_INVALID, PARAM_INVALID};
	param_t _fmpc_mode_handle{PARAM_INVALID};
	param_t _fmpc_act_en_handle{PARAM_INVALID};
	param_t _fmpc_sign_ok_handle{PARAM_INVALID};
	param_t _fmpc_act_lim_handle{PARAM_INVALID};
	param_t _fmpc_bnd_en_handle{PARAM_INVALID};
	param_t _fmpc_bnd_lim_handle{PARAM_INVALID};
	param_t _fmpc_sat_margin_handle{PARAM_INVALID};
	param_t _fmpc_full_lim_handle{PARAM_INVALID};
	param_t _fmpc_full_slew_handle{PARAM_INVALID};
	param_t _fmpc_error_gain_handle{PARAM_INVALID};
	param_t _fmpc_full_src_handle{PARAM_INVALID};
	param_t _fmpc_full_trim_en_handle{PARAM_INVALID};
	param_t _fmpc_trim_a_handle{PARAM_INVALID};
	param_t _fmpc_g_basemin_handle{PARAM_INVALID};
	param_t _fmpc_g_lim_handle{PARAM_INVALID};
	param_t _fmpc_g_lowlim_handle{PARAM_INVALID};
	param_t _fmpc_roll_max_handle{PARAM_INVALID};
	param_t _fmpc_rate_max_handle{PARAM_INVALID};
	param_t _fmpc_cand_max_handle{PARAM_INVALID};
	param_t _fmpc_full_cand_max_handle{PARAM_INVALID};
	param_t _hy_roll_max_handle{PARAM_INVALID};
	param_t _com_rc_loss_t_handle{PARAM_INVALID};
	hrt_abstime _full_mode_start_timestamp{0};
	uint8_t _previous_mode{static_cast<uint8_t>(FmpcMode::SHADOW_ONLY)};
	float _full_trim_latched_A{0.f};
	float _full_trim_latched_B{0.f};
	bool _full_trim_latched_valid{false};
	hrt_abstime _mode6_entry_timestamp{0};
	uint32_t _mode6_command_sequence{0};
	uint32_t _mode6_session_id{0};
	Mode6SessionBinding _mode6_binding{};
	StaticFullTrim _static_trim{};
	Diagnostics _last_diag_snapshot{};
};

static int print_stopped_status()
{
	PX4_INFO("fixed_mpc_shadow: stopped");
	PX4_INFO("mode: SHADOW_ONLY");
	PX4_INFO("active_output_allowed: false");
	PX4_INFO("actuator_sign_confirmed: false");
	return 0;
}

} // namespace fixed_mpc_shadow

extern "C" __EXPORT int fixed_mpc_shadow_main(int argc, char *argv[])
{
	using fixed_mpc_shadow::FixedMpcShadow;

	if (argc <= 1 || !strcmp(argv[1], "help") || !strcmp(argv[1], "-h")) {
		return FixedMpcShadow::print_usage();
	}

	if (!strcmp(argv[1], "start")) {
		if (FixedMpcShadow::running()) {
			PX4_INFO("fixed_mpc_shadow: already running");
			return 0;
		}

		const int ret = FixedMpcShadow::task_spawn(argc - 1, argv + 1);

		if (ret == 0) {
			PX4_INFO("fixed_mpc_shadow: started");
			PX4_INFO("mode: QGC-switchable shadow/tiny-active framework");
			PX4_INFO("active_output_allowed: false by default");
		}

		return ret;
	}

	if (!strcmp(argv[1], "stop")) {
		if (!FixedMpcShadow::running()) {
			PX4_INFO("fixed_mpc_shadow: not running");
			return 0;
		}

		const int ret = FixedMpcShadow::stop_command();

		if (ret == 0) {
			PX4_INFO("fixed_mpc_shadow: stopped");
		}

		return ret;
	}

	if (!strcmp(argv[1], "status")) {
		if (!FixedMpcShadow::running()) {
			return fixed_mpc_shadow::print_stopped_status();
		}

		return FixedMpcShadow::instance()->print_status();
	}

	return FixedMpcShadow::custom_command(argc - 1, argv + 1);
}
