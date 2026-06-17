#include <drivers/drv_hrt.h>
#include <inttypes.h>
#include <lib/matrix/matrix/math.hpp>
#include <parameters/param.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <stdio.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>

#include "FixedLinearMPC.hpp"
#include "FixedMPCDiagnostics.hpp"
#include "FixedMPCParams.hpp"
#include "SafetyGate.hpp"

using namespace time_literals;

namespace fixed_mpc_shadow
{

static constexpr uint32_t kIntervalMs = 20;
static constexpr uint32_t kIntervalUs = kIntervalMs * 1000;
static constexpr float kRadToDeg = 57.29577951308232f;
static constexpr float kSensorAgeLimitMs = 200.f;
static constexpr float kBaseCommandAgeLimitMs = 200.f;
static constexpr float kDiagnosticTolerance = 1.0e-6f;
static constexpr const char *kWorkQueueName = "lp_default";

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

		_last_diag_snapshot = make_base_snapshot(hrt_absolute_time(), 0.f);
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
		PX4_INFO_RAW("mode: SHADOW_ONLY\n");
		PX4_INFO_RAW("active output: compile-time disabled\n");
		PX4_INFO_RAW("actuator_sign_confirmed: false\n");
		PX4_INFO_RAW("forbidden publications: actuator/hydro setpoints are not published\n");
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
		const Diagnostics snapshot = read_snapshot();

		PX4_INFO("fixed_mpc_shadow: running");
		PX4_INFO("work_queue: %s", kWorkQueueName);
		PX4_INFO("interval_ms: %lu", (unsigned long)kIntervalMs);
		PX4_INFO("loop_count: %" PRIu64, snapshot.loop_count);
		PX4_INFO("last_run_elapsed_ms: %.3f", (double)snapshot.last_run_elapsed_ms);
		PX4_INFO("last_solve_time_us: %" PRIu32, snapshot.solve_time_us);
		PX4_INFO("solver_success: %s", snapshot.solver_success ? "true" : "false");
		PX4_INFO("attitude_valid: %s", snapshot.attitude_valid ? "true" : "false");
		PX4_INFO("angular_velocity_valid: %s", snapshot.angular_velocity_valid ? "true" : "false");
		PX4_INFO("attitude_age_ms: %.3f", (double)snapshot.attitude_age_ms);
		PX4_INFO("angular_velocity_age_ms: %.3f", (double)snapshot.angular_velocity_age_ms);
		PX4_INFO("roll_rad: %.6f", (double)snapshot.phi_rad);
		PX4_INFO("roll_deg: %.3f", (double)snapshot.roll_deg);
		PX4_INFO("roll_rate_rad_s: %.6f", (double)snapshot.p_rad_s);
		PX4_INFO("roll_rate_deg_s: %.3f", (double)snapshot.roll_rate_deg_s);
		PX4_INFO("pitch_deg: %.3f", (double)snapshot.pitch_deg);
		PX4_INFO("yaw_deg: %.3f", (double)snapshot.yaw_deg);
		PX4_INFO("roll_rate_source: vehicle_angular_velocity.xyz[0] (PX4 body-frame roll rate)");
		PX4_INFO("base_command_valid: %s", snapshot.base_command_valid ? "true" : "false");
		PX4_INFO("base_command_source: hydro_servos");
		PX4_INFO("base_command_age_ms: %.3f", (double)snapshot.base_command_age_ms);
		PX4_INFO("hydro_torque_setpoint_valid: %s", snapshot.hydro_torque_setpoint_valid ? "true" : "false");
		PX4_INFO("hydro_torque_setpoint_age_ms: %.3f", (double)snapshot.hydro_torque_setpoint_age_ms);
		PX4_INFO("hydro_torque_roll_cmd: %.6f", (double)snapshot.hydro_torque_roll_cmd);
		PX4_INFO("left_servo_base: %.6f", (double)snapshot.left_servo_base);
		PX4_INFO("right_servo_base: %.6f", (double)snapshot.right_servo_base);
		PX4_INFO("delta_base_A: %.6f", (double)snapshot.delta_base_A);
		PX4_INFO("delta_base_B: %.6f", (double)snapshot.delta_base_B);
		PX4_INFO("delta_mpc_raw: %.6f", (double)snapshot.delta_mpc_raw);
		PX4_INFO("delta_mpc_amplitude_limited: %.6f", (double)snapshot.delta_mpc_amplitude_limited);
		PX4_INFO("delta_mpc_amplitude_saturation_flag: %s",
			 snapshot.delta_mpc_amplitude_saturation_flag ? "true" : "false");
		PX4_INFO("delta_mpc_amplitude_saturation_ratio: %.6f", (double)snapshot.delta_mpc_amplitude_saturation_ratio);
		PX4_INFO("delta_mpc_rate_limit_reference: %.6f", (double)snapshot.delta_mpc_rate_limit_reference);
		PX4_INFO("delta_mpc_rate_limit_delta: %.6f", (double)snapshot.delta_mpc_rate_limit_delta);
		PX4_INFO("delta_mpc_rate_limit_max_step: %.6f", (double)snapshot.delta_mpc_rate_limit_max_step);
		PX4_INFO("delta_mpc_rate_limited: %.6f", (double)snapshot.delta_mpc_rate_limited);
		PX4_INFO("delta_mpc_rate_saturation_flag: %s", snapshot.delta_mpc_rate_saturation_flag ? "true" : "false");
		PX4_INFO("delta_mpc_rate_saturation_ratio: %.6f", (double)snapshot.delta_mpc_rate_saturation_ratio);
		PX4_INFO("delta_mpc_clipped: %.6f", (double)snapshot.delta_mpc_clipped);
		PX4_INFO("delta_mpc_saturation_flag: %s", snapshot.delta_mpc_saturation_flag ? "true" : "false");
		PX4_INFO("delta_mpc_saturation_ratio: %.6f", (double)snapshot.delta_mpc_saturation_ratio);
		PX4_INFO("delta_mpc_pre_gate: %.6f", (double)snapshot.delta_mpc_pre_gate);
		PX4_INFO("delta_mpc_shadow_correction: %.6f", (double)snapshot.delta_mpc_shadow_correction);
		PX4_INFO("delta_mpc_shadow_correction_limit: %.6f", (double)snapshot.delta_mpc_shadow_correction_limit);
		PX4_INFO("delta_mpc_shadow_correction_saturation_flag: %s",
			 snapshot.delta_mpc_shadow_correction_saturation_flag ? "true" : "false");
		PX4_INFO("delta_mpc_shadow_correction_A: %.6f", (double)snapshot.delta_mpc_shadow_correction_A);
		PX4_INFO("delta_mpc_shadow_correction_B: %.6f", (double)snapshot.delta_mpc_shadow_correction_B);
		PX4_INFO("delta_candidate_shadow_A_raw: %.6f", (double)snapshot.delta_candidate_shadow_A_raw);
		PX4_INFO("delta_candidate_shadow_A_limited: %.6f", (double)snapshot.delta_candidate_shadow_A_limited);
		PX4_INFO("delta_candidate_shadow_A_saturation_flag: %s",
			 snapshot.delta_candidate_shadow_A_saturation_flag ? "true" : "false");
		PX4_INFO("delta_candidate_shadow_A_saturation_ratio: %.6f",
			 (double)snapshot.delta_candidate_shadow_A_saturation_ratio);
		PX4_INFO("delta_candidate_shadow_B_raw: %.6f", (double)snapshot.delta_candidate_shadow_B_raw);
		PX4_INFO("delta_candidate_shadow_B_limited: %.6f", (double)snapshot.delta_candidate_shadow_B_limited);
		PX4_INFO("delta_candidate_shadow_B_saturation_flag: %s",
			 snapshot.delta_candidate_shadow_B_saturation_flag ? "true" : "false");
		PX4_INFO("delta_candidate_shadow_B_saturation_ratio: %.6f",
			 (double)snapshot.delta_candidate_shadow_B_saturation_ratio);
		PX4_INFO("delta_mpc_post_gate: %.6f", (double)snapshot.delta_mpc_post_gate);
		PX4_INFO("delta_rate_mpc_pre_gate: %.6f", (double)snapshot.delta_rate_mpc_pre_gate);
		PX4_INFO("delta_rate_mpc_post_gate: %.6f", (double)snapshot.delta_rate_mpc_post_gate);
		PX4_INFO("delta_candidate_post_gate_A: %.6f", (double)snapshot.delta_candidate_post_gate_A);
		PX4_INFO("delta_candidate_post_gate_B: %.6f", (double)snapshot.delta_candidate_post_gate_B);
		PX4_INFO("diag_consistency_ok: %s", snapshot.diag_consistency_ok ? "true" : "false");
		PX4_INFO("diag_consistency_error_count: %" PRIu32, snapshot.diag_consistency_error_count);
		PX4_INFO("diag_error_amplitude_limit: %s", snapshot.diag_error_amplitude_limit ? "true" : "false");
		PX4_INFO("diag_error_legacy_fields: %s", snapshot.diag_error_legacy_fields ? "true" : "false");
		PX4_INFO("diag_error_pre_gate_rate_limit: %s", snapshot.diag_error_pre_gate_rate_limit ? "true" : "false");
		PX4_INFO("diag_error_shadow_correction: %s", snapshot.diag_error_shadow_correction ? "true" : "false");
		PX4_INFO("diag_error_post_gate: %s", snapshot.diag_error_post_gate ? "true" : "false");
		PX4_INFO("diag_error_base_command: %s", snapshot.diag_error_base_command ? "true" : "false");
		PX4_INFO("diag_error_candidate_shadow: %s", snapshot.diag_error_candidate_shadow ? "true" : "false");
		PX4_INFO("diag_error_candidate_post_gate: %s", snapshot.diag_error_candidate_post_gate ? "true" : "false");
		PX4_INFO("active_output_allowed: false");
		PX4_INFO("actuator_sign_confirmed: false");
		PX4_INFO("safety_gate_pass: %s", snapshot.safety_gate_pass ? "true" : "false");
		PX4_INFO("fallback_reason: %s", fallback_reason_string(static_cast<FallbackReason>(snapshot.fallback_reason)));
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

		MpcState state{};
		Diagnostics snapshot = make_base_snapshot(now, last_run_elapsed_ms);
		update_base_command_snapshot(now, snapshot);

		if (!update_inputs(now, state, snapshot)) {
			evaluate_consistency(snapshot);
			publish_shadow_diagnostics(snapshot);
			publish_snapshot(snapshot);
			return;
		}

		const hrt_abstime solve_start = hrt_absolute_time();
		MpcResult result = _mpc.solve_lqr_clipping(state);
		snapshot.solve_time_us = hrt_elapsed_time(&solve_start);
		result.solve_time_us = snapshot.solve_time_us;
		snapshot.solver_success = result.solver_success;
		snapshot.delta_logged = _previous_delta;
		snapshot.delta_mpc_raw = result.delta_raw;
		snapshot.delta_mpc_amplitude_limited = result.delta_amplitude_limited;
		snapshot.delta_mpc_amplitude_saturation_flag = fabsf(result.delta_raw) > (kDeltaMax + kDiagnosticTolerance);
		snapshot.delta_mpc_amplitude_saturation_ratio = (kDeltaMax > 0.f) ?
				math::min(fabsf(snapshot.delta_mpc_amplitude_limited) / kDeltaMax, 1.0f) : 0.f;
		snapshot.delta_mpc_rate_limit_reference = _previous_delta;
		snapshot.delta_mpc_rate_limit_delta = result.delta_rate_limited - snapshot.delta_mpc_rate_limit_reference;
		snapshot.delta_mpc_rate_limit_max_step = kDDeltaMax;
		snapshot.delta_mpc_rate_limited = result.delta_rate_limited;
		snapshot.delta_mpc_rate_saturation_flag =
			fabsf(snapshot.delta_mpc_rate_limited - snapshot.delta_mpc_amplitude_limited) > kDiagnosticTolerance;
		snapshot.delta_mpc_rate_saturation_ratio = (kDDeltaMax > 0.f) ?
				math::min(fabsf(snapshot.delta_mpc_amplitude_limited - snapshot.delta_mpc_rate_limit_reference) / kDDeltaMax, 1.0f) : 0.f;
		snapshot.delta_mpc_clipped = snapshot.delta_mpc_amplitude_limited;
		snapshot.delta_mpc_saturation_flag = snapshot.delta_mpc_amplitude_saturation_flag;
		snapshot.delta_mpc_saturation_ratio = snapshot.delta_mpc_amplitude_saturation_ratio;
		snapshot.delta_mpc_pre_gate = snapshot.delta_mpc_rate_limited;
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
		snapshot.fallback_reason = base_command_fallback(snapshot);

		evaluate_consistency(snapshot);
		publish_shadow_diagnostics(snapshot);
		publish_snapshot(snapshot);

		_previous_delta = snapshot.delta_mpc_pre_gate;
	}

	void update_base_command_snapshot(const hrt_abstime now, Diagnostics &snapshot)
	{
		_hydro_servos_sub.update(&_hydro_servos);
		_hydro_torque_setpoint_sub.update(&_hydro_torque_setpoint);

		snapshot.base_command_age_ms = (_hydro_servos.timestamp > 0) ? (now - _hydro_servos.timestamp) / 1000.f : 1.0e9f;
		snapshot.hydro_torque_setpoint_age_ms = (_hydro_torque_setpoint.timestamp > 0) ?
				(now - _hydro_torque_setpoint.timestamp) / 1000.f : 1.0e9f;

		if (PX4_ISFINITE(_hydro_torque_setpoint.xyz[0]) && _hydro_torque_setpoint.timestamp > 0 &&
		    snapshot.hydro_torque_setpoint_age_ms <= kBaseCommandAgeLimitMs) {
			snapshot.hydro_torque_setpoint_valid = true;
			snapshot.hydro_torque_roll_cmd = _hydro_torque_setpoint.xyz[0];
		}

		int left_servo_index = -1;
		int right_servo_index = -1;

		if (!resolve_hydro_servo_indices(left_servo_index, right_servo_index)) {
			snapshot.base_command_valid = false;
			return;
		}

		if (_hydro_servos.timestamp == 0) {
			snapshot.base_command_valid = false;
			return;
		}

		snapshot.left_servo_base = _hydro_servos.control[left_servo_index];
		snapshot.right_servo_base = _hydro_servos.control[right_servo_index];

		if (!PX4_ISFINITE(snapshot.left_servo_base) || !PX4_ISFINITE(snapshot.right_servo_base)) {
			snapshot.base_command_valid = false;
			return;
		}

		snapshot.delta_base_A = snapshot.left_servo_base - snapshot.right_servo_base;
		snapshot.delta_base_B = snapshot.right_servo_base - snapshot.left_servo_base;

		if (snapshot.base_command_age_ms > kBaseCommandAgeLimitMs) {
			snapshot.base_command_valid = false;
			return;
		}

		snapshot.base_command_valid = true;
	}

	bool update_inputs(const hrt_abstime now, MpcState &state, Diagnostics &snapshot)
	{
		vehicle_attitude_s attitude{};
		vehicle_angular_velocity_s angular_velocity{};

		if (!_vehicle_attitude_sub.copy(&attitude) || !_vehicle_angular_velocity_sub.copy(&angular_velocity)) {
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::MISSING_SENSOR_DATA);
			return false;
		}

		snapshot.attitude_age_ms = (attitude.timestamp > 0) ? (now - attitude.timestamp) / 1000.f : 1.0e9f;
		snapshot.angular_velocity_age_ms = (angular_velocity.timestamp > 0) ? (now - angular_velocity.timestamp) / 1000.f : 1.0e9f;

		for (float q_component : attitude.q) {
			if (!PX4_ISFINITE(q_component)) {
				snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::MISSING_SENSOR_DATA);
				return false;
			}
		}

		if (!PX4_ISFINITE(angular_velocity.xyz[0])) {
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::MISSING_SENSOR_DATA);
			return false;
		}

		if (snapshot.attitude_age_ms > kSensorAgeLimitMs || snapshot.angular_velocity_age_ms > kSensorAgeLimitMs) {
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::STALE_SENSOR_DATA);
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
			snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::MISSING_SENSOR_DATA);
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

	void update_candidate_shadow_snapshot(Diagnostics &snapshot) const
	{
		if (!snapshot.base_command_valid) {
			return;
		}

		snapshot.delta_mpc_shadow_correction_A = snapshot.delta_mpc_shadow_correction;
		snapshot.delta_mpc_shadow_correction_B = -snapshot.delta_mpc_shadow_correction_A;
		snapshot.delta_candidate_shadow_A_raw = snapshot.delta_base_A + snapshot.delta_mpc_shadow_correction_A;
		snapshot.delta_candidate_shadow_B_raw = snapshot.delta_base_B + snapshot.delta_mpc_shadow_correction_B;
		snapshot.delta_candidate_shadow_A_limited =
			math::constrain(snapshot.delta_candidate_shadow_A_raw, -kDeltaMax, kDeltaMax);
		snapshot.delta_candidate_shadow_B_limited =
			math::constrain(snapshot.delta_candidate_shadow_B_raw, -kDeltaMax, kDeltaMax);
		snapshot.delta_candidate_shadow_A_saturation_flag =
			fabsf(snapshot.delta_candidate_shadow_A_raw) > (kDeltaMax + kDiagnosticTolerance);
		snapshot.delta_candidate_shadow_B_saturation_flag =
			fabsf(snapshot.delta_candidate_shadow_B_raw) > (kDeltaMax + kDiagnosticTolerance);
		snapshot.delta_candidate_shadow_A_saturation_ratio = (kDeltaMax > 0.f) ?
				math::min(fabsf(snapshot.delta_candidate_shadow_A_limited) / kDeltaMax, 1.0f) : 0.f;
		snapshot.delta_candidate_shadow_B_saturation_ratio = (kDeltaMax > 0.f) ?
				math::min(fabsf(snapshot.delta_candidate_shadow_B_limited) / kDeltaMax, 1.0f) : 0.f;
	}

	void publish_shadow_diagnostics(const Diagnostics &diag)
	{
		(void)diag;
	}

	Diagnostics make_base_snapshot(const hrt_abstime now, const float last_run_elapsed_ms) const
	{
		Diagnostics snapshot{};
		snapshot.timestamp_us = now;
		snapshot.loop_count = _loop_count;
		snapshot.last_run_elapsed_ms = last_run_elapsed_ms;
		snapshot.attitude_age_ms = 1.0e9f;
		snapshot.angular_velocity_age_ms = 1.0e9f;
		snapshot.base_command_age_ms = 1.0e9f;
		snapshot.hydro_torque_setpoint_age_ms = 1.0e9f;
		snapshot.delta_mpc_shadow_correction_limit = kShadowCorrectionLimit;
		snapshot.delta_mpc_rate_limit_max_step = kDDeltaMax;
		snapshot.diag_consistency_ok = true;
		snapshot.fallback_reason = static_cast<uint8_t>(FallbackReason::UNKNOWN);
		snapshot.active_output_allowed = false;
		snapshot.actuator_sign_confirmed = false;
		snapshot.safety_gate_pass = false;
		return snapshot;
	}

	void evaluate_consistency(Diagnostics &snapshot) const
	{
		const float expected_amplitude_limited =
			math::constrain(snapshot.delta_mpc_raw, -kDeltaMax, kDeltaMax);
		const float expected_amplitude_ratio = (kDeltaMax > 0.f) ?
			math::min(fabsf(expected_amplitude_limited) / kDeltaMax, 1.0f) : 0.f;
		const bool expected_amplitude_flag = fabsf(snapshot.delta_mpc_raw) > (kDeltaMax + kDiagnosticTolerance);
		const float expected_shadow_correction = math::constrain(
			snapshot.delta_mpc_pre_gate, -snapshot.delta_mpc_shadow_correction_limit,
			snapshot.delta_mpc_shadow_correction_limit);
		const bool mpc_chain_available = snapshot.attitude_valid && snapshot.angular_velocity_valid && snapshot.solver_success;

		snapshot.diag_error_amplitude_limit =
			!nearly_equal(snapshot.delta_mpc_amplitude_limited, expected_amplitude_limited) ||
			!nearly_equal(snapshot.delta_mpc_amplitude_saturation_ratio, expected_amplitude_ratio) ||
			(snapshot.delta_mpc_amplitude_saturation_flag != expected_amplitude_flag) ||
			(nearly_equal(snapshot.delta_mpc_raw, 0.f) &&
			 nearly_equal(snapshot.delta_mpc_amplitude_limited, 0.f) &&
			 snapshot.delta_mpc_amplitude_saturation_flag);

		snapshot.diag_error_legacy_fields =
			!nearly_equal(snapshot.delta_mpc_clipped, snapshot.delta_mpc_amplitude_limited) ||
			(snapshot.delta_mpc_saturation_flag != snapshot.delta_mpc_amplitude_saturation_flag) ||
			!nearly_equal(snapshot.delta_mpc_saturation_ratio, snapshot.delta_mpc_amplitude_saturation_ratio);

		snapshot.diag_error_pre_gate_rate_limit =
			!nearly_equal(snapshot.delta_mpc_pre_gate, snapshot.delta_mpc_rate_limited);

		snapshot.diag_error_shadow_correction =
			!nearly_equal(snapshot.delta_mpc_shadow_correction, expected_shadow_correction);

		snapshot.diag_error_post_gate =
			(!snapshot.active_output_allowed &&
			 (!nearly_equal(snapshot.delta_mpc_post_gate, 0.f) ||
			  !nearly_equal(snapshot.delta_rate_mpc_post_gate, 0.f) ||
			  snapshot.safety_gate_pass));

		snapshot.diag_error_base_command = false;

		if (snapshot.base_command_valid) {
			snapshot.diag_error_base_command =
				!nearly_equal(snapshot.delta_base_A, snapshot.left_servo_base - snapshot.right_servo_base) ||
				!nearly_equal(snapshot.delta_base_B, snapshot.right_servo_base - snapshot.left_servo_base);
		}

		snapshot.diag_error_candidate_shadow = false;

		if (snapshot.base_command_valid && mpc_chain_available) {
			const float expected_candidate_shadow_a_raw = snapshot.delta_base_A + snapshot.delta_mpc_shadow_correction_A;
			const float expected_candidate_shadow_b_raw = snapshot.delta_base_B + snapshot.delta_mpc_shadow_correction_B;
			const float expected_candidate_shadow_a_limited =
				math::constrain(expected_candidate_shadow_a_raw, -kDeltaMax, kDeltaMax);
			const float expected_candidate_shadow_b_limited =
				math::constrain(expected_candidate_shadow_b_raw, -kDeltaMax, kDeltaMax);
			const bool expected_candidate_shadow_a_flag =
				fabsf(expected_candidate_shadow_a_raw) > (kDeltaMax + kDiagnosticTolerance);
			const bool expected_candidate_shadow_b_flag =
				fabsf(expected_candidate_shadow_b_raw) > (kDeltaMax + kDiagnosticTolerance);
			const float expected_candidate_shadow_a_ratio = (kDeltaMax > 0.f) ?
				math::min(fabsf(expected_candidate_shadow_a_limited) / kDeltaMax, 1.0f) : 0.f;
			const float expected_candidate_shadow_b_ratio = (kDeltaMax > 0.f) ?
				math::min(fabsf(expected_candidate_shadow_b_limited) / kDeltaMax, 1.0f) : 0.f;

			snapshot.diag_error_candidate_shadow =
				!nearly_equal(snapshot.delta_mpc_shadow_correction_B, -snapshot.delta_mpc_shadow_correction_A) ||
				!nearly_equal(snapshot.delta_candidate_shadow_A_raw, expected_candidate_shadow_a_raw) ||
				!nearly_equal(snapshot.delta_candidate_shadow_B_raw, expected_candidate_shadow_b_raw) ||
				!nearly_equal(snapshot.delta_candidate_shadow_A_limited, expected_candidate_shadow_a_limited) ||
				!nearly_equal(snapshot.delta_candidate_shadow_B_limited, expected_candidate_shadow_b_limited) ||
				(snapshot.delta_candidate_shadow_A_saturation_flag != expected_candidate_shadow_a_flag) ||
				(snapshot.delta_candidate_shadow_B_saturation_flag != expected_candidate_shadow_b_flag) ||
				!nearly_equal(snapshot.delta_candidate_shadow_A_saturation_ratio, expected_candidate_shadow_a_ratio) ||
				!nearly_equal(snapshot.delta_candidate_shadow_B_saturation_ratio, expected_candidate_shadow_b_ratio);
		}

		snapshot.diag_error_candidate_post_gate =
			(!snapshot.active_output_allowed &&
			 (!nearly_equal(snapshot.delta_candidate_post_gate_A, 0.f) ||
			  !nearly_equal(snapshot.delta_candidate_post_gate_B, 0.f)));

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

		// HydroAllocator.cpp assigns HY_SV0_IDX -> x[0][0] (left hydro servo) and
		// HY_SV1_IDX -> x[1][0] (right hydro servo). We keep the resulting dual-sign
		// diagnostics without asserting final physical sign confirmation.
		left_servo_index = left_servo_param - 1;
		right_servo_index = right_servo_param - 1;
		return true;
	}

	uint8_t base_command_fallback(const Diagnostics &snapshot) const
	{
		if (!snapshot.base_command_valid) {
			if (snapshot.base_command_age_ms > kBaseCommandAgeLimitMs) {
				return static_cast<uint8_t>(FallbackReason::STALE_BASE_COMMAND);
			}

			return static_cast<uint8_t>(FallbackReason::MISSING_BASE_COMMAND);
		}

		return static_cast<uint8_t>(FallbackReason::SIGN_NOT_CONFIRMED);
	}

	static const char *fallback_reason_string(FallbackReason reason)
	{
		switch (reason) {
		case FallbackReason::NONE:
			return "NONE";
		case FallbackReason::SIGN_NOT_CONFIRMED:
			return "SIGN_NOT_CONFIRMED";
		case FallbackReason::RC_DISABLED:
			return "RC_DISABLED";
		case FallbackReason::NOT_ARMED_OR_INVALID_ARM_STATE:
			return "NOT_ARMED_OR_INVALID_ARM_STATE";
		case FallbackReason::ROLL_LIMIT:
			return "ROLL_LIMIT";
		case FallbackReason::ROLL_RATE_LIMIT:
			return "ROLL_RATE_LIMIT";
		case FallbackReason::DELTA_LIMIT:
			return "DELTA_LIMIT";
		case FallbackReason::DELTA_RATE_LIMIT:
			return "DELTA_RATE_LIMIT";
		case FallbackReason::LOW_BATTERY:
			return "LOW_BATTERY";
		case FallbackReason::SOLVER_FAIL:
			return "SOLVER_FAIL";
		case FallbackReason::NAN_DETECTED:
			return "NAN_DETECTED";
		case FallbackReason::SOLVE_TIME_EXCEEDED:
			return "SOLVE_TIME_EXCEEDED";
		case FallbackReason::MISSING_SENSOR_DATA:
			return "MISSING_SENSOR_DATA";
		case FallbackReason::STALE_SENSOR_DATA:
			return "STALE_SENSOR_DATA";
		case FallbackReason::MISSING_BASE_COMMAND:
			return "MISSING_BASE_COMMAND";
		case FallbackReason::STALE_BASE_COMMAND:
			return "STALE_BASE_COMMAND";
		case FallbackReason::UNKNOWN:
		default:
			return "UNKNOWN";
		}
	}

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _hydro_servos_sub{ORB_ID(hydro_servos)};
	uORB::Subscription _hydro_torque_setpoint_sub{ORB_ID(hydro_torque_setpoint)};

	FixedLinearMPC _mpc{};

	hrt_abstime _last_run_timestamp{0};
	uint64_t _loop_count{0};
	float _previous_delta{0.f};
	actuator_servos_s _hydro_servos{};
	vehicle_torque_setpoint_s _hydro_torque_setpoint{};
	param_t _hy_sv_idx_param_handles[2]{PARAM_INVALID, PARAM_INVALID};
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
			PX4_INFO("mode: SHADOW_ONLY");
			PX4_INFO("active_output_allowed: false");
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
