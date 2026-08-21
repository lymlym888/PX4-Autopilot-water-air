/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "HydroRateControl.hpp"

#include <csignal>
#include <cstdlib>
#include <math.h>

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::interpolate;
using math::radians;

namespace
{
volatile sig_atomic_t safe_roll_watch_should_exit{0};

void safeRollWatchSignalHandler(int)
{
	safe_roll_watch_should_exit = 1;
}
}

HydroRateControl::HydroRateControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();
}

HydroRateControl::~HydroRateControl()
{
	perf_free(_loop_perf);
}

bool
HydroRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
HydroRateControl::parameters_update()
{
	const Vector3f rate_p = Vector3f(_param_hy_rr_p.get(), _param_hy_pr_p.get(), _param_hy_yr_p.get());
	const Vector3f rate_i = Vector3f(_param_hy_rr_i.get(), _param_hy_pr_i.get(), _param_hy_yr_i.get());
	const Vector3f rate_d = Vector3f(_param_hy_rr_d.get(), _param_hy_pr_d.get(), _param_hy_yr_d.get());

	_rate_control.setPidGains(rate_p, rate_i, rate_d);

	_rate_control.setIntegratorLimit(
		Vector3f(_param_hy_rr_imax.get(), _param_hy_pr_imax.get(), _param_hy_yr_imax.get()));

	_rate_control.setFeedForwardGain(
		// set FF gains to 0 as we add the FF control outside of the rate controller
		Vector3f(0.f, 0.f, 0.f));

	//CADRC部分
	const float J0    = _param_hy_cadrc_j0.get();
	const float kp1   = _param_hy_cadrc_kp1.get();
	const float kp2   = _param_hy_cadrc_kp2.get();
	const float o1    = _param_hy_cadrc_o1.get();
	const float o2    = _param_hy_cadrc_o2.get();
	const float alpha = _param_hy_cadrc_alp.get();

	_cadrc_roll.setJ0(J0);
	_cadrc_pitch.setJ0(J0);

	_cadrc_roll.setGains(kp1, o1, kp2, o2);
	_cadrc_pitch.setGains(kp1, o1, kp2, o2);

	_cadrc_roll.setAlpha(alpha);
	_cadrc_pitch.setAlpha(alpha);


	return PX4_OK;
}




float HydroRateControl::get_airspeed_and_update_scaling()
{
	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_hy_airspd_trim.get();

	const float airspeed_constrained = constrain(constrain(airspeed, _param_hy_airspd_stall.get(),
					   _param_hy_airspd_max.get()), 0.1f, 1000.0f);

	_airspeed_scaling = (_param_hy_arsp_scale_en.get()) ? (_param_hy_airspd_trim.get() / airspeed_constrained) : 1.0f;

	return airspeed_constrained;
}

void HydroRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if angular velocity changed
	if (_vehicle_angular_velocity_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) {

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		float dt = 0.f;

		static constexpr float DT_MIN = 0.002f;
		static constexpr float DT_MAX = 0.04f;

		vehicle_angular_velocity_s vehicle_angular_velocity{};

		if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
			dt = math::constrain((vehicle_angular_velocity.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = vehicle_angular_velocity.timestamp_sample;
		}

		if (dt < DT_MIN || dt > DT_MAX) {
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
		}

		Vector3f rates(vehicle_angular_velocity.xyz);
		Vector3f angular_accel{vehicle_angular_velocity.xyz_derivative};

			_vehicle_status_sub.update(&_vehicle_status);
			_hydro_allocate_message_sub.update(&_hydro_allocate_message);
			_haar_safe_roll_status_sub.update(&_haar_safe_roll_status);

// --------------------是否切换模式进入stab，是则reset--------------------------------------------------------------
		// detect transition into STAB (edge)
			const bool entered_stab =
			(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB) &&
			(_prev_nav_state != vehicle_status_s::NAVIGATION_STATE_STAB);
			const bool exited_stab =
			(_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_STAB) &&
			(_prev_nav_state == vehicle_status_s::NAVIGATION_STATE_STAB);

		if (entered_stab) {
		// reset CADRC (and optionally other integrators)
		_cadrc_roll.reset();
			_cadrc_pitch.reset();
			}

			if (entered_stab || exited_stab) {
				_safe_roll_controller.reset(false);
			}
		// update previous nav state
		_prev_nav_state = _vehicle_status.nav_state;

//---------------------------------小改动，为了无遥控测试用---------------------------------------
		// //读手动操控信息
		_manual_control_setpoint_sub.copy(&_manual_control_setpoint);
		// // 读手动操控信息：如果没检测到(未发布/无效)，就用0
		// manual_control_setpoint_s manual_sp{};
		// const bool got_manual = _manual_control_setpoint_sub.update(&manual_sp);

		// if (got_manual && manual_sp.valid && manual_sp.timestamp != 0) {
		// 	_manual_control_setpoint = manual_sp;
		// } else {
		// 	// 没RC/无效 -> 全部置0，避免用到脏数据
		// 	_manual_control_setpoint = {};
		// 	_manual_control_setpoint.valid = false;
		// }
//----------------------------------结束--------------------------------------------------------

		//特技模式下，操控量映射到rate_sp
		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO)
		{
			_rates_sp.roll = _manual_control_setpoint.roll * radians(_param_hy_acro_x_max.get());
			_rates_sp.yaw = _manual_control_setpoint.yaw * radians(_param_hy_acro_z_max.get());
			_rates_sp.pitch = -_manual_control_setpoint.pitch * radians(_param_hy_acro_y_max.get());
			_rates_sp.timestamp = hrt_absolute_time();
			_rates_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;

			_rate_sp_pub.publish(_rates_sp);

		}
		//手动模式下，操控量映射到torque和thrust
		else if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL)
		{
			_hydro_torque_setpoint.xyz[0] = math::constrain(_manual_control_setpoint.roll * _param_hy_man_r_sc.get() +
							_param_trim_roll.get(), -1.f, 1.f);
			_hydro_torque_setpoint.xyz[1] = math::constrain(-_manual_control_setpoint.pitch * _param_hy_man_p_sc.get() +
							_param_trim_pitch.get(), -1.f, 1.f);
			_hydro_torque_setpoint.xyz[2] = math::constrain(_manual_control_setpoint.yaw * _param_hy_man_y_sc.get() +
							_param_trim_yaw.get(), -1.f, 1.f);

			_thrust_sp_norm = math::constrain((_manual_control_setpoint.throttle + 1.f) * .5f, 0.f, 1.f);
		}

		//在非手动模式下，执行控制算法
		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO) {

			//CADRC算法增加部分-------------开始
			// 读取姿态角（实时角度 P）
				_vehicle_attitude_sub.update(&_vehicle_attitude);
				const Quatf attitude_quaternion(_vehicle_attitude.q);
				Eulerf euler_angles(attitude_quaternion);

			const float roll  = euler_angles.phi();    // rad
			const float pitch = euler_angles.theta();  // rad

			// 实时角速度 Wx
			const float p = rates(0); // roll rate rad/s
			const float q = rates(1); // pitch rate rad/s

			// STAB 模式：由遥控器映射目标角度 P_sp；没RC则用0
			float roll_sp = 0.f;
			float pitch_sp = 0.f;
			if (_manual_control_setpoint.valid && _manual_control_setpoint.timestamp != 0) {
				roll_sp  = _manual_control_setpoint.roll  * radians(_param_hy_roll_max.get());
				pitch_sp = -_manual_control_setpoint.pitch * radians(_param_hy_pitch_max.get());
			}

			// tau：支持地面站覆盖（0=用真实dt）
			const float tau_override = _param_hy_cadrc_tau.get();
			const float dt_used = (tau_override > 0.f) ? tau_override : dt;

			if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB
			&& _param_hy_cadrc_en.get()) {

				// CADRC：P_sp, P, Wx -> Wx_p -> Uc
				_cadrc_roll.update(roll, roll_sp, p, dt_used);
				_cadrc_pitch.update(pitch, pitch_sp, q, dt_used);

				// 记录
				_hydro_cadrc_control_message.roll_sp = roll_sp;
				_hydro_cadrc_control_message.pitch_sp = pitch_sp;
				_hydro_cadrc_control_message.roll_z11 = _cadrc_roll._Z1(0,0);
				_hydro_cadrc_control_message.roll_z12 = _cadrc_roll._Z1(1,0);
				_hydro_cadrc_control_message.pitch_z11 = _cadrc_pitch._Z1(0,0);
				_hydro_cadrc_control_message.pitch_z12 = _cadrc_pitch._Z1(1,0);

				// 把 Wx_p 发出来做日志（方便在QGC plot看“期望角速度”）
				// _rates_sp.roll = _cadrc_roll.getRateSp();
				// _rates_sp.pitch = _cadrc_pitch.getRateSp();
				// _rates_sp.yaw = 0.f;
				// _rates_sp.timestamp = hrt_absolute_time();
				// _rate_sp_pub.publish(_rates_sp);

				// Uc -> 归一化 torque（交给 control allocation）
				const float u_scale = _param_hy_cadrc_u_scale.get();
				float u_roll  = math::constrain(_cadrc_roll.getUc()  * u_scale, -1.f, 1.f);
				float u_pitch = math::constrain(_cadrc_pitch.getUc() * u_scale, -1.f, 1.f);

				// 记录
				_hydro_cadrc_control_message.u_roll = _cadrc_roll.getUc()  * u_scale;
				_hydro_cadrc_control_message.u_pitch = _cadrc_pitch.getUc() * u_scale;

				// yaw：先保持最简单的手动映射（后面再决定是否也用CADRC）
				float u_yaw = math::constrain(_manual_control_setpoint.yaw * _param_hy_man_y_sc.get()
								+ _param_trim_yaw.get(), -1.f, 1.f);

					// 加 trim（保持与原框架一致）
					u_roll  = math::constrain(u_roll  + _param_trim_roll.get(),  -1.f, 1.f);
					u_pitch = math::constrain(u_pitch + _param_trim_pitch.get(), -1.f, 1.f);
					const float legacy_u_roll = u_roll;
					SafeRollController::Input safe_roll_input{};
					safe_roll_input.now = hrt_absolute_time();
					safe_roll_input.angular_velocity_timestamp_sample = vehicle_angular_velocity.timestamp_sample;
					safe_roll_input.attitude_timestamp = _vehicle_attitude.timestamp;
					safe_roll_input.allocator_timestamp = _hydro_allocate_message.timestamp;
					safe_roll_input.requested_mode = static_cast<uint8_t>(_param_hy_sr_mode.get());
					safe_roll_input.armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
					safe_roll_input.in_stab = true;
					safe_roll_input.quaternion_finite = attitude_quaternion.isAllFinite();
					safe_roll_input.angular_velocity_finite = rates.isAllFinite();
					safe_roll_input.allocator_valid = _hydro_allocate_message.allocator_valid;
					safe_roll_input.allocator_converged = _hydro_allocate_message.allocator_converged;
					safe_roll_input.allocator_finite = _hydro_allocate_message.allocator_finite;
					safe_roll_input.allocator_bounded = _hydro_allocate_message.allocator_bounded;
					safe_roll_input.allocator_saturation_mask =
						_hydro_allocate_message.allocator_saturation_mask;
					safe_roll_input.foil_gate_enabled = _hydro_allocate_message.foil_gate_enabled;
					safe_roll_input.aux_gate_enabled_raw =
						_hydro_allocate_message.aux_gate_enabled_raw;
					safe_roll_input.nav_gate_enabled_raw =
						_hydro_allocate_message.nav_gate_enabled_raw;
					safe_roll_input.aux_gate_data_valid =
						_hydro_allocate_message.aux_gate_data_valid;
					safe_roll_input.aux_gate_value_finite =
						_hydro_allocate_message.aux_gate_value_finite;
					safe_roll_input.gate_close_reason = _hydro_allocate_message.gate_close_reason;
					safe_roll_input.foil_aux_parameter = _hydro_allocate_message.foil_aux_parameter;
					safe_roll_input.foil_aux_value = _hydro_allocate_message.foil_aux_value;
					safe_roll_input.foil_threshold_parameter =
						_hydro_allocate_message.foil_threshold_parameter;
					safe_roll_input.foil_aux_rc_channel = _hydro_allocate_message.foil_aux_rc_channel;
					safe_roll_input.foil_aux_mapping_valid =
						_hydro_allocate_message.foil_aux_mapping_valid;
					safe_roll_input.manual_control_timestamp =
						_hydro_allocate_message.manual_control_timestamp;
					safe_roll_input.manual_control_timestamp_sample =
						_hydro_allocate_message.manual_control_timestamp_sample;
					safe_roll_input.manual_control_age_us =
						_hydro_allocate_message.manual_control_age_us;
					safe_roll_input.manual_control_data_source =
						_hydro_allocate_message.manual_control_data_source;
					safe_roll_input.manual_control_valid =
						_hydro_allocate_message.manual_control_valid;
					safe_roll_input.rc_signal_lost = _hydro_allocate_message.rc_signal_lost;
					safe_roll_input.rc_failsafe = _hydro_allocate_message.rc_failsafe;
					safe_roll_input.logical_mapping_valid = _hydro_allocate_message.logical_mapping_matches_expected;
					safe_roll_input.physical_mapping_verified = _param_hy_sr_map_ok.get() == 1;
					safe_roll_input.servo_ownership_verified = _param_hy_sr_own_ok.get() == 1;
					safe_roll_input.attitude_reset_counter = _vehicle_attitude.quat_reset_counter;
					safe_roll_input.manual_roll = _manual_control_setpoint.roll;
					safe_roll_input.phi_cmd = roll_sp;
					safe_roll_input.phi = roll;
					safe_roll_input.p = p;
					safe_roll_input.legacy_u_roll = legacy_u_roll;
					safe_roll_input.generic_servo4 = _haar_safe_roll_status.generic_servo4;
					safe_roll_input.generic_servo5 = _haar_safe_roll_status.generic_servo5;
					safe_roll_input.previous_final_servo4 = _haar_safe_roll_status.final_servo4;
					safe_roll_input.previous_final_servo5 = _haar_safe_roll_status.final_servo5;
					safe_roll_input.previous_u_effective_model = _haar_safe_roll_status.u_effective_model;
					safe_roll_input.allocation_context.lift_coefficient = _hydro_allocate_message.lift_coefficient;
					safe_roll_input.allocation_context.speed_squared = _hydro_allocate_message.speed_squared;
					safe_roll_input.allocation_context.body_pitch = _hydro_allocate_message.body_pitch;
					safe_roll_input.allocation_context.max_hydrofoil_angle =
						_hydro_allocate_message.context_max_hydrofoil_angle;
					safe_roll_input.allocation_context.max_thruster_thrust =
						_hydro_allocate_message.max_thruster_thrust;
					safe_roll_input.allocation_context.available_thruster_thrust =
						_hydro_allocate_message.available_thruster_thrust;

					for (int row = 0; row < 4; ++row) {
						for (int column = 0; column < 3; ++column) {
							safe_roll_input.allocation_context.virtual_control_matrix(row, column) =
								_hydro_allocate_message.virtual_control_matrix[row * 3 + column];
						}
					}

					safe_roll_input.allocation_command.force_x = _hydro_allocate_message.command_force_x;
					safe_roll_input.allocation_command.force_z = _hydro_allocate_message.command_force_z;
					safe_roll_input.allocation_command.u_phi = _hydro_allocate_message.command_u_phi;
					safe_roll_input.allocation_command.u_theta = _hydro_allocate_message.command_u_theta;
					safe_roll_input.allocation_command.u_psi = _hydro_allocate_message.command_u_psi;
					SafeRollController::Config &config = safe_roll_input.config;
					config.enable = static_cast<uint8_t>(_param_hy_sr_enable.get());
					const bool requested_field_mode = _param_hy_sr_field.get() != 0;

					if (!_safe_roll_field_initialized
					    || safe_roll_input.requested_mode
					    == static_cast<uint8_t>(SafeRollController::Mode::Legacy)) {
						_safe_roll_field_effective = requested_field_mode;
						_safe_roll_field_initialized = true;
					}

					config.field_mode = _safe_roll_field_effective;
					config.shadow = _param_hy_sr_shadow.get() != 0;
					config.trial_id = static_cast<uint16_t>(math::constrain(_param_hy_sr_trial.get(),
							  static_cast<int32_t>(0), static_cast<int32_t>(65535)));
					config.reset_id = static_cast<uint32_t>(_param_hy_sr_rst_id.get());
					config.blend_time = _param_hy_sr_blend.get();
					config.reference_wn = _param_hy_sr_rfwn.get();
					config.reference_zeta = _param_hy_sr_rfz.get();
					config.reference_rate_max = radians(_param_hy_sr_rfpmax.get());
					config.reference_accel_max = radians(_param_hy_sr_rfamax.get());
					config.a0 = _param_hy_sr_a0.get();
					config.b0 = _param_hy_sr_b0.get();
					config.b_min = _param_hy_sr_bmin.get();
					config.b_max = _param_hy_sr_bmax.get();
					config.phi_safe = radians(_param_hy_sr_phis.get());
					config.u_max = _param_hy_sr_umax.get();
					config.du_max_nominal = _param_hy_sr_dumax.get();
					config.foil_angle_max = radians(_param_hy_sr_dmax.get());
					config.foil_rate_max = radians(_param_hy_sr_drate.get());
					config.foil_remainder = radians(_param_hy_sr_drem.get());
					config.filter_lambda = _param_hy_sr_flam.get();
					config.b_adaptation_gain = _param_hy_sr_gb.get();
					config.b_adaptation_epsilon = _param_hy_sr_epsb.get();
					config.filtered_u_min = _param_hy_sr_ufmin.get();
					config.innovation_gate = _param_hy_sr_igate.get();
					config.b_update_max = _param_hy_sr_bdmax.get();
					config.disturbance_gain = _param_hy_sr_ld.get();
					config.disturbance_radius_initial = _param_hy_sr_rd0.get();
					config.disturbance_variation_nominal = _param_hy_sr_ddb.get();
					config.effectiveness_variation_nominal = _param_hy_sr_dbb.get();
					config.incremental_u_min = _param_hy_sr_iumin.get();
					config.incremental_residual = _param_hy_sr_ires.get();
					config.incremental_gate_factor = _param_hy_sr_igfac.get();
					config.b_radius_initial = _param_hy_sr_rb0.get();
					config.k1 = _param_hy_sr_k1.get();
					config.k2 = _param_hy_sr_k2.get();
					config.gamma1 = _param_hy_sr_g1.get();
					config.gamma2 = _param_hy_sr_g2.get();
					config.robust_gain_floor = _param_hy_sr_bfloor.get();

					const hrt_abstime safe_roll_start = hrt_absolute_time();
					SafeRollController::Output safe_roll_output{};

					// Mode 0 is a direct legacy path: no candidate safe controller is evaluated.
					if (safe_roll_input.requested_mode == static_cast<uint8_t>(SafeRollController::Mode::Legacy)) {
						safe_roll_output = _safe_roll_controller.observeLegacy(safe_roll_input);

					} else {
						safe_roll_output = _safe_roll_controller.update(safe_roll_input);
					}

					u_roll = safe_roll_output.u_selected;
					const uint32_t safe_roll_cycle_time_us = hrt_elapsed_time(&safe_roll_start);

					if (safe_roll_output.first_fault_code == 0) {
						_first_fault_cycle_time_us = 0;

					} else if (_previous_safe_roll_first_fault_code == 0) {
						_first_fault_cycle_time_us = safe_roll_cycle_time_us;
					}

					if (safe_roll_input.requested_mode != _previous_safe_roll_requested_mode) {
						if (safe_roll_input.requested_mode != static_cast<uint8_t>(SafeRollController::Mode::Legacy)) {
							PX4_INFO("HY_SR_MODE requested=%u active=%u fault=0x%08x",
								 unsigned(safe_roll_input.requested_mode),
								 unsigned(safe_roll_output.active_code),
								 unsigned(safe_roll_output.fault_flags));
						}

						_previous_safe_roll_requested_mode = safe_roll_input.requested_mode;
					}

					if (safe_roll_output.first_fault_code != 0
					    && safe_roll_output.first_fault_code != _previous_safe_roll_first_fault_code) {
						PX4_WARN("SafeRoll fallback first_fault=0x%08x stages=0x%08x",
							 unsigned(safe_roll_output.first_fault_code),
							 unsigned(safe_roll_output.execution_stage_mask));
					}

					_previous_safe_roll_first_fault_code = safe_roll_output.first_fault_code;

					_hydro_cadrc_control_message.roll_z21 = _cadrc_roll._Z2(0,0);
				_hydro_cadrc_control_message.roll_z22 = _cadrc_roll._Z2(1,0);
				_hydro_cadrc_control_message.pitch_z21 = _cadrc_pitch._Z2(0,0);
					_hydro_cadrc_control_message.pitch_z22 = _cadrc_pitch._Z2(1,0);
					_hydro_cadrc_control_message.timestamp = hrt_absolute_time();
					_hydro_cadrc_control_message.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
					_hydro_cadrc_control_message.depth_setpoint = _param_hy_dc_sp.get();
					_hydro_cadrc_control_message.requested_mode = safe_roll_output.requested_code;
					_hydro_cadrc_control_message.active_mode = safe_roll_output.active_code;
					_hydro_cadrc_control_message.reset_count = safe_roll_output.reset_count;
					_hydro_cadrc_control_message.fault_flags = safe_roll_output.fault_flags;
					_hydro_cadrc_control_message.first_fault_code = safe_roll_output.first_fault_code;
					_hydro_cadrc_control_message.execution_stage_mask = safe_roll_output.execution_stage_mask;
					_hydro_cadrc_control_message.first_fault_stage_mask = safe_roll_output.first_fault_stage_mask;
					_hydro_cadrc_control_message.first_fault_total_candidate_time_us =
						safe_roll_output.first_fault_total_candidate_time_us;
					_hydro_cadrc_control_message.first_fault_reference_filter_time_us =
						safe_roll_output.first_fault_reference_filter_time_us;
					_hydro_cadrc_control_message.first_fault_estimator_time_us =
						safe_roll_output.first_fault_estimator_time_us;
					_hydro_cadrc_control_message.first_fault_certificate_time_us =
						safe_roll_output.first_fault_certificate_time_us;
					_hydro_cadrc_control_message.first_fault_allocation_query_time_us =
						safe_roll_output.first_fault_allocation_query_time_us;
					_hydro_cadrc_control_message.first_fault_nominal_time_us =
						safe_roll_output.first_fault_nominal_time_us;
					_hydro_cadrc_control_message.first_fault_cbf_time_us =
						safe_roll_output.first_fault_cbf_time_us;
					_hydro_cadrc_control_message.first_fault_nonlinear_check_time_us =
						safe_roll_output.first_fault_nonlinear_check_time_us;
					_hydro_cadrc_control_message.first_fault_cycle_time_us = _first_fault_cycle_time_us;
					_hydro_cadrc_control_message.runtime_soft_budget_us = safe_roll_output.runtime_soft_budget_us;
					_hydro_cadrc_control_message.runtime_hard_budget_us = safe_roll_output.runtime_hard_budget_us;
					_hydro_cadrc_control_message.consecutive_runtime_overrun_count =
						safe_roll_output.consecutive_runtime_overrun_count;
					_hydro_cadrc_control_message.allocator_valid_streak =
						safe_roll_output.allocator_valid_streak;
					_hydro_cadrc_control_message.allocator_invalid_streak =
						safe_roll_output.allocator_invalid_streak;
					_hydro_cadrc_control_message.activation_required_streak =
						safe_roll_output.activation_required_streak;
					_hydro_cadrc_control_message.activation_pending = safe_roll_output.activation_pending;
					_hydro_cadrc_control_message.temporary_fallback_active =
						safe_roll_output.temporary_fallback_active;
					_hydro_cadrc_control_message.healthy = safe_roll_output.healthy;
					_hydro_cadrc_control_message.fallback_active = safe_roll_output.fallback_active;
					_hydro_cadrc_control_message.dt_raw = safe_roll_output.dt_raw;
					_hydro_cadrc_control_message.dt_used = safe_roll_output.dt_used;
					_hydro_cadrc_control_message.manual_roll = _manual_control_setpoint.roll;
					_hydro_cadrc_control_message.phi = roll;
					_hydro_cadrc_control_message.p = p;
					_hydro_cadrc_control_message.u_legacy = legacy_u_roll;
					_hydro_cadrc_control_message.u_nominal = safe_roll_output.u_nominal;
					_hydro_cadrc_control_message.u_selected = u_roll;
					_hydro_cadrc_control_message.physical_mapping_verified = safe_roll_input.physical_mapping_verified;
					_hydro_cadrc_control_message.servo_ownership_verified = safe_roll_input.servo_ownership_verified;
					_hydro_cadrc_control_message.safe_roll_hardware_ready = safe_roll_output.safe_roll_hardware_ready;
					_hydro_cadrc_control_message.cycle_time_us = safe_roll_cycle_time_us;
					_hydro_cadrc_control_message.requested_code = safe_roll_output.requested_code;
					_hydro_cadrc_control_message.active_code = safe_roll_output.active_code;
					_hydro_cadrc_control_message.trial_id = safe_roll_output.trial_id;
					_hydro_cadrc_control_message.configuration_crc32 = safe_roll_output.configuration_crc32;
					_hydro_cadrc_control_message.shadow_active = safe_roll_output.shadow_active;
					_hydro_cadrc_control_message.output_enabled = safe_roll_output.output_enabled;
					_hydro_cadrc_control_message.phi_cmd = safe_roll_output.phi_cmd;
					_hydro_cadrc_control_message.phi_ref = safe_roll_output.phi_ref;
					_hydro_cadrc_control_message.phi_ref_dot = safe_roll_output.phi_ref_dot;
					_hydro_cadrc_control_message.phi_ref_ddot = safe_roll_output.phi_ref_ddot;
					_hydro_cadrc_control_message.u_candidate = safe_roll_output.u_candidate;
					_hydro_cadrc_control_message.u_blended = safe_roll_output.u_blended;
					_hydro_cadrc_control_message.u_effective_model = safe_roll_output.u_effective_model;
					_hydro_cadrc_control_message.delta_left_commanded = safe_roll_output.delta_left_commanded;
					_hydro_cadrc_control_message.delta_right_commanded = safe_roll_output.delta_right_commanded;
					_hydro_cadrc_control_message.foil_rate_left_commanded =
						safe_roll_output.foil_rate_left_commanded;
					_hydro_cadrc_control_message.foil_rate_right_commanded =
						safe_roll_output.foil_rate_right_commanded;
					_hydro_cadrc_control_message.candidate_valid = safe_roll_output.candidate_valid;
					_hydro_cadrc_control_message.interval_valid = safe_roll_output.interval_valid;
					_hydro_cadrc_control_message.estimator_ready = safe_roll_output.estimator_ready;
					_hydro_cadrc_control_message.certificate_ready = safe_roll_output.certificate_ready;
					_hydro_cadrc_control_message.certificate_warmup = safe_roll_output.certificate_warmup;
					_hydro_cadrc_control_message.allocation_interval_degraded =
						safe_roll_output.allocation_interval_degraded;
					haar_safe_roll_estimator_status_s estimator_status{};
					estimator_status.timestamp = _hydro_cadrc_control_message.timestamp;
					estimator_status.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
					estimator_status.requested_code = safe_roll_output.requested_code;
					estimator_status.active_code = safe_roll_output.active_code;
					estimator_status.trial_id = safe_roll_output.trial_id;
					estimator_status.configuration_crc32 = safe_roll_output.configuration_crc32;
					estimator_status.first_fault_code = safe_roll_output.first_fault_code;
					estimator_status.execution_stage_mask = safe_roll_output.execution_stage_mask;
					estimator_status.first_fault_stage_mask = safe_roll_output.first_fault_stage_mask;
					estimator_status.first_fault_total_candidate_time_us =
						safe_roll_output.first_fault_total_candidate_time_us;
					estimator_status.first_fault_reference_filter_time_us =
						safe_roll_output.first_fault_reference_filter_time_us;
					estimator_status.first_fault_estimator_time_us =
						safe_roll_output.first_fault_estimator_time_us;
					estimator_status.first_fault_certificate_time_us =
						safe_roll_output.first_fault_certificate_time_us;
					estimator_status.first_fault_allocation_query_time_us =
						safe_roll_output.first_fault_allocation_query_time_us;
					estimator_status.first_fault_nominal_time_us =
						safe_roll_output.first_fault_nominal_time_us;
					estimator_status.first_fault_cbf_time_us = safe_roll_output.first_fault_cbf_time_us;
					estimator_status.first_fault_nonlinear_check_time_us =
						safe_roll_output.first_fault_nonlinear_check_time_us;
					estimator_status.first_fault_cycle_time_us = _first_fault_cycle_time_us;
					estimator_status.runtime_soft_budget_us = safe_roll_output.runtime_soft_budget_us;
					estimator_status.runtime_hard_budget_us = safe_roll_output.runtime_hard_budget_us;
					estimator_status.consecutive_runtime_overrun_count =
						safe_roll_output.consecutive_runtime_overrun_count;
					estimator_status.allocator_recovery_streak = safe_roll_output.allocator_recovery_streak;
					estimator_status.allocator_invalid_duration_us =
						safe_roll_output.allocator_invalid_duration_us;
					estimator_status.allocator_degraded_active = safe_roll_output.allocator_degraded_active;
					estimator_status.temporary_fallback_active = safe_roll_output.temporary_fallback_active;
					estimator_status.safety_degraded_active = safe_roll_output.safety_degraded_active;
					estimator_status.last_feasible_u = safe_roll_output.last_feasible_u;
					estimator_status.phi_cmd = safe_roll_output.phi_cmd;
					estimator_status.phi_ref = safe_roll_output.phi_ref;
					estimator_status.phi_ref_dot = safe_roll_output.phi_ref_dot;
					estimator_status.phi_ref_ddot = safe_roll_output.phi_ref_ddot;
					estimator_status.u_candidate = safe_roll_output.u_candidate;
					estimator_status.u_blended = safe_roll_output.u_blended;
					estimator_status.b_hat = safe_roll_output.b_hat;
					estimator_status.b_lower = safe_roll_output.b_lower;
					estimator_status.b_upper = safe_roll_output.b_upper;
					estimator_status.rho_b = safe_roll_output.rho_b;
					estimator_status.d_hat = safe_roll_output.d_hat;
					estimator_status.rho_d = safe_roll_output.rho_d;
					estimator_status.certified_width = safe_roll_output.certified_width;
					estimator_status.filtered_u = safe_roll_output.filtered_u;
					estimator_status.filtered_r = safe_roll_output.filtered_r;
					estimator_status.filtered_innovation = safe_roll_output.filtered_innovation;
					estimator_status.b_update_active = safe_roll_output.b_update_active;
					estimator_status.certificate_update_active = safe_roll_output.certificate_update_active;
					estimator_status.certificate_conflict = safe_roll_output.certificate_conflict;
					estimator_status.interval_valid = safe_roll_output.interval_valid;
					estimator_status.estimator_ready = safe_roll_output.estimator_ready;
					estimator_status.certificate_ready = safe_roll_output.certificate_ready;
					estimator_status.certificate_warmup = safe_roll_output.certificate_warmup;
					estimator_status.allocation_interval_degraded =
						safe_roll_output.allocation_interval_degraded;
					estimator_status.accepted_lag_mask = safe_roll_output.accepted_lag_mask;
					estimator_status.conflict_lag_mask = safe_roll_output.conflict_lag_mask;
					estimator_status.u_amp_lower = safe_roll_output.u_amp.lower;
					estimator_status.u_amp_upper = safe_roll_output.u_amp.upper;
					estimator_status.u_rate_lower = safe_roll_output.u_rate.lower;
					estimator_status.u_rate_upper = safe_roll_output.u_rate.upper;
					estimator_status.u_alloc_lower = safe_roll_output.u_alloc.lower;
					estimator_status.u_alloc_upper = safe_roll_output.u_alloc.upper;
					estimator_status.u_hard_lower = safe_roll_output.u_hard.lower;
					estimator_status.u_hard_upper = safe_roll_output.u_hard.upper;
					estimator_status.u_cbf_lower = safe_roll_output.u_cbf.lower;
					estimator_status.u_cbf_upper = safe_roll_output.u_cbf.upper;
					estimator_status.u_safe_lower = safe_roll_output.u_safe.lower;
					estimator_status.u_safe_upper = safe_roll_output.u_safe.upper;
					estimator_status.epsilon_relax = safe_roll_output.epsilon_relax;
					estimator_status.h_upper = safe_roll_output.h_upper;
					estimator_status.h_lower = safe_roll_output.h_lower;
					estimator_status.psi_upper = safe_roll_output.psi_upper;
					estimator_status.psi_lower = safe_roll_output.psi_lower;
					estimator_status.psi_min = safe_roll_output.psi_min;
					estimator_status.reference_filter_time_us = safe_roll_output.reference_filter_time_us;
					estimator_status.estimator_time_us = safe_roll_output.estimator_time_us;
					estimator_status.certificate_time_us = safe_roll_output.certificate_time_us;
					estimator_status.cbf_time_us = safe_roll_output.cbf_time_us;
					estimator_status.allocation_query_time_us = safe_roll_output.allocation_query_time_us;
					estimator_status.nominal_time_us = safe_roll_output.nominal_time_us;
					estimator_status.nonlinear_check_time_us = safe_roll_output.nonlinear_check_time_us;
					estimator_status.total_candidate_time_us = safe_roll_output.total_candidate_time_us;
					_haar_safe_roll_estimator_status_pub.publish(estimator_status);
					const SafeRollController::FirstFaultSnapshot &fault_snapshot =
						safe_roll_output.first_fault_snapshot;
					haar_safe_roll_fault_snapshot_s fault_status{};
					fault_status.timestamp = _hydro_cadrc_control_message.timestamp;
					fault_status.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
					fault_status.first_fault_code = safe_roll_output.first_fault_code;
					fault_status.prerequisite_fault_flags = fault_snapshot.prerequisite_fault_flags;
					fault_status.reset_count = fault_snapshot.reset_count;
					fault_status.requested_mode = fault_snapshot.requested_mode;
					fault_status.attitude_reset_counter = fault_snapshot.attitude_reset_counter;
					fault_status.history_count = fault_snapshot.history_count;
					fault_status.allocator_timestamp = fault_snapshot.allocator_timestamp;
					fault_status.allocator_age_us = fault_snapshot.allocator_age_us;
					fault_status.activation_pending = fault_snapshot.activation_pending;
					fault_status.mode_activated = fault_snapshot.mode_activated;
					fault_status.in_stab = fault_snapshot.in_stab;
					fault_status.allocator_timestamp_valid = fault_snapshot.allocator_timestamp_valid;
					fault_status.allocator_valid = fault_snapshot.allocator_valid;
					fault_status.allocator_converged = fault_snapshot.allocator_converged;
					fault_status.allocator_finite = fault_snapshot.allocator_finite;
					fault_status.allocator_bounded = fault_snapshot.allocator_bounded;
					fault_status.foil_gate_enabled = fault_snapshot.foil_gate_enabled;
					fault_status.foil_aux_above_threshold = fault_snapshot.foil_aux_above_threshold;
					fault_status.logical_mapping_valid = fault_snapshot.logical_mapping_valid;
					fault_status.estimator_initialized = fault_snapshot.estimator_initialized;
					fault_status.foil_aux_parameter = fault_snapshot.foil_aux_parameter;
					fault_status.foil_aux_value = fault_snapshot.foil_aux_value;
					fault_status.foil_threshold_parameter = fault_snapshot.foil_threshold_parameter;
					fault_status.previous_u_effective_model = fault_snapshot.previous_u_effective_model;
					fault_status.b_hat = fault_snapshot.b_hat;
					fault_status.b_lower = fault_snapshot.b_lower;
					fault_status.b_upper = fault_snapshot.b_upper;
					fault_status.rho_b = fault_snapshot.rho_b;
					fault_status.d_hat = fault_snapshot.d_hat;
					fault_status.rho_d = fault_snapshot.rho_d;
					fault_status.filtered_u = fault_snapshot.filtered_u;
					fault_status.filtered_r = fault_snapshot.filtered_r;
					_haar_safe_roll_fault_snapshot_pub.publish(fault_status);
					haar_safe_roll_gate_status_s gate_status{};
					gate_status.timestamp = _hydro_cadrc_control_message.timestamp;
					gate_status.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
					gate_status.requested_code = safe_roll_output.requested_code;
					gate_status.active_code = safe_roll_output.active_code;
					gate_status.trial_id = safe_roll_output.trial_id;
					gate_status.configuration_crc32 = safe_roll_output.configuration_crc32;
					gate_status.aux_gate_enabled_raw = safe_roll_input.aux_gate_enabled_raw;
					gate_status.nav_gate_enabled_raw = safe_roll_input.nav_gate_enabled_raw;
					gate_status.foil_gate_enabled = safe_roll_input.foil_gate_enabled;
					gate_status.foil_aux_value = safe_roll_input.foil_aux_value;
					gate_status.foil_aux_above_threshold = safe_roll_input.aux_gate_enabled_raw;
					gate_status.foil_threshold_parameter = safe_roll_input.foil_threshold_parameter;
					gate_status.foil_aux_parameter = safe_roll_input.foil_aux_parameter;
					gate_status.foil_aux_rc_channel = safe_roll_input.foil_aux_rc_channel;
					gate_status.foil_aux_mapping_valid = safe_roll_input.foil_aux_mapping_valid;
					gate_status.manual_control_timestamp = safe_roll_input.manual_control_timestamp;
					gate_status.manual_control_timestamp_sample =
						safe_roll_input.manual_control_timestamp_sample;
					gate_status.manual_control_age_us = safe_roll_input.manual_control_age_us;
					gate_status.manual_control_data_source = safe_roll_input.manual_control_data_source;
					gate_status.manual_control_valid = safe_roll_input.manual_control_valid;
					gate_status.rc_signal_lost = safe_roll_input.rc_signal_lost;
					gate_status.rc_failsafe = safe_roll_input.rc_failsafe;
					gate_status.aux_gate_low_streak = safe_roll_output.aux_gate_low_streak;
					gate_status.aux_gate_high_streak = safe_roll_output.aux_gate_high_streak;
					gate_status.aux_gate_inhibit_active = safe_roll_output.aux_gate_inhibit_active;
					gate_status.gate_close_reason = safe_roll_input.gate_close_reason;
					_haar_safe_roll_gate_status_pub.publish(gate_status);
					haar_safe_roll_field_status_s field_status{};
					field_status.timestamp = _hydro_cadrc_control_message.timestamp;
					field_status.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
					field_status.requested_code = safe_roll_output.requested_code;
					field_status.active_code = safe_roll_output.active_code;
					field_status.trial_id = safe_roll_output.trial_id;
					field_status.configuration_crc32 = safe_roll_output.configuration_crc32;
					field_status.field_mode_active = safe_roll_output.field_mode_active;
					field_status.best_feasible_active = safe_roll_output.best_feasible_active;
					field_status.temporary_fallback_active = safe_roll_output.temporary_fallback_active;
					field_status.safety_degraded_active = safe_roll_output.safety_degraded_active;
					field_status.severe_fault_code = safe_roll_output.severe_fault_code;
					field_status.cumulative_best_feasible_time_us =
						safe_roll_output.cumulative_best_feasible_time_us;
					field_status.cumulative_temporary_fallback_time_us =
						safe_roll_output.cumulative_temporary_fallback_time_us;
					field_status.cumulative_requested_mode_active_time_us =
						safe_roll_output.cumulative_requested_mode_active_time_us;
					_haar_safe_roll_field_status_pub.publish(field_status);

				_hydro_torque_setpoint.xyz[0] = u_roll;
				_hydro_torque_setpoint.xyz[1] = u_pitch;
				_hydro_torque_setpoint.xyz[2] = u_yaw;

				// 推力：STAB 下也由遥控器给一个基准
				_thrust_sp_norm = math::constrain((_manual_control_setpoint.throttle + 1.f) * 0.5f, 0.f, 1.f);

				// 发布!!!!!!
				_hydro_cadrc_control_message_pub.publish(_hydro_cadrc_control_message);

			} else {

			//CADRC算法增加部分----------结束

				const float airspeed = get_airspeed_and_update_scaling();

				/* reset integrals where needed */
				if (_rates_sp.reset_integral) {
					_rate_control.resetIntegral();
				}

				/* bi-linear interpolation over airspeed for actuator trim scheduling */
				Vector3f trim(_param_trim_roll.get(), _param_trim_pitch.get(), _param_trim_yaw.get());

				if (airspeed < _param_hy_airspd_trim.get()) {
					trim(0) += interpolate(airspeed, _param_hy_airspd_min.get(), _param_hy_airspd_trim.get(),
							_param_hy_dtrim_r_vmin.get(),
							0.0f);
					trim(1) += interpolate(airspeed, _param_hy_airspd_min.get(), _param_hy_airspd_trim.get(),
							_param_hy_dtrim_p_vmin.get(),
							0.0f);
					trim(2) += interpolate(airspeed, _param_hy_airspd_min.get(), _param_hy_airspd_trim.get(),
							_param_hy_dtrim_y_vmin.get(),
							0.0f);

				} else {
					trim(0) += interpolate(airspeed, _param_hy_airspd_trim.get(), _param_hy_airspd_max.get(), 0.0f,
							_param_hy_dtrim_r_vmax.get());
					trim(1) += interpolate(airspeed, _param_hy_airspd_trim.get(), _param_hy_airspd_max.get(), 0.0f,
							_param_hy_dtrim_p_vmax.get());
					trim(2) += interpolate(airspeed, _param_hy_airspd_trim.get(), _param_hy_airspd_max.get(), 0.0f,
							_param_hy_dtrim_y_vmax.get());
				}

				_rates_sp_sub.update(&_rates_sp);

				Vector3f body_rates_setpoint = Vector3f(_rates_sp.roll, _rates_sp.pitch, _rates_sp.yaw);

				// Run attitude RATE controllers which need the desired attitudes from above, add trim.
				const Vector3f angular_acceleration_setpoint = _rate_control.update(rates, body_rates_setpoint, angular_accel, dt,
						false);

				const Vector3f gain_ff(_param_hy_rr_ff.get(), _param_hy_pr_ff.get(), _param_hy_yr_ff.get());
				const Vector3f feedforward = gain_ff.emult(body_rates_setpoint) * _airspeed_scaling;

				Vector3f control_u = angular_acceleration_setpoint * _airspeed_scaling * _airspeed_scaling + feedforward;

				// Special case yaw in Acro: if the parameter HY_ACRO_YAW_CTL is not set then don't control yaw
				if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO && !_param_hy_acro_yaw_en.get()) {
					control_u(2) = _manual_control_setpoint.yaw * _param_hy_man_y_sc.get();
					_rate_control.resetIntegral(2);
				}

				if (control_u.isAllFinite()) {
					matrix::constrain(control_u + trim, -1.f, 1.f).copyTo(_hydro_torque_setpoint.xyz);

				} else {
					_rate_control.resetIntegral();
					trim.copyTo(_hydro_torque_setpoint.xyz);
				}

				/* throttle passed through if it is finite */
				_thrust_sp_norm = PX4_ISFINITE(_rates_sp.thrust_body[0]) ? _rates_sp.thrust_body[0] : 0.0f;

				/* scale effort by battery status */
				if (_param_hy_bat_scale_en.get() && _thrust_sp_norm > 0.1f) {

					if (_battery_status_sub.updated()) {
						battery_status_s battery_status{};

						if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
							_battery_scale = battery_status.scale;
						}
					}

					_thrust_sp_norm *= _battery_scale;
				}
			}

		} else {
			_rate_control.resetIntegral();
		}

		//执行补偿操作和滑行深度控制
		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL)
		{
			// Add feed-forward from roll control output to yaw control output
			// This can be used to counteract the adverse yaw effect when rolling the plane
			_hydro_torque_setpoint.xyz[2] = math::constrain(_hydro_torque_setpoint.xyz[2] + _param_hy_rll_to_yaw_ff.get() *
			_hydro_torque_setpoint.xyz[0], -1.f, 1.f);

			//推力前馈到pit轴力矩上
			_hydro_torque_setpoint.xyz[1] = math::constrain(_hydro_torque_setpoint.xyz[1] + _param_thr_to_pit_ff.get() *
			_thrust_sp_norm, -1.f, 1.f);

			//读取消息
			_vehicle_attitude_sub.update(&_vehicle_attitude);
			Eulerf euler_angles(Quatf(_vehicle_attitude.q));
			_depth_fusion_sub.update(&_depth_fusion);
			_debug_key_value_sub.update(&_debug_key_value);
			_vehicle_air_data_sub.update(&_vehicle_air_data);
			_depth_estimated_sub.update(&_depth_estimated);
			_debug_vect_sub.update(&_debug_vect);

			// 深度（高度）真值和设定值获取，//! 向下为正，越深越正
			// float depth = _depth_fusion.fudepth * 0.01f;//换算单位
			// float depth = - _debug_key_value.value;
			// float depth = _vehicle_air_data.baro_alt_meter * 0.01f;
			float depth = _depth_estimated.depth_estimated;
			float depth_rate = 0;
			// float depth = _debug_vect.x;
			// float depth_rate = _debug_vect.y;
			float depth_setpoint = _param_hy_dc_sp.get();

			_hydro_depth_control_message.depth = depth;
			_hydro_depth_control_message.depth_rate = depth_rate;
			_hydro_depth_control_message.depth_setpoint = depth_setpoint;

			// 水平推力比例限幅，限制遥控器的最大命令
			float throttle_limited;
			throttle_limited = math::constrain(_thrust_sp_norm, -_param_hy_dc_max_thr.get(), _param_hy_dc_max_thr.get());
			_hydro_depth_control_message.throttle_limited = throttle_limited;

			//水平推力计算，向前为正，最大为水下推进器推力的2倍
			//! 注意，水翼的推力计算都用真值，单位是N，但力矩仍用归一化值
			float hydro_horizontal_thrust_setpoint = throttle_limited * 2 * _param_hy_rt_max_thrust.get();
			_hydro_depth_control_message.horizontal_thrust = hydro_horizontal_thrust_setpoint;

			/* 非光滑反馈 START */
			//TODO 非光滑反馈还有一个对舵机角度进行反馈的项，但这个项目前得不到
			// 非光滑反馈误差计算
			float nsf_e1 = depth_setpoint - depth;
			float nsf_e2 = - depth_rate;
			float nsf_s = nsf_e1 + nsf_e2*_param_hy_nsf_pt.get();
			float nsf_s_norm = nsf_s / _param_hy_nsf_norm.get();
			float nsf_s_norm_power = std::pow(nsf_s_norm, _param_hy_nsf_power.get());
			// 非光滑反馈控制量计算
			float nsf_u = nsf_s_norm_power * _param_hy_nsf_kp.get();
			// 非光滑反馈控制量限幅
			float nsf_u_limited =\
				math::constrain(nsf_u, _param_hy_nsf_umin.get(), _param_hy_nsf_umax.get());
			// 记录
			_hydro_depth_control_message.nsf_e1 = nsf_e1;
			_hydro_depth_control_message.nsf_e2 = nsf_e2;
			_hydro_depth_control_message.nsf_s = nsf_s;
			_hydro_depth_control_message.nsf_s_norm = nsf_s_norm;
			_hydro_depth_control_message.nsf_s_norm_power = nsf_s_norm_power;
			_hydro_depth_control_message.nsf_u = nsf_u;
			_hydro_depth_control_message.nsf_u_limited = nsf_u_limited;
			/* 非光滑反馈 END */

			/* 高阶滑模 START */
			//控制量计算
			float hsmc_x1 = depth - depth_setpoint;
			float hsmc_x2 = depth_rate;
			float hsmc_surface =\
				_param_hy_hsmc_mass.get() * _param_hy_hsmc_alpha.get() * _param_hy_hsmc_alpha.get() * hsmc_x1 +\
				_param_hy_hsmc_mass.get() * 2 * _param_hy_hsmc_alpha.get() * hsmc_x2 + _hsmc_x3;
			float hsmc_u =\
				- _param_hy_hsmc_mass.get() * _param_hy_hsmc_alpha.get() * _param_hy_hsmc_alpha.get() * hsmc_x2 -\
				2 * _param_hy_hsmc_alpha.get() * _hsmc_x3 -\
				_param_hy_hsmc_eta.get() * math::signNoZero(hsmc_surface);
			//积分
			_hsmc_x3 += hsmc_u * _param_hy_hsmc_ts.get();
			_hsmc_x3 = math::constrain(_hsmc_x3, _param_hy_hsmc_imin.get(), _param_hy_hsmc_imax.get());
			//记录
			_hydro_depth_control_message.hsmc_x1 = hsmc_x1;
			_hydro_depth_control_message.hsmc_x2 = hsmc_x2;
			_hydro_depth_control_message.hsmc_x3 = _hsmc_x3;
			_hydro_depth_control_message.hsmc_surface = hsmc_surface;
			_hydro_depth_control_message.hsmc_u = hsmc_u;
			/* 高阶滑模 END */

			// 根据模式选择竖直推力，向下为正
				float hydro_vertical_thrust_setpoint;
				int32_t dc_mode = _param_hy_dc_mode.get();
			if(dc_mode == 1)
				hydro_vertical_thrust_setpoint = nsf_u_limited + _param_hy_dc_ff.get();
			else if(dc_mode == 2)
				hydro_vertical_thrust_setpoint = _hsmc_x3 + _param_hy_dc_ff.get();
			else if(dc_mode == 3)
				hydro_vertical_thrust_setpoint = _manual_control_setpoint.pitch * _param_hy_man_ratio.get() + _param_hy_dc_ff.get();
			else if(dc_mode == 4)
				hydro_vertical_thrust_setpoint = _param_hy_dc_ff.get();
				else
					hydro_vertical_thrust_setpoint = 0;

				// Read-only depth-chain diagnostics. The selected value below is the
				// original production depth output used to form force_z; it is not a
				// SafeRoll replacement or a separately synthesized command.
				_hydro_depth_control_message.timestamp_sample =
					vehicle_angular_velocity.timestamp_sample;
				_hydro_depth_control_message.depth_controller_mode = dc_mode;
				_hydro_depth_control_message.depth_controller_enabled = dc_mode >= 1 && dc_mode <= 4;
				_hydro_depth_control_message.depth_controller_output_finite =
					PX4_ISFINITE(hydro_vertical_thrust_setpoint);
				_hydro_depth_control_message.vertical_thrust = hydro_vertical_thrust_setpoint;
				_hydro_depth_control_message.vertical_thrust_limited = hydro_vertical_thrust_setpoint;

				//滑行时，机身的俯仰角近似为自然攻角，实际攻角等于翼面偏转角度加上自然攻角
			float alpha0 = euler_angles.theta();

			//水平和竖直推力转换为机身坐标系下的推力，使用二维坐标转换
			_hydro_thrust_setpoint.xyz[0] = hydro_horizontal_thrust_setpoint * std::cos(alpha0) - hydro_vertical_thrust_setpoint * std::sin(alpha0);
			_hydro_thrust_setpoint.xyz[2] = hydro_horizontal_thrust_setpoint * std::sin(alpha0) + hydro_vertical_thrust_setpoint * std::cos(alpha0);

			//y方向推力始终为0
			_hydro_thrust_setpoint.xyz[1] = 0;

			_hydro_depth_control_message.timestamp = hrt_absolute_time();
			_hydro_depth_control_message_pub.publish(_hydro_depth_control_message);
		}

		//发布hydro的setpoint；其他情况下，也要发布0
		if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL)
		{
			;
		}
		else
		{
			_hydro_thrust_setpoint.xyz[0] = 0;
			_hydro_thrust_setpoint.xyz[1] = 0;
			_hydro_thrust_setpoint.xyz[2] = 0;
			_hydro_torque_setpoint.xyz[0] = 0;
			_hydro_torque_setpoint.xyz[1] = 0;
			_hydro_torque_setpoint.xyz[2] = 0;
		}
		_hydro_thrust_setpoint.timestamp = hrt_absolute_time();
		_hydro_thrust_setpoint.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
		_hydro_thrust_setpoint_pub.publish(_hydro_thrust_setpoint);

		_hydro_torque_setpoint.timestamp = hrt_absolute_time();
		_hydro_torque_setpoint.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
		_hydro_torque_setpoint_pub.publish(_hydro_torque_setpoint);
	}

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

int HydroRateControl::task_spawn(int argc, char *argv[])
{
	HydroRateControl *instance = new HydroRateControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int HydroRateControl::custom_command(int argc, char *argv[])
{
	if (argc >= 1 && strcmp(argv[0], "safe_roll_watch") == 0) {
		bool full_format = false;
		float rate_hz = 5.f;
		int rate_argument = 1;

		if (argc >= 2) {
			if (strcmp(argv[1], "compact") == 0) {
				rate_argument = 2;

			} else if (strcmp(argv[1], "full") == 0) {
				full_format = true;
				rate_argument = 2;
			}
		}

		if (argc > rate_argument) {
			char *end = nullptr;
			rate_hz = strtof(argv[rate_argument], &end);

			if (end == argv[rate_argument] || *end != '\0' || !PX4_ISFINITE(rate_hz)
			    || rate_hz < 1.f || rate_hz > 20.f) {
				PX4_ERR("rate_hz must be in [1, 20]");
				return PX4_ERROR;
			}
		}

		if (argc > rate_argument + 1) {
			PX4_ERR("usage: safe_roll_watch [compact|full] [rate_hz]");
			return PX4_ERROR;
		}

		uORB::Subscription status_sub{ORB_ID(haar_safe_roll_status)};
		uORB::Subscription estimator_sub{ORB_ID(haar_safe_roll_estimator_status)};
		uORB::Subscription allocator_sub{ORB_ID(hydro_allocate_message)};
		uORB::Subscription vehicle_status_sub{ORB_ID(vehicle_status)};
		uORB::Subscription attitude_sub{ORB_ID(vehicle_attitude)};
		haar_safe_roll_status_s status{};
		haar_safe_roll_estimator_status_s estimator{};
		hydro_allocate_message_s allocator{};
		vehicle_status_s vehicle_status{};
		vehicle_attitude_s attitude{};
		const uint32_t interval_us = static_cast<uint32_t>(1e6f / rate_hz);
		safe_roll_watch_should_exit = 0;
		const auto previous_handler = signal(SIGINT, safeRollWatchSignalHandler);

		while (!safe_roll_watch_should_exit) {
			if (status_sub.copy(&status)) {
				estimator_sub.copy(&estimator);
				allocator_sub.copy(&allocator);
				vehicle_status_sub.copy(&vehicle_status);
				attitude_sub.copy(&attitude);

				if (full_format) {
					PX4_INFO_RAW("%llu %u %u %.4f %.4f %.6f %.6f %u %u %.6f 0x%08x %u %u %u "
						     "0x%08x %u %llu %.6f %.6f %.6f %.4f %.6f %.6f %.4f "
						     "%.4f %.6f %.6f %.6f %.6f %.4f %.4f %u %u "
						     "%.6f %u %u %u %u 0x%08x %u %u %u\n",
						     static_cast<unsigned long long>(status.timestamp),
						     static_cast<unsigned>(status.requested_code),
						     static_cast<unsigned>(status.active_code),
						     static_cast<double>(math::degrees(status.phi)),
						     static_cast<double>(math::degrees(status.p)),
						     static_cast<double>(status.u_nominal),
						     static_cast<double>(status.u_selected),
						     static_cast<unsigned>(status.allocator_valid),
						     static_cast<unsigned>(status.allocator_converged),
						     static_cast<double>(status.allocator_residual),
						     static_cast<unsigned>(status.allocator_saturation_mask),
						     static_cast<unsigned>(status.allocator_invalid_streak),
						     static_cast<unsigned>(status.allocation_interval_degraded),
						     static_cast<unsigned>(status.temporary_fallback_active),
						     static_cast<unsigned>(status.first_fault_code),
						     static_cast<unsigned>((status.fault_flags
								       & SafeRollController::FallbackLatchedFault) != 0),
						     static_cast<unsigned long long>(allocator.timestamp),
						     static_cast<double>(allocator.command_force_x),
						     static_cast<double>(allocator.command_force_z),
						     static_cast<double>(allocator.available_thruster_thrust),
						     static_cast<double>(math::degrees(allocator.body_pitch)),
						     static_cast<double>(allocator.lift_coefficient),
						     static_cast<double>(allocator.speed_squared),
						     static_cast<double>(math::degrees(allocator.context_max_hydrofoil_angle)),
						     static_cast<double>(allocator.command_u_phi),
						     static_cast<double>(allocator.command_u_theta),
						     static_cast<double>(allocator.command_u_psi),
						     static_cast<double>(status.thrust_l_commanded),
						     static_cast<double>(status.thrust_r_commanded),
						     static_cast<double>(math::degrees(status.delta_l_commanded)),
						     static_cast<double>(math::degrees(status.delta_r_commanded)),
						     static_cast<unsigned>(status.allocator_iterations),
						     static_cast<unsigned>(allocator.allocator_solve_status),
						     static_cast<double>(estimator.last_feasible_u),
						     static_cast<unsigned>(status.candidate_valid),
						     static_cast<unsigned>(estimator.allocator_degraded_active),
						     static_cast<unsigned>(estimator.allocator_recovery_streak),
						     static_cast<unsigned>(status.reset_count),
						     static_cast<unsigned>(status.configuration_crc32),
						     static_cast<unsigned>(vehicle_status.arming_state),
						     static_cast<unsigned>(vehicle_status.nav_state),
						     static_cast<unsigned>(attitude.quat_reset_counter));

				} else {
					PX4_INFO_RAW("%llu %u %u %.4f %.4f %.6f %.6f %u %u %.6f 0x%08x %u %u %u 0x%08x %u\n",
						     static_cast<unsigned long long>(status.timestamp),
						     static_cast<unsigned>(status.requested_code),
						     static_cast<unsigned>(status.active_code),
						     static_cast<double>(math::degrees(status.phi)),
						     static_cast<double>(math::degrees(status.p)),
						     static_cast<double>(status.u_nominal),
						     static_cast<double>(status.u_selected),
						     static_cast<unsigned>(status.allocator_valid),
						     static_cast<unsigned>(status.allocator_converged),
						     static_cast<double>(status.allocator_residual),
						     static_cast<unsigned>(status.allocator_saturation_mask),
						     static_cast<unsigned>(status.allocator_invalid_streak),
						     static_cast<unsigned>(status.allocation_interval_degraded),
						     static_cast<unsigned>(status.temporary_fallback_active),
						     static_cast<unsigned>(status.first_fault_code),
						     static_cast<unsigned>((status.fault_flags
								       & SafeRollController::FallbackLatchedFault) != 0));
				}
			}

			px4_usleep(interval_us);
		}

		signal(SIGINT, previous_handler);
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int HydroRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
hydro_rate_control is the water-air cross medium rate controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hydro_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND_DESCR("safe_roll_watch", "Watch compact/full SafeRoll state (Ctrl+C to stop)");
	PRINT_MODULE_USAGE_ARG("[compact|full]", "Output format, default compact", true);
	PRINT_MODULE_USAGE_ARG("<rate_hz>", "Output rate, default 5 Hz, range 1..20 Hz", true);
	/*PRINT_MODULE_USAGE_NAME("hydro_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();*/

	return 0;
}

extern "C" __EXPORT int hydro_rate_control_main(int argc, char *argv[])
{
	return HydroRateControl::main(argc, argv);
}
