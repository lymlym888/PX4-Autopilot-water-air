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

#include <math.h>
#include <string.h>

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::interpolate;
using math::radians;

HydroRateControl::HydroRateControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();
	applyNewControllerParameters(readNewControllerParameters());
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

bool HydroRateControl::isNewController(HydroRollControllerId controller)
{
	return controller == HydroRollControllerId::Ladrc
	       || controller == HydroRollControllerId::MceoAdrc
	       || controller == HydroRollControllerId::AmceoAdrc
	       || controller == HydroRollControllerId::Nadrc;
}

bool HydroRateControl::isImplementedNewController(HydroRollControllerId controller)
{
	return controller == HydroRollControllerId::Ladrc
	       || controller == HydroRollControllerId::MceoAdrc
	       || controller == HydroRollControllerId::AmceoAdrc
	       || controller == HydroRollControllerId::Nadrc;
}

HydroRateControl::NewControllerParameters HydroRateControl::readNewControllerParameters() const
{
	return {
		_param_hy_rll_b0.get(),
		_param_hy_rll_aphi.get(),
		_param_hy_rll_k1.get(),
		_param_hy_rll_k2.get(),
		_param_hy_ladrc_wo.get(),
		_param_hy_mceo_wo.get(),
		_param_hy_rll_dt_max.get(),
		_param_hy_am_wmin.get(),
		_param_hy_am_wmax.get(),
		_param_hy_am_rho0.get(),
		_param_hy_am_ce.get(),
		_param_hy_am_cu.get(),
		_param_hy_am_trho.get(),
		_param_hy_am_e1bar.get(),
		_param_hy_am_e2bar.get(),
		_param_hy_na_wo.get(),
		_param_hy_na_tdr.get(),
		_param_hy_na_tdn0.get(),
		_param_hy_na_oa1.get(),
		_param_hy_na_oa2.get(),
		_param_hy_na_odel.get(),
		_param_hy_na_ca1.get(),
		_param_hy_na_ca2.get(),
		_param_hy_na_cdel.get()
	};
}

bool HydroRateControl::newControllerParametersValid(const NewControllerParameters &parameters)
{
	return PX4_ISFINITE(parameters.b0) && parameters.b0 > 0.f
	       && PX4_ISFINITE(parameters.a_phi)
	       && PX4_ISFINITE(parameters.k_phi1) && parameters.k_phi1 > 0.f
	       && PX4_ISFINITE(parameters.k_phi2) && parameters.k_phi2 > 0.f
	       && PX4_ISFINITE(parameters.ladrc_wo) && parameters.ladrc_wo > 0.f
	       && PX4_ISFINITE(parameters.mceo_wo) && parameters.mceo_wo > 0.f
	       && PX4_ISFINITE(parameters.dt_max) && parameters.dt_max > 0.f
	       && PX4_ISFINITE(parameters.am_omega_min) && parameters.am_omega_min > 0.f
	       && PX4_ISFINITE(parameters.am_omega_max) && parameters.am_omega_max > parameters.am_omega_min
	       && PX4_ISFINITE(parameters.am_rho0) && parameters.am_rho0 >= 0.f && parameters.am_rho0 <= 1.f
	       && PX4_ISFINITE(parameters.am_ce) && parameters.am_ce >= 0.f
	       && PX4_ISFINITE(parameters.am_cu) && parameters.am_cu >= 0.f
	       && PX4_ISFINITE(parameters.am_tau_rho) && parameters.am_tau_rho > 0.f
	       && PX4_ISFINITE(parameters.am_e1_bar) && parameters.am_e1_bar > 0.f
	       && PX4_ISFINITE(parameters.am_e2_bar) && parameters.am_e2_bar > 0.f
	       && PX4_ISFINITE(parameters.na_wo) && parameters.na_wo > 0.f
	       && PX4_ISFINITE(parameters.na_td_r) && parameters.na_td_r > 0.f
	       && PX4_ISFINITE(parameters.na_td_n0) && parameters.na_td_n0 >= 1.f
	       && PX4_ISFINITE(parameters.na_observer_alpha1) && parameters.na_observer_alpha1 > 0.f
	       && PX4_ISFINITE(parameters.na_observer_alpha2) && parameters.na_observer_alpha2 > 0.f
	       && PX4_ISFINITE(parameters.na_observer_delta) && parameters.na_observer_delta > 0.f
	       && PX4_ISFINITE(parameters.na_controller_alpha1) && parameters.na_controller_alpha1 > 0.f
	       && PX4_ISFINITE(parameters.na_controller_alpha2) && parameters.na_controller_alpha2 > 0.f
	       && PX4_ISFINITE(parameters.na_controller_delta) && parameters.na_controller_delta > 0.f;
}

bool HydroRateControl::newControllerParametersEqual(const NewControllerParameters &lhs,
		const NewControllerParameters &rhs)
{
	static_assert(sizeof(NewControllerParameters) == 24 * sizeof(float), "unexpected new-controller parameter padding");
	return memcmp(&lhs, &rhs, sizeof(NewControllerParameters)) == 0;
}

bool HydroRateControl::applyNewControllerParameters(const NewControllerParameters &parameters)
{
	if (!newControllerParametersValid(parameters)) {
		return false;
	}

	const HydroRollControllerConfig ladrc_config {
		parameters.b0, parameters.a_phi, parameters.k_phi1, parameters.k_phi2, parameters.ladrc_wo
	};
	const HydroRollControllerConfig mceo_config {
		parameters.b0, parameters.a_phi, parameters.k_phi1, parameters.k_phi2, parameters.mceo_wo
	};
	const AMCEOControllerConfig amceo_config {
		{parameters.b0, parameters.a_phi, parameters.k_phi1, parameters.k_phi2, parameters.am_omega_min},
		parameters.am_omega_min,
		parameters.am_omega_max,
		parameters.am_rho0,
		parameters.am_ce,
		parameters.am_cu,
		parameters.am_tau_rho,
		parameters.am_e1_bar,
		parameters.am_e2_bar
	};
	const ClassicalNADRCConfig nadrc_config {
		parameters.b0,
		parameters.k_phi1,
		parameters.k_phi2,
		parameters.na_wo,
		parameters.na_td_r,
		parameters.na_td_n0,
		parameters.na_observer_alpha1,
		parameters.na_observer_alpha2,
		parameters.na_observer_delta,
		parameters.na_controller_alpha1,
		parameters.na_controller_alpha2,
		parameters.na_controller_delta
	};
	const bool valid = _ladrc_roll.setParameters(ladrc_config)
			   && _mceo_adrc_roll.setParameters(mceo_config)
			   && _amceo_adrc_roll.setParameters(amceo_config)
			   && _nadrc_roll.setParameters(nadrc_config);

	if (valid) {
		_applied_new_controller_parameters = parameters;
		_new_controller_parameters_initialized = true;
	}

	return valid;
}

void HydroRateControl::resetNewController(HydroRollControllerId controller, const HydroRollControllerInput &input)
{
	switch (controller) {
	case HydroRollControllerId::Ladrc:
		_ladrc_roll.reset(input);
		break;

	case HydroRollControllerId::MceoAdrc:
		_mceo_adrc_roll.reset(input);
		break;

	case HydroRollControllerId::AmceoAdrc:
		_amceo_adrc_roll.reset(input);
		break;

	case HydroRollControllerId::Nadrc:
		_nadrc_roll.reset(input);
		break;

	case HydroRollControllerId::Legacy:
	case HydroRollControllerId::Cadrc:
		break;
	}
}

HydroRollControllerOutput HydroRateControl::updateNewController(HydroRollControllerId controller,
		const HydroRollControllerInput &input)
{
	switch (controller) {
	case HydroRollControllerId::Ladrc:
		return _ladrc_roll.update(input);

	case HydroRollControllerId::MceoAdrc:
		return _mceo_adrc_roll.update(input);

	case HydroRollControllerId::AmceoAdrc:
		return _amceo_adrc_roll.update(input);

	case HydroRollControllerId::Nadrc:
		return _nadrc_roll.update(input);

	case HydroRollControllerId::Legacy:
	case HydroRollControllerId::Cadrc:
		return {NAN, false, false};
	}

	return {NAN, false, false};
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

		bool selector_edge = false;
		bool controller_reset_event = false;
		bool fallback_active = false;
		bool diagnostic_output_valid = false;
		bool diagnostic_reference_valid = false;
		bool diagnostic_magnitude_limited = false;
		bool diagnostic_rate_limited = false;
		bool diagnostic_dt_invalid = false;
		bool timing_fault_event = false;
		float diagnostic_observer_bandwidth = NAN;
		float diagnostic_rho = NAN;
		float diagnostic_sigma_e = NAN;
		float diagnostic_sigma_u = NAN;
		float diagnostic_phi = NAN;
		float diagnostic_p = NAN;
		float diagnostic_manual_roll_raw = NAN;
		float diagnostic_command_roll_latched = NAN;
		float diagnostic_manual_age_s = NAN;
		float diagnostic_phi_ref = NAN;
		float diagnostic_phi_ref_dot = NAN;
		float diagnostic_phi_ref_ddot = NAN;
		float diagnostic_u_requested = NAN;
		float diagnostic_u_applied = NAN;
		float diagnostic_dt_used = dt;
		float diagnostic_state_1 = NAN;
		float diagnostic_state_2 = NAN;
		float diagnostic_state_3 = NAN;
		float diagnostic_state_4 = NAN;
		float diagnostic_disturbance_estimate = NAN;
		uint8_t diagnostic_manual_data_source = manual_control_setpoint_s::SOURCE_UNKNOWN;
		bool diagnostic_manual_valid = false;
		bool new_roll_output_available = false;
		bool controller_changed = false;
		float new_roll_applied = 0.f;
		HydroRollControllerId requested_controller = _last_requested_controller;

		_vehicle_status_sub.update(&_vehicle_status);
		const bool armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

		// Critical new-controller gains are read on every parameter update, but
		// are applied only while disarmed. CADRC parameter behavior above remains
		// unchanged. A deferred set is atomically applied before the next new-
		// controller update and forces its observer to reset.
		if (params_updated) {
			const NewControllerParameters candidate = readNewControllerParameters();

			if (!_new_controller_parameters_initialized
			    || !newControllerParametersEqual(candidate, _applied_new_controller_parameters)) {
				_pending_new_controller_parameters = candidate;
				_new_controller_parameters_pending = true;
				_new_controller_parameter_fault_reported = false;
			}
		}

		if (_new_controller_parameters_pending) {
			if (!newControllerParametersValid(_pending_new_controller_parameters)) {
				if (!_new_controller_parameter_fault_reported) {
					++_roll_controller_fault_counter;
					PX4_WARN("invalid new roll controller parameters rejected");
					_new_controller_parameter_fault_reported = true;
				}

			} else if (armed) {
				if (!_new_controller_parameter_fault_reported) {
					++_roll_controller_fault_counter;
					PX4_WARN("armed new roll gain update deferred until disarm");
					_new_controller_parameter_fault_reported = true;
				}

			} else if (applyNewControllerParameters(_pending_new_controller_parameters)) {
				_new_controller_parameters_pending = false;
				_new_controller_parameter_fault_reported = false;
				_new_controller_parameter_reset_pending = isNewController(_active_controller);
			}
		}

		// Selector state machine runs in every navigation mode. A change while
		// armed is recorded but cannot change the active controller. A blocked
		// request is accepted only after disarm.
		const int32_t selector_value = _param_hy_rll_ctrl_sel.get();
		const bool selector_valid = selector_value >= static_cast<int32_t>(HydroRollControllerId::Legacy)
				    && selector_value <= static_cast<int32_t>(HydroRollControllerId::Nadrc);
		requested_controller = selector_valid ? static_cast<HydroRollControllerId>(selector_value)
				       : HydroRollControllerId::Legacy;
		selector_edge = requested_controller != _last_requested_controller;

		if (selector_edge) {
			_last_requested_controller = requested_controller;
		}

		if (!selector_valid && selector_edge) {
			++_roll_controller_fault_counter;
			PX4_WARN("invalid roll controller selector, requesting legacy");
		}

		if (selector_edge && armed && requested_controller != _active_controller) {
			_roll_controller_switch_blocked = true;
			++_roll_controller_fault_counter;
			PX4_WARN("armed roll controller switch blocked (%u -> %u)",
				 static_cast<unsigned>(_active_controller), static_cast<unsigned>(requested_controller));

		} else if ((!armed && selector_edge && requested_controller != _active_controller)
			   || (!armed && _roll_controller_switch_blocked && requested_controller != _active_controller)) {
			HydroRollControllerInput reset_input{};
			reset_input.dt_measured = dt;
			reset_input.dt_used = dt;
			reset_input.u_applied_prev = _hydro_torque_setpoint.xyz[0];
			reset_input.timestamp_sample = vehicle_angular_velocity.timestamp_sample;

			if (isNewController(_active_controller)) {
				resetNewController(_active_controller, reset_input);
				_manual_roll_latch.reset(0.f);
				_roll_reference_generator.reset(0.f, 0.f);
				_new_roll_limiter.reset(_hydro_torque_setpoint.xyz[0]);
			}

			_previous_controller = _active_controller;
			_active_controller = requested_controller;
			_roll_controller_switch_blocked = false;
			_new_controller_input_fault = false;
			_new_controller_timing_fault = false;
			_new_controller_parameter_reset_pending = false;
			controller_changed = true;
			controller_reset_event = true;
			++_roll_controller_reset_counter;

			if (_active_controller == HydroRollControllerId::Cadrc) {
				_cadrc_roll.reset();
				_cadrc_pitch.reset();

			} else if (_active_controller == HydroRollControllerId::Legacy) {
				_rate_control.resetIntegral();

			} else {
				_manual_roll_latch.reset(0.f);
				_roll_reference_generator.reset(0.f, 0.f);
				_new_roll_limiter.reset(_hydro_torque_setpoint.xyz[0]);

				if (_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_STAB
				    && _vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_ACRO) {
					resetNewController(_active_controller, reset_input);
				}

			}

		} else if (requested_controller == _active_controller) {
			_roll_controller_switch_blocked = false;
		}

// --------------------是否切换模式进入stab，是则reset--------------------------------------------------------------
		// detect transition into STAB (edge)
		const bool entered_stab =
		(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB) &&
		(_prev_nav_state != vehicle_status_s::NAVIGATION_STATE_STAB);
		const bool left_stab =
		(_vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_STAB) &&
		(_prev_nav_state == vehicle_status_s::NAVIGATION_STATE_STAB);

		if (entered_stab) {
		// reset CADRC (and optionally other integrators)
		_cadrc_roll.reset();
		_cadrc_pitch.reset();
		++_roll_controller_reset_counter;
		controller_reset_event = true;
		}

		if (left_stab) {
			if (isNewController(_active_controller)) {
				HydroRollControllerInput reset_input{};
				resetNewController(_active_controller, reset_input);
				++_roll_controller_reset_counter;
				controller_reset_event = true;
			}

			_manual_roll_latch.reset(0.f);
			_roll_reference_generator.reset(0.f, 0.f);
			_new_roll_limiter.reset(_hydro_torque_setpoint.xyz[0]);
			_new_controller_input_fault = false;
			_new_controller_timing_fault = false;
		}
		// update previous nav state
		_prev_nav_state = _vehicle_status.nav_state;

		// Preserve the publication edge for the independent new-controller latch.
		const bool got_manual = _manual_control_setpoint_sub.updated();
		// Read the latest manual command, including between publications.
		_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

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
			Eulerf euler_angles(Quatf(_vehicle_attitude.q));

			const float roll  = euler_angles.phi();    // rad
			const float pitch = euler_angles.theta();  // rad

			// 实时角速度 Wx
			const float p = rates(0); // roll rate rad/s
			const float q = rates(1); // pitch rate rad/s
			diagnostic_phi = roll;
			diagnostic_p = p;

			if ((entered_stab || controller_changed || _new_controller_parameter_reset_pending)
			    && isNewController(_active_controller)) {
				const bool parameter_reset = _new_controller_parameter_reset_pending;
				HydroRollControllerInput reset_input{};
				reset_input.phi = roll;
				reset_input.p = p;
				reset_input.command_roll = roll;
				reset_input.phi_ref = roll;
				reset_input.dt_measured = dt;
				reset_input.dt_used = dt;
				reset_input.u_applied_prev = _hydro_torque_setpoint.xyz[0];
				reset_input.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
				_manual_roll_latch.reset(roll);
				_roll_reference_generator.reset(roll, roll);
				_new_roll_limiter.reset(_hydro_torque_setpoint.xyz[0]);
				resetNewController(_active_controller, reset_input);
				_new_controller_parameter_reset_pending = false;

				if (parameter_reset && !entered_stab && !controller_changed) {
					++_roll_controller_reset_counter;
					controller_reset_event = true;
				}
			}

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
			    && isNewController(_active_controller)) {
				fallback_active = true;
				const bool timing_guard_required = isImplementedNewController(_active_controller);
				diagnostic_dt_invalid = timing_guard_required
							&& !HydroRollTimingGuard::isValid(dt,
								_applied_new_controller_parameters.dt_max);

				if (diagnostic_dt_invalid) {
					// A scheduler stall is not integrated with a fictitious clamped
					// interval. Preserve controller state and use legacy fallback until
					// all new-controller state is reset on a later valid sample.
					if (!_new_controller_timing_fault) {
						_new_controller_timing_fault = true;
						timing_fault_event = true;
						++_roll_controller_fault_counter;
						PX4_WARN("new roll controller timing fault: dt %.6f s", (double)dt);
					}

				} else {
					if (_new_controller_timing_fault) {
						// Recovery below resets all integration histories once inputs
						// are valid. A timing fault alone must not erase the RC latch
						// and add a wait for another manual publication.
						_new_controller_timing_fault = false;
					}

				const bool nadrc_active = _active_controller == HydroRollControllerId::Nadrc;
				const bool latch_was_valid = _manual_roll_latch.valid();
				const hrt_abstime now = hrt_absolute_time();
				const manual_control_setpoint_s &manual_sp = _manual_control_setpoint;
				const bool manual_sample_valid = got_manual && manual_sp.valid && manual_sp.timestamp != 0
							 && PX4_ISFINITE(manual_sp.roll);
				_manual_roll_latch.update(true, got_manual, manual_sample_valid, manual_sp.roll,
							 manual_sp.data_source, manual_sp.timestamp, now,
							 radians(_param_hy_roll_max.get()), roll);

				const bool latch_reset_event = latch_was_valid && !_manual_roll_latch.valid();

				if (latch_reset_event) {
					_roll_reference_generator.reset(_manual_roll_latch.commandRoll(), roll);
					controller_reset_event = true;
					++_roll_controller_fault_counter;
					++_roll_controller_reset_counter;
				}

				diagnostic_manual_roll_raw = _manual_roll_latch.manualRollRaw();
				diagnostic_command_roll_latched = _manual_roll_latch.commandRoll();
				diagnostic_manual_age_s = _manual_roll_latch.ageSeconds();
				diagnostic_manual_data_source = _manual_roll_latch.dataSource();
				diagnostic_manual_valid = _manual_roll_latch.valid();

				// FALLBACK -> RECOVERY: use the last command actually sent downstream,
				// not the limiter's stale pre-fallback history. Wait for finite inputs;
				// the surrounding timing gate has already accepted this sample. Keep
				// the valid manual latch (publication gaps are NOT input failures).
				if (_new_controller_recovery_pending && _manual_roll_latch.valid()
				    && PX4_ISFINITE(roll) && PX4_ISFINITE(p)
				    && PX4_ISFINITE(_manual_roll_latch.commandRoll())
				    && PX4_ISFINITE(_hydro_torque_setpoint.xyz[0])) {
					HydroRollControllerInput recovery_input{};
					recovery_input.phi = roll;
					recovery_input.p = p;
					recovery_input.command_roll = _manual_roll_latch.commandRoll();
					recovery_input.phi_ref = roll;
					recovery_input.dt_measured = dt;
					recovery_input.dt_used = dt;
					recovery_input.u_applied_prev = _hydro_torque_setpoint.xyz[0];
					recovery_input.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
					recovery_input.u_max = _param_hy_rll_u_max.get();
					_roll_reference_generator.reset(recovery_input.command_roll, roll);
					_new_roll_limiter.reset(recovery_input.u_applied_prev);
					resetNewController(_active_controller, recovery_input);
					_new_controller_recovery_pending = false;
					controller_reset_event = true;
					++_roll_controller_reset_counter;
				}

				// IDs 2-4 use the common second-order reference generator. Classical
				// NADRC (ID 5) consumes the latched command directly and owns its TD.
				if (_manual_roll_latch.valid() && !nadrc_active) {
					diagnostic_reference_valid = _roll_reference_generator.update(_manual_roll_latch.commandRoll(),
							     _param_hy_rref_w.get(), _param_hy_rref_zeta.get(), dt);

				} else if (_manual_roll_latch.valid()) {
					diagnostic_reference_valid = true;
				}

				if (!nadrc_active) {
					diagnostic_phi_ref = _roll_reference_generator.phiRef();
					diagnostic_phi_ref_dot = _roll_reference_generator.phiRefDot();
					diagnostic_phi_ref_ddot = _roll_reference_generator.phiRefDdot();
				}

				diagnostic_dt_used = dt;

				HydroRollControllerInput controller_input{};
				controller_input.phi = roll;
				controller_input.p = p;
				controller_input.command_roll = _manual_roll_latch.commandRoll();
				// The generic reference members are not consumed by ClassicalNADRC.
				// Keep them finite for the common input-validity contract while its
				// actual reference is command_roll -> internal Han TD.
				controller_input.phi_ref = nadrc_active ? controller_input.command_roll : diagnostic_phi_ref;
				controller_input.phi_ref_dot = nadrc_active ? 0.f : diagnostic_phi_ref_dot;
				controller_input.phi_ref_ddot = nadrc_active ? 0.f : diagnostic_phi_ref_ddot;
				controller_input.dt_measured = dt;
				controller_input.dt_used = dt;
				controller_input.u_applied_prev = _new_roll_limiter.appliedPrevious();
				controller_input.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
				controller_input.u_max = _param_hy_rll_u_max.get();

				if (latch_reset_event && nadrc_active) {
					// Reset the private TD/NESO together after a stale/invalid manual
					// command. The next valid sample still enters through the limiter.
					resetNewController(_active_controller, controller_input);
					_new_roll_limiter.reset(_hydro_torque_setpoint.xyz[0]);
				}

				const bool input_finite = PX4_ISFINITE(controller_input.phi) && PX4_ISFINITE(controller_input.p)
							  && PX4_ISFINITE(controller_input.command_roll)
							  && PX4_ISFINITE(controller_input.phi_ref)
							  && PX4_ISFINITE(controller_input.phi_ref_dot)
							  && PX4_ISFINITE(controller_input.phi_ref_ddot)
							  && PX4_ISFINITE(controller_input.dt_measured)
							  && PX4_ISFINITE(controller_input.dt_used)
							  && controller_input.dt_used > 0.f
							  && PX4_ISFINITE(controller_input.u_applied_prev)
							  && PX4_ISFINITE(controller_input.u_max)
							  && controller_input.u_max > 0.f;

				if (diagnostic_reference_valid && input_finite && !_new_controller_recovery_pending) {
					const HydroRollControllerOutput output = updateNewController(_active_controller, controller_input);
					diagnostic_u_requested = output.u_requested;
					controller_reset_event = controller_reset_event || output.reset_event;

					if (output.valid && PX4_ISFINITE(output.u_requested)) {
						const HydroRollLimiterResult limited = _new_roll_limiter.update(output.u_requested,
								_param_trim_roll.get(), dt, _param_hy_rll_u_max.get(), _param_hy_rll_du_max.get());
						diagnostic_magnitude_limited = limited.magnitude_limited;
						diagnostic_rate_limited = limited.rate_limited;
						diagnostic_output_valid = limited.valid;

						if (limited.valid) {
							new_roll_applied = limited.u_applied;
							new_roll_output_available = true;
							fallback_active = false;
							_new_controller_input_fault = false;
						}

						if (!limited.valid && !_new_controller_input_fault) {
							_new_controller_input_fault = true;
							++_roll_controller_fault_counter;
							++_roll_controller_reset_counter;
							controller_reset_event = true;
							resetNewController(_active_controller, controller_input);
							_roll_reference_generator.reset(controller_input.command_roll, controller_input.phi);
						}

					} else if ((output.valid || isImplementedNewController(_active_controller))
						   && !_new_controller_input_fault) {
						// Every new controller ID is implemented and must return a finite
						// normalized request. Invalid output enters legacy fallback.
						_new_controller_input_fault = true;
						++_roll_controller_fault_counter;
						++_roll_controller_reset_counter;
						controller_reset_event = true;
						resetNewController(_active_controller, controller_input);
						_new_roll_limiter.reset(_hydro_torque_setpoint.xyz[0]);
						_roll_reference_generator.reset(controller_input.command_roll, controller_input.phi);
					}

				} else if (_manual_roll_latch.valid() && !_new_controller_input_fault) {
					_new_controller_input_fault = true;
					++_roll_controller_fault_counter;
					++_roll_controller_reset_counter;
					controller_reset_event = true;
					resetNewController(_active_controller, controller_input);
					_new_roll_limiter.reset(_hydro_torque_setpoint.xyz[0]);
					_roll_reference_generator.reset(controller_input.command_roll, controller_input.phi);
				}

				if (_active_controller == HydroRollControllerId::Ladrc) {
					diagnostic_state_1 = _ladrc_roll.z1();
					diagnostic_state_2 = _ladrc_roll.z2();
					diagnostic_state_3 = _ladrc_roll.z3();
					diagnostic_state_4 = NAN;
					diagnostic_disturbance_estimate = _ladrc_roll.z3();
					diagnostic_observer_bandwidth = _ladrc_roll.observerBandwidth();

				} else if (_active_controller == HydroRollControllerId::MceoAdrc) {
					diagnostic_state_1 = _mceo_adrc_roll.z1();
					diagnostic_state_2 = _mceo_adrc_roll.z2();
					diagnostic_state_3 = _mceo_adrc_roll.z3();
					diagnostic_state_4 = _mceo_adrc_roll.z4();
					diagnostic_disturbance_estimate = _mceo_adrc_roll.z3();
					diagnostic_observer_bandwidth = _mceo_adrc_roll.observerBandwidth();

				} else if (_active_controller == HydroRollControllerId::AmceoAdrc) {
					diagnostic_state_1 = _amceo_adrc_roll.z1();
					diagnostic_state_2 = _amceo_adrc_roll.z2();
					diagnostic_state_3 = _amceo_adrc_roll.z3();
					diagnostic_state_4 = _amceo_adrc_roll.z4();
					diagnostic_disturbance_estimate = _amceo_adrc_roll.z3();
					diagnostic_observer_bandwidth = _amceo_adrc_roll.observerBandwidth();
					diagnostic_rho = _amceo_adrc_roll.rho();
					diagnostic_sigma_e = _amceo_adrc_roll.sigmaE();
					diagnostic_sigma_u = _amceo_adrc_roll.sigmaU();

				} else if (_active_controller == HydroRollControllerId::Nadrc) {
					diagnostic_state_1 = _nadrc_roll.z1();
					diagnostic_state_2 = _nadrc_roll.z2();
					diagnostic_state_3 = _nadrc_roll.z3();
					diagnostic_state_4 = NAN;
					diagnostic_disturbance_estimate = _nadrc_roll.z3();
					diagnostic_observer_bandwidth = _nadrc_roll.observerBandwidth();
					diagnostic_phi_ref = _nadrc_roll.v1();
					diagnostic_phi_ref_dot = _nadrc_roll.v2();
					diagnostic_phi_ref_ddot = _nadrc_roll.tdAcceleration();
					diagnostic_reference_valid = diagnostic_reference_valid
							     && PX4_ISFINITE(diagnostic_phi_ref)
							     && PX4_ISFINITE(diagnostic_phi_ref_dot)
							     && PX4_ISFINITE(diagnostic_phi_ref_ddot);
				}
				}
				// Include manual invalidity, timing faults, invalid reference/controller
				// output and limiter failure. Only a successfully applied new command
				// may continue without recovery on the following sample.
				_new_controller_recovery_pending = !new_roll_output_available;
			}

			if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB
			&& _param_hy_cadrc_en.get()
			&& _active_controller == HydroRollControllerId::Cadrc) {

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
				diagnostic_u_requested = _cadrc_roll.getUc() * u_scale;

				// yaw：先保持最简单的手动映射（后面再决定是否也用CADRC）
				float u_yaw = math::constrain(_manual_control_setpoint.yaw * _param_hy_man_y_sc.get()
								+ _param_trim_yaw.get(), -1.f, 1.f);

				// 加 trim（保持与原框架一致）
				u_roll  = math::constrain(u_roll  + _param_trim_roll.get(),  -1.f, 1.f);
				u_pitch = math::constrain(u_pitch + _param_trim_pitch.get(), -1.f, 1.f);
				diagnostic_magnitude_limited = fabsf(diagnostic_u_requested) > 1.f
							       || fabsf(math::constrain(diagnostic_u_requested, -1.f, 1.f)
								+ _param_trim_roll.get()) > 1.f;
				_hydro_cadrc_control_message.roll_z21 = _cadrc_roll._Z2(0,0);
				_hydro_cadrc_control_message.roll_z22 = _cadrc_roll._Z2(1,0);
				_hydro_cadrc_control_message.pitch_z21 = _cadrc_pitch._Z2(0,0);
				_hydro_cadrc_control_message.pitch_z22 = _cadrc_pitch._Z2(1,0);

				_hydro_torque_setpoint.xyz[0] = u_roll;
				_hydro_torque_setpoint.xyz[1] = u_pitch;
				_hydro_torque_setpoint.xyz[2] = u_yaw;
				diagnostic_output_valid = PX4_ISFINITE(diagnostic_u_requested) && PX4_ISFINITE(u_roll);
				diagnostic_reference_valid = _manual_control_setpoint.valid
							     && _manual_control_setpoint.timestamp != 0;
				diagnostic_manual_valid = diagnostic_reference_valid;
				diagnostic_manual_roll_raw = diagnostic_manual_valid ? _manual_control_setpoint.roll : NAN;
				diagnostic_manual_data_source = diagnostic_manual_valid ? _manual_control_setpoint.data_source
							       : manual_control_setpoint_s::SOURCE_UNKNOWN;
				diagnostic_phi_ref = roll_sp;
				diagnostic_dt_used = dt_used;
				diagnostic_state_1 = _cadrc_roll._Z1(0, 0);
				diagnostic_state_2 = _cadrc_roll._Z1(1, 0);
				diagnostic_state_3 = _cadrc_roll._Z2(0, 0);
				diagnostic_state_4 = _cadrc_roll._Z2(1, 0);
				diagnostic_disturbance_estimate = _cadrc_roll._Z2(1, 0);

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

			if (new_roll_output_available) {
				// Implemented new controllers replace only the legacy roll contribution.
				_hydro_torque_setpoint.xyz[0] = new_roll_applied;
			}

			if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO
			    || _active_controller == HydroRollControllerId::Legacy) {
				diagnostic_output_valid = PX4_ISFINITE(_hydro_torque_setpoint.xyz[0]);

			} else if (_active_controller == HydroRollControllerId::Cadrc && !_param_hy_cadrc_en.get()) {
				fallback_active = true;
				diagnostic_output_valid = PX4_ISFINITE(_hydro_torque_setpoint.xyz[0]);
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

		diagnostic_u_applied = _hydro_torque_setpoint.xyz[0];
		const hrt_abstime diagnostic_now = hrt_absolute_time();
		const bool publish_roll_status = selector_edge || controller_reset_event || timing_fault_event
					 || _last_roll_controller_status_publish == 0
					 || diagnostic_now - _last_roll_controller_status_publish >= 5_ms;

		if (publish_roll_status) {
			_hydro_roll_controller_status = {};
			_hydro_roll_controller_status.timestamp = diagnostic_now;
			_hydro_roll_controller_status.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
			_hydro_roll_controller_status.controller_id = static_cast<uint8_t>(_active_controller);
			_hydro_roll_controller_status.requested_controller_id = static_cast<uint8_t>(requested_controller);
			_hydro_roll_controller_status.previous_controller_id = static_cast<uint8_t>(_previous_controller);
			_hydro_roll_controller_status.reset_counter = _roll_controller_reset_counter;
			_hydro_roll_controller_status.fault_counter = _roll_controller_fault_counter;
			_hydro_roll_controller_status.output_valid = diagnostic_output_valid;
			_hydro_roll_controller_status.reference_valid = diagnostic_reference_valid;
			_hydro_roll_controller_status.manual_valid = diagnostic_manual_valid;
			_hydro_roll_controller_status.magnitude_limited = diagnostic_magnitude_limited;
			_hydro_roll_controller_status.rate_limited = diagnostic_rate_limited;
			_hydro_roll_controller_status.dt_invalid = diagnostic_dt_invalid;
			_hydro_roll_controller_status.timing_fault = _new_controller_timing_fault || diagnostic_dt_invalid;
			_hydro_roll_controller_status.selector_edge = selector_edge;
			_hydro_roll_controller_status.switch_blocked = _roll_controller_switch_blocked;
			_hydro_roll_controller_status.fallback_active = fallback_active;
			_hydro_roll_controller_status.phi = diagnostic_phi;
			_hydro_roll_controller_status.p = diagnostic_p;
			_hydro_roll_controller_status.manual_roll_raw = diagnostic_manual_roll_raw;
			_hydro_roll_controller_status.command_roll_latched = diagnostic_command_roll_latched;
			_hydro_roll_controller_status.manual_command_age_s = diagnostic_manual_age_s;
			_hydro_roll_controller_status.manual_data_source = diagnostic_manual_data_source;
			_hydro_roll_controller_status.phi_ref = diagnostic_phi_ref;
			_hydro_roll_controller_status.phi_ref_dot = diagnostic_phi_ref_dot;
			_hydro_roll_controller_status.phi_ref_ddot = diagnostic_phi_ref_ddot;
			_hydro_roll_controller_status.u_requested = diagnostic_u_requested;
			_hydro_roll_controller_status.u_applied = diagnostic_u_applied;
			_hydro_roll_controller_status.dt_measured = dt;
			_hydro_roll_controller_status.dt_used = diagnostic_dt_used;
			_hydro_roll_controller_status.state_1 = diagnostic_state_1;
			_hydro_roll_controller_status.state_2 = diagnostic_state_2;
			_hydro_roll_controller_status.state_3 = diagnostic_state_3;
			_hydro_roll_controller_status.state_4 = diagnostic_state_4;
			_hydro_roll_controller_status.disturbance_estimate = diagnostic_disturbance_estimate;
			_hydro_roll_controller_status.observer_bandwidth = diagnostic_observer_bandwidth;
			_hydro_roll_controller_status.rho = diagnostic_rho;
			_hydro_roll_controller_status.sigma_e = diagnostic_sigma_e;
			_hydro_roll_controller_status.sigma_u = diagnostic_sigma_u;
			_hydro_roll_controller_status_pub.publish(_hydro_roll_controller_status);
			_last_roll_controller_status_publish = diagnostic_now;
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
