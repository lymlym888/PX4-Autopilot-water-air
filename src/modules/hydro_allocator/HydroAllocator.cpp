/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "HydroAllocator.hpp"

#include <lib/matrix/matrix/math.hpp>

using namespace time_literals;
using namespace matrix;

namespace
{
enum GateCloseReason : uint8_t {
	GateReasonNone = 0,
	GateReasonAuxThreshold = 1,
	GateReasonNavState = 2,
	GateReasonAuxNonfinite = 3,
	GateReasonAuxStale = 4,
	GateReasonConfigInvalid = 5
};
}

HydroAllocator::HydroAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_RT%u_IDX", i);
		_param_handles.hy_rt_idx[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_SV%u_IDX", i);
		_param_handles.hy_sv_idx[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_VZRT%u_PIT_R", i);
		_param_handles.hy_vzrt_pit_r[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_VZRT%u_YAW_R", i);
		_param_handles.hy_vzrt_yaw_r[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_VZRT%u_ROL_R", i);
		_param_handles.hy_vzrt_rol_r[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_VXRT%u_PIT_R", i);
		_param_handles.hy_vxrt_pit_r[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_VXRT%u_YAW_R", i);
		_param_handles.hy_vxrt_yaw_r[i] = param_find(buffer);
	}
	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_VXRT%u_ROL_R", i);
		_param_handles.hy_vxrt_rol_r[i] = param_find(buffer);
	}
}

HydroAllocator::~HydroAllocator()
{
	perf_free(_loop_perf);
}

bool
HydroAllocator::init()
{
	if (!_hydro_torque_setpoint_sub.registerCallback() || !_hydro_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
HydroAllocator::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}

	//更新多实例参数
	for (int i = 0; i < 2; ++i) {
		param_get(_param_handles.hy_rt_idx[i], &_params.hy_rt_idx[i]);
		param_get(_param_handles.hy_sv_idx[i], &_params.hy_sv_idx[i]);
		param_get(_param_handles.hy_vzrt_pit_r[i], &_params.hy_vzrt_pit_r[i]);
		param_get(_param_handles.hy_vzrt_yaw_r[i], &_params.hy_vzrt_yaw_r[i]);
		param_get(_param_handles.hy_vzrt_rol_r[i], &_params.hy_vzrt_rol_r[i]);
		param_get(_param_handles.hy_vxrt_pit_r[i], &_params.hy_vxrt_pit_r[i]);
		param_get(_param_handles.hy_vxrt_yaw_r[i], &_params.hy_vxrt_yaw_r[i]);
		param_get(_param_handles.hy_vxrt_rol_r[i], &_params.hy_vxrt_rol_r[i]);
	}
}

void HydroAllocator::Run()
{
	if (should_exit()) {
		_hydro_torque_setpoint_sub.unregisterCallback();
		_hydro_thrust_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	parameters_update();

	if (_hydro_torque_setpoint_sub.update(&_hydro_torque_setpoint_msg))
	{
		;
	}
	if (_hydro_thrust_setpoint_sub.update(&_hydro_thrust_setpoint_msg))
	{
		;
	}
	_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

	_vehicle_status_sub.update(&_vehicle_status);
	_rc_channels_sub.update(&_rc_channels);
	_input_rc_sub.update(&_input_rc);

	_vehicle_attitude_sub.copy(&_vehicle_attitude);
	Eulerf euler_angles(Quatf(_vehicle_attitude.q));

	// 机翼折叠功能
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

	float foldwing_delta_a = dt / _param_hy_fdw_cta.get();
	float foldwing_delta_b = dt / _param_hy_fdw_ctb.get();
	float foldwing_sp_final;

	float fdw_aux;

	switch (_param_hy_fdw_aux.get()) {
		case 0:
		fdw_aux = 0;
		break;

		case 1:
		fdw_aux = _manual_control_setpoint.aux1;
		break;

		case 2:
		fdw_aux = _manual_control_setpoint.aux2;
		break;

		case 3:
		fdw_aux = _manual_control_setpoint.aux3;
		break;

		case 4:
		fdw_aux = _manual_control_setpoint.aux4;
		break;

		case 5:
		fdw_aux = _manual_control_setpoint.aux5;
		break;

		case 6:
		fdw_aux = _manual_control_setpoint.aux6;
		break;

		case 7:
		fdw_aux = -1.0f;
		break;

		case 8:
		fdw_aux = 1.0f;
		break;

		default:
		fdw_aux = 0;
	}

	fdw_aux *= _param_hy_fdw_auxgain.get();

	if(fdw_aux < _param_hy_fdw_dnthr.get()) {
		foldwing_sp_final = -1;
	}
	else if(fdw_aux > _param_hy_fdw_upthr.get()){
		foldwing_sp_final = 1;
	}
	else {
		foldwing_sp_final = 0;
	}

	if(foldwing_sp_final > _foldwing_sp) {
		if(foldwing_sp_final - _foldwing_sp > foldwing_delta_a)
			_foldwing_sp += foldwing_delta_a;
		else
			_foldwing_sp = foldwing_sp_final;
	}
	else {
		if(_foldwing_sp - foldwing_sp_final > foldwing_delta_b)
			_foldwing_sp -= foldwing_delta_b;
		else
			_foldwing_sp = foldwing_sp_final;
	}
	_foldwing_sp = math::constrain(_foldwing_sp, -1.0f, 1.0f);

	_last_run = now;




	hydro_allocate_message_s hydro_allocate_message_msg{0};
	haar_depth_allocator_status_s depth_allocator_status{};

	//执行器顺序：x0 z0 x1 z1
	//力矩顺序：  rol pit yaw
	float allocate_matrix_array[4][3] = {
		{_params.hy_vxrt_rol_r[0], _params.hy_vxrt_pit_r[0], _params.hy_vxrt_yaw_r[0]},
		{_params.hy_vzrt_rol_r[0], _params.hy_vzrt_pit_r[0], _params.hy_vzrt_yaw_r[0]},
		{_params.hy_vxrt_rol_r[1], _params.hy_vxrt_pit_r[1], _params.hy_vxrt_yaw_r[1]},
		{_params.hy_vzrt_rol_r[1], _params.hy_vzrt_pit_r[1], _params.hy_vzrt_yaw_r[1]}
	};

	hydrofoil::AllocationContext allocation_context{};
	allocation_context.body_pitch = euler_angles.theta();
	allocation_context.lift_coefficient = _param_hy_wing_kl.get();
	allocation_context.max_hydrofoil_angle = _param_hy_wing_max_a.get();
	allocation_context.max_thruster_thrust = _param_hy_rt_max_thrust.get();
	allocation_context.available_thruster_thrust = math::constrain(_param_hy_th_max_gain.get()
			* (_manual_control_setpoint.throttle + 1.f) * 0.5f, 0.f, 1.f) * _param_hy_rt_max_thrust.get();
	allocation_context.virtual_control_matrix = Matrix<float, 4, 3>(allocate_matrix_array);

	debug_vect_s debug_vect_msg{};
	const bool new_speed_sample = _debug_vect_sub.update(&debug_vect_msg);
	const uint64_t speed_sample_timestamp =
		new_speed_sample && debug_vect_msg.timestamp != 0 ? debug_vect_msg.timestamp : now;
	const hydrofoil::SpeedContextCache::Result speed_context = _speed_context_cache.update(
			_param_hy_speed_select.get() != 0, _param_hy_alt_speed.get(), now,
			new_speed_sample, speed_sample_timestamp, debug_vect_msg.z);
	allocation_context.speed_squared = speed_context.speed_squared;

	hydrofoil::AllocationCommand allocation_command{};
	allocation_command.force_x = _hydro_thrust_setpoint_msg.xyz[0];
	allocation_command.force_z = _hydro_thrust_setpoint_msg.xyz[2];
	allocation_command.u_phi = _hydro_torque_setpoint_msg.xyz[0];
	allocation_command.u_theta = _hydro_torque_setpoint_msg.xyz[1];
	allocation_command.u_psi = _hydro_torque_setpoint_msg.xyz[2];
	const hydrofoil::AllocationSolution allocation_solution =
		hydrofoil::HydrofoilAllocationModel::solve(allocation_context, allocation_command);
	const hydrofoil::AllocationLinearization linearization =
		hydrofoil::HydrofoilAllocationModel::linearize(allocation_context, allocation_solution);
	const bool allocator_bounded =
		hydrofoil::HydrofoilAllocationModel::isFiniteBounded(allocation_context, allocation_solution);
	const int32_t safe_roll_mode = _param_hy_sr_mode.get();
	// HEAD 5ae2fd95 emitted every finite constrained Newton iterate. Preserve
	// that complete-vector behavior for legacy and operational SafeRoll modes;
	// Mode 4 retains its deliberately strict allocator semantics.
	const bool bounded_best_feasible_allowed = safe_roll_mode >= 0 && safe_roll_mode <= 3;
	const hydrofoil::ExecutableAllocation executable =
		hydrofoil::HydrofoilAllocationModel::selectExecutable(
			allocation_context, allocation_solution, bounded_best_feasible_allowed,
			_previous_executed_delta_left, _previous_executed_delta_right,
			math::radians(_param_hy_sr_drate.get()), dt);
	const hydrofoil::AllocationSolution &executed_solution =
		executable.usable ? executable.solution : allocation_solution;
	const hydrofoil::AllocationForwardResult forward_result =
		hydrofoil::HydrofoilAllocationModel::forward(allocation_context, executed_solution);

	float x[2][2] {
		{executed_solution.delta_left_commanded, executed_solution.thrust_left_commanded},
		{executed_solution.delta_right_commanded, executed_solution.thrust_right_commanded}
	};
	hydro_allocate_message_msg.allocator_delta_left_requested = allocation_solution.delta_left_commanded;
	hydro_allocate_message_msg.allocator_delta_right_requested = allocation_solution.delta_right_commanded;
	hydro_allocate_message_msg.allocator_thrust_left_requested = allocation_solution.thrust_left_commanded;
	hydro_allocate_message_msg.allocator_thrust_right_requested = allocation_solution.thrust_right_commanded;
	hydro_allocate_message_msg.allocator_output_suppressed = false;
	depth_allocator_status.full_best_feasible_solution_used = executable.best_feasible;
	depth_allocator_status.best_feasible_rate_limited = executable.rate_limited;
	depth_allocator_status.depth_degradation_active = executable.best_feasible;
	depth_allocator_status.depth_degradation_cumulative_time_us = _depth_degradation_cumulative_time_us;

	if (!executable.usable) {
		x[0][0] = 0.f;
		x[1][0] = 0.f;
		x[0][1] = 0.f;
		x[1][1] = 0.f;
		hydro_allocate_message_msg.allocator_output_suppressed = true;
	}
	depth_allocator_status.allocator_output_suppressed =
		hydro_allocate_message_msg.allocator_output_suppressed;

	if (allocation_solution.status.valid && allocator_bounded) {
		_last_feasible_delta_left = allocation_solution.delta_left_commanded;
		_last_feasible_delta_right = allocation_solution.delta_right_commanded;
		_last_feasible_solution_valid = true;
	}

	if (executable.usable) {
		_previous_executed_delta_left = executed_solution.delta_left_commanded;
		_previous_executed_delta_right = executed_solution.delta_right_commanded;
	}

	allocation_solution.virtual_forces_control.copyTo(hydro_allocate_message_msg.thrust_xz_control);
	allocation_solution.virtual_forces_commanded.copyTo(hydro_allocate_message_msg.thrust_xz_limited);

	hydro_allocate_message_msg.actuator_optimed[0] = x[0][0];
	hydro_allocate_message_msg.actuator_optimed[1] = x[1][0];
	hydro_allocate_message_msg.actuator_optimed[2] = x[0][1];
	hydro_allocate_message_msg.actuator_optimed[3] = x[1][1];
	hydro_allocate_message_msg.timestamp_sample = _hydro_torque_setpoint_msg.timestamp_sample;
	hydro_allocate_message_msg.allocator_valid = allocation_solution.status.valid;
	hydro_allocate_message_msg.allocator_converged = allocation_solution.status.converged;
	hydro_allocate_message_msg.allocator_finite = allocation_solution.status.finite;
	hydro_allocate_message_msg.allocator_bounded = allocator_bounded;
	hydro_allocate_message_msg.allocator_iterations = allocation_solution.status.iterations;
	hydro_allocate_message_msg.allocator_residual = allocation_solution.status.residual;
	hydro_allocate_message_msg.allocator_saturation_mask = allocation_solution.status.saturation_mask;
	hydro_allocate_message_msg.allocator_solve_status = static_cast<uint8_t>(allocation_solution.status.solve_status);
	hydro_allocate_message_msg.jacobian_condition = allocation_solution.status.jacobian_condition;
	hydro_allocate_message_msg.u_phi_effective_model = forward_result.u_phi_effective_model;
	hydro_allocate_message_msg.u_effective_is_model_based = true;
	hydro_allocate_message_msg.foil_position_feedback_available = false;
	hydro_allocate_message_msg.g_left = linearization.g_left;
	hydro_allocate_message_msg.g_right = linearization.g_right;
	hydro_allocate_message_msg.logical_left_servo_index = _params.hy_sv_idx[0] - 1;
	hydro_allocate_message_msg.logical_right_servo_index = _params.hy_sv_idx[1] - 1;
	hydro_allocate_message_msg.logical_mapping_matches_expected =
		(hydro_allocate_message_msg.logical_left_servo_index == 4)
		&& (hydro_allocate_message_msg.logical_right_servo_index == 3);
	hydro_allocate_message_msg.lift_coefficient = allocation_context.lift_coefficient;
	hydro_allocate_message_msg.speed_squared = allocation_context.speed_squared;
	hydro_allocate_message_msg.speed_context_cached = speed_context.cached;
	hydro_allocate_message_msg.speed_context_stale = speed_context.stale;
	hydro_allocate_message_msg.speed_context_age_us = speed_context.age_us;
	hydro_allocate_message_msg.body_pitch = allocation_context.body_pitch;
	hydro_allocate_message_msg.context_max_hydrofoil_angle = allocation_context.max_hydrofoil_angle;
	hydro_allocate_message_msg.max_thruster_thrust = allocation_context.max_thruster_thrust;
	hydro_allocate_message_msg.available_thruster_thrust = allocation_context.available_thruster_thrust;
	allocation_context.virtual_control_matrix.copyTo(hydro_allocate_message_msg.virtual_control_matrix);
	hydro_allocate_message_msg.command_force_x = allocation_command.force_x;
	hydro_allocate_message_msg.command_force_z = allocation_command.force_z;
	hydro_allocate_message_msg.command_u_phi = allocation_command.u_phi;
	hydro_allocate_message_msg.command_u_theta = allocation_command.u_theta;
	hydro_allocate_message_msg.command_u_psi = allocation_command.u_psi;
	const hydrofoil::FoilCommandComponents requested_components =
		hydrofoil::HydrofoilAllocationModel::decomposeFoilCommand(
			allocation_solution.delta_left_commanded, allocation_solution.delta_right_commanded);
	const hydrofoil::FoilCommandComponents realized_components =
		hydrofoil::HydrofoilAllocationModel::decomposeFoilCommand(x[0][0], x[1][0]);
	const hydrofoil::FoilCommandComponents last_feasible_components =
		hydrofoil::HydrofoilAllocationModel::decomposeFoilCommand(
			_last_feasible_delta_left, _last_feasible_delta_right);
	depth_allocator_status.timestamp_sample = _hydro_torque_setpoint_msg.timestamp_sample;
	depth_allocator_status.allocator_valid = allocation_solution.status.valid;
	depth_allocator_status.allocator_converged = allocation_solution.status.converged;
	depth_allocator_status.allocator_finite = allocation_solution.status.finite;
	depth_allocator_status.allocator_bounded = allocator_bounded;
	depth_allocator_status.allocator_residual = allocation_solution.status.residual;
	depth_allocator_status.allocator_iterations = allocation_solution.status.iterations;
	depth_allocator_status.allocator_saturation_mask = allocation_solution.status.saturation_mask;
	depth_allocator_status.speed_squared = speed_context.speed_squared;
	depth_allocator_status.speed_source = _param_hy_speed_select.get() == 0 ? 0 : 1;
	depth_allocator_status.speed_context_stale = speed_context.stale;
	depth_allocator_status.requested_force_z = allocation_command.force_z;
	depth_allocator_status.realized_force_z =
		forward_result.valid
		? forward_result.virtual_forces_reconstructed(1) + forward_result.virtual_forces_reconstructed(3) : NAN;
	depth_allocator_status.requested_symmetric_foil_component =
		requested_components.valid ? requested_components.symmetric : NAN;
	depth_allocator_status.realized_symmetric_foil_component =
		realized_components.valid ? realized_components.symmetric : NAN;
	depth_allocator_status.requested_differential_foil_component =
		requested_components.valid ? requested_components.differential : NAN;
	depth_allocator_status.realized_differential_foil_component =
		realized_components.valid ? realized_components.differential : NAN;
	depth_allocator_status.last_feasible_symmetric_foil_component =
		_last_feasible_solution_valid && last_feasible_components.valid ? last_feasible_components.symmetric : NAN;
	depth_allocator_status.last_feasible_differential_foil_component =
		_last_feasible_solution_valid && last_feasible_components.valid ? last_feasible_components.differential : NAN;

	//归一化，并再次限幅确保安全
	x[0][0] = math::constrain(x[0][0] / _param_hy_wing_max_a.get(), -1.f, 1.f);
	x[1][0] = math::constrain(x[1][0] / _param_hy_wing_max_a.get(), -1.f, 1.f);
	x[0][1] = math::constrain(x[0][1] / _param_hy_rt_max_thrust.get(), 0.f, 1.f);
	x[1][1] = math::constrain(x[1][1] / _param_hy_rt_max_thrust.get(), 0.f, 1.f);

	//根据参数设置的对应关系填入数据并发送
	actuator_motors_s hydro_motors_msg{0};
	actuator_servos_s hydro_servos_msg{0};

	hydro_motors_msg.timestamp = hrt_absolute_time();
	hydro_motors_msg.timestamp_sample = _hydro_thrust_setpoint_msg.timestamp_sample;

	hydro_servos_msg.timestamp = hrt_absolute_time();
	hydro_servos_msg.timestamp_sample = _hydro_torque_setpoint_msg.timestamp_sample;

	// 主螺旋桨倾转功能
	hydro_tilt_message_s hydro_tilt_message_msg{0};
	float tilt_aux;
	switch (_param_hy_tilt_aux.get()) {
		case 0:
		tilt_aux = 0;
		break;

		case 1:
		tilt_aux = _manual_control_setpoint.aux1;
		break;

		case 2:
		tilt_aux = _manual_control_setpoint.aux2;
		break;

		case 3:
		tilt_aux = _manual_control_setpoint.aux3;
		break;

		case 4:
		tilt_aux = _manual_control_setpoint.aux4;
		break;

		case 5:
		tilt_aux = _manual_control_setpoint.aux5;
		break;

		case 6:
		tilt_aux = _manual_control_setpoint.aux6;
		break;

		case 7:
		tilt_aux = -1.0f;
		break;

		case 8:
		tilt_aux = 1.0f;
		break;

		default:
		tilt_aux = 0;
	}
	tilt_aux *= _param_hy_tilt_auxgain.get();

	float tilt_output = 0.0f;
	float tilt_sp = 0.0f;

	float tilt_sp_simple;
	float tilt_sp_complex;

	Quatf q(_vehicle_attitude.q);
	Vector3f Oz_in_A; // O系z轴在A系下的表示
	Oz_in_A(0) = 2*(q(1)*q(3) - q(0)*q(2));
	Oz_in_A(1) = 2*(q(2)*q(3) + q(0)*q(1));
	Oz_in_A(2) = 1 - 2*(q(1)*q(1) + q(2)*q(2));

	Vector3f Ax_in_A(1,0,0);// A系x轴在A系下的表示
	Vector3f Ay_in_A(0,1,0);// A系y轴在A系下的表示，也即倾转平面的法向量
	Vector3f Az_in_A(0,0,1);// A系z轴在A系下的表示

	float angle_simple = acos((Ax_in_A * Oz_in_A) / (Ax_in_A.norm() * Oz_in_A.norm()));
	tilt_sp_simple = 2.0f - ( angle_simple / (float)M_PI_2 );

	Vector3f F_in_A; // 期望的推力方向，相当于Oz_in_A投影到A系xz平面
	F_in_A(0) = -Oz_in_A(0);
	F_in_A(1) = 0;
	F_in_A(2) = -Oz_in_A(2);

	if(F_in_A.norm() < 0.1f){
		;//防抖
	}

	float angle_complex = acos((Ax_in_A * F_in_A) / (Ax_in_A.norm() * F_in_A.norm()));

	Vector3f y1_complex = Ax_in_A.cross(F_in_A);
	if(y1_complex * Ay_in_A < 0) angle_complex *= -1.0f;

	tilt_sp_complex = angle_complex / (float)M_PI_2;

	int32_t mode = _param_hy_tilt_mode.get();
	if(mode == 0){
		tilt_sp = 0;
	}
	else if(mode == 1){
		tilt_sp = tilt_sp_simple;
	}
	else if(mode == 2) {
		tilt_sp = tilt_sp_complex;
	}
	else {
		tilt_sp = _hydro_torque_setpoint_msg.xyz[1];
	}
	tilt_sp = tilt_sp * _param_hy_tilt_k.get() + _param_hy_tilt_b.get();
	tilt_sp = math::constrain(tilt_sp, -1.0f, 1.0f);

	if(tilt_aux > _param_hy_tilt_thr.get() && (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL)) {
		//启用倾转
		tilt_output = tilt_sp;
	}
	else{
		//禁用倾转
		tilt_output = _param_hy_tilt_dis.get();
	}

	hydro_tilt_message_msg.tilt_sp_simple = tilt_sp_simple;
	hydro_tilt_message_msg.tilt_sp_complex = tilt_sp_complex;
	hydro_tilt_message_msg.tilt_sp = tilt_sp;
	hydro_tilt_message_msg.tilt_output = tilt_output;
	hydro_tilt_message_msg.angle_simple = angle_simple;
	hydro_tilt_message_msg.angle_complex = angle_complex;
	hydro_tilt_message_msg.oz_in_a[0] = Oz_in_A(0);
	hydro_tilt_message_msg.oz_in_a[1] = Oz_in_A(1);
	hydro_tilt_message_msg.oz_in_a[2] = Oz_in_A(2);



	//水翼启用与禁用功能
	const int32_t foil_aux_parameter = _param_hy_foil_aux.get();
	float foil_aux;
	switch (foil_aux_parameter) {
		case 0:
		foil_aux = 0;
		break;

		case 1:
		foil_aux = _manual_control_setpoint.aux1;
		break;

		case 2:
		foil_aux = _manual_control_setpoint.aux2;
		break;

		case 3:
		foil_aux = _manual_control_setpoint.aux3;
		break;

		case 4:
		foil_aux = _manual_control_setpoint.aux4;
		break;

		case 5:
		foil_aux = _manual_control_setpoint.aux5;
		break;

		case 6:
		foil_aux = _manual_control_setpoint.aux6;
		break;

		case 7:
		foil_aux = -1.0f;
		break;

		case 8:
		foil_aux = 1.0f;
		break;

		default:
		foil_aux = 0;
	}
	foil_aux *= _param_hy_foil_auxgain.get();
	const bool aux_uses_manual_input = foil_aux_parameter >= 1 && foil_aux_parameter <= 6;
	const bool gate_config_valid = foil_aux_parameter >= 0 && foil_aux_parameter <= 8
				       && PX4_ISFINITE(_param_hy_foil_auxgain.get())
				       && PX4_ISFINITE(_param_hy_foil_thr.get());
	const bool aux_value_finite = PX4_ISFINITE(foil_aux);
	const uint64_t manual_age_us =
		_manual_control_setpoint.timestamp_sample != 0 && now >= _manual_control_setpoint.timestamp_sample
		? now - _manual_control_setpoint.timestamp_sample : UINT64_MAX;
	const uint64_t manual_timeout_us = static_cast<uint64_t>(
			math::max(_param_com_rc_loss_t.get(), 0.f) * 1e6f);
	int8_t foil_aux_rc_channel = 0;
	bool foil_aux_mapping_valid = !aux_uses_manual_input;

	if (aux_uses_manual_input
	    && _manual_control_setpoint.data_source == manual_control_setpoint_s::SOURCE_RC) {
		const int function = rc_channels_s::FUNCTION_AUX_1 + foil_aux_parameter - 1;
		const int channel = _rc_channels.function[function];
		foil_aux_rc_channel = channel >= 0 ? static_cast<int8_t>(channel + 1) : 0;
		foil_aux_mapping_valid = channel >= 0 && channel < _rc_channels.channel_count;

	} else if (aux_uses_manual_input) {
		foil_aux_mapping_valid =
			_manual_control_setpoint.data_source != manual_control_setpoint_s::SOURCE_UNKNOWN;
	}

	const bool manual_data_fresh = !aux_uses_manual_input
				       || (_manual_control_setpoint.valid
					   && _manual_control_setpoint.timestamp_sample != 0
					   && now >= _manual_control_setpoint.timestamp_sample
					   && manual_age_us <= manual_timeout_us
					   && foil_aux_mapping_valid);
	const bool aux_gate_enabled_raw = gate_config_valid && aux_value_finite && manual_data_fresh
					  && foil_aux > _param_hy_foil_thr.get();
	const bool nav_gate_enabled_raw =
		_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB
		|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO
		|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL;
	const bool foil_gate_enabled = aux_gate_enabled_raw && nav_gate_enabled_raw;
	const int32_t allocate_mode = _param_hy_allocate_mode.get();
	uint8_t gate_close_reason = GateReasonNone;

	if (!gate_config_valid) {
		gate_close_reason = GateReasonConfigInvalid;

	} else if (!aux_value_finite) {
		gate_close_reason = GateReasonAuxNonfinite;

	} else if (!manual_data_fresh) {
		gate_close_reason = GateReasonAuxStale;

	} else if (!aux_gate_enabled_raw) {
		gate_close_reason = GateReasonAuxThreshold;

	} else if (!nav_gate_enabled_raw) {
		gate_close_reason = GateReasonNavState;
	}

	float man_delta_motor = allocation_command.u_psi * _param_hy_alct_yaw_man.get();
	float man_delta_servo = allocation_command.u_phi * _param_hy_alct_rol_man.get();
	float man_manual_motor = (_manual_control_setpoint.throttle+1)*0.5f * _param_hy_alct_mt_man.get();
	float man_manual_servo = _manual_control_setpoint.pitch * _param_hy_alct_sv_man.get();

	hydrofoil::AllocationSolution legacy_fixed_solution{};
	legacy_fixed_solution.delta_left_commanded =
		math::constrain(_param_hy_sv_l_fm.get(), -1.f, 1.f) * allocation_context.max_hydrofoil_angle;
	legacy_fixed_solution.delta_right_commanded =
		math::constrain(_param_hy_sv_r_fm.get(), -1.f, 1.f) * allocation_context.max_hydrofoil_angle;
	legacy_fixed_solution.thrust_left_commanded =
		math::constrain(allocation_solution.virtual_forces_commanded(0), 0.f,
				allocation_context.max_thruster_thrust);
	legacy_fixed_solution.thrust_right_commanded =
		math::constrain(allocation_solution.virtual_forces_commanded(2), 0.f,
				allocation_context.max_thruster_thrust);

	hydrofoil::AllocationSolution legacy_manual_solution{};
	legacy_manual_solution.delta_left_commanded =
		math::constrain(man_manual_servo + man_delta_servo, -1.f, 1.f)
		* allocation_context.max_hydrofoil_angle;
	legacy_manual_solution.delta_right_commanded =
		math::constrain(man_manual_servo - man_delta_servo, -1.f, 1.f)
		* allocation_context.max_hydrofoil_angle;
	legacy_manual_solution.thrust_left_commanded =
		math::constrain(man_manual_motor + man_delta_motor, 0.f, 1.f)
		* allocation_context.max_thruster_thrust;
	legacy_manual_solution.thrust_right_commanded =
		math::constrain(man_manual_motor - man_delta_motor, 0.f, 1.f)
		* allocation_context.max_thruster_thrust;

	const hydrofoil::AllocationModeOutput mode_output =
		hydrofoil::HydrofoilAllocationModel::selectAllocationModeOutput(
			executable, allocate_mode, static_cast<uint8_t>(safe_roll_mode),
			legacy_fixed_solution, legacy_manual_solution);
	if (!mode_output.selected) {
		hydro_allocate_message_msg.allocator_output_suppressed = true;
		depth_allocator_status.allocator_output_suppressed = true;
	}
	const hydrofoil::FoilCommandComponents executable_components =
		hydrofoil::HydrofoilAllocationModel::decomposeFoilCommand(
			executed_solution.delta_left_commanded, executed_solution.delta_right_commanded);
	const hydrofoil::FoilCommandComponents selected_components =
		hydrofoil::HydrofoilAllocationModel::decomposeFoilCommand(
			mode_output.solution.delta_left_commanded, mode_output.solution.delta_right_commanded);

	hydro_allocate_message_msg.man_delta_motor = man_delta_motor;
	hydro_allocate_message_msg.man_delta_servo = man_delta_servo;
	hydro_allocate_message_msg.man_manual_motor = man_manual_motor;
	hydro_allocate_message_msg.man_manual_servo = man_manual_servo;

	hydro_allocate_message_msg.foil_gate_enabled = foil_gate_enabled;
	hydro_allocate_message_msg.aux_gate_enabled_raw = aux_gate_enabled_raw;
	hydro_allocate_message_msg.nav_gate_enabled_raw = nav_gate_enabled_raw;
	hydro_allocate_message_msg.aux_gate_data_valid = gate_config_valid && manual_data_fresh;
	hydro_allocate_message_msg.aux_gate_value_finite = aux_value_finite;
	hydro_allocate_message_msg.gate_close_reason = gate_close_reason;
	hydro_allocate_message_msg.foil_aux_parameter = foil_aux_parameter;
	hydro_allocate_message_msg.foil_aux_value = foil_aux;
	hydro_allocate_message_msg.foil_threshold_parameter = _param_hy_foil_thr.get();
	hydro_allocate_message_msg.foil_aux_rc_channel = foil_aux_rc_channel;
	hydro_allocate_message_msg.foil_aux_mapping_valid = foil_aux_mapping_valid;
	hydro_allocate_message_msg.manual_control_timestamp = _manual_control_setpoint.timestamp;
	hydro_allocate_message_msg.manual_control_timestamp_sample = _manual_control_setpoint.timestamp_sample;
	hydro_allocate_message_msg.manual_control_age_us =
		static_cast<uint32_t>(math::min(manual_age_us, static_cast<uint64_t>(UINT32_MAX)));
	hydro_allocate_message_msg.manual_control_data_source = _manual_control_setpoint.data_source;
	hydro_allocate_message_msg.manual_control_valid = _manual_control_setpoint.valid;
	hydro_allocate_message_msg.rc_signal_lost = _rc_channels.signal_lost;
	hydro_allocate_message_msg.rc_failsafe = _input_rc.rc_failsafe;
	hydro_allocate_message_msg.allocate_mode_parameter = _param_hy_allocate_mode.get();
	hydro_allocate_message_msg.wing_max_angle_parameter = _param_hy_wing_max_a.get();
	_last_foil_aux_value = foil_aux;
	_last_foil_gate_enabled = foil_gate_enabled;

	if (foil_gate_enabled && mode_output.selected) {
		hydro_motors_msg.control[_params.hy_rt_idx[0] - 1] =
			math::constrain(mode_output.solution.thrust_left_commanded
					/ allocation_context.max_thruster_thrust, 0.f, 1.f);
		hydro_motors_msg.control[_params.hy_rt_idx[1] - 1] =
			math::constrain(mode_output.solution.thrust_right_commanded
					/ allocation_context.max_thruster_thrust, 0.f, 1.f);
		hydro_servos_msg.control[_params.hy_sv_idx[0] - 1] =
			math::constrain(mode_output.solution.delta_left_commanded
					/ allocation_context.max_hydrofoil_angle, -1.f, 1.f);
		hydro_servos_msg.control[_params.hy_sv_idx[1] - 1] =
			math::constrain(mode_output.solution.delta_right_commanded
					/ allocation_context.max_hydrofoil_angle, -1.f, 1.f);
	}

	//机翼的通道
	int32_t fdw_idx = _param_hy_fdw_idx.get();
	if(fdw_idx >=1 && fdw_idx <= 8)
	{
		hydro_servos_msg.control[fdw_idx - 1] = _foldwing_sp;
	}

	//倾转的通道
	int32_t tilt_idx = _param_hy_tilt_idx.get();
	if(tilt_idx >=1 && tilt_idx <= 8)
	{
		hydro_servos_msg.control[tilt_idx - 1] = tilt_output;
	}

	_hydro_motors_pub.publish(hydro_motors_msg);
	_hydro_servos_pub.publish(hydro_servos_msg);

	// Reconstruct what was actually published by the hydro allocator, including
	// gate and allocation-mode effects. These are commanded/model-based values.
	hydrofoil::AllocationSolution published_solution = allocation_solution;
	published_solution.delta_left_commanded =
		hydro_servos_msg.control[_params.hy_sv_idx[0] - 1] * _param_hy_wing_max_a.get();
	published_solution.delta_right_commanded =
		hydro_servos_msg.control[_params.hy_sv_idx[1] - 1] * _param_hy_wing_max_a.get();
	published_solution.thrust_left_commanded =
		hydro_motors_msg.control[_params.hy_rt_idx[0] - 1] * _param_hy_rt_max_thrust.get();
	published_solution.thrust_right_commanded =
		hydro_motors_msg.control[_params.hy_rt_idx[1] - 1] * _param_hy_rt_max_thrust.get();
	const hydrofoil::FoilCommandComponents published_components =
		hydrofoil::HydrofoilAllocationModel::decomposeFoilCommand(
			published_solution.delta_left_commanded, published_solution.delta_right_commanded);
	const hydrofoil::AllocationForwardResult published_forward =
		hydrofoil::HydrofoilAllocationModel::forward(allocation_context, published_solution);
	const bool complete_solution_selected =
		mode_output.source == hydrofoil::AllocationOutputSource::CompleteExecutable;
	depth_allocator_status.realized_symmetric_foil_component =
		published_components.valid ? published_components.symmetric : NAN;
	depth_allocator_status.realized_differential_foil_component =
		published_components.valid ? published_components.differential : NAN;
	depth_allocator_status.realized_force_z =
		published_forward.valid
		? published_forward.virtual_forces_reconstructed(1)
		+ published_forward.virtual_forces_reconstructed(3) : NAN;
	depth_allocator_status.allocate_mode = allocate_mode;
	depth_allocator_status.executable_solution_selected =
		mode_output.selected && complete_solution_selected;
	depth_allocator_status.selected_solution_source =
		static_cast<uint8_t>(mode_output.source);
	depth_allocator_status.mode_branch_overrode_solution =
		mode_output.mode_branch_overrode_solution;
	depth_allocator_status.mode_branch_symmetric_before =
		executable_components.valid ? executable_components.symmetric : NAN;
	depth_allocator_status.mode_branch_symmetric_after =
		selected_components.valid ? selected_components.symmetric : NAN;
	depth_allocator_status.mode_branch_differential_before =
		executable_components.valid ? executable_components.differential : NAN;
	depth_allocator_status.mode_branch_differential_after =
		selected_components.valid ? selected_components.differential : NAN;
	depth_allocator_status.selected_thrust_left =
		mode_output.selected ? mode_output.solution.thrust_left_commanded : NAN;
	depth_allocator_status.selected_thrust_right =
		mode_output.selected ? mode_output.solution.thrust_right_commanded : NAN;
	depth_allocator_status.published_hydro_left =
		published_solution.delta_left_commanded;
	depth_allocator_status.published_hydro_right =
		published_solution.delta_right_commanded;
	constexpr float symmetric_loss_threshold = math::radians(0.1f);
	depth_allocator_status.symmetric_depth_lost =
		executable_components.valid && published_components.valid
		&& fabsf(executable_components.symmetric) > symmetric_loss_threshold
		&& fabsf(published_components.symmetric) <= symmetric_loss_threshold;
	depth_allocator_status.symmetric_loss_reason =
		haar_depth_allocator_status_s::SYMMETRIC_LOSS_NONE;

	if (depth_allocator_status.symmetric_depth_lost) {
		if (!foil_gate_enabled) {
			depth_allocator_status.symmetric_loss_reason =
				haar_depth_allocator_status_s::SYMMETRIC_LOSS_FOIL_GATE;

		} else if (!mode_output.selected) {
			depth_allocator_status.symmetric_loss_reason =
				haar_depth_allocator_status_s::SYMMETRIC_LOSS_ALLOCATOR_SUPPRESSED;

		} else if (mode_output.mode_branch_overrode_solution) {
			depth_allocator_status.symmetric_loss_reason =
				haar_depth_allocator_status_s::SYMMETRIC_LOSS_MODE_BRANCH_OVERRIDE;
		}
	}

	const bool complete_best_feasible_applied =
		executable.best_feasible && foil_gate_enabled
		&& mode_output.selected && complete_solution_selected;
	depth_allocator_status.full_best_feasible_solution_used = complete_best_feasible_applied;
	depth_allocator_status.best_feasible_rate_limited =
		complete_best_feasible_applied && executable.rate_limited;
	depth_allocator_status.depth_degradation_active = complete_best_feasible_applied;

	if (complete_best_feasible_applied) {
		_depth_degradation_cumulative_time_us += static_cast<uint64_t>(dt * 1e6f);
	}

	depth_allocator_status.depth_degradation_cumulative_time_us =
		_depth_degradation_cumulative_time_us;

	hydro_allocate_message_msg.timestamp = hrt_absolute_time();
	_hydro_allocate_message_pub.publish(hydro_allocate_message_msg);
	depth_allocator_status.timestamp = hrt_absolute_time();
	_haar_depth_allocator_status_pub.publish(depth_allocator_status);

	hydro_tilt_message_msg.timestamp = hrt_absolute_time();
	_hydro_tilt_message_pub.publish(hydro_tilt_message_msg);

	// backup schedule
	ScheduleDelayed(100_ms);

	perf_end(_loop_perf);
}

int HydroAllocator::print_status()
{
	PX4_INFO("foil gate: AUX selection=%" PRId32 ", value=%.3f, threshold=%.3f, result=%s",
		 _param_hy_foil_aux.get(), static_cast<double>(_last_foil_aux_value),
		 static_cast<double>(_param_hy_foil_thr.get()), _last_foil_gate_enabled ? "enabled" : "disabled");
	PX4_INFO("allocation mode=%" PRId32 ", logical L/R servo indexes=%" PRId32 "/%" PRId32,
		 _param_hy_allocate_mode.get(), _params.hy_sv_idx[0], _params.hy_sv_idx[1]);
	perf_print_counter(_loop_perf);
	return PX4_OK;
}

int HydroAllocator::task_spawn(int argc, char *argv[])
{
	HydroAllocator *instance = new HydroAllocator();

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

int HydroAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HydroAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
HydroAllocator.

)DESCR_STR");

	return 0;
}

extern "C" __EXPORT int hydro_allocator_main(int argc, char *argv[])
{
	return HydroAllocator::main(argc, argv);
}
