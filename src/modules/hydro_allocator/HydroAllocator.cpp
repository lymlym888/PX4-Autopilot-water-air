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

	_param_handles.fmpc_mode = param_find("FMPC_MODE");
	_param_handles.fmpc_act_en = param_find("FMPC_ACT_EN");
	_param_handles.fmpc_sign_ok = param_find("FMPC_SIGN_OK");
	_param_handles.fmpc_act_lim = param_find("FMPC_ACT_LIM");
	_param_handles.fmpc_bnd_en = param_find("FMPC_BND_EN");
	_param_handles.fmpc_bnd_lim = param_find("FMPC_BND_LIM");
	_param_handles.fmpc_sat_margin = param_find("FMPC_SAT_MARGIN");
	_param_handles.fmpc_full_en = param_find("FMPC_FULL_EN");
	_param_handles.fmpc_full_lim = param_find("FMPC_FULL_LIM");
	_param_handles.fmpc_full_trim_en = param_find("FMPC_FTRIM_EN");
	_param_handles.fmpc_fcand_max = param_find("FMPC_FCAND_MAX");
	_param_handles.fmpc_g_basemin = param_find("FMPC_G_BASEMIN");
	_param_handles.fmpc_g_lowlim = param_find("FMPC_G_LOWLIM");
	_param_handles.fmpc_cand_max = param_find("FMPC_CAND_MAX");
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

	if (_param_handles.fmpc_mode != PARAM_INVALID) { param_get(_param_handles.fmpc_mode, &_params.fmpc_mode); }
	if (_param_handles.fmpc_act_en != PARAM_INVALID) { param_get(_param_handles.fmpc_act_en, &_params.fmpc_act_en); }
	if (_param_handles.fmpc_sign_ok != PARAM_INVALID) { param_get(_param_handles.fmpc_sign_ok, &_params.fmpc_sign_ok); }
	if (_param_handles.fmpc_act_lim != PARAM_INVALID) { param_get(_param_handles.fmpc_act_lim, &_params.fmpc_act_lim); }
	if (_param_handles.fmpc_bnd_en != PARAM_INVALID) { param_get(_param_handles.fmpc_bnd_en, &_params.fmpc_bnd_en); }
	if (_param_handles.fmpc_bnd_lim != PARAM_INVALID) { param_get(_param_handles.fmpc_bnd_lim, &_params.fmpc_bnd_lim); }
	if (_param_handles.fmpc_sat_margin != PARAM_INVALID) { param_get(_param_handles.fmpc_sat_margin, &_params.fmpc_sat_margin); }
	if (_param_handles.fmpc_full_en != PARAM_INVALID) { param_get(_param_handles.fmpc_full_en, &_params.fmpc_full_en); }
	if (_param_handles.fmpc_full_lim != PARAM_INVALID) { param_get(_param_handles.fmpc_full_lim, &_params.fmpc_full_lim); }
	if (_param_handles.fmpc_full_trim_en != PARAM_INVALID) { param_get(_param_handles.fmpc_full_trim_en, &_params.fmpc_full_trim_en); }
	if (_param_handles.fmpc_fcand_max != PARAM_INVALID) { param_get(_param_handles.fmpc_fcand_max, &_params.fmpc_fcand_max); }
	if (_param_handles.fmpc_g_basemin != PARAM_INVALID) { param_get(_param_handles.fmpc_g_basemin, &_params.fmpc_g_basemin); }
	if (_param_handles.fmpc_g_lowlim != PARAM_INVALID) { param_get(_param_handles.fmpc_g_lowlim, &_params.fmpc_g_lowlim); }
	if (_param_handles.fmpc_cand_max != PARAM_INVALID) { param_get(_param_handles.fmpc_cand_max, &_params.fmpc_cand_max); }
}

//待优化求解的方程，优化求解的目标就是让这个方程等于0
Vector2f HydroAllocator::func(Vector2f x, NfParams p)
{
	float alpha = x(0);
	float T = x(1);

	Vector2f out;

	out(0) = p.Fx - p.KL*p.v2*(alpha+p.alpha0)*std::sin(p.alpha0) - T*std::cos(alpha);
	out(1) = p.Fz + p.KL*p.v2*(alpha+p.alpha0)*std::cos(p.alpha0) + T*std::sin(alpha);

	return out;
}

//待优化方程的雅可比矩阵
SquareMatrix<float, 2> HydroAllocator::J_func(Vector2f x, NfParams p)
{
	float alpha = x(0);
	float T = x(1);

	SquareMatrix<float, 2> J;

	J(0, 0) = -p.KL*p.v2*std::sin(p.alpha0) + T*std::sin(alpha);
	J(0, 1) = -std::cos(alpha);
	J(1, 0) = p.KL*p.v2*std::cos(p.alpha0) + T*std::cos(alpha);
	J(1, 1) = std::sin(alpha);

	return J;
}

//使用牛顿法优化求解
void HydroAllocator::optim(float x_array[2], NfParams p)
{
	Vector2f x(x_array);
	SquareMatrix<float, 2> J;
	Vector2f func_out;
	SquareMatrix<float, 2> J_inv;
	Vector2f delta_x;
	Vector2f x_new;

	for(int i=0; i<10; i++)
	{
		J = J_func(x, p);
		func_out = func(x, p);

		inv(J, J_inv);

		delta_x = - J_inv * func_out;

		if(delta_x.norm_squared() < (float)1e-6)
			break;

		x_new = x + delta_x;

		x_new(0) = math::constrain(x_new(0), - _param_hy_wing_max_a.get(), _param_hy_wing_max_a.get());
		float x1_max = math::constrain(_param_hy_th_max_gain.get() * (_manual_control_setpoint.throttle+1)*0.5f, 0.f, 1.f) * _param_hy_rt_max_thrust.get();
		x_new(1) = math::constrain(x_new(1), 0.f, x1_max);

		x = x_new;
	}

	x.copyTo(x_array);
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

	//0是左边，1是右边
	float thrust_x[2];
	float thrust_z[2];
	thrust_x[0] = _hydro_thrust_setpoint_msg.xyz[0] / 2;
	thrust_x[1] = thrust_x[0];
	thrust_z[0] = _hydro_thrust_setpoint_msg.xyz[2] / 2;
	thrust_z[1] = thrust_z[0];

	//执行器顺序：x0 z0 x1 z1
	//力矩顺序：  rol pit yaw
	float allocate_matrix_array[4][3] = {
		{_params.hy_vxrt_rol_r[0], _params.hy_vxrt_pit_r[0], _params.hy_vxrt_yaw_r[0]},
		{_params.hy_vzrt_rol_r[0], _params.hy_vzrt_pit_r[0], _params.hy_vzrt_yaw_r[0]},
		{_params.hy_vxrt_rol_r[1], _params.hy_vxrt_pit_r[1], _params.hy_vxrt_yaw_r[1]},
		{_params.hy_vzrt_rol_r[1], _params.hy_vzrt_pit_r[1], _params.hy_vzrt_yaw_r[1]}
	};

	//计算torque_setpoint导致的各虚拟执行器的期望推力变化量
	Matrix<float, 4, 3> allocate_matrix(allocate_matrix_array);
	Vector3f torque_vector(_hydro_torque_setpoint_msg.xyz);
	Vector4f delta_force_vector;

	delta_force_vector = allocate_matrix * torque_vector;

	thrust_x[0] += delta_force_vector(0);
	thrust_z[0] += delta_force_vector(1);
	thrust_x[1] += delta_force_vector(2);
	thrust_z[1] += delta_force_vector(3);

	hydro_allocate_message_msg.thrust_xz_control[0] = thrust_x[0];
	hydro_allocate_message_msg.thrust_xz_control[1] = thrust_z[0];
	hydro_allocate_message_msg.thrust_xz_control[2] = thrust_x[1];
	hydro_allocate_message_msg.thrust_xz_control[3] = thrust_z[1];

	//避免thrust_x为负，因为这可能导致优化求解出问题
	float sum_thrust_x = thrust_x[0] + thrust_x[1];
	if(sum_thrust_x < 0)
	{
		sum_thrust_x *= 0.5f;
		thrust_x[0] -= sum_thrust_x;
		thrust_x[1] -= sum_thrust_x;
	}
	if(thrust_x[0] < 0)
	{
		thrust_x[1] += thrust_x[0];
		thrust_x[0] = 0;
	}
	else if(thrust_x[1] < 0)
	{
		thrust_x[0] += thrust_x[1];
		thrust_x[1] = 0;
	}

	hydro_allocate_message_msg.thrust_xz_limited[0] = thrust_x[0];
	hydro_allocate_message_msg.thrust_xz_limited[1] = thrust_z[0];
	hydro_allocate_message_msg.thrust_xz_limited[2] = thrust_x[1];
	hydro_allocate_message_msg.thrust_xz_limited[3] = thrust_z[1];

	//定义待优化求解的变量，第一行是左侧水翼，第二行是右侧水翼，第一列是攻角，第二列是推力
	//攻角的初始值为0，推力初始值为x方向期望推力的一半
	float x[2][2] = {
		{0, _hydro_thrust_setpoint_msg.xyz[0] / 2},
		{0, _hydro_thrust_setpoint_msg.xyz[0] / 2}
	};

	//填入方程的参数
	_nf_params.alpha0 = euler_angles.theta();
	_nf_params.KL = _param_hy_wing_kl.get();

	if(_param_hy_speed_select.get() == 0)//使用替代速度
	{
		_nf_params.v2 = _param_hy_alt_speed.get() * _param_hy_alt_speed.get();
	}
	else // 使用实际速度
	{
		debug_vect_s debug_vect_msg;
		if(_debug_vect_sub.update(&debug_vect_msg))
		{
			float real_v2 = debug_vect_msg.z;
			if(real_v2 < 1.0f)
				real_v2 = 1.0f;	// 小于1时按照1来处理
			_nf_params.v2 = real_v2;
		}
	}

	//代入虚拟执行器的期望力，并进行优化求解
	_nf_params.Fx = thrust_x[0];
	_nf_params.Fz = thrust_z[0];
	optim(x[0], _nf_params);
	_nf_params.Fx = thrust_x[1];
	_nf_params.Fz = thrust_z[1];
	optim(x[1], _nf_params);

	hydro_allocate_message_msg.actuator_optimed[0] = x[0][0];
	hydro_allocate_message_msg.actuator_optimed[1] = x[1][0];
	hydro_allocate_message_msg.actuator_optimed[2] = x[0][1];
	hydro_allocate_message_msg.actuator_optimed[3] = x[1][1];

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
	float foil_aux;
	switch (_param_hy_foil_aux.get()) {
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

	float man_delta_motor = torque_vector(2) * _param_hy_alct_yaw_man.get();
	float man_delta_servo = torque_vector(0) * _param_hy_alct_rol_man.get();
	float man_manual_motor = (_manual_control_setpoint.throttle+1)*0.5f * _param_hy_alct_mt_man.get();
	float man_manual_servo = _manual_control_setpoint.pitch * _param_hy_alct_sv_man.get();

	hydro_allocate_message_msg.man_delta_motor = man_delta_motor;
	hydro_allocate_message_msg.man_delta_servo = man_delta_servo;
	hydro_allocate_message_msg.man_manual_motor = man_manual_motor;
	hydro_allocate_message_msg.man_manual_servo = man_manual_servo;

	if(foil_aux > _param_hy_foil_thr.get() && (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO || _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL))
	{
		int32_t allocate_mode = _param_hy_allocate_mode.get();
		if(allocate_mode == 0){
			hydro_motors_msg.control[_params.hy_rt_idx[0] - 1] = x[0][1];
			hydro_motors_msg.control[_params.hy_rt_idx[1] - 1] = x[1][1];

			hydro_servos_msg.control[_params.hy_sv_idx[0] - 1] = x[0][0];
			hydro_servos_msg.control[_params.hy_sv_idx[1] - 1] = x[1][0];
		}
		else if(allocate_mode == 1){
			hydro_motors_msg.control[_params.hy_rt_idx[0] - 1] =
				math::constrain(thrust_x[0] / _param_hy_rt_max_thrust.get(), 0.f, 1.f);
			hydro_motors_msg.control[_params.hy_rt_idx[1] - 1] =
				math::constrain(thrust_x[1] / _param_hy_rt_max_thrust.get(), 0.f, 1.f);

			hydro_servos_msg.control[_params.hy_sv_idx[0] - 1] = _param_hy_sv_l_fm.get();
			hydro_servos_msg.control[_params.hy_sv_idx[1] - 1] = _param_hy_sv_r_fm.get();
		}
		else{
			hydro_motors_msg.control[_params.hy_rt_idx[0] - 1] =
				math::constrain(man_manual_motor + man_delta_motor, 0.f, 1.f);
			hydro_motors_msg.control[_params.hy_rt_idx[1] - 1] =
				math::constrain(man_manual_motor - man_delta_motor, 0.f, 1.f);
			hydro_servos_msg.control[_params.hy_sv_idx[0] - 1] =
				math::constrain(man_manual_servo + man_delta_servo, -1.f, 1.f);
			hydro_servos_msg.control[_params.hy_sv_idx[1] - 1] =
				math::constrain(man_manual_servo - man_delta_servo, -1.f, 1.f);
		}
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

	fixed_mpc_allocator_status_s fixed_mpc_allocator_status_msg{};
	fixed_mpc_allocator_status_msg.timestamp = hrt_absolute_time();
	fixed_mpc_allocator_status_msg.requested_mode = static_cast<uint8_t>(
		math::constrain(_params.fmpc_mode, int32_t{0}, int32_t{6}));
	fixed_mpc_allocator_status_msg.fb = true;
	fixed_mpc_allocator_status_msg.fb_r = static_cast<uint8_t>(fixed_mpc_shadow::FallbackReason::BASE_COMMAND_INVALID);

	const auto same_sign = [](float a, float b) {
		if (fabsf(a) <= 1.0e-6f || fabsf(b) <= 1.0e-6f) {
			return true;
		}
		return (a > 0.f && b > 0.f) || (a < 0.f && b < 0.f);
	};

	const int left_servo_index = _params.hy_sv_idx[0] - 1;
	const int right_servo_index = _params.hy_sv_idx[1] - 1;
	fixed_mpc_allocator_status_msg.left_index = static_cast<int8_t>(left_servo_index);
	fixed_mpc_allocator_status_msg.right_index = static_cast<int8_t>(right_servo_index);

	if (left_servo_index >= 0 && left_servo_index < actuator_servos_s::NUM_CONTROLS &&
	    right_servo_index >= 0 && right_servo_index < actuator_servos_s::NUM_CONTROLS &&
	    left_servo_index != right_servo_index) {
		const float servo_l_base = hydro_servos_msg.control[left_servo_index];
		const float servo_r_base = hydro_servos_msg.control[right_servo_index];
		const float d_base_a = servo_l_base - servo_r_base;
		const float d_base_b = servo_r_base - servo_l_base;
		const float mean_servo = 0.5f * (servo_l_base + servo_r_base);

		fixed_mpc_allocator_status_msg.base_ok = PX4_ISFINITE(servo_l_base) && PX4_ISFINITE(servo_r_base) && PX4_ISFINITE(d_base_a);
		fixed_mpc_allocator_status_msg.fallback_valid = fixed_mpc_allocator_status_msg.base_ok;
		fixed_mpc_allocator_status_msg.base_src = static_cast<uint8_t>(fixed_mpc_shadow::FmpcBaseSource::ALLOCATOR_STATUS);
		fixed_mpc_allocator_status_msg.srv_l_b = servo_l_base;
		fixed_mpc_allocator_status_msg.srv_r_b = servo_r_base;
		fixed_mpc_allocator_status_msg.d_base_a = d_base_a;
		fixed_mpc_allocator_status_msg.d_base_b = d_base_b;
		fixed_mpc_allocator_status_msg.srv_l_o = servo_l_base;
		fixed_mpc_allocator_status_msg.srv_r_o = servo_r_base;

		_fixed_mpc_tiny_correction_sub.update(&_fixed_mpc_tiny_correction_msg);
		fixed_mpc_allocator_status_msg.source_timestamp = _fixed_mpc_tiny_correction_msg.timestamp;
		fixed_mpc_allocator_status_msg.mode_entry_timestamp = _fixed_mpc_tiny_correction_msg.mode_entry_timestamp;
		fixed_mpc_allocator_status_msg.selector_epoch = _fixed_mpc_tiny_correction_msg.selector_epoch;
		fixed_mpc_allocator_status_msg.source_sequence = _fixed_mpc_tiny_correction_msg.command_sequence;
		fixed_mpc_allocator_status_msg.mode_session_id = _fixed_mpc_tiny_correction_msg.mode_session_id;
		fixed_mpc_allocator_status_msg.source_mode = _fixed_mpc_tiny_correction_msg.mode;
		fixed_mpc_allocator_status_msg.requested_target_a = _fixed_mpc_tiny_correction_msg.requested_target_a;
		const float corr_age_ms = (_fixed_mpc_tiny_correction_msg.timestamp > 0) ? (hrt_absolute_time() - _fixed_mpc_tiny_correction_msg.timestamp) / 1000.f : 1.0e9f;
		const bool corr_topic_fresh = corr_age_ms <= fixed_mpc_shadow::kAllocatorCorrectionTopicMaxAgeMs;
		const bool mode_active_tiny = _params.fmpc_mode == static_cast<int32_t>(fixed_mpc_shadow::FmpcMode::ACTIVE_TINY);
		const bool mode_active_bounded = _params.fmpc_mode == static_cast<int32_t>(fixed_mpc_shadow::FmpcMode::ACTIVE_BOUNDED_MPC);
		const bool mode_full_roll = _params.fmpc_mode == static_cast<int32_t>(fixed_mpc_shadow::FmpcMode::FULL_ROLL_MPC_TEST);
		const bool any_active_mode = mode_active_tiny || mode_active_bounded || mode_full_roll;
		const bool active_param_enabled = mode_full_roll ||
			((_params.fmpc_act_en == 1) && (_params.fmpc_sign_ok == 1));
		const bool mode_enabled = mode_active_tiny ||
			(mode_active_bounded && (_params.fmpc_bnd_en == 1)) ||
			mode_full_roll;
		const bool corr_msg_active = any_active_mode && (_fixed_mpc_tiny_correction_msg.mode == static_cast<uint8_t>(_params.fmpc_mode));

		if (fixed_mpc_allocator_status_msg.base_ok && corr_topic_fresh && any_active_mode && active_param_enabled &&
		    mode_enabled && corr_msg_active && _fixed_mpc_tiny_correction_msg.act_allow && !_fixed_mpc_tiny_correction_msg.fb &&
		    _fixed_mpc_tiny_correction_msg.diag_ok && _fixed_mpc_tiny_correction_msg.sen_ok && _fixed_mpc_tiny_correction_msg.base_ok) {
			const float act_lim = math::constrain(_params.fmpc_act_lim, 0.f, fixed_mpc_shadow::kFmpcActLimHardMax);
			const float bnd_lim = math::constrain(_params.fmpc_bnd_lim, 0.f, fixed_mpc_shadow::kFmpcBndLimHardMax);
			const float full_lim = math::constrain(_params.fmpc_full_lim, 0.f, fixed_mpc_shadow::kFmpcFullLimHardMax);
			const bool full_trim_en = (_params.fmpc_full_trim_en == 1);
			const float full_cand_max = math::constrain(_params.fmpc_fcand_max,
				fixed_mpc_shadow::kFmpcFullCandMaxHardMin,
				fixed_mpc_shadow::kFmpcFullCandMaxHardMax);
			const float sat_margin = math::constrain(_params.fmpc_sat_margin, 0.f, 0.2f);
			const float abs_limit = math::max(1.f - sat_margin, 0.f);
			float corr_a = 0.f;
			float corr_b = 0.f;
			float target_diff_a = d_base_a;
			float target_diff_b = d_base_b;
			bool allow_apply = true;
			uint8_t reject_reason = static_cast<uint8_t>(fixed_mpc_shadow::FallbackReason::NONE);

			if (mode_active_tiny) {
				corr_a = math::constrain(_fixed_mpc_tiny_correction_msg.corr_a, -act_lim, act_lim);
				corr_b = -corr_a;

				if (fabsf(d_base_a) >= _params.fmpc_g_basemin) {
					if (!same_sign(corr_a, d_base_a)) {
						allow_apply = false;
						reject_reason = static_cast<uint8_t>(fixed_mpc_shadow::FallbackReason::DIRECTION_CONFLICT);
					}
				} else {
					const float low_limit = math::min(_params.fmpc_g_lowlim, act_lim);
					corr_a = math::constrain(corr_a, -low_limit, low_limit);
					corr_b = -corr_a;
				}

				target_diff_a = d_base_a + corr_a;
				target_diff_b = d_base_b + corr_b;
			}

			if (mode_active_bounded) {
				corr_a = math::constrain(_fixed_mpc_tiny_correction_msg.corr_a, -bnd_lim, bnd_lim);
				corr_b = -corr_a;

				if (fabsf(d_base_a) >= _params.fmpc_g_basemin) {
					if (!same_sign(corr_a, d_base_a)) {
						allow_apply = false;
						reject_reason = static_cast<uint8_t>(fixed_mpc_shadow::FallbackReason::DIRECTION_CONFLICT);
					}
				} else {
					const float low_limit = math::min(_params.fmpc_g_lowlim, bnd_lim);
					corr_a = math::constrain(corr_a, -low_limit, low_limit);
					corr_b = -corr_a;
				}

				target_diff_a = d_base_a + corr_a;
				target_diff_b = d_base_b + corr_b;
			}

			if (mode_full_roll) {
				if (!PX4_ISFINITE(_fixed_mpc_tiny_correction_msg.cand_a) ||
				    !PX4_ISFINITE(_fixed_mpc_tiny_correction_msg.cand_b) ||
				    !PX4_ISFINITE(_fixed_mpc_tiny_correction_msg.base_a) ||
				    !PX4_ISFINITE(_fixed_mpc_tiny_correction_msg.base_b)) {
					allow_apply = false;
					reject_reason = static_cast<uint8_t>(fixed_mpc_shadow::FallbackReason::FULL_SOURCE_INVALID);

				} else if (full_trim_en) {
					target_diff_a = _fixed_mpc_tiny_correction_msg.cand_a;
					target_diff_b = _fixed_mpc_tiny_correction_msg.cand_b;
				} else {
					target_diff_a = math::constrain(_fixed_mpc_tiny_correction_msg.cand_a, -full_lim, full_lim);
					target_diff_b = math::constrain(_fixed_mpc_tiny_correction_msg.cand_b, -full_lim, full_lim);
				}

				corr_a = target_diff_a - _fixed_mpc_tiny_correction_msg.base_a;
				corr_b = target_diff_b - _fixed_mpc_tiny_correction_msg.base_b;
			}

			const float candidate_limit =
				(mode_full_roll && full_trim_en) ? full_cand_max : _params.fmpc_cand_max;

			if (allow_apply && mode_full_roll) {
				target_diff_a = math::constrain(target_diff_a, -candidate_limit, candidate_limit);
				target_diff_b = -target_diff_a;

			} else if (allow_apply &&
				   (fabsf(target_diff_a) > candidate_limit || fabsf(target_diff_b) > candidate_limit)) {
				allow_apply = false;
				reject_reason = static_cast<uint8_t>(fixed_mpc_shadow::FallbackReason::CANDIDATE_LIMIT);
			}

			if (allow_apply && mode_full_roll) {
				const float max_abs_diff = 2.f * math::max(abs_limit - fabsf(mean_servo), 0.f);
				target_diff_a = math::constrain(target_diff_a, -max_abs_diff, max_abs_diff);
				target_diff_b = -target_diff_a;
			}

			const float servo_l_candidate = mean_servo + 0.5f * target_diff_a;
			const float servo_r_candidate = mean_servo - 0.5f * target_diff_a;

			if (allow_apply && mode_active_bounded &&
			    (fabsf(servo_l_candidate) > abs_limit || fabsf(servo_r_candidate) > abs_limit)) {
				allow_apply = false;
				reject_reason = static_cast<uint8_t>(fixed_mpc_shadow::FallbackReason::SATURATION_NO_HEADROOM);
			}

			if (allow_apply) {
				const float servo_l_out = math::constrain(servo_l_candidate, -1.f, 1.f);
				const float servo_r_out = math::constrain(servo_r_candidate, -1.f, 1.f);
				hydro_servos_msg.control[left_servo_index] = servo_l_out;
				hydro_servos_msg.control[right_servo_index] = servo_r_out;
				const float applied_base_a = mode_full_roll ? _fixed_mpc_tiny_correction_msg.base_a : d_base_a;
				const float applied_base_b = mode_full_roll ? _fixed_mpc_tiny_correction_msg.base_b : d_base_b;
				fixed_mpc_allocator_status_msg.app_ca = (servo_l_out - servo_r_out) - applied_base_a;
				fixed_mpc_allocator_status_msg.app_cb = (servo_r_out - servo_l_out) - applied_base_b;
				fixed_mpc_allocator_status_msg.srv_l_o = servo_l_out;
				fixed_mpc_allocator_status_msg.srv_r_o = servo_r_out;
				fixed_mpc_allocator_status_msg.source_implemented_target_a = servo_l_out - servo_r_out;
				fixed_mpc_allocator_status_msg.source_target_clamped = mode_full_roll &&
					fabsf(fixed_mpc_allocator_status_msg.source_implemented_target_a -
					       fixed_mpc_allocator_status_msg.requested_target_a) > 1.0e-6f;
				fixed_mpc_allocator_status_msg.command_valid = mode_full_roll;
				fixed_mpc_allocator_status_msg.act_ap = true;
				fixed_mpc_allocator_status_msg.fb = false;
				fixed_mpc_allocator_status_msg.fb_r = static_cast<uint8_t>(fixed_mpc_shadow::FallbackReason::NONE);
			} else {
				fixed_mpc_allocator_status_msg.fb = true;
				fixed_mpc_allocator_status_msg.fb_r = reject_reason;
			}
		} else {
			fixed_mpc_allocator_status_msg.fb = true;
			if (any_active_mode && active_param_enabled && !mode_enabled) {
				if (mode_active_bounded) {
					fixed_mpc_allocator_status_msg.fb_r = static_cast<uint8_t>(fixed_mpc_shadow::FallbackReason::BOUNDED_ACTIVE_DISABLED);
				} else if (mode_full_roll) {
					fixed_mpc_allocator_status_msg.fb_r = static_cast<uint8_t>(fixed_mpc_shadow::FallbackReason::FULL_ACTIVE_DISABLED);
				} else {
					fixed_mpc_allocator_status_msg.fb_r = static_cast<uint8_t>(fixed_mpc_shadow::FallbackReason::PARAM_ACTIVE_DISABLED);
				}
			} else {
				fixed_mpc_allocator_status_msg.fb_r = _fixed_mpc_tiny_correction_msg.fb_r;
			}
		}
	}

	_hydro_motors_pub.publish(hydro_motors_msg);
	_hydro_servos_pub.publish(hydro_servos_msg);
	_fixed_mpc_allocator_status_pub.publish(fixed_mpc_allocator_status_msg);

	hydro_allocate_message_msg.timestamp = hrt_absolute_time();
	_hydro_allocate_message_pub.publish(hydro_allocate_message_msg);

	hydro_tilt_message_msg.timestamp = hrt_absolute_time();
	_hydro_tilt_message_pub.publish(hydro_tilt_message_msg);

	// backup schedule
	ScheduleDelayed(100_ms);

	perf_end(_loop_perf);
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
