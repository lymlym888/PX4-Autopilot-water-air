// LADRC.cpp
#include "LADRC.h"

#include <cmath>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

LADRCController::LADRCController()
{
	updateObserverGains();
	reset(0.0f);
}

void LADRCController::reset(float y0)
{
	if (!PX4_ISFINITE(y0)) {
		y0 = 0.0f;
	}

	_initialized = true;

	// 参考值从当前角速度开始，避免切入时突然产生大角速度误差
	_v1 = y0;

	// LESO 初值从当前测量角速度开始
	_z1 = y0;
	_z2 = 0.0f;
	_z3 = 0.0f;

	_u_prev = 0.0f;

	_error = 0.0f;
	_u_raw = 0.0f;
	_u = 0.0f;
}

void LADRCController::setParams(float b0,
				float omega_c,
				float omega_o,
				float tau_td,
				float u_lim,
				int32_t eso_order,
				float eso_gain_scale)
{
	_b0 = math::max(fabsf(b0), 0.0001f);
	_omega_c = math::max(omega_c, 0.0f);
	_omega_o = math::max(omega_o, 0.0f);
	_tau_td = math::max(tau_td, 0.0f);

	_u_lim = fabsf(u_lim);

	if (_u_lim < 0.0001f) {
		_u_lim = 0.0001f;
	}

	_eso_order = (eso_order == 3) ? 3 : 2;
	_eso_gain_scale = math::max(eso_gain_scale, 0.01f);

	updateObserverGains();
}

void LADRCController::updateObserverGains()
{
	const float w = math::max(_omega_o * _eso_gain_scale, 0.0f);

	if (_eso_order == 3) {
		// 三阶 LESO：
		// beta1 = 3*w
		// beta2 = 3*w^2
		// beta3 = w^3
		_beta1 = 3.0f * w;
		_beta2 = 3.0f * w * w;
		_beta3 = w * w * w;

	} else {
		// 二阶 LESO：
		// beta1 = 2*w
		// beta2 = w^2
		_beta1 = 2.0f * w;
		_beta2 = w * w;
		_beta3 = 0.0f;
	}
}

void LADRCController::updateReference(float r_rate, float dt)
{
	if (_tau_td <= 0.0001f) {
		_v1 = r_rate;
		return;
	}

	const float alpha = math::constrain(dt / _tau_td, 0.0f, 1.0f);
	_v1 += alpha * (r_rate - _v1);
}

void LADRCController::updateLESO(float y_rate, float dt)
{
	// PDF 中的一阶对象形式：
	// x_dot = b0 * u + f
	// y = x
	//
	// 对姿态稳定移植时：
	// x -> 某轴角速度 p/q
	// u -> 归一化力矩
	// f -> 未建模水动力、耦合、扰动等总扰动

	const float e = y_rate - _z1;
	const float u_for_observer = math::constrain(_u_prev, -_u_lim, _u_lim);

	float z1_dot = 0.0f;
	float z2_dot = 0.0f;
	float z3_dot = 0.0f;

	if (_eso_order == 3) {
		// 三阶 LESO
		z1_dot = _z2 + _b0 * u_for_observer + _beta1 * e;
		z2_dot = _z3 + _beta2 * e;
		z3_dot = _beta3 * e;

	} else {
		// 二阶 LESO
		z1_dot = _z2 + _b0 * u_for_observer + _beta1 * e;
		z2_dot = _beta2 * e;
		z3_dot = 0.0f;
	}

	const float z1_new = _z1 + dt * z1_dot;
	const float z2_new = _z2 + dt * z2_dot;
	const float z3_new = (_eso_order == 3) ? (_z3 + dt * z3_dot) : 0.0f;

	if (PX4_ISFINITE(z1_new) && PX4_ISFINITE(z2_new) && PX4_ISFINITE(z3_new)) {
		_z1 = z1_new;
		_z2 = z2_new;
		_z3 = z3_new;

	} else {
		reset(y_rate);
	}
}

float LADRCController::update(float r_rate, float y_rate, float dt)
{
	if (!PX4_ISFINITE(r_rate) || !PX4_ISFINITE(y_rate)) {
		_u_raw = 0.0f;
		_u = 0.0f;
		_u_prev = 0.0f;
		return 0.0f;
	}

	const float dt_used = math::constrain(dt, 0.0005f, 0.05f);

	if (!_initialized) {
		reset(y_rate);
	}

	// 1. 参考角速度一阶平滑
	updateReference(r_rate, dt_used);

	// 2. LESO 使用上一次实际控制量更新观测状态
	updateLESO(y_rate, dt_used);

	// 3. LADRC 控制律
	// u0 = omega_c * (v - z1)
	// u  = (u0 - z2) / b0
	//
	// 二阶 LESO: z2 作为总扰动估计
	// 三阶 LESO: z2 仍作为主要扰动补偿项，z3 用于日志观察扰动变化趋势
	_error = _v1 - _z1;

	const float u0 = _omega_c * _error;
	_u_raw = (u0 - _z2) / _b0;

	if (!PX4_ISFINITE(_u_raw)) {
		_u_raw = 0.0f;
	}

	_u = math::constrain(_u_raw, -_u_lim, _u_lim);

	// 记录本次实际输出，供下一轮 LESO 使用
	_u_prev = _u;

	return _u;
}
