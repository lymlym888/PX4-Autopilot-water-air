// DepthADRC.cpp
#include "DepthADRC.h"

#include <cmath>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

DepthADRCController::DepthADRCController()
{
	reset(0.0f);
}

void DepthADRCController::reset(float depth_now)
{
	_initialized = PX4_ISFINITE(depth_now);

	if (!_initialized) {
		depth_now = 0.0f;
		_initialized = true;
	}

	// TD 从当前深度开始，避免刚切入控制模式时突然给大控制量
	_td_x1 = depth_now;
	_td_x2 = 0.0f;
	_td_x2_dot = 0.0f;

	// ESO 也从当前深度开始
	_eso_z1 = depth_now;
	_eso_z2 = 0.0f;
	_eso_z3 = 0.0f;

	// KF 初值
	_kf_x1 = depth_now;
	_kf_x2 = 0.0f;

	_p00 = 1.0f;
	_p01 = 0.0f;
	_p10 = 0.0f;
	_p11 = 1.0f;

	_e1 = 0.0f;
	_e2 = 0.0f;
	_u_raw = 0.0f;
	_u_limited = 0.0f;
}

void DepthADRCController::setParams(float b0,
				    float td_r,
				    float eso_beta01,
				    float eso_beta02,
				    float eso_beta03,
				    float eso_alpha1,
				    float eso_alpha2,
				    float eso_delta,
				    float nlsef_alpha1,
				    float nlsef_alpha2,
				    float nlsef_delta,
				    float nlsef_beta1,
				    float nlsef_beta2,
				    float kf_q_pos,
				    float kf_q_vel,
				    float kf_r_meas,
				    float u_min,
				    float u_max)
{
	_b0 = math::max(b0, 0.0001f);

	_td_r = math::max(td_r, 0.0001f);

	_eso_beta01 = math::max(eso_beta01, 0.0f);
	_eso_beta02 = math::max(eso_beta02, 0.0f);
	_eso_beta03 = math::max(eso_beta03, 0.0f);

	_eso_alpha1 = math::constrain(eso_alpha1, 0.01f, 1.0f);
	_eso_alpha2 = math::constrain(eso_alpha2, 0.01f, 1.0f);
	_eso_delta = math::max(eso_delta, 1e-6f);

	_nlsef_alpha1 = math::constrain(nlsef_alpha1, 0.01f, 1.0f);
	_nlsef_alpha2 = math::constrain(nlsef_alpha2, 0.01f, 1.0f);
	_nlsef_delta = math::max(nlsef_delta, 1e-6f);

	_nlsef_beta1 = nlsef_beta1;
	_nlsef_beta2 = nlsef_beta2;

	_kf_q_pos = math::max(kf_q_pos, 0.0f);
	_kf_q_vel = math::max(kf_q_vel, 0.0f);
	_kf_r_meas = math::max(kf_r_meas, 1e-6f);

	_u_min = u_min;
	_u_max = u_max;

	if (_u_min > _u_max) {
		const float tmp = _u_min;
		_u_min = _u_max;
		_u_max = tmp;
	}
}

float DepthADRCController::fal(float e, float alpha, float delta) const
{
	const float abs_e = fabsf(e);

	if (abs_e > delta) {
		return powf(abs_e, alpha) * ((e >= 0.0f) ? 1.0f : -1.0f);

	} else {
		return e / powf(delta, 1.0f - alpha);
	}
}

float DepthADRCController::fst(float e, float v2, float r_set, float h0) const
{
	const float r = math::max(r_set, 0.0001f);
	const float h = math::constrain(h0, 0.0005f, 0.05f);

	const float d = r * h;
	const float d0 = r * h * h;
	const float y = e;

	const float a0 = sqrtf(d * d + 8.0f * r * fabsf(y));

	float a1 = 0.0f;

	if (fabsf(y) > d0) {
		a1 = v2 + (a0 - d) * 0.5f * ((y >= 0.0f) ? 1.0f : -1.0f);

	} else {
		a1 = v2 + y / h;
	}

	if (fabsf(a1) <= d) {
		return -r * a1 / d;

	} else {
		return -r * ((a1 >= 0.0f) ? 1.0f : -1.0f);
	}
}

void DepthADRCController::updateTD(float depth_setpoint, float dt)
{
	// 对应 Python: x2_a = fst(x_1 - h_set, x_2, r, h)
	_td_x2_dot = fst(_td_x1 - depth_setpoint, _td_x2, _td_r, dt);

	const float x1_new = _td_x1 + dt * _td_x2;
	const float x2_new = _td_x2 + dt * _td_x2_dot;

	_td_x1 = x1_new;
	_td_x2 = x2_new;
}

void DepthADRCController::updateKF(float depth_meas, float control_u, float dt)
{
	// 状态：
	// x1 = depth
	// x2 = depth_rate
	//
	// F = [1 dt; 0 1]
	// G = [0; dt*b0]
	// H = [1 0]

	const float x1_pred = _kf_x1 + dt * _kf_x2;
	const float x2_pred = _kf_x2 + dt * _b0 * control_u;

	const float p00_pred = _p00 + dt * (_p10 + _p01) + dt * dt * _p11 + _kf_q_pos;
	const float p01_pred = _p01 + dt * _p11;
	const float p10_pred = _p10 + dt * _p11;
	const float p11_pred = _p11 + _kf_q_vel;

	const float innovation = depth_meas - x1_pred;
	const float s = p00_pred + _kf_r_meas;

	if (s > 1e-9f && PX4_ISFINITE(s)) {
		const float k0 = p00_pred / s;
		const float k1 = p10_pred / s;

		_kf_x1 = x1_pred + k0 * innovation;
		_kf_x2 = x2_pred + k1 * innovation;

		_p00 = (1.0f - k0) * p00_pred;
		_p01 = (1.0f - k0) * p01_pred;
		_p10 = p10_pred - k1 * p00_pred;
		_p11 = p11_pred - k1 * p01_pred;

		// 数值对称化，避免长时间运行后 P01/P10 漂移太多
		const float p01_sym = 0.5f * (_p01 + _p10);
		_p01 = p01_sym;
		_p10 = p01_sym;

	} else {
		_kf_x1 = x1_pred;
		_kf_x2 = x2_pred;

		_p00 = p00_pred;
		_p01 = p01_pred;
		_p10 = p10_pred;
		_p11 = p11_pred;
	}

	if (!PX4_ISFINITE(_kf_x1) || !PX4_ISFINITE(_kf_x2)) {
		reset(depth_meas);
	}
}

void DepthADRCController::updateESO(float y_sys_out, float control_u, float dt)
{
	// 对应 Python:
	// e1 = z1 - y
	// z1 += h * (z2 - beta01 * e1)
	// z2 += h * (z3 - beta02 * fal(e1, alpha1, delta) + b0 * u)
	// z3 += h * (-beta03) * fal(e1, alpha2, delta)

	const float e1 = _eso_z1 - y_sys_out;

	const float z1_new = _eso_z1 + dt * (_eso_z2 - _eso_beta01 * e1);
	const float z2_new = _eso_z2 + dt * (_eso_z3 - _eso_beta02 * fal(e1, _eso_alpha1, _eso_delta)
					     + _b0 * control_u);
	const float z3_new = _eso_z3 + dt * (-_eso_beta03) * fal(e1, _eso_alpha2, _eso_delta);

	if (PX4_ISFINITE(z1_new) && PX4_ISFINITE(z2_new) && PX4_ISFINITE(z3_new)) {
		_eso_z1 = z1_new;
		_eso_z2 = z2_new;
		_eso_z3 = z3_new;
	}
}

float DepthADRCController::update(float depth_setpoint, float depth_meas, float dt)
{
	if (!PX4_ISFINITE(depth_meas) || !PX4_ISFINITE(depth_setpoint)) {
		_u_raw = 0.0f;
		_u_limited = 0.0f;
		return 0.0f;
	}

	const float dt_used = math::constrain(dt, 0.0005f, 0.05f);

	if (!_initialized) {
		reset(depth_meas);
	}

	// 1. TD
	updateTD(depth_setpoint, dt_used);

	// 2. NLSEF
	_e1 = _td_x1 - _eso_z1;
	_e2 = _td_x2 - _eso_z2;

	const float u0 = _nlsef_beta1 * fal(_e1, _nlsef_alpha1, _nlsef_delta)
		       + _nlsef_beta2 * fal(_e2, _nlsef_alpha2, _nlsef_delta);

	_u_raw = u0 - _eso_z3 / _b0;

	if (!PX4_ISFINITE(_u_raw)) {
		_u_raw = 0.0f;
	}

	_u_limited = math::constrain(_u_raw, _u_min, _u_max);

	// 3. KF
	// 注意：这里用限幅后的控制量参与 KF/ESO，因为真机实际执行的是限幅后的推力
	updateKF(depth_meas, _u_limited, dt_used);

	// 4. ESO
	updateESO(_kf_x1, _u_limited, dt_used);

	return _u_limited;
}
