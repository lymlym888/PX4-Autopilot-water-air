#include "MFESO.h"
#include <cmath>
#include <mathlib/mathlib.h>

using matrix::Matrix;

MFESOController::MFESOController()
{
    initGains();
    reset();
}

void MFESOController::reset()
{
    _Zs.setZero();
    _Zf.setZero();
    _pid_integral = 0.f;
    _Uc = 0.f;
}

void MFESOController::setModelParams(float a0, float b0)
{
    _a0 = a0;
    _b0 = math::max(b0, 0.001f);
    _b0_inv = 1.0f / _b0;
}

void MFESOController::setPIDGains(float kp, float ki, float kd, float int_limit)
{
    _pid_kp = kp;
    _pid_ki = ki;
    _pid_kd = kd;
    _pid_int_limit = int_limit;
}

void MFESOController::setBandwidths(float wc_calm, float zeta_calm,
                                    float wc_rough, float zeta_rough,
                                    float wc_safe, float zeta_safe)
{
    _calm_wc = wc_calm;     _calm_zeta = zeta_calm;
    _rough_wc = wc_rough;   _rough_zeta = zeta_rough;
    _safe_wc = wc_safe;     _safe_zeta = zeta_safe;
}

void MFESOController::fillLMatrix(Matrix<float, 3, 2>& L, float bandwidth)
{
    // 注意：这里的系数(4.1, 5.3875 等)仅作为演示逼近。
    // 在上板子前，请务必用你在 MATLAB 脚本中打印出来的确切 C 数组覆盖对应矩阵！
    L.setZero();
    L(0, 0) = 4.1000f * bandwidth;
    L(1, 0) = 5.3875f * bandwidth * bandwidth;
    L(2, 0) = 2.3625f * bandwidth * bandwidth * bandwidth;
}

void MFESOController::initGains()
{
    fillLMatrix(_L_s_calm, 5.0f);   fillLMatrix(_L_f_calm, 12.0f);
    fillLMatrix(_L_s_rough, 8.5f);  fillLMatrix(_L_f_rough, 14.0f);
    fillLMatrix(_L_s_safe, 6.5f);   fillLMatrix(_L_f_safe, 12.0f);
    fillLMatrix(_L_s_fixed, 6.0f);  fillLMatrix(_L_f_fixed, 12.0f);

    fillLMatrix(_L_ff_single_calm, 6.0f);
    fillLMatrix(_L_ff_single_rough, 8.5f);
    fillLMatrix(_L_ff_single_safe, 8.0f);
}

void MFESOController::observerStepAngleRate(Matrix<float, 3, 1>& Z, float input_u, float y_angle, float y_rate,
                                            const Matrix<float, 3, 2>& L_gain, float dt)
{
    float err_angle = Z(0, 0) - y_angle;
    float err_rate  = Z(1, 0) - y_rate;

    float z_dot1 = Z(1, 0)           - L_gain(0, 0) * err_angle - L_gain(0, 1) * err_rate;
    float z_dot2 = Z(2, 0) + input_u - L_gain(1, 0) * err_angle - L_gain(1, 1) * err_rate;
    float z_dot3 = 0.0f              - L_gain(2, 0) * err_angle - L_gain(2, 1) * err_rate;

    Z(0, 0) += dt * z_dot1;
    Z(1, 0) += dt * z_dot2;
    Z(2, 0) += dt * z_dot3;
}

void MFESOController::observerStepAngleOnly(Matrix<float, 3, 1>& Z, float input_u, float y_angle, float wo, float dt)
{
    float err_angle = Z(0, 0) - y_angle;
    float beta1 = 3.0f * wo;
    float beta2 = 3.0f * wo * wo;
    float beta3 = wo * wo * wo;

    float z_dot1 = Z(1, 0)           - beta1 * err_angle;
    float z_dot2 = Z(2, 0) + input_u - beta2 * err_angle;
    float z_dot3 = 0.0f              - beta3 * err_angle;

    Z(0, 0) += dt * z_dot1;
    Z(1, 0) += dt * z_dot2;
    Z(2, 0) += dt * z_dot3;
}

void MFESOController::update(float P_current, float P_target, float Wx_meas, float dt,
                             MFESO_SeaState sea_state, float u_prev_applied, MFESO_ControlMode mode,
                             float ref_rate, float ref_accel)
{
    // PX4 标准安全保护：约束最小/最大 dt 避免积分飞车
    const float tau = math::constrain(dt, 0.0005f, 0.02f);

    float wc = _calm_wc;
    float zeta = _calm_zeta;

    switch (mode) {
        case MFESO_ControlMode::PROPOSED: {
            Matrix<float, 3, 2> *Ls = &_L_s_calm;
            Matrix<float, 3, 2> *Lf = &_L_f_calm;

            if (sea_state == MFESO_SeaState::ROUGH) {
                wc = _rough_wc; zeta = _rough_zeta;
                Ls = &_L_s_rough; Lf = &_L_f_rough;
            } else if (sea_state == MFESO_SeaState::SAFE) {
                wc = _safe_wc; zeta = _safe_zeta;
                Ls = &_L_s_safe; Lf = &_L_f_safe;
            }

            // 更新双 ESO
            float virtual_prev = _a0 + _b0 * u_prev_applied;
            observerStepAngleRate(_Zs, virtual_prev, P_current, Wx_meas, *Ls, tau);
            float dhat_s = _Zs(2, 0);

            observerStepAngleRate(_Zf, virtual_prev + dhat_s, P_current, Wx_meas, *Lf, tau);
            float dhat_f = _Zf(2, 0);

            // 控制律计算
            float rate_fb = (1.0f - _rate_fusion) * _Zs(1, 0) + _rate_fusion * Wx_meas;
            float kp = wc * wc;
            float kd = 2.0f * zeta * wc;
            float u0 = ref_accel + kp * (P_target - _Zs(0, 0)) + kd * (ref_rate - rate_fb);

            float dhat_total = dhat_s + _fast_weight * dhat_f;
            float u_virtual = u0 - dhat_total;
            _Uc = (u_virtual - _a0) * _b0_inv;
            break;
        }

        case MFESO_ControlMode::FF_SINGLE: {
            Matrix<float, 3, 2> *Lz = &_L_ff_single_calm;

            if (sea_state == MFESO_SeaState::ROUGH) {
                wc = _rough_wc; zeta = _rough_zeta; Lz = &_L_ff_single_rough;
            } else if (sea_state == MFESO_SeaState::SAFE) {
                wc = _safe_wc; zeta = _safe_zeta; Lz = &_L_ff_single_safe;
            }

            float virtual_prev = _a0 + _b0 * u_prev_applied;
            observerStepAngleRate(_Zs, virtual_prev, P_current, Wx_meas, *Lz, tau);

            float rate_fb = (1.0f - _rate_fusion) * _Zs(1, 0) + _rate_fusion * Wx_meas;
            float kp = wc * wc;
            float kd = 2.0f * zeta * wc;
            float u0 = ref_accel + kp * (P_target - _Zs(0, 0)) + kd * (ref_rate - rate_fb);

            _Uc = (u0 - _Zs(2, 0) - _a0) * _b0_inv;
            break;
        }

        case MFESO_ControlMode::DUAL_NO_FF: {
            Matrix<float, 3, 2> *Ls = &_L_s_calm;
            Matrix<float, 3, 2> *Lf = &_L_f_calm;

            if (sea_state == MFESO_SeaState::ROUGH) {
                wc = _rough_wc; zeta = _rough_zeta; Ls = &_L_s_rough; Lf = &_L_f_rough;
            } else if (sea_state == MFESO_SeaState::SAFE) {
                wc = _safe_wc; zeta = _safe_zeta; Ls = &_L_s_safe; Lf = &_L_f_safe;
            }

            float virtual_prev_no_ff = 0.0f + _b0 * u_prev_applied;
            observerStepAngleRate(_Zs, virtual_prev_no_ff, P_current, Wx_meas, *Ls, tau);
            float dhat_s = _Zs(2, 0);

            observerStepAngleRate(_Zf, virtual_prev_no_ff + dhat_s, P_current, Wx_meas, *Lf, tau);
            float dhat_f = _Zf(2, 0);

            float rate_fb = (1.0f - _rate_fusion) * _Zs(1, 0) + _rate_fusion * Wx_meas;
            float kp = wc * wc;
            float kd = 2.0f * zeta * wc;
            float u0 = ref_accel + kp * (P_target - _Zs(0, 0)) + kd * (ref_rate - rate_fb);

            float dhat_total = dhat_s + _fast_weight * dhat_f;
            _Uc = (u0 - dhat_total) * _b0_inv;
            break;
        }

        case MFESO_ControlMode::DUAL_FIXED: {
            wc = _fixed_wc;
            zeta = _fixed_zeta;

            float virtual_prev = _a0 + _b0 * u_prev_applied;
            observerStepAngleRate(_Zs, virtual_prev, P_current, Wx_meas, _L_s_fixed, tau);
            float dhat_s = _Zs(2, 0);

            observerStepAngleRate(_Zf, virtual_prev + dhat_s, P_current, Wx_meas, _L_f_fixed, tau);
            float dhat_f = _Zf(2, 0);

            float rate_fb = (1.0f - _rate_fusion) * _Zs(1, 0) + _rate_fusion * Wx_meas;
            float kp = wc * wc;
            float kd = 2.0f * zeta * wc;
            float u0 = ref_accel + kp * (P_target - _Zs(0, 0)) + kd * (ref_rate - rate_fb);

            float dhat_total = dhat_s + 1.0f * dhat_f;
            _Uc = (u0 - dhat_total - _a0) * _b0_inv;
            break;
        }

        case MFESO_ControlMode::SINGLE_ESO: {
            wc = _single_wc;
            zeta = _single_zeta;

            float input_u = _b0 * u_prev_applied;
            observerStepAngleOnly(_Zs, input_u, P_current, _single_wo, tau);

            float kp = wc * wc;
            float kd = 2.0f * zeta * wc;
            float u0 = ref_accel + kp * (P_target - _Zs(0, 0)) + kd * (ref_rate - _Zs(1, 0));
            _Uc = (u0 - _Zs(2, 0)) * _b0_inv;
            break;
        }

        case MFESO_ControlMode::PID: {
            float err = P_target - P_current;
            float err_rate = ref_rate - Wx_meas;

            _pid_integral += err * tau;

            // 积分抗饱和限幅 (Anti-windup)
            _pid_integral = math::constrain(_pid_integral, -_pid_int_limit, _pid_int_limit);

            _Uc = _pid_kp * err + _pid_ki * _pid_integral + _pid_kd * err_rate;
            break;
        }
    }

    // 异常浮点状态拦截，防止非法数据污染执行机构
    if (!PX4_ISFINITE(_Uc) || !PX4_ISFINITE(_Zs(0,0)) || !PX4_ISFINITE(_Zf(0,0))) {
        reset();
        _Uc = 0.f;
    }
}
