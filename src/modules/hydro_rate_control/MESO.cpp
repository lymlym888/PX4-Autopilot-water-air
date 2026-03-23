#include "MESO.h"
#include <mathlib/mathlib.h>

// 基类构造
MESOController::MESOController(MESOType type, bool adaptive,
                               float k_phi1, float k_phi2,
                               float Kp, float Kd,
                               float omega_min, float omega_max,
                               float delta_lim_deg,
                               float gamma, float band_mid, float lpf_c)
    : _type(type), _adaptive(adaptive),
      _k_phi1(k_phi1), _k_phi2(k_phi2),
      _Kp(Kp), _Kd(Kd),
      _gamma(gamma), _band_mid(band_mid), _lpf_c(lpf_c),
      _omega_min(omega_min), _omega_max(omega_max),
      _delta_lim_rad(delta_lim_deg * M_PI / 180.0f)
{
    reset();
}

void MESOController::reset()
{
    _z1 = 0.0f;
    _z2 = 0.0f;
    _z3 = 0.0f;
    _z4 = 0.0f;
    _omega = _adaptive ? _omega_min : _omega_fixed;
    _adapt_metric = 0.0f;
    _delta_a = 0.0f;
    _phi_ref_dot = 0.0f;
    _phi_ref_ddot = 0.0f;
    _p_est = 0.0f;
    _prev_u = 0.0f;
    _adapt_metric_prev = 0.0f;
}

void MESOController::updateAdaptiveBandwidth(float dt, float z3, float p_est, float phi_ref_dot)
{
    if (!_adaptive) return;

    float obs_signal = fabsf(z3) + 0.5f * fabsf(p_est) + fabsf(phi_ref_dot);
    float adapt_metric = _adapt_metric_prev + dt * _lpf_c * (obs_signal - _adapt_metric_prev);
    _adapt_metric = adapt_metric;
    _adapt_metric_prev = adapt_metric;

    float s = 1.0f / (1.0f + expf(-_gamma * (adapt_metric - _band_mid) * 25.0f));
    _omega = _omega_min + (_omega_max - _omega_min) * s;
}

void MESOController::update(float phi_current, float phi_ref, float dt, float measured_omega)
{
    float tau = math::constrain(dt, 0.0005f, 0.02f);

    // 计算参考信号导数（简单差分，实际可使用外部提供的值）
    static float phi_ref_prev = phi_ref;
    _phi_ref_dot = (phi_ref - phi_ref_prev) / tau;
    _phi_ref_ddot = 0.0f; // 可根据需要实现
    phi_ref_prev = phi_ref;

    // 带宽选择
    if (!_adaptive) {
        _omega = _omega_fixed;
    }

    // 观测器更新
    float z1n, z2n, z3n, z4n;
    mesoStep(phi_current, _prev_u, _z1, _z2, _z3, _z4, _omega, tau, z1n, z2n, z3n, z4n);
    _z1 = z1n;
    _z2 = z2n;
    _z3 = z3n;
    _z4 = z4n;

    _p_est = _z2; // 可使用 measured_omega 替代

    updateAdaptiveBandwidth(tau, _z3, _p_est, _phi_ref_dot);

    // 控制律
    float e = phi_ref - phi_current;
    float ed = _phi_ref_dot - _z2;
    float d_hat = _z3;

    float gain_scale = 1.0f;
    float ddot_ff = 0.0f;
    switch (_type) {
    case MESO1:
        gain_scale = 1.00f; ddot_ff = _phi_ref_ddot;
        break;
    case MESO2:
        gain_scale = 1.05f; ddot_ff = _phi_ref_ddot;
        break;
    case MESO3:
        gain_scale = 1.00f; ddot_ff = 0.0f;
        break;
    case MESO4:
        gain_scale = 1.00f; ddot_ff = 0.0f;
        break;
    }

    float u = (gain_scale * _Kp * e + _Kd * ed + ddot_ff - d_hat) / _k_phi2;
    u = math::constrain(u, -_delta_lim_rad, _delta_lim_rad);

    _delta_a = u;
    _prev_u = u;
}

// ---------- MESO1 ----------
void MESO1Controller::mesoStep(float y, float u_prev, float z1, float z2, float z3, float z4,
                               float omega, float tau, float& z1n, float& z2n, float& z3n, float& z4n)
{
    float a = -_k_phi1;
    float beta1 = 3.0f * omega - a;
    float beta2 = 3.0f * omega * omega - a * beta1;
    float beta3 = omega * omega * omega;

    float ey = y - z1;
    float z1_dot = z2 + beta1 * ey;
    float z2_dot = -a * z2 + z3 + _k_phi2 * u_prev + beta2 * ey;
    float z3_dot = beta3 * ey;
    float z4_dot = 0.0f;

    z1n = z1 + tau * z1_dot;
    z2n = z2 + tau * z2_dot;
    z3n = z3 + tau * z3_dot;
    z4n = z4 + tau * z4_dot;
}

// ---------- MESO2 ----------
void MESO2Controller::mesoStep(float y, float u_prev, float z1, float z2, float z3, float z4,
                               float omega, float tau, float& z1n, float& z2n, float& z3n, float& z4n)
{
    float a = -_k_phi1;
    float beta1 = 3.0f * omega - a;
    float beta2 = 3.0f * omega * omega - a * beta1;
    float beta3 = omega * omega * omega - a * beta2;

    float ey = y - z1;
    float z1_dot = z2 + beta1 * ey;
    float z2_dot = z3 + _k_phi2 * u_prev + beta2 * ey;
    float z3_dot = -a * z3 - a * _k_phi2 * u_prev + beta3 * ey;
    float z4_dot = 0.0f;

    z1n = z1 + tau * z1_dot;
    z2n = z2 + tau * z2_dot;
    z3n = z3 + tau * z3_dot;
    z4n = z4 + tau * z4_dot;
}

// ---------- MESO3 ----------
void MESO3Controller::mesoStep(float y, float u_prev, float z1, float z2, float z3, float z4,
                               float omega, float tau, float& z1n, float& z2n, float& z3n, float& z4n)
{
    float a = -_k_phi1;
    float beta1 = 2.0f * omega - a;
    float beta2 = omega * omega;

    float hatz2 = z1;
    float hatz3 = z4;

    float hatz2_alg = z2 - beta1 * y;
    float hatz3_alg = z3 - beta2 * y;

    float term1 = (-a * beta1 - beta1 * beta1 + beta2) * y;
    float hatz2_dot = term1 - (a + beta1) * hatz2_alg + hatz3_alg + _k_phi2 * u_prev;
    float hatz3_dot = -beta2 * hatz2_alg - beta1 * beta2 * y;

    float z2_est = hatz2_alg + beta1 * y;
    float z3_est = hatz3_alg + beta2 * y;

    float hatz2_n = hatz2 + tau * hatz2_dot;
    float hatz3_n = hatz3 + tau * hatz3_dot;

    z1n = hatz2_n;
    z2n = z2_est;
    z3n = z3_est;
    z4n = hatz3_n;
}

// ---------- MESO4 ----------
void MESO4Controller::mesoStep(float y, float u_prev, float z1, float z2, float z3, float z4,
                               float omega, float tau, float& z1n, float& z2n, float& z3n, float& z4n)
{
    float a = -_k_phi1;
    float beta1 = 2.0f * omega;
    float beta2 = omega * omega - a * beta1;

    float hatz2 = z1;
    float hatz3 = z4;

    float hatz2_alg = z2 - beta1 * y;
    float hatz3_alg = z3 - beta2 * y;

    float hatz2_dot = (beta2 - beta1 * beta1) * y - beta1 * hatz2_alg + hatz3_alg + _k_phi2 * u_prev;
    float hatz3_dot = (-a * beta2 - beta1 * beta2) * y - beta2 * hatz2_alg - a * hatz3_alg - a * _k_phi2 * u_prev;

    float z2_est = hatz2_alg + beta1 * y;
    float z3_est = hatz3_alg + beta2 * y;

    float hatz2_n = hatz2 + tau * hatz2_dot;
    float hatz3_n = hatz3 + tau * hatz3_dot;

    z1n = hatz2_n;
    z2n = z2_est;
    z3n = z3_est;
    z4n = hatz3_n;
}
