#include "IALADRC.h"
#include <math.h>
#include <mathlib/mathlib.h>


IALADRCController::IALADRCController(float dt, float b0, float wc, float wo_min, float wo_max,
                                     float wcut, float eta, float wLcut, float e0)
    : _dt(dt), _b0(b0), _wc(wc), _wo_min(wo_min), _wo_max(wo_max),
      _wo(wo_min), _wcut(wcut), _eta(eta), _wLcut(wLcut), _e0(e0),
      _x1_hat(0.0f), _x2_hat(0.0f), _x3_hat(0.0f), _x3_hat_prev(0.0f),
      _Uc(0.0f), _Wx_sp(0.0f), _last_u(0.0f), _compensation(0.0f)
{
    // 初始化所有数组
    for (int i = 0; i < FILTER_ORDER; ++i) {
        _b_filter[i] = 0.0f;
        _a_filter[i] = 0.0f;
        _filter_x[i] = 0.0f;
        _filter_y[i] = 0.0f;
    }
    for (int i = 0; i < LPF_ORDER; ++i) {
        _b_lpf[i] = 0.0f;
        _a_lpf[i] = 0.0f;
        _lpf_x[i] = 0.0f;
        _lpf_y[i] = 0.0f;
    }
    for (int i = 0; i < E1_HISTORY_SIZE; ++i) {
        _e1_history[i] = 0.0f;
    }

    updateGains();
    designThirdOrderFilter(_wcut, _dt);
    designSecondOrderLPF(_wLcut, _dt);
    reset();
}

IALADRCController::~IALADRCController() {}

void IALADRCController::reset()
{
    _x1_hat = 0.0f;
    _x2_hat = 0.0f;
    _x3_hat = 0.0f;
    _x3_hat_prev = 0.0f;
    _wo = _wo_min;
    _Uc = 0.0f;
    _Wx_sp = 0.0f;
    _last_u = 0.0f;
    _compensation = 0.0f;

    for (int i = 0; i < FILTER_ORDER; ++i) {
        _filter_x[i] = 0.0f;
        _filter_y[i] = 0.0f;
    }
    for (int i = 0; i < LPF_ORDER; ++i) {
        _lpf_x[i] = 0.0f;
        _lpf_y[i] = 0.0f;
    }
    for (int i = 0; i < E1_HISTORY_SIZE; ++i) {
        _e1_history[i] = 0.0f;
    }

    updateGains();
}

void IALADRCController::updateGains()
{
    _l1 = 3.0f * _wo;
    _l2 = 3.0f * _wo * _wo;
    _l3 = _wo * _wo * _wo;
    _kp = _wc * _wc;
    _kd = 2.0f * _wc;
}

void IALADRCController::setGains(float wc, float wo_min, float wo_max)
{
    _wc = wc;
    _wo_min = wo_min;
    _wo_max = wo_max;
    if (_wo < _wo_min) _wo = _wo_min;
    if (_wo > _wo_max) _wo = _wo_max;
    updateGains();
}

void IALADRCController::setB0(float b0)
{
    _b0 = math::max(b0, 0.001f);
}

void IALADRCController::setCutoff(float wcut)
{
    _wcut = wcut;
    designThirdOrderFilter(_wcut, _dt);
}

void IALADRCController::setAdaptiveParams(float eta, float wLcut, float e0)
{
    _eta = eta;
    _wLcut = wLcut;
    _e0 = e0;
    designSecondOrderLPF(_wLcut, _dt);
}

void IALADRCController::designThirdOrderFilter(float wcut, float T)
{
    float wcut2 = wcut * wcut;
    float wcut3 = wcut2 * wcut;

    float fs = 2.0f / T;
    float fs2 = fs * fs;
    float fs3 = fs2 * fs;

    float den0 = fs3 + 3.0f * wcut * fs2 + 3.0f * wcut2 * fs + wcut3;
    float den1 = -3.0f * fs3 - 3.0f * wcut * fs2 + 3.0f * wcut2 * fs + 3.0f * wcut3;
    float den2 = 3.0f * fs3 - 3.0f * wcut * fs2 - 3.0f * wcut2 * fs + 3.0f * wcut3;
    float den3 = -fs3 + 3.0f * wcut * fs2 - 3.0f * wcut2 * fs + wcut3;

    float num0 = wcut3;
    float num1 = 3.0f * wcut3;
    float num2 = 3.0f * wcut3;
    float num3 = wcut3;

    if (fabsf(den0) < 1e-10f) {
        float alpha = wcut * T / (1.0f + wcut * T);
        _b_filter[0] = alpha;
        _b_filter[1] = 0.0f;
        _b_filter[2] = 0.0f;
        _b_filter[3] = 0.0f;
        _a_filter[0] = 1.0f;
        _a_filter[1] = -(1.0f - alpha);
        _a_filter[2] = 0.0f;
        _a_filter[3] = 0.0f;
        return;
    }

    float norm_factor = 1.0f / den0;
    _b_filter[0] = num0 * norm_factor;
    _b_filter[1] = num1 * norm_factor;
    _b_filter[2] = num2 * norm_factor;
    _b_filter[3] = num3 * norm_factor;
    _a_filter[0] = 1.0f;
    _a_filter[1] = den1 * norm_factor;
    _a_filter[2] = den2 * norm_factor;
    _a_filter[3] = den3 * norm_factor;

    // 重置滤波器状态
    for (int i = 0; i < FILTER_ORDER; ++i) {
        _filter_x[i] = 0.0f;
        _filter_y[i] = 0.0f;
    }
}

void IALADRCController::designSecondOrderLPF(float wLcut, float T)
{
    float alpha = wLcut * T / (1.0f + wLcut * T);
    _b_lpf[0] = alpha * alpha;
    _b_lpf[1] = 2.0f * alpha * (1.0f - alpha);
    _b_lpf[2] = (1.0f - alpha) * (1.0f - alpha);
    _a_lpf[0] = 1.0f;
    _a_lpf[1] = 0.0f;
    _a_lpf[2] = 0.0f;

    // 重置滤波器状态
    for (int i = 0; i < LPF_ORDER; ++i) {
        _lpf_x[i] = 0.0f;
        _lpf_y[i] = 0.0f;
    }
}

float IALADRCController::applyThirdOrderFilter(float x)
{
    // 更新输入历史
    for (int i = FILTER_ORDER - 1; i > 0; --i) _filter_x[i] = _filter_x[i-1];
    _filter_x[0] = x;

    // 计算输出
    float y = _b_filter[0] * _filter_x[0];
    y += _b_filter[1] * _filter_x[1];
    y += _b_filter[2] * _filter_x[2];
    y += _b_filter[3] * _filter_x[3];
    y -= _a_filter[1] * _filter_y[0];
    y -= _a_filter[2] * _filter_y[1];
    y -= _a_filter[3] * _filter_y[2];

    y = math::constrain(y, -1e6f, 1e6f);

    // 更新输出历史
    for (int i = FILTER_ORDER - 1; i > 0; --i) _filter_y[i] = _filter_y[i-1];
    _filter_y[0] = y;

    return y;
}

float IALADRCController::applySecondOrderLPF(float x)
{
    // 更新输入历史
    for (int i = LPF_ORDER - 1; i > 0; --i) _lpf_x[i] = _lpf_x[i-1];
    _lpf_x[0] = x;

    // 计算输出
    float y = _b_lpf[0] * _lpf_x[0];
    y += _b_lpf[1] * _lpf_x[1];
    y += _b_lpf[2] * _lpf_x[2];
    y -= _a_lpf[1] * _lpf_y[0];
    y -= _a_lpf[2] * _lpf_y[1];

    // 更新输出历史
    for (int i = LPF_ORDER - 1; i > 0; --i) _lpf_y[i] = _lpf_y[i-1];
    _lpf_y[0] = y;

    return y;
}

float IALADRCController::computeAdaptiveBandwidth(float d_hat, float d_hat_prev, float dt)
{
    float d_dot = (d_hat - d_hat_prev) / dt;
    float filtered_dot = applySecondOrderLPF(d_dot);

    float gamma = _eta * filtered_dot * 0.001f;
    float abs_gamma = fabsf(gamma);
    float exp_term = (abs_gamma > 50.0f) ? expf(50.0f) : expf(abs_gamma);
    float adaptive_factor = (exp_term - 1.0f) / (exp_term + 1.0f);

    float new_wo = _wo_min + (_wo_max - _wo_min) * adaptive_factor;
    new_wo = math::constrain(new_wo, _wo_min, _wo_max);
    return new_wo;
}

float IALADRCController::satFunction(float e) const
{
    float abs_e = fabsf(e);
    if (abs_e <= _e0) return 1.0f;
    else return 10.0f * _e0 / abs_e;
}

float IALADRCController::computeGeuCompensationWithSaturation(float e1, float dt)
{
    float sat_weight = satFunction(e1);

    // 更新误差历史
    for (int i = E1_HISTORY_SIZE - 1; i > 0; --i) _e1_history[i] = _e1_history[i-1];
    _e1_history[0] = e1;

    float e1_hat = 0.0f;
    float e1_dot = (_e1_history[0] - _e1_history[1]) / dt;
    float e1_ddot = (_e1_history[0] - 2.0f * _e1_history[1] + _e1_history[2]) / (dt * dt);
    e1_hat = e1_ddot + _kd * e1_dot + _kp * _e1_history[0];

    float filtered = applyThirdOrderFilter(e1_hat);
    return sat_weight * filtered;
}

void IALADRCController::update(float P_current, float P_target, float Wx_meas, float dt)
{
    float tau = math::constrain(dt, 0.0005f, 0.02f);

    // 保存上一时刻扰动估计
    _x3_hat_prev = _x3_hat;

    // 观测误差
    float e = P_current - _x1_hat;
    e = math::constrain(e, -1e6f, 1e6f);

    // 扩张状态观测器
    _x1_hat += tau * (_x2_hat + _l1 * e);
    _x2_hat += tau * (_b0 * _last_u + _x3_hat + _l2 * e);
    _x3_hat += tau * (_l3 * e);

    _x1_hat = math::constrain(_x1_hat, -1e6f, 1e6f);
    _x2_hat = math::constrain(_x2_hat, -1e6f, 1e6f);
    _x3_hat = math::constrain(_x3_hat, -1e6f, 1e6f);

    // 自适应带宽
    _wo = computeAdaptiveBandwidth(_x3_hat, _x3_hat_prev, tau);
    updateGains();

    // 状态误差
    float e1 = P_target - _x1_hat;
    float e2 = 0.0f - _x2_hat;
    e1 = math::constrain(e1, -1e6f, 1e6f);
    e2 = math::constrain(e2, -1e6f, 1e6f);

    // 带饱和的 Geu 补偿
    _compensation = computeGeuCompensationWithSaturation(e1, tau);

    // 控制律
    _Uc = (_kp * e1 + _kd * e2 - _x3_hat + _compensation) / _b0;

    // 期望角速度
    _Wx_sp = _kp * e1 + _kd * e2 + _compensation;

    _last_u = _Uc;

    if (!PX4_ISFINITE(_Uc) || !PX4_ISFINITE(_Wx_sp)) {
        reset();
        _Uc = 0.0f;
        _Wx_sp = 0.0f;
    }
}
