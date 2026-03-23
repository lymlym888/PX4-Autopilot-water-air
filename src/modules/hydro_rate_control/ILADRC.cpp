#include "ILADRC.h"
#include <math.h>
#include <mathlib/mathlib.h>

ILADRCController::ILADRCController(float dt, float b0, float wc, float wo, float wcut)
    : _dt(dt), _b0(b0), _wc(wc), _wo(wo), _wcut(wcut),
      _x1_hat(0.0f), _x2_hat(0.0f), _x3_hat(0.0f),
      _Uc(0.0f), _Wx_sp(0.0f), _last_u(0.0f), _compensation(0.0f)
{
    // 初始化数组
    for (int i = 0; i < FILTER_ORDER; ++i) {
        _b_filter[i] = 0.0f;
        _a_filter[i] = 0.0f;
        _filter_x[i] = 0.0f;
        _filter_y[i] = 0.0f;
    }
    for (int i = 0; i < E1_HISTORY_SIZE; ++i) {
        _e1_history[i] = 0.0f;
    }
    updateGains();
    designThirdOrderFilter(_wcut, _dt);
    reset();
}

ILADRCController::~ILADRCController() {}

void ILADRCController::reset()
{
    _x1_hat = 0.0f;
    _x2_hat = 0.0f;
    _x3_hat = 0.0f;
    _Uc = 0.0f;
    _Wx_sp = 0.0f;
    _last_u = 0.0f;
    _compensation = 0.0f;

    for (int i = 0; i < FILTER_ORDER; ++i) {
        _filter_x[i] = 0.0f;
        _filter_y[i] = 0.0f;
    }
    for (int i = 0; i < E1_HISTORY_SIZE; ++i) {
        _e1_history[i] = 0.0f;
    }
}

void ILADRCController::updateGains()
{
    _l1 = 3.0f * _wo;
    _l2 = 3.0f * _wo * _wo;
    _l3 = _wo * _wo * _wo;
    _kp = _wc * _wc;
    _kd = 2.0f * _wc;
}

void ILADRCController::setGains(float wc, float wo)
{
    _wc = wc;
    _wo = wo;
    updateGains();
}

void ILADRCController::setB0(float b0)
{
    _b0 = math::max(b0, 0.001f);
}

void ILADRCController::setCutoff(float wcut)
{
    _wcut = wcut;
    designThirdOrderFilter(_wcut, _dt);
}

void ILADRCController::designThirdOrderFilter(float wcut, float T)
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

float ILADRCController::applyThirdOrderFilter(float x)
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

float ILADRCController::computeGeuCompensation(float e1, float dt)
{
    // 更新误差历史
    for (int i = E1_HISTORY_SIZE - 1; i > 0; --i) _e1_history[i] = _e1_history[i-1];
    _e1_history[0] = e1;

    float e1_hat = 0.0f;
    float e1_dot = (_e1_history[0] - _e1_history[1]) / dt;
    float e1_ddot = (_e1_history[0] - 2.0f * _e1_history[1] + _e1_history[2]) / (dt * dt);
    e1_hat = e1_ddot + _kd * e1_dot + _kp * _e1_history[0];

    return applyThirdOrderFilter(e1_hat);
}

void ILADRCController::update(float P_current, float P_target, float Wx_meas, float dt)
{
    float tau = math::constrain(dt, 0.0005f, 0.02f);

    float e = P_current - _x1_hat;
    e = math::constrain(e, -1e6f, 1e6f);

    _x1_hat += tau * (_x2_hat + _l1 * e);
    _x2_hat += tau * (_b0 * _last_u + _x3_hat + _l2 * e);
    _x3_hat += tau * (_l3 * e);

    _x1_hat = math::constrain(_x1_hat, -1e6f, 1e6f);
    _x2_hat = math::constrain(_x2_hat, -1e6f, 1e6f);
    _x3_hat = math::constrain(_x3_hat, -50.0f, 50.0f);

    float e1 = P_target - _x1_hat;
    float e2 = 0.0f - _x2_hat;
    e1 = math::constrain(e1, -1e6f, 1e6f);
    e2 = math::constrain(e2, -1e6f, 1e6f);

    _compensation = computeGeuCompensation(e1, tau);
    _Uc = (_kp * e1 + _kd * e2 - _x3_hat + _compensation) / _b0;
    _Wx_sp = _kp * e1 + _kd * e2 + _compensation;

    _last_u = _Uc;

    if (!PX4_ISFINITE(_Uc) || !PX4_ISFINITE(_Wx_sp)) {
        reset();
        _Uc = 0.0f;
        _Wx_sp = 0.0f;
    }
}
