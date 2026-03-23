#include "LADRC.h"
#include <math.h>
#include <mathlib/mathlib.h>

LADRCController::LADRCController(float dt, float b0, float wc, float wo)
    : _dt(dt), _b0(b0), _wc(wc), _wo(wo),
      _x1_hat(0.0f), _x2_hat(0.0f), _x3_hat(0.0f),
      _Uc(0.0f), _Wx_sp(0.0f), _last_u(0.0f)
{
    updateGains();
    reset();
}

void LADRCController::reset()
{
    _x1_hat = 0.0f;
    _x2_hat = 0.0f;
    _x3_hat = 0.0f;
    _Uc = 0.0f;
    _Wx_sp = 0.0f;
    _last_u = 0.0f;
}

void LADRCController::updateGains()
{
    _l1 = 3.0f * _wo;
    _l2 = 3.0f * _wo * _wo;
    _l3 = _wo * _wo * _wo;
    _kp = _wc * _wc;
    _kd = 2.0f * _wc;
}

void LADRCController::setGains(float wc, float wo)
{
    _wc = wc;
    _wo = wo;
    updateGains();
}

void LADRCController::setB0(float b0)
{
    _b0 = math::max(b0, 0.001f);
}

void LADRCController::update(float P_current, float P_target, float Wx_meas, float dt)
{
    float tau = math::constrain(dt, 0.0005f, 0.02f);

    // 观测误差
    float e = P_current - _x1_hat;
    e = math::constrain(e, -1e6f, 1e6f);

    // 扩张状态观测器（欧拉法）
    _x1_hat += tau * (_x2_hat + _l1 * e);
    _x2_hat += tau * (_b0 * _last_u + _x3_hat + _l2 * e);
    _x3_hat += tau * (_l3 * e);

    // 状态限幅
    _x1_hat = math::constrain(_x1_hat, -1e6f, 1e6f);
    _x2_hat = math::constrain(_x2_hat, -1e6f, 1e6f);
    _x3_hat = math::constrain(_x3_hat, -50.0f, 50.0f);

    // 状态误差（期望速度为零）
    float e1 = P_target - _x1_hat;
    float e2 = 0.0f - _x2_hat;
    e1 = math::constrain(e1, -1e6f, 1e6f);
    e2 = math::constrain(e2, -1e6f, 1e6f);

    // 控制律
    _Uc = (_kp * e1 + _kd * e2 - _x3_hat) / _b0;

    // 期望角速度（用于外环监控）
    _Wx_sp = _kp * e1 + _kd * e2;

    // 更新上一时刻控制量
    _last_u = _Uc;

    // 防止数值异常
    if (!PX4_ISFINITE(_Uc) || !PX4_ISFINITE(_Wx_sp)) {
        reset();
        _Uc = 0.0f;
        _Wx_sp = 0.0f;
    }
}
