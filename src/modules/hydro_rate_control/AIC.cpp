#include "AIC.h"
#include <cmath>

ADRC_AIC::ADRC_AIC(float Kp, float Kd, float b0, float wo)
{
    setGains(Kp, Kd, b0, wo);
    reset();
}

void ADRC_AIC::setGains(float Kp, float Kd, float b0, float wo)
{
    _Kp = Kp;
    _Kd = Kd;
    _b0 = math::max(b0, 0.001f); // 防止除零异常
    _wo = wo;
    updateObserverGains();
}

void ADRC_AIC::updateObserverGains()
{
    // 带宽参数化配置 LESO 极点
    _l1 = 3.0f * _wo;
    _l2 = 3.0f * _wo * _wo;
    _l3 = _wo * _wo * _wo;
}

void ADRC_AIC::setAIC(float k_f, float ku, float Ks_max)
{
    _k_f = k_f;
    _ku = ku;
    _Ks_max = Ks_max;
}

void ADRC_AIC::setLimits(float u_min, float u_max)
{
    _u_min = u_min;
    _u_max = u_max;
}

void ADRC_AIC::reset()
{
    _z1 = 0.0f;
    _z2 = 0.0f;
    _z3 = 0.0f;
    _z_lpf = 0.0f;
    _u_prev = 0.0f;
    _u = 0.0f;
}

void ADRC_AIC::updateLESO(float y, float u, float dt)
{
    float e = y - _z1;

    // 前向欧拉法离散化三阶 LESO
    float z1_dot = _z2 + _l1 * e;
    float z2_dot = _z3 + _b0 * u + _l2 * e;
    float z3_dot = _l3 * e;

    _z1 += dt * z1_dot;
    _z2 += dt * z2_dot;
    _z3 += dt * z3_dot;
}

float ADRC_AIC::aicCompensation(float z3, float dt)
{
    // 如果未启用 AIC 或参数异常，则不进行补偿
    if (_ku <= 0.0f || _k_f <= 0.0f) {
        return 0.0f;
    }

    // 后向欧拉法离散化低通滤波器
    float z_lpf_new = (_z_lpf + _k_f * dt * z3) / (1.0f + _k_f * dt);

    // 提取高频扰动特征
    float y_diff = _k_f * (z3 - z_lpf_new);

    // 计算自适应增益并进行限幅
    float Ks = _ku * std::fabs(z3);
    Ks = math::min(Ks, _Ks_max);

    // 计算前馈补偿量
    float T_comp = -Ks * y_diff;

    // 更新状态
    _z_lpf = z_lpf_new;

    return T_comp;
}

void ADRC_AIC::update(float y, float ref, float ref_dot, float dt)
{
    // 限制时间步长范围，防止启动瞬间或系统阻塞导致的 dt 异常（积分爆炸）
    const float tau = math::constrain(dt, 0.0005f, 0.02f);

    // 1. 更新扩张状态观测器 (使用上一时刻的受限控制量)
    updateLESO(y, _u_prev, tau);

    // 2. 计算 LADRC 跟踪误差
    float e = ref - _z1;
    float edot = ref_dot - _z2;

    // 3. PD 反馈与总扰动前馈补偿
    float u0 = _Kp * e + _Kd * edot;
    float u_ladrc = (u0 - _z3) / _b0;

    // 4. 主动干扰补偿 (AIC)
    float T_comp = aicCompensation(_z3, tau);

    // 5. 组合总控制量
    _u = u_ladrc + T_comp;

    // 6. 执行器限幅抗饱和
    _u = math::constrain(_u, _u_min, _u_max);

    // 7. 安全性检查：如果输出为 NaN 或 Inf，立刻重置并安全输出 0
    if (!PX4_ISFINITE(_u) || !PX4_ISFINITE(_z1)) {
        reset();
        _u = 0.0f;
    }

    // 8. 缓存当前实际输出的控制量，供下一周期 LESO 使用
    _u_prev = _u;
}
