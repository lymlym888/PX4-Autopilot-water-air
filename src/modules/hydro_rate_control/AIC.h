#pragma once

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

class ADRC_AIC {
public:
    // 构造函数：初始化默认参数
    ADRC_AIC(float Kp = 0.0f, float Kd = 0.0f, float b0 = 1.0f, float wo = 20.0f);

    // 设置基础控制增益
    void setGains(float Kp, float Kd, float b0, float wo);

    // 设置 AIC (主动干扰补偿) 参数
    void setAIC(float k_f, float ku, float Ks_max);

    // 设置控制输出限幅
    void setLimits(float u_min, float u_max);

    // 核心控制更新逻辑：输入当前测量值 y、目标值 ref、目标微分 ref_dot、时间步长 dt(s)
    void update(float y, float ref, float ref_dot, float dt);

    // 获取总控制输出 u
    float getControlOutput() const { return _u; }

    // 获取总扰动估计值 z3
    float getZ3() const { return _z3; }

    // 重置控制器状态
    void reset();

private:
    // 内部运算函数
    void updateLESO(float y, float u, float dt);
    float aicCompensation(float z3, float dt);
    void updateObserverGains();

    // 基础控制参数
    float _Kp{0.0f};           // 比例增益
    float _Kd{0.0f};           // 微分增益
    float _b0{1.0f};           // 控制增益 (最大归一化扭矩产生的角加速度)
    float _wo{20.0f};          // 观测器带宽

    // LESO 增益
    float _l1{0.0f};
    float _l2{0.0f};
    float _l3{0.0f};

    // AIC 参数
    float _k_f{0.0f};          // 低通滤波器截止频率 (rad/s)
    float _ku{0.0f};           // 自适应增益系数
    float _Ks_max{1.0f};       // 最大补偿增益

    // 观测器和控制器状态
    float _z1{0.0f};           // 状态估计: 跟踪信号
    float _z2{0.0f};           // 状态估计: 信号微分
    float _z3{0.0f};           // 状态估计: 总扰动
    float _z_lpf{0.0f};        // AIC 低通滤波器状态

    float _u_prev{0.0f};       // 上一时刻的控制输出 (用于 LESO 更新)
    float _u{0.0f};            // 当前时刻的总控制输出

    // 输出限幅
    float _u_min{-1e6f};       // 默认下限
    float _u_max{1e6f};        // 默认上限
};
