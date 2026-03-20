#pragma once

#include <matrix/math.hpp>

class LADRCController {
public:
    LADRCController(float dt = 0.001f, float b0 = 10.1746f,
                    float wc = 40.0f, float wo = 120.0f);

    void reset();

    // 输入：当前角度 P_current (rad)，目标角度 P_target (rad)，当前角速度 Wx_meas (rad/s)，采样周期 dt (s)
    void update(float P_current, float P_target, float Wx_meas, float dt);

    float getUc() const { return _Uc; }        // 控制输出（未归一化）
    float getWx_sp() const { return _Wx_sp; }  // 期望角速度

    void setGains(float wc, float wo);
    void setB0(float b0);
    void setDt(float dt) { _dt = dt; }

    float getX1_hat() const { return _x1_hat; }
    float getX2_hat() const { return _x2_hat; }
    float getX3_hat() const { return _x3_hat; }

private:
    void updateGains();

    float _dt;          // 采样时间
    float _b0;          // 系统增益
    float _wc;          // 控制器带宽
    float _wo;          // 观测器带宽

    float _l1, _l2, _l3; // 观测器增益
    float _kp, _kd;      // 控制器增益

    float _x1_hat;      // 位置估计
    float _x2_hat;      // 速度估计
    float _x3_hat;      // 扰动估计

    float _Uc;          // 控制输出
    float _Wx_sp;       // 期望角速度

    float _last_u;      // 上一时刻控制输入
};
