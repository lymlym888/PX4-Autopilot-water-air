#pragma once

#include <cmath>
#include <matrix/math.hpp>

class MESOController {
public:
    enum MESOType {
        MESO1 = 1,
        MESO2 = 2,
        MESO3 = 3,
        MESO4 = 4
    };

    MESOController(MESOType type, bool adaptive = true,
                   float k_phi1 = 0.0f, float k_phi2 = 10.1746f,
                   float Kp = 180.0f, float Kd = 18.0f,
                   float omega_min = 8.0f, float omega_max = 24.0f,
                   float delta_lim_deg = 30.0f,
                   float gamma = 0.28f, float band_mid = 0.10f, float lpf_c = 18.0f);

    virtual ~MESOController() = default;

    void reset();
    void update(float phi_current, float phi_ref, float dt, float measured_omega = 0.0f);

    float getControl() const { return _delta_a; }
    float getEstimatedPhi() const { return _z1; }
    float getEstimatedOmega() const { return _z2; }
    float getEstimatedDisturbance() const { return _z3; }
    float getBandwidth() const { return _omega; }

    void setAdaptive(bool adaptive) { _adaptive = adaptive; }
    void setOmegaFixed(float omega_fixed) { _omega_fixed = omega_fixed; }

protected:
    // 由派生类实现具体的MESO观测器步进
    virtual void mesoStep(float y, float u_prev, float z1, float z2, float z3, float z4,
                          float omega, float tau, float& z1n, float& z2n, float& z3n, float& z4n) = 0;

    void updateAdaptiveBandwidth(float dt, float z3, float p_est, float phi_ref_dot);

    // 成员变量声明顺序与构造函数初始化列表一致，避免 -Wreorder
    MESOType _type;
    bool _adaptive;
    float _omega_fixed;           // 固定带宽值（当 adaptive=false 时使用）
    float _delta_lim_rad;         // 舵面限制（弧度）

    float _k_phi1;                // 被控对象参数
    float _k_phi2;                // 被控对象参数

    float _Kp;                    // 比例增益
    float _Kd;                    // 微分增益

    // 自适应带宽参数
    float _gamma;
    float _band_mid;
    float _lpf_c;
    float _omega_min;
    float _omega_max;

    // 状态变量
    float _z1{0.0f};              // 角度估计
    float _z2{0.0f};              // 角速度估计
    float _z3{0.0f};              // 总扰动估计
    float _z4{0.0f};              // 辅助状态（MESO3/4 使用）
    float _omega{0.0f};           // 当前带宽
    float _adapt_metric{0.0f};    // 自适应度量
    float _delta_a{0.0f};         // 控制输出

    // 辅助变量
    float _phi_ref_dot{0.0f};
    float _phi_ref_ddot{0.0f};
    float _p_est{0.0f};
    float _prev_u{0.0f};

    float _adapt_metric_prev{0.0f};
};

// ---------- 派生类 ----------
class MESO1Controller : public MESOController {
public:
    MESO1Controller(bool adaptive = true,
                    float k_phi1 = -0.8f, float k_phi2 = 10.1746f,
                    float Kp = 180.0f, float Kd = 18.0f,
                    float omega_min = 8.0f, float omega_max = 24.0f,
                    float delta_lim_deg = 30.0f,
                    float gamma = 0.28f, float band_mid = 0.10f, float lpf_c = 18.0f)
        : MESOController(MESO1, adaptive, k_phi1, k_phi2, Kp, Kd, omega_min, omega_max,
                         delta_lim_deg, gamma, band_mid, lpf_c) {}

protected:
    void mesoStep(float y, float u_prev, float z1, float z2, float z3, float z4,
                  float omega, float tau, float& z1n, float& z2n, float& z3n, float& z4n) override;
};

class MESO2Controller : public MESOController {
public:
    MESO2Controller(bool adaptive = true,
                    float k_phi1 = -0.8f, float k_phi2 = 10.1746f,
                    float Kp = 145.0f, float Kd = 20.0f,
                    float omega_min = 8.0f, float omega_max = 24.0f,
                    float delta_lim_deg = 30.0f,
                    float gamma = 0.28f, float band_mid = 0.10f, float lpf_c = 18.0f)
        : MESOController(MESO2, adaptive, k_phi1, k_phi2, Kp, Kd, omega_min, omega_max,
                         delta_lim_deg, gamma, band_mid, lpf_c) {}

protected:
    void mesoStep(float y, float u_prev, float z1, float z2, float z3, float z4,
                  float omega, float tau, float& z1n, float& z2n, float& z3n, float& z4n) override;
};

class MESO3Controller : public MESOController {
public:
    MESO3Controller(bool adaptive = true,
                    float k_phi1 = -1.5f, float k_phi2 = 10.1746f,
                    float Kp = 1200.0f, float Kd = 0.0f,
                    float omega_min = 8.0f, float omega_max = 24.0f,
                    float delta_lim_deg = 30.0f,
                    float gamma = 0.28f, float band_mid = 0.10f, float lpf_c = 18.0f)
        : MESOController(MESO3, adaptive, k_phi1, k_phi2, Kp, Kd, omega_min, omega_max,
                         delta_lim_deg, gamma, band_mid, lpf_c) {}

protected:
    void mesoStep(float y, float u_prev, float z1, float z2, float z3, float z4,
                  float omega, float tau, float& z1n, float& z2n, float& z3n, float& z4n) override;
};

class MESO4Controller : public MESOController {
public:
    MESO4Controller(bool adaptive = true,
                    float k_phi1 = -1.2f, float k_phi2 = 10.1746f,
                    float Kp = 500.0f, float Kd = 0.0f,
                    float omega_min = 8.0f, float omega_max = 24.0f,
                    float delta_lim_deg = 30.0f,
                    float gamma = 0.28f, float band_mid = 0.10f, float lpf_c = 18.0f)
        : MESOController(MESO4, adaptive, k_phi1, k_phi2, Kp, Kd, omega_min, omega_max,
                         delta_lim_deg, gamma, band_mid, lpf_c) {}

protected:
    void mesoStep(float y, float u_prev, float z1, float z2, float z3, float z4,
                  float omega, float tau, float& z1n, float& z2n, float& z3n, float& z4n) override;
};
