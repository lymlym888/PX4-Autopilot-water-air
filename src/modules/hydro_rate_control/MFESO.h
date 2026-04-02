#pragma once

#include <matrix/math.hpp>

// ---------- 定义控制算法模式与海况 ----------
enum class MFESO_ControlMode {
    PROPOSED = 0,   // MF + Dual-ESO ADRC (主推)
    FF_SINGLE,      // 带模型前馈的单 ESO ADRC
    DUAL_NO_FF,     // 双 ESO (无前馈)
    DUAL_FIXED,     // 双 ESO (固定参数)
    SINGLE_ESO,     // 传统单 ESO
    PID             // 普通 PID
};

enum class MFESO_SeaState {
    CALM = 0,
    ROUGH = 1,
    SAFE = 2
};

class MFESOController {
public:
    MFESOController();

    // 重置状态与积分器
    void reset();

    // 核心控制步进函数
    // 输入：当前角度、目标角度、当前角速度、dt、海况、上一时刻输出的指令、控制模式
    void update(float P_current, float P_target, float Wx_meas, float dt,
                MFESO_SeaState sea_state, float u_prev_applied, MFESO_ControlMode mode,
                float ref_rate = 0.f, float ref_accel = 0.f);

    // 获取控制输出
    float getUc() const { return _Uc; }

    // 参数设置接口
    void setModelParams(float a0, float b0);
    void setPIDGains(float kp, float ki, float kd, float int_limit);
    void setBandwidths(float wc_calm, float zeta_calm,
                       float wc_rough, float zeta_rough,
                       float wc_safe, float zeta_safe);

    // 开放出来的状态变量（方便在 uORB 中记录 Log 进行绘图）
    matrix::Matrix<float, 3, 1> _Zs{}; // 慢速 ESO 状态 [角度, 速度, 主扰动]
    matrix::Matrix<float, 3, 1> _Zf{}; // 快速 ESO 状态 [角度, 速度, 残余扰动]

private:
    void initGains();
    void fillLMatrix(matrix::Matrix<float, 3, 2>& L, float bandwidth);

    // ESO 内部迭代步进 (Angle + Rate 双输入模式)
    void observerStepAngleRate(matrix::Matrix<float, 3, 1>& Z, float input_u, float y_angle, float y_rate,
                               const matrix::Matrix<float, 3, 2>& L_gain, float dt);

    // ESO 内部迭代步进 (仅 Angle 单输入模式，用于基线对比)
    void observerStepAngleOnly(matrix::Matrix<float, 3, 1>& Z, float input_u, float y_angle, float wo, float dt);

    // ---------- 控制参数 ----------
    float _a0{0.22f};
    float _b0{10.1746f};
    float _b0_inv{1.0f / 10.1746f};

    float _rate_fusion{0.50f};
    float _fast_weight{0.40f};

    // 带宽参数
    float _calm_wc{4.8f},   _calm_zeta{0.95f};
    float _rough_wc{4.4f},  _rough_zeta{1.55f};
    float _safe_wc{2.8f},   _safe_zeta{1.25f};
    float _single_wc{3.6f}, _single_zeta{1.30f}, _single_wo{12.0f};
    float _fixed_wc{4.0f},  _fixed_zeta{1.20f};

    // PID 参数
    float _pid_kp{1.70f}, _pid_ki{0.45f}, _pid_kd{0.55f};
    float _pid_int_limit{0.209f}; // 约 12 度

    // ---------- 观测器增益矩阵缓存 ----------
    matrix::Matrix<float, 3, 2> _L_s_calm{};
    matrix::Matrix<float, 3, 2> _L_f_calm{};
    matrix::Matrix<float, 3, 2> _L_s_rough{};
    matrix::Matrix<float, 3, 2> _L_f_rough{};
    matrix::Matrix<float, 3, 2> _L_s_safe{};
    matrix::Matrix<float, 3, 2> _L_f_safe{};
    matrix::Matrix<float, 3, 2> _L_s_fixed{};
    matrix::Matrix<float, 3, 2> _L_f_fixed{};
    matrix::Matrix<float, 3, 2> _L_ff_single_calm{};
    matrix::Matrix<float, 3, 2> _L_ff_single_rough{};
    matrix::Matrix<float, 3, 2> _L_ff_single_safe{};

    // ---------- 运行状态 ----------
    float _Uc{0.f};
    float _pid_integral{0.f};
};
