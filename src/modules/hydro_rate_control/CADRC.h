// CADRC.h
#pragma once

#include <matrix/math.hpp>

class ADRCController {
public:
    ADRCController(float J0 = 10.1746f,
                   float kp1 = 500.0f,
                   float omega_o1 = 15.0f,
                   float kp2 = 1000.0f,
                   float omega_o2 = 15.0f,
                   float alpha = 0.0f);

    void reset();

    // 输入：角度P( rad )、目标角度P_sp( rad )、角速度Wx( rad/s )、dt( s )
    void update(float P_current, float P_target, float Wx_meas, float dt);

    float getUc() const { return _Uc; }        // 控制输出（未归一化）
    float getRateSp() const { return _Wx_p; }  // 期望角速度

    void setJ0(float J0);
    void setGains(float kp1, float omega_o1, float kp2, float omega_o2);
    void setAlpha(float alpha) { _alpha = alpha; }

    matrix::Matrix<float, 2, 1> _Z1{};
    matrix::Matrix<float, 2, 1> _Z2{};

private:
    void updateBetaGains();
    void initWeightMatrix();

    // 参数
    float _J0{10.1746f};
    float _Jc_inv{1.0f / 10.1746f};

    float _kp1{500.f};
    float _omega_o1{15.f};
    float _beta11{0.f}, _beta12{0.f};

    float _kp2{1000.f};
    float _omega_o2{15.f};
    float _beta21{0.f}, _beta22{0.f};

    float _alpha{0.f};

    static constexpr int32_t m = 4;

    // 状态
    int32_t _nn{0};

    matrix::Matrix<float, 2, 1> _Lc1{};
    matrix::Matrix<float, 2, 1> _Lc2{};


    matrix::Matrix<float, 1, m> _Wx_vec{};
    matrix::Matrix<float, m, m> _Q1{};
    matrix::Matrix<float, 1, m> _DeltaXe1_vec{};
    matrix::Matrix<float, m, 1> _Gamma1{};

    matrix::Matrix<float, m, m> _A{};
    matrix::Matrix<float, m, m> _A_inv{};

    float _Xe1{0.f};
    float _Xe2{0.f};

    float _Wx_meas{0.f};
    float _Wx_p{0.f};
    float _Uc{0.f};

    float _Z1_dot_prev[2]{0.f, 0.f};
};
