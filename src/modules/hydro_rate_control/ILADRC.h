#pragma once

#include <matrix/math.hpp>

class ILADRCController {
public:
    ILADRCController(float dt = 0.001f, float b0 = 10.1746f,
                     float wc = 40.0f, float wo = 120.0f,
                     float wcut = 100.0f * static_cast<float>(M_PI));

    ~ILADRCController();

    void reset();

    void update(float P_current, float P_target, float Wx_meas, float dt);

    float getUc() const { return _Uc; }
    float getWx_sp() const { return _Wx_sp; }
    float getCompensation() const { return _compensation; }

    void setGains(float wc, float wo);
    void setB0(float b0);
    void setCutoff(float wcut);
    void setDt(float dt) { _dt = dt; }

    float getX1_hat() const { return _x1_hat; }
    float getX2_hat() const { return _x2_hat; }
    float getX3_hat() const { return _x3_hat; }

private:
    void updateGains();
    void designThirdOrderFilter(float wcut, float T);
    float applyThirdOrderFilter(float x);
    float computeGeuCompensation(float e1, float dt);

    float _dt;
    float _b0;
    float _wc;
    float _wo;
    float _wcut;

    float _l1, _l2, _l3;
    float _kp, _kd;

    float _x1_hat;
    float _x2_hat;
    float _x3_hat;

    float _Uc;
    float _Wx_sp;
    float _last_u;
    float _compensation;

    static constexpr int FILTER_ORDER = 4;
    float _b_filter[FILTER_ORDER];
    float _a_filter[FILTER_ORDER];
    float _filter_x[FILTER_ORDER];
    float _filter_y[FILTER_ORDER];

    static constexpr int E1_HISTORY_SIZE = 3;
    float _e1_history[E1_HISTORY_SIZE];
};
