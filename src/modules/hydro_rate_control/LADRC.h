// LADRC.h
#pragma once

#include <stdint.h>

class LADRCController {
public:
	LADRCController();

	void reset(float y0 = 0.0f);

	void setParams(float b0,
		       float omega_c,
		       float omega_o,
		       float tau_td,
		       float u_lim,
		       int32_t eso_order,
		       float eso_gain_scale);

	// 输入：
	// r_rate    : 期望角速度，rad/s
	// y_rate    : 实际角速度，rad/s
	// dt        : 控制周期，s
	//
	// 输出：
	// normalized torque command，范围由 u_lim 限幅
	float update(float r_rate, float y_rate, float dt);

	float getZ1() const { return _z1; }
	float getZ2() const { return _z2; }
	float getZ3() const { return _z3; }

	float getReference() const { return _v1; }
	float getError() const { return _error; }

	float getURaw() const { return _u_raw; }
	float getU() const { return _u; }

	int32_t getEsoOrder() const { return _eso_order; }

private:
	void updateObserverGains();
	void updateReference(float r_rate, float dt);
	void updateLESO(float y_rate, float dt);

	bool _initialized{false};

	// LADRC parameters
	float _b0{1.0f};
	float _omega_c{4.0f};
	float _omega_o{15.0f};
	float _tau_td{0.05f};
	float _u_lim{0.4f};
	float _eso_gain_scale{1.0f};

	int32_t _eso_order{2};

	// LESO gains
	float _beta1{0.0f};
	float _beta2{0.0f};
	float _beta3{0.0f};

	// Reference tracking
	float _v1{0.0f};

	// LESO states
	// z1: angular-rate estimate
	// z2: total disturbance estimate for second-order LESO;
	//     for third-order LESO, z2 is still used as the main compensation term.
	// z3: disturbance derivative estimate for third-order LESO.
	float _z1{0.0f};
	float _z2{0.0f};
	float _z3{0.0f};

	// Previous applied input, used by LESO
	float _u_prev{0.0f};

	// Debug values
	float _error{0.0f};
	float _u_raw{0.0f};
	float _u{0.0f};
};
