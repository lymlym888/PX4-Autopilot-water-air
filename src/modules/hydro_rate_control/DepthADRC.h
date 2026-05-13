// DepthADRC.h
#pragma once

class DepthADRCController {
public:
	DepthADRCController();

	void reset(float depth_now = 0.0f);

	void setParams(float b0,
		       float td_r,
		       float eso_beta01,
		       float eso_beta02,
		       float eso_beta03,
		       float eso_alpha1,
		       float eso_alpha2,
		       float eso_delta,
		       float nlsef_alpha1,
		       float nlsef_alpha2,
		       float nlsef_delta,
		       float nlsef_beta1,
		       float nlsef_beta2,
		       float kf_q_pos,
		       float kf_q_vel,
		       float kf_r_meas,
		       float u_min,
		       float u_max);

	// 输入：目标深度、测量深度、控制周期
	// 输出：竖直推力控制量，单位按现有 Hydro Depth Control 逻辑使用 N，向下为正
	float update(float depth_setpoint, float depth_meas, float dt);

	float getTdX1() const { return _td_x1; }
	float getTdX2() const { return _td_x2; }

	float getKfDepth() const { return _kf_x1; }
	float getKfRate() const { return _kf_x2; }

	float getEsoZ1() const { return _eso_z1; }
	float getEsoZ2() const { return _eso_z2; }
	float getEsoZ3() const { return _eso_z3; }

	float getE1() const { return _e1; }
	float getE2() const { return _e2; }

	float getURaw() const { return _u_raw; }
	float getULimited() const { return _u_limited; }

	bool initialized() const { return _initialized; }

private:
	float fal(float e, float alpha, float delta) const;
	float fst(float e, float v2, float r_set, float h0) const;
	void updateTD(float depth_setpoint, float dt);
	void updateKF(float depth_meas, float control_u, float dt);
	void updateESO(float y_sys_out, float control_u, float dt);

	bool _initialized{false};

	// 参数
	float _b0{1.2f};

	float _td_r{1.0f};

	float _eso_beta01{25.0f};
	float _eso_beta02{30.0f};
	float _eso_beta03{40.0f};
	float _eso_alpha1{0.5f};
	float _eso_alpha2{0.35f};
	float _eso_delta{0.002f};

	float _nlsef_alpha1{0.275f};
	float _nlsef_alpha2{0.275f};
	float _nlsef_delta{0.01f};
	float _nlsef_beta1{8.0f};
	float _nlsef_beta2{2.0f};

	float _kf_q_pos{1e-6f};
	float _kf_q_vel{1e-3f};
	float _kf_r_meas{0.05f};

	float _u_min{-60.0f};
	float _u_max{60.0f};

	// TD 状态
	float _td_x1{0.0f};
	float _td_x2{0.0f};
	float _td_x2_dot{0.0f};

	// ESO 状态
	float _eso_z1{0.0f};
	float _eso_z2{0.0f};
	float _eso_z3{0.0f};

	// KF 状态：x = [depth, depth_rate]
	float _kf_x1{0.0f};
	float _kf_x2{0.0f};

	// KF 协方差矩阵 P
	float _p00{1.0f};
	float _p01{0.0f};
	float _p10{0.0f};
	float _p11{1.0f};

	// 记录量
	float _e1{0.0f};
	float _e2{0.0f};
	float _u_raw{0.0f};
	float _u_limited{0.0f};
};
