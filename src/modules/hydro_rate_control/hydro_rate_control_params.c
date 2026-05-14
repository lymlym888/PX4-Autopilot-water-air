/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hydro_rate_control_params.c
 *
 * Parameters defined by the fixed-wing rate control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */


/**
 * Enable CADRC in STAB mode
 *
 * @boolean
 * @group HY CADRC
 */
PARAM_DEFINE_INT32(HY_CADRC_EN, 0);

/**
 * CADRC J0 (inertia-like parameter)
 *
 * @min 0.001
 * @group HY CADRC
 */
PARAM_DEFINE_FLOAT(HY_CADRC_J0, 10.1746f);

/**
 * CADRC kp1 (position loop gain)
 *
 * @min 0.0
 * @group HY CADRC
 */
PARAM_DEFINE_FLOAT(HY_CADRC_KP1, 500.0f);

/**
 * CADRC kp2 (rate loop gain)
 *
 * @min 0.0
 * @group HY CADRC
 */
PARAM_DEFINE_FLOAT(HY_CADRC_KP2, 1000.0f);

/**
 * CADRC omega_o1 (ESO bandwidth for position loop)
 *
 * @min 0.0
 * @group HY CADRC
 */
PARAM_DEFINE_FLOAT(HY_CADRC_O1, 15.0f);

/**
 * CADRC omega_o2 (ESO bandwidth for rate loop)
 *
 * @min 0.0
 * @group HY CADRC
 */
PARAM_DEFINE_FLOAT(HY_CADRC_O2, 15.0f);

/**
 * CADRC alpha (adaptive gain). 0 disables adaptation
 *
 * @min 0.0
 * @group HY CADRC
 */
PARAM_DEFINE_FLOAT(HY_CADRC_ALP, 0.0f);

/**
 * CADRC tau override (s). 0 = use dt from gyro/scheduler
 *
 * @min 0.0
 * @max 0.02
 * @group HY CADRC
 */
PARAM_DEFINE_FLOAT(HY_CADRC_TAU, 0.0025f);

/**
 * Scale CADRC Uc to normalized torque [-1, 1]
 *
 * @min 0.000001
 * @group HY CADRC
 */
PARAM_DEFINE_FLOAT(HY_CADRC_U_SCALE, 0.001f);

/**
 * STAB max roll angle from RC (deg)
 *
 * @min 0.0
 * @max 80.0
 * @group HY CADRC
 */
PARAM_DEFINE_FLOAT(HY_ROLL_MAX, 30.0f);

/**
 * STAB max pitch angle from RC (deg)
 *
 * @min 0.0
 * @max 80.0
 * @group HY CADRC
 */
PARAM_DEFINE_FLOAT(HY_PITCH_MAX, 20.0f);


/**
 * Enable LADRC in STAB mode
 *
 * When enabled, LADRC has higher priority than CADRC.
 *
 * @boolean
 * @group HY LADRC
 */
PARAM_DEFINE_INT32(HY_LADRC_EN, 0);

/**
 * LADRC LESO order
 *
 * 2: second-order LESO
 * 3: third-order LESO
 *
 * @min 2
 * @max 3
 * @group HY LADRC
 */
PARAM_DEFINE_INT32(HY_LADRC_ORD, 2);

/**
 * LADRC input gain b0
 *
 * The nominal gain from normalized torque command to body-rate acceleration.
 *
 * @min 0.0001
 * @decimal 4
 * @group HY LADRC
 */
PARAM_DEFINE_FLOAT(HY_LADRC_B0, 1.0f);

/**
 * LADRC controller bandwidth omega_c
 *
 * @min 0.0
 * @decimal 3
 * @group HY LADRC
 */
PARAM_DEFINE_FLOAT(HY_LADRC_WC, 4.0f);

/**
 * LADRC observer bandwidth omega_o
 *
 * @min 0.0
 * @decimal 3
 * @group HY LADRC
 */
PARAM_DEFINE_FLOAT(HY_LADRC_WO, 15.0f);

/**
 * LADRC reference smoothing time constant
 *
 * 0 disables smoothing.
 *
 * @unit s
 * @min 0.0
 * @decimal 4
 * @group HY LADRC
 */
PARAM_DEFINE_FLOAT(HY_LADRC_TAU, 0.05f);

/**
 * LADRC normalized output limit
 *
 * The LADRC output is normalized torque command before trim.
 *
 * @min 0.0001
 * @max 1.0
 * @decimal 3
 * @group HY LADRC
 */
PARAM_DEFINE_FLOAT(HY_LADRC_ULIM, 0.4f);

/**
 * LADRC outer attitude proportional gain
 *
 * roll_rate_sp = AP * roll_error
 * pitch_rate_sp = AP * pitch_error
 *
 * @min 0.0
 * @decimal 3
 * @group HY LADRC
 */
PARAM_DEFINE_FLOAT(HY_LADRC_AP, 2.0f);

/**
 * LADRC body-rate setpoint limit
 *
 * @unit rad/s
 * @min 0.01
 * @decimal 3
 * @group HY LADRC
 */
PARAM_DEFINE_FLOAT(HY_LADRC_RLIM, 0.8f);

/**
 * LADRC LESO gain scale
 *
 * This scales beta1, beta2 and beta3 together.
 *
 * @min 0.01
 * @decimal 3
 * @group HY LADRC
 */
PARAM_DEFINE_FLOAT(HY_LADRC_GS, 1.0f);




/**
 * Body attack feedforward.
 *
 * Please refer to the comments in HydroRateControl.cpp for details
 *
 * @decimal 3
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_ATTACK_FF, 0.00f);

/**
 * Pitch rate proportional gain.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_PR_P, 0.08f);

/**
 * Pitch rate derivative gain.
 *
 * Pitch rate differential gain.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_PR_D, 0.f);

/**
 * Pitch rate integrator gain.
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_PR_I, 0.1f);

/**
 * Pitch rate integrator limit
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_PR_IMAX, 0.4f);

/**
 * Roll rate proportional gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_RR_P, 0.05f);

/**
 * Roll rate derivative gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_RR_D, 0.0f);

/**
 * Roll rate integrator gain
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.01
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_RR_I, 0.1f);

/**
 * Roll integrator limit
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_RR_IMAX, 0.2f);

/**
 * Yaw rate proportional gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_YR_P, 0.05f);

/**
 * Yaw rate derivative gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_YR_D, 0.0f);

/**
 * Yaw rate integrator gain
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 1
 * @increment 0.5
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_YR_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_YR_IMAX, 0.2f);

/**
 * Roll rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_RR_FF, 0.5f);

/**
 * Pitch rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_PR_FF, 0.5f);

/**
 * Yaw rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_YR_FF, 0.3f);

/**
 * Acro body roll max rate setpoint
 *
 * @min 10
 * @max 720
 * @unit deg
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_ACRO_X_MAX, 90);

/**
 * Acro body pitch max rate setpoint
 *
 * @min 10
 * @max 720
 * @unit deg
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_ACRO_Y_MAX, 90);

/**
 * Acro body yaw max rate setpoint
 *
 * @min 10
 * @max 720
 * @unit deg
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_ACRO_Z_MAX, 45);

/**
 * Enable throttle scale by battery level
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery.
 *
 * @boolean
 * @group Hydro Rate Control
 */
PARAM_DEFINE_INT32(HY_BAT_SCALE_EN, 0);

/**
 * Enable airspeed scaling
 *
 * This enables a logic that automatically adjusts the output of the rate controller to take
 * into account the real torque produced by an aerodynamic control surface given
 * the current deviation from the trim airspeed (HY_AIRSPD_TRIM).
 *
 * Enable when using aerodynamic control surfaces (e.g.: plane)
 * Disable when using rotor wings (e.g.: autogyro)
 *
 * @boolean
 * @group Hydro Rate Control
 */
PARAM_DEFINE_INT32(HY_ARSP_SCALE_EN, 1);

/**
* Roll trim increment at minimum airspeed
*
* This increment is added to TRIM_ROLL when airspeed is HY_AIRSPD_MIN.
 *
 * @group Hydro Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(HY_DTRIM_R_VMIN, 0.0f);

/**
* Pitch trim increment at minimum airspeed
*
* This increment is added to TRIM_PITCH when airspeed is HY_AIRSPD_MIN.
 *
 * @group Hydro Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(HY_DTRIM_P_VMIN, 0.0f);

/**
* Yaw trim increment at minimum airspeed
*
* This increment is added to TRIM_YAW when airspeed is HY_AIRSPD_MIN.
 *
 * @group Hydro Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(HY_DTRIM_Y_VMIN, 0.0f);

/**
* Roll trim increment at maximum airspeed
*
* This increment is added to TRIM_ROLL when airspeed is HY_AIRSPD_MAX.
 *
 * @group Hydro Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(HY_DTRIM_R_VMAX, 0.0f);

/**
* Pitch trim increment at maximum airspeed
*
* This increment is added to TRIM_PITCH when airspeed is HY_AIRSPD_MAX.
 *
 * @group Hydro Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(HY_DTRIM_P_VMAX, 0.0f);

/**
* Yaw trim increment at maximum airspeed
*
* This increment is added to TRIM_YAW when airspeed is HY_AIRSPD_MAX.
 *
 * @group Hydro Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(HY_DTRIM_Y_VMAX, 0.0f);

/**
 * Manual roll scale
 *
 * Scale factor applied to the desired roll actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_MAN_R_SC, 1.0f);

/**
 * Manual pitch scale
 *
 * Scale factor applied to the desired pitch actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_MAN_P_SC, 1.0f);

/**
 * Manual yaw scale
 *
 * Scale factor applied to the desired yaw actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_MAN_Y_SC, 1.0f);

/**
 * Roll control to yaw control feedforward gain.
 *
 * This gain can be used to counteract the "adverse yaw" effect for fixed wings.
 * When the plane enters a roll it will tend to yaw the nose out of the turn.
 * This gain enables the use of a yaw actuator to counteract this effect.
 *
 * @min 0.0
 * @decimal 1
 * @increment 0.01
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_RLL_TO_YAW_FF, 0.0f);

/**
 * Enable yaw rate controller in Acro
 *
 * If this parameter is set to 1, the yaw rate controller is enabled in Fixed-wing Acro mode.
 * Otherwise the pilot commands directly the yaw actuator.
 * It is disabled by default because an active yaw rate controller will fight against the
 * natural turn coordination of the plane.
 *
 * @boolean
 * @group Hydro Rate Control
 */
PARAM_DEFINE_INT32(HY_ACRO_YAW_EN, 0);

/**
 * Thrust to pit control feedforward gain.
 *
 * Thrust to pit control feedforward gain.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_THR_TO_PIT_FF, 0.0f);


/**
 * Minimum Airspeed (CAS)
 *
 * The minimal airspeed (calibrated airspeed) the user is able to command.
 * Further, if the airspeed falls below this value, the TECS controller will try to
 * increase airspeed more aggressively.
 * Has to be set according to the vehicle's stall speed (which should be set in HYDRO_AIRSPD_STALL),
 * with some margin between the stall speed and minimum airspeed.
 * This value corresponds to the desired minimum speed with the default load factor (level flight, default weight),
 * and is automatically adpated to the current load factor (calculated from roll setpoint and WEIGHT_GROSS/WEIGHT_BASE).
 *
 * @unit m/s
 * @min 0.5
 * @decimal 1
 * @increment 0.5
 * @group Hydro Rate Control
 */
PARAM_DEFINE_FLOAT(HY_AIRSPD_MIN, 10.0f); //最小空速

/**
 * Maximum manually added yaw rate
 *
 * This is the maximally added yaw rate setpoint from the yaw stick in any attitude controlled flight mode.
 * It is added to the yaw rate setpoint generated by the controller for turn coordination.
 *
 * @unit deg/s
 * @min 0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(HY_MAN_YR_MAX, 30.f);

//深度控制通用参数

/**
 * Depth control mode selection
 *
 * @group Hydro Depth Control
 * @value 0 Disabled
 * @value 1 NSF
 * @value 2 HSMC
 * @value 3 Manual
 * @value 4 Feedforward Only
 */
PARAM_DEFINE_INT32(HY_DC_MODE, 0);
//深度控制模式选择

/**
 * Depth control gravity feedforward
 *
 * @unit N
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_DC_FF, -20.f);
//深度控制重力补偿，向下为正

/**
 * Depth control depth setpoint
 *
 * @unit m
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_DC_SP, 0.05f);
//深度控制深度设定点，向下为正，即水面为0，水下为正

/**
 * Depth control max throttle
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_DC_MAX_THR, 1.0f);
//深度控制最大推力比例


//非光滑反馈参数

/**
 * NSF predict time
 *
 * @decimal 3
 * @group Hydro Depth Control
 * @unit s
 */
PARAM_DEFINE_FLOAT(HY_NSF_PT, 0.2f);
//非光滑反馈预测时间

/**
 * NSF err norm point
 *
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_NSF_NORM, 0.025f);
//非光滑反馈误差归一化点

/**
 * NSF err power
 *
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_NSF_POWER, 1.0f);
//非光滑反馈的误差幂次

/**
 * NSF kp
 *
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_NSF_KP, 1.0f);
//非光滑反馈的控制增益

/**
 * NSF u_max
 *
 * @unit N
 * @min 0.0
 * @max 60.0
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_NSF_UMAX, 60.0f);
//非光滑反馈的输出上限

/**
 * NSF u_min
 *
 * @unit N
 * @min -60.0
 * @max 0.0
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_NSF_UMIN, -60.0f);
//非光滑反馈的输出下限

//高阶滑模参数

/**
 * HSMC alpha
 *
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_HSMC_ALPHA, 5.0f);
//alpha越大，滑模面越陡峭，收敛越快

/**
 * HSMC eta
 *
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_HSMC_ETA, 60.0f);
//eta越大，理论上鲁棒性越好，到达滑模面的速度越快

/**
 * HSMC mass
 *
 * @unit kg
 * @decimal 4
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_HSMC_MASS, 2.0f);
//高阶滑模的质量参数

/**
 * HSMC Ts
 *
 * @unit s
 * @decimal 4
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_HSMC_TS, 0.0025f);
//高阶滑模的积分时间长度

/**
 * HSMC integrater max
 *
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_HSMC_IMAX, 20.0f);
//高阶滑模的积分上限

/**
 * HSMC integrater min
 *
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_HSMC_IMIN, -20.0f);
//高阶滑模的积分下限

//手动控制参数

/**
 * Manual vertical force ratio
 *
 * @decimal 3
 * @group Hydro Depth Control
 */
PARAM_DEFINE_FLOAT(HY_MAN_RATIO, 0.0f);
//手动模式下纵向力的比率


