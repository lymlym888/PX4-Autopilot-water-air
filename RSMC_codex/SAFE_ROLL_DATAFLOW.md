# HAAR/PX4 safe-roll 完整数据流

状态标签同主审计报告。所有数组索引均按 C/C++ 0-based；参数名中的 IDX 是 1-based。

## 1. 完整调用图

```text
IMU gyro driver
  -> sensor_gyro / sensor_gyro_fifo
  -> VehicleAngularVelocity::Run(), CalibrateAndPublish()
     [rate_ctrl WQ; calibration/filter/bias; rad/s]
  -> vehicle_angular_velocity.xyz
       -> HydroRateControl::Run() callback [nav_and_controllers WQ]
          p = xyz[0], q = xyz[1]

IMU integrated data -> vehicle_imu / sensor_combined
  -> EKF2::Run() -> setIMUData() -> PublishAttitude()
  -> (multi-EKF: EKF2Selector::PublishVehicleAttitude())
  -> vehicle_attitude.q
       -> HydroRateControl::Run() -> Quatf -> Eulerf -> phi/theta [rad]

manual_control_setpoint.roll
  -> roll_sp = roll * radians(HY_ROLL_MAX) in STAB
  -> ADRCController::update(phi, roll_sp, p, dt_used)
  -> Uc * HY_CADRC_U_SCALE -> constrain [-1,1] -> TRIM_ROLL
  -> hydro_torque_setpoint.xyz[0] = normalized u_phi

hydro_torque_setpoint.xyz = [u_phi,u_theta,u_psi]
hydro_thrust_setpoint.xyz = [Fx,0,Fz] N
  -> HydroAllocator::Run() [rate_ctrl WQ]
  -> virtual force A*u + F/2
  -> optim(left), optim(right): [delta_h,T]
  -> normalize/limit
  -> hydro_servos + hydro_motors
  -> ControlAllocator::publish_actuator_controls()
     generic actuator setpoint + hydro contribution -> constrain
  -> actuator_servos / actuator_motors
  -> FunctionServos / FunctionMotors -> MixingOutput
  -> PWMOut::updateOutputs() -> physical MAIN outputs
  -> actuator_outputs (actual commanded PWM values, not angle feedback)
```

## 2. 逐级证据

| 级 | 状态 | topic/变量、单位与方向 | 文件、函数、行号 |
|---|---|---|---|
| gyro 原始输入 | VERIFIED | `sensor_gyro(_fifo)`；传感器系经校准旋转到机体系 | `VehicleAngularVelocity.cpp:823-857,867-903`，`CalibrateAndPublish():921-940` |
| 角速度 | VERIFIED | `vehicle_angular_velocity.xyz`，SI rad/s；PX4 FRD 机体系，x/y/z 正向为前/右/下，正角速度遵循右手系 | `VehicleAngularVelocity.cpp:927-940`；`HydroRateControl.cpp:155-169,239-241` |
| 姿态估计 | VERIFIED | EKF 以 `vehicle_imu`/`sensor_combined` 更新并发布 quaternion；有效时才发布 | `EKF2.cpp:506-653,948-965`；多实例 selector stale gate `EKF2Selector.cpp:396-413` |
| roll angle | VERIFIED | `phi=Eulerf(Quatf(vehicle_attitude.q)).phi()`，rad，FRD/NED conventional positive roll | `HydroRateControl.cpp:232-240` |
| roll reference | VERIFIED | 当前 CADRC 路径直接来自 `manual_control_setpoint.roll * radians(HY_ROLL_MAX)`，仅 valid 且 timestamp 非零，否则 0；不是 `hydro_attitude_setpoint` | `HydroRateControl.cpp:187-200,243-249` |
| CADRC | VERIFIED | `_cadrc_roll.update(roll,roll_sp,p,dt_used)`；两级 ESO 输出未归一 `_Uc` | `HydroRateControl.cpp:251-260`；`CADRC.cpp:168-260` |
| normalized roll | VERIFIED | `u_phi=constrain(Uc*U_SCALE,-1,1)`，再加 `TRIM_ROLL` 再限幅；无符号翻转 | `HydroRateControl.cpp:277-300` |
| hydro setpoint | VERIFIED | torque 为归一化虚拟控制；thrust 是 N。时间基于 angular velocity sample | `HydroRateControl.cpp:431-434,496-504,524-530` |
| 虚拟分配 | VERIFIED | `[FxL,FzL,FxR,FzR]=[Fx/2,Fz/2,Fx/2,Fz/2]+A[u_phi,u_theta,u_psi]` | `HydroAllocator.cpp:303-338` |
| 非线性求解 | VERIFIED | 两侧 `delta_h` rad、`T` N；解析方程/Jacobian + 最多 10 次 Newton | `HydroAllocator.cpp:130-191,364-408` |
| 请求输出 | VERIFIED | `hydro_servos.control[]` 是归一化请求，`hydro_motors.control[]` 是归一化推力 | `HydroAllocator.cpp:404-418,591-637` |
| 合成输出 | VERIFIED | 通用 CA setpoint 与 hydro 值相加再限 `[-1,1]` | `ControlAllocator.cpp:795-864` |
| PWM/actual command | VERIFIED | output function 取 actuator topic，映射 MIN/MAX/REV，发布 `actuator_outputs` | `FunctionServos.hpp:49-60`；`mixer_module.cpp:528-644` |
| 实际翼角反馈 | UNKNOWN | 工程无左右水翼位置传感 topic；`actuator_outputs` 只是 PWM 命令，不是测得角度 | 全仓库未发现 hydrofoil angle feedback consumer |

## 3. 更新时间、频率与线程

- **VERIFIED** `HydroRateControl` 由 `vehicle_angular_velocity` callback 唤醒，并有 20 ms backup（`HydroRateControl.cpp:59-68,123-135,533-536`）。它运行于 `nav_and_controllers`（构造器 `:45-48`）。实际调用频率等于 `vehicle_angular_velocity` 发布频率，源码未硬编码 400 Hz。
- **VERIFIED** controller dt 取相邻 `timestamp_sample`，先限 `[0.002,0.04]`（`:150-160`）；默认 `HY_CADRC_TAU=0.0025` 覆盖 dt（`:251-254`，参数 `hydro_rate_control_params.c:100-115`）。因此默认 CADRC 数值积分固定 400 Hz，但调度/调用频率仍由 gyro topic 决定。
- **VERIFIED** `HydroAllocator` 同时注册 torque、thrust callback（`HydroAllocator.cpp:93-101`），运行于 `rate_ctrl`（`:41-44`），每次读取两 topic 的最新值并发布 hydro outputs（`:194-219,636-648`）。
- **VERIFIED** 通用 `ControlAllocator` 也在 `rate_ctrl`，消费 `hydro_motors/hydro_servos` 并发布标准 actuator topic（`ControlAllocator.hpp:182-191`；`ControlAllocator.cpp:663-686,795-864`）。输出 driver被切到同一 work queue（`mixer_module.cpp:240-247`）。
- **UNKNOWN** 目标板实际 gyro publish rate、每段延迟、jitter 和 WCET；现有池试日志早于 CADRC，不能测当前链。应使用已有 cycle perf 和 `control latency` 现场测量。

## 4. 坐标、符号、范围

```text
u_phi > 0
  -> historical A roll column: Fz_L += -10*u_phi, Fz_R += +10*u_phi
  -> nonlinear solver determines delta_L/delta_R
  -> left normalized request -> actuator_servos.control[4]
  -> right normalized request -> actuator_servos.control[3]
```

- **VERIFIED** `phi` 和 `p` 均正绕 body +X；`roll_sp` 同符号。代码层没有负号。物理翼偏转符号还叠加 MAIN12 REV，必须以台架定义“正 delta”对应的几何方向。
- **VERIFIED** CADRC raw `_Uc` 无明确物理单位注释，只标记“未归一化”（`CADRC.h:17-21`）；接口靠经验参数 `U_SCALE` 转归一化虚拟 roll。
- **VERIFIED** `hydro_torque_setpoint.xyz[0..2]` 被 allocator 当作 `[roll,pitch,yaw]` 虚拟控制量，正常应在 `[-1,1]`（`HydroRateControl.cpp:277-300`，`HydroAllocator.cpp:314-328`）。
- **VERIFIED** `hydro_thrust_setpoint.xyz[0]` 为机体 x 力、`xyz[2]` 为机体 z 力，单位 N；由水平/竖直力按 pitch/attack angle 旋转（`HydroRateControl.cpp:431-434,496-504`）。

## 5. 左右请求、实际输出和通道表

| 物理意图 | allocator 数组 | hydro topic | 标准 topic | 历史 PWM function | MAIN 通道 | REV | 状态 |
|---|---:|---:|---:|---:|---:|---:|---|
| 左水翼 | `x[0][0]` | `hydro_servos.control[4]` | `actuator_servos.control[4]` (Servo5) | 205 | MAIN12 | 是 | 软件/参数 VERIFIED，接线 UNKNOWN |
| 右水翼 | `x[1][0]` | `hydro_servos.control[3]` | `actuator_servos.control[3]` (Servo4) | 204 | MAIN11 | 否 | 软件/参数 VERIFIED，接线 UNKNOWN |
| 左水下推进器 | `x[0][1]` | `hydro_motors.control[0]` | `actuator_motors.control[0]` (Motor1) | 101 | MAIN4 | 否 | 软件/参数 VERIFIED，接线 UNKNOWN |
| 右水下推进器 | `x[1][1]` | `hydro_motors.control[1]` | `actuator_motors.control[1]` (Motor2) | 102 | MAIN5 | 否 | 软件/参数 VERIFIED，接线 UNKNOWN |

证据：左右定义与写入 `HydroAllocator.cpp:306,364-368,591-600`；IDX 参数 `release/1.0/hydro3-250110.params:721-728`；function `:881-895`；REV `:926`；function 数值含义 `output_functions.yaml:10-15`。

注意：这里的“实际输出”仅能追到 `actuator_outputs.output[MAIN-1]` 的 PWM 命令。没有位置传感器，无法由飞控确认水翼真实角度、卡滞或回差。

## 6. 限位链

```text
CADRC raw Uc
 -> U_SCALE and [-1,1]
 -> trim and [-1,1]
 -> virtual force mapping
 -> negative Fx redistribution
 -> Newton each step: delta ±WING_MAX_A, T [0, throttle limit]
 -> normalized delta [-1,1], T [0,1]
 -> optional foil AUX/mode gate
 -> add generic allocator output
 -> final actuator [-1,1]
 -> MAIN REV
 -> channel PWM MIN/MAX; disarmed/ramp/failsafe
```

对应位置：`HydroRateControl.cpp:277-300`；`HydroAllocator.cpp:340-357,170-188,404-408,537-620`；`ControlAllocator.cpp:798-853`；`mixer_module.cpp:528-631`。水翼偏转自身无 slew/rate limit，机械限位未知。

**UNKNOWN 当前水翼 gate**：`HY_FOIL_AUX` 新版默认 0，严格 `foil_aux > HY_FOIL_THR` 在默认 0/0 时为 false（`hydro_allocator/module.yaml:201-233`；`HydroAllocator.cpp:537-591`）。历史 params 没有这些新增字段，必须读取当前实机参数才能确认是否实际输出。通用 control allocator 虽有 slew limit，但它发生在 hydro contribution 相加之前（`ControlAllocator.cpp:434-448,795-853`），不能提供水翼速率保护。

## 7. 分配器查询与反算数据路径

当前只能从 `hydro_allocate_message.actuator_optimed` 获得未归一化左右请求（字段定义 `HydroAllocateMessage.msg:3-10`，赋值 `HydroAllocator.cpp:335-402`）；无法查询任意候选 `u_phi`，无法直接得到 solver status 或 `g_L/g_R`。

不复制代码的正确结构是把 `func/J_func/optim` 抽成共用纯模型：

```text
AllocationContext = {Fx,Fz,u_theta,u_psi,alpha0,v2,KL,limits,throttle}
solve(context, candidate_u_phi)
  -> {delta_L,delta_R,T_L,T_R,residual,iterations,saturation,valid}
linearize(context, solution)
  -> g_L, g_R via -J^-1 * A_phi_side
forward(solution)
  -> effective forces -> least-squares u_phi^e
```

运行 allocator 与 safe controller 调同一个库函数；候选查询不得发布 uORB 或改变 warm-start 状态。若 400 Hz 内多次候选求解超预算，使用一次 nominal solve + 解析 `g_L/g_R` 构造局部可行区间，并对最终命令只做一次完整 solve。
