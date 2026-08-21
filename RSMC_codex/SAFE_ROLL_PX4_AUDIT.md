# HAAR/PX4 safe-roll 底层只读审计

审计日期：2026-07-22。审计对象：工作区 `PX4-Autopilot-water-air`，HEAD `5ae2fd95ae`，分支 `RSMC`（本地 `cadc` 指向同一提交）。状态含义：**VERIFIED**＝可由当前源码/受控参数文件直接证明；**LIKELY**＝证据一致但缺少实机或运行时闭环；**UNKNOWN**＝仓库内没有足够证据。

## 0. 审计边界与关键结论

- **VERIFIED** 本阶段只读检查了源码、Git 历史、已有 build 元数据、参数与原始日志目录；只新增本报告、`SAFE_ROLL_DATAFLOW.md`、`SAFE_ROLL_IMPLEMENTATION_PLAN.md`，未修改源码、参数默认值、构建文件或原始日志。
- **VERIFIED** 当前版本字符串为 `v1.15.0-beta1-346-g5ae2fd95ae`；实物发布说明指定 `make matek_h743_dummy upload`（`release/1.0/readme:4-5`），目标板配置为 STM32H743/Cortex-M7（`boards/matek/h743/default.px4board:1-2`、`boards/matek/h743/nuttx-config/nsh/defconfig:59`）。
- **VERIFIED** 固定翼 airframe 为 `SYS_AUTOSTART=2100`（`release/1.0/hydro3-250110.params:1129`），启动链为 `rc.fw_apps` 中的 `control_allocator`、标准固定翼控制器以及三个 hydro 模块（`ROMFS/px4fmu_common/init.d/rc.fw_apps:9-19,36-46`）。
- **VERIFIED** safe-roll 的最小接入点是 `HydroRateControl::Run()` 中 legacy CADRC 已算出归一化 `u_roll` 后、写入 `_hydro_torque_setpoint.xyz[0]` 前（`src/modules/hydro_rate_control/HydroRateControl.cpp:277-300`）。默认路径必须原样输出 legacy CADRC。
- **VERIFIED** 当前 CADRC 仅在 STAB 且 `HY_CADRC_EN=1` 时运行（同文件 `:228-260`）；ACRO 走旧 rate PID，MANUAL 直接杆量映射（`:203-226,308-385`）。
- **VERIFIED** 目前没有水翼角速率限制、分配器输出新鲜度检查、解锁/上锁 CADRC 复位、姿态/角速度 freshness/validity gate 或 Newton 求解失败标志。这些是上线新控制器前的高优先级风险。

## 1. 工程、分支、构建与启动

### 1.1 版本与历史 CADRC

- **VERIFIED** HEAD 为 `5ae2fd95ae`，分支 `RSMC`；`cadc` 同指该提交。CADRC 首次完整加入提交为 `6abec104d4`（分支 `new`），其加入 `CADRC.cpp/.h`、消息、参数、logger 与 CMake；当前 HEAD 相对该提交只在 `HydroRateControl.cpp:187-200` 将“只有新 RC update 才接受，否则清零”改回无条件 `copy()`。因此 `6abec104d4` 是当前 legacy CADRC 的明确代码模板，当前 HEAD 是其现场操控变体。
- **LIKELY** 用户所称“实物验证的旧 CADRC”对应 `new`/当前 `RSMC` 链；仓库没有带 CADRC 字段的池试参数或池试 readme 来明确标注哪次试验验证了提交 `6abec104d4`。2025 年 `ulog_sync` 日志早于 CADRC 提交，不能证明 CADRC 实物验证。
- **VERIFIED** 已有 `build/matek_h743_dummy` 的 build header 标记 `914ec24dab... / v1.14.0-1.0.0-dirty`（`build/matek_h743_dummy/src/lib/version/build_git_version.h:6-8`），不是当前 HEAD 的可信 clean-build 证明。`ninja -n` 只显示需要重新运行 CMake。因此当前 HEAD 的完整无写入编译状态为 **UNKNOWN**。

### 1.2 编译接入

- **VERIFIED** `boards/matek/h743/dummy.px4board:20-22` 使能 `HYDRO_ALLOCATOR/HYDRO_ATT_CONTROL/HYDRO_RATE_CONTROL`；同板继承 default 配置中的 `CONTROL_ALLOCATOR`、EKF2、PWM 等（`boards/matek/h743/default.px4board:20,29-36,41-53`）。
- **VERIFIED** `hydro_rate_control/CMakeLists.txt:34-45` 将 `HydroRateControl.cpp/.hpp` 与 `CADRC.cpp/.h` 编为同一模块；`Kconfig:1-5` 声明开关。模块 Kconfig 由 `src/modules/Kconfig:1` 的通配 `rsource` 纳入。
- **VERIFIED** `hydro_allocator/CMakeLists.txt:34-43`、`hydro_att_control/CMakeLists.txt:34-46` 分别接入构建；各自 `Kconfig:1-5` 接入板级配置。
- **VERIFIED** 自启动由 `ROMFS/px4fmu_common/init.d/rc.fw_apps:37-39` 完成，不是独立 airframe 脚本；2100 airframe 先加载 `rc.fw_defaults` 并建立通用固定翼控制分配（`ROMFS/.../airframes/2100_standard_plane:11-23`）。

### 1.3 与本审计有关的自定义面

相对 `origin/main...HEAD` 的相关自定义/修改文件如下（目录中的 CMake/Kconfig/module.yaml 同属接入面）：

- **VERIFIED 控制**：`src/modules/hydro_rate_control/*`、`hydro_att_control/*`、`hydro_allocator/*`。
- **VERIFIED 水面/深度**：`src/drivers/water_level/*`、`src/modules/depth_estimator/*`、`depth_fusion/*`、`src/modules/sensors/vehicle_depth_data/*`、`slide_estimator/*`、`vtrack/*`、`mode_control/*`、`mavdebug/*`。
- **VERIFIED 通用 PX4 改动**：`src/modules/control_allocator/ControlAllocator.cpp/.hpp`、`module.yaml` 及若干 `ActuatorEffectiveness/*`、`ControlAllocationPseudoInverse.cpp`；`src/modules/logger/logged_topics.cpp`；`ROMFS/.../rc.fw_apps`；`boards/matek/h743/dummy.px4board`。
- **VERIFIED uORB**：`msg/HydroAllocateMessage.msg`、`HydroCadrcControlMessage.msg`、`HydroDepthControlMessage.msg`、`HydroTiltMessage.msg`、`DepthEstimated.msg`、`DepthFusion.msg`、`SlideEstimated.msg`、`WaterLevel.msg`；以及为 hydro topic 别名修改的 `ActuatorMotors.msg:14`、`ActuatorServos.msg:10`、`VehicleTorqueSetpoint.msg:9`、`VehicleThrustSetpoint.msg:9` 和 `msg/CMakeLists.txt:257,262`。
- **VERIFIED 历史资料**：`release/1.0/*`；`ulog_sync/250116滑行测试/*`、`ulog_sync/25011601滑行测试/*`。未发现独立 HAAR/safe-roll 目录、独立编译脚本或独立 CADRC 启动脚本。

精确控制文件清单：

```text
src/modules/hydro_rate_control/{CADRC.cpp,CADRC.h,CMakeLists.txt,HydroRateControl.cpp,
  HydroRateControl.hpp,Kconfig,hydro_rate_control_params.c}
src/modules/hydro_att_control/{CMakeLists.txt,HydroAttitudeControl.cpp,HydroAttitudeControl.hpp,Kconfig,
  hydro_att_control_params.c,hydro_pitch_controller.cpp,hydro_pitch_controller.h,
  hydro_roll_controller.cpp,hydro_roll_controller.h,hydro_yaw_controller.cpp,hydro_yaw_controller.h}
src/modules/hydro_allocator/{CMakeLists.txt,HydroAllocator.cpp,HydroAllocator.hpp,Kconfig,module.yaml}
src/modules/control_allocator/{ControlAllocator.cpp,ControlAllocator.hpp,module.yaml,
  ControlAllocation/ControlAllocationPseudoInverse.cpp,
  ActuatorEffectiveness/ActuatorEffectivenessControlSurfaces.cpp,
  ActuatorEffectiveness/ActuatorEffectivenessCustom.cpp,
  ActuatorEffectiveness/ActuatorEffectivenessCustom.hpp,
  ActuatorEffectiveness/ActuatorEffectivenessFixedWing.cpp,
  ActuatorEffectiveness/ActuatorEffectivenessRotors.cpp,
  ActuatorEffectiveness/ActuatorEffectivenessRotors.hpp}
```

精确外围自定义目录为 `src/drivers/water_level`、`src/modules/depth_estimator`、`depth_fusion`、`sensors/vehicle_depth_data`、`slide_estimator`、`vtrack`、`mode_control`、`mavdebug`；各目录内全部受 Git 跟踪的 `.cpp/.hpp/.h/CMakeLists.txt/Kconfig/module.yaml/params.c` 均为定制面。它们不在本次 roll 主闭环内，但 depth、滑行状态和未来 mode gate 可能引用它们。

## 2. CADRC 完整审计

### 2.1 状态、参数、初始化

- **VERIFIED 参数**：`J0/Jc_inv`、`kp1/omega_o1/beta11/beta12`、`kp2/omega_o2/beta21/beta22`、`alpha`（`CADRC.h:34-48`）；PX4 参数 `HY_CADRC_EN/J0/KP1/KP2/O1/O2/ALP/TAU/U_SCALE` 与角度上限（`hydro_rate_control_params.c:44-133`，绑定于 `HydroRateControl.hpp:245-256`）。默认 `TAU=0.0025 s`，因此默认算法积分步长固定为 400 Hz 而非测得 dt。
- **VERIFIED 动态状态**：计数 `_nn`；两级 ESO `_Z1/_Z2`；增益向量 `_Lc1/_Lc2`；4 点角速度历史 `_Wx_vec`；权矩阵 `_Q1`；自适应量 `_DeltaXe1_vec/_Gamma1/_A/_A_inv`；误差 `_Xe1/_Xe2`；测量/目标率/输出 `_Wx_meas/_Wx_p/_Uc`；上一时刻 `_Z1_dot_prev[2]`（`CADRC.h:27-72`）。
- **VERIFIED 初始化**：构造器设置参数、构造权矩阵、observer gains 后 `reset()`（`CADRC.cpp:96-110`）；PX4 模块构造时读参（`HydroRateControl.cpp:45-52`），`parameters_update()` 将参数写入 roll/pitch 两实例（`:86-101`）。
- **VERIFIED reset** 清空全部动态矩阵、误差、计数、输出和导数（`CADRC.cpp:127-145`）。目前唯一外部 reset 条件是进入 STAB 的边沿（`HydroRateControl.cpp:173-185`）；内部仅在 `_Uc` 或 `_Wx_p` 非有限时 reset 并输出 0（`CADRC.cpp:254-260`）。

### 2.2 算法、范围与模式行为

- **VERIFIED** `update(P_current,P_target,Wx_meas,dt)` 将 dt 限于 `[0.0005,0.02] s`，计算角误差、位置 ESO、可选自适应项、目标角速度、速度 ESO和 `_Uc=Jc_inv(kp2 Z21+Z22)`；只对 `Z22` 限于 `[-50,50]`（`CADRC.cpp:168-254`）。
- **VERIFIED** roll/pitch `_Uc` 乘 `HY_CADRC_U_SCALE` 后限于 `[-1,1]`，再加 trim 并二次限幅；yaw 是手动杆量（`HydroRateControl.cpp:277-300`）。正负号遵循 PX4 FRD 机体系右手系：正 roll/p 为绕机体 +X（机头向前轴）右手旋转；代码没有额外反号（`:236-241,277-299`）。
- **VERIFIED** STAB 退出时 CADRC 状态不会 reset，只是不再调用；再次进入 STAB 时才 reset（`:173-185,255-260,308-385`）。MANUAL/ACRO/其他模式的输出见 `:203-229,308-389,510-530`。
- **VERIFIED** 非 STAB/ACRO/MANUAL 时 hydro torque/thrust 被显式置零并发布（`:510-530`）。但 allocator/control_allocator 对消息年龄没有检查；若发布者停止或 work item 故障，最后值可能保留。
- **VERIFIED 缺失行为**：没有 arming 状态订阅或 arm/disarm reset；没有 attitude/angular-velocity timestamp age gate；没有 estimator validity flag；没有 timestamp backward/jump reset。外层 dt 已先 constrain 到 `[0.002,0.04]`，随后的越界 fallback 实际不可达（`:150-166`）；备份调度在无新陀螺时可能继续使用已缓存样本和 0.002 s。
- **VERIFIED 日志缺陷**：`hydro_cadrc_control_message` 发布前没有设置 `timestamp`，`depth_setpoint` 也未赋值（`:262-306`；字段定义 `msg/HydroCadrcControlMessage.msg:1-15`），两字段将保持零。

## 3. 非线性 hydrofoil allocator

### 3.1 输入与数学关系

定义 `u=[u_phi,u_theta,u_psi]^T = hydro_torque_setpoint.xyz`，机体系合力 `F=[Fx,0,Fz] = hydro_thrust_setpoint.xyz`。代码先平分基准力，再用 4×3 参数矩阵增加虚拟控制力（`HydroAllocator.cpp:303-338`）：

```text
[Fx_L]   [Fx/2]   [rLx_phi rLx_theta rLx_psi] u
[Fz_L] = [Fz/2] + [rLz_phi rLz_theta rLz_psi] u
[Fx_R]   [Fx/2]   [rRx_phi rRx_theta rRx_psi] u
[Fz_R]   [Fz/2]   [rRz_phi rRz_theta rRz_psi] u
```

池试参数给出 roll 列 `[0,-10,0,+10]^T`，yaw 列 `[+30,0,-30,0]^T`，pitch 列全零（`release/1.0/hydro3-250110.params:731-742`）。因此 **VERIFIED** `u_phi` 是左右 z 向虚拟力差动，`u_psi` 是左右 x 向推进力差动；`u_theta` 在该参数文件中不通过这四个虚拟力实现。

每侧以 `x=[delta_h,T]` 求以下两式为零（`HydroAllocator::func/J_func/optim`，`:130-191`）：

```text
0 = Fx_i - KL v^2 (delta_i + alpha0) sin(alpha0) - T_i cos(delta_i)
0 = Fz_i + KL v^2 (delta_i + alpha0) cos(alpha0) + T_i sin(delta_i)
```

`alpha0=theta`，`v²` 来自固定替代速度或 `debug_vect.z`（最低 1）（`:371-389`）。Newton 最多 10 次，解析 Jacobian 在 `:144-157`；每步把翼角限于 `±HY_WING_MAX_A`，推力限于 `[0, throttle*gain*Tmax]`（`:170-188`）。结果再归一化和限幅（`:399-408`）。

### 3.2 限幅、饱和与速率

- **VERIFIED 软件角限位**：Newton 每步 `±HY_WING_MAX_A`，输出再除该值并限 `[-1,1]`（`:184,404-406`）。历史值为 `1.134464 rad≈65°`（params `:744`）。
- **VERIFIED 推力限位**：x 虚拟力负值重分配（`:340-357`）；Newton 推力受油门比例与 `HY_RT_MAX_THRUST` 限制（`:184-186`）；归一化再限 `[0,1]`（`:407-408`）；control_allocator 合成后再限 `[-1,1]`（`ControlAllocator.cpp:798-823`）。
- **VERIFIED 输出 PWM 限位**：mixing output 将归一化值插值到各通道 MIN/MAX，支持 REV，disarmed/ramp/failsafe 状态（`src/lib/mixer_module/mixer_module.cpp:528-631`）。
- **UNKNOWN 机械限位**：仓库只含 PWM MIN/MAX 与算法角上限，没有连杆几何、舵机标定曲线、硬止挡角或负载下可达范围。
- **VERIFIED 无水翼速率限制**：allocator 对翼角没有上一拍状态/`SlewRate`；唯一显式速率限制是折叠翼 `_foldwing_sp`（`HydroAllocator.cpp:222-299`），与滚转水翼无关。
- **VERIFIED 求解缺陷**：`inv(J,J_inv)` 返回值未检查，非有限结果、残差、迭代收敛与饱和状态均未发布（`:170-191`）；实速模式若没有新的 `debug_vect`，`_nf_params.v2` 可能保留零/旧值（`:379-389`）。
- **UNKNOWN 当前启用状态**：新版 `HY_FOIL_AUX` 默认 0、阈值默认 0，判断使用严格 `>`，所以仅用默认值时水翼输出分支不执行（`module.yaml:201-233`；`HydroAllocator.cpp:537-591`）。2025 历史 params 尚无 `HY_FOIL_*`/`HY_ALLOCATE_MODE` 字段，无法证明当前实机保存值；上电后必须用 `param show` 核对。

### 3.3 可反算性与复用接口

- **VERIFIED** 可以数学上由当前 `delta_i,T_i,alpha0,v²` 正算各侧有效 `Fx_i,Fz_i`，再在已知基准力和 `u_theta/u_psi` 后，对 roll 列作带饱和标志的最小二乘反算 `u_phi^e`。若 roll 列 `a_phi` 非零：`u_phi^e=(a_phi^T w)/(a_phi^T a_phi)`，`w=F_effective-F_base-A_other u_other`。饱和时该值只是“实现等效值”，不是原命令。
- **VERIFIED** 局部灵敏度可用隐函数：对第 i 侧，`[d delta_i/du_phi,dT_i/du_phi]^T = -J_i^{-1}[r_ix_phi,r_iz_phi]^T`。因此 `g_L/g_R` 是上式第一分量。解析 `J_func` 已存在（`:144-157`）。
- **VERIFIED** 当前没有公开查询 API、有限差分 API或纯函数 solver；`func/J_func/optim` 均为 `HydroAllocator` private（`HydroAllocator.hpp:87-99`），`optim` 还读取成员参数和 RC throttle。不能由 rate controller 安全复用。
- **LIKELY 推荐** 把方程、Jacobian、约束 solver 抽成无状态 `HydrofoilAllocationModel` 库，并让运行 allocator 与 safe controller 共用；提供 `solve(candidate_u) / forward(delta,T) / linearize(candidate_u)`，而不是复制公式。详见实施计划。

## 4. 左右翼与输出通道

- **VERIFIED 逻辑左右**：源码注释明确索引 0 左、1 右（`HydroAllocator.cpp:306,364-368`）；正常分配将左/右分别写 `HY_SV0_IDX-1`、`HY_SV1_IDX-1`（`:593-600`）。
- **VERIFIED 历史台架配置**：`HY_SV0_IDX=5`、`HY_SV1_IDX=4`（params `:727-728`），故 `actuator_servos.control[4]=左`、`control[3]=右`。`PWM_MAIN_FUNC11=204(Servo4)`、`PWM_MAIN_FUNC12=205(Servo5)`（params `:883-884`；功能编号 `output_functions.yaml:10-15`），故 MAIN11=右、MAIN12=左。`FunctionServos::value()` 完成 ServoN 到 `control[N-1]` 的索引（`FunctionServos.hpp:49-60`）。
- **VERIFIED 方向配置**：`PWM_MAIN_REV=6273` 的置位通道为 1、8、12、13；因此 MAIN12（左）在 PWM 输出层反向，MAIN11（右）不反向。历史 MIN/MAX：两通道均 800/2200，DIS=1500（params `:853-854,898-899,913-914,926`）。
- **UNKNOWN 物理接线**：图片只显示机体，不能读出 FC 插针；仓库无线束表或带签名台架验收记录。人工必须断桨、限能量，用 actuator test 验证 MAIN11 实际驱动右、MAIN12 实际驱动左，并记录正命令对应前缘/后缘与实际角度。

## 5. uORB、logger 与实时性

- **VERIFIED `hydro_cadrc_control_message`**：timestamp、roll/pitch sp、depth_setpoint、u_roll/u_pitch、roll/pitch 两级 ESO 各 4 状态（`msg/HydroCadrcControlMessage.msg:1-15`）。
- **VERIFIED `hydro_allocate_message`**：控制前/限幅后 `[Lx,Lz,Rx,Rz]`、优化结果 `[L翼,R翼,L推,R推]`、四个 manual allocation 中间量（`msg/HydroAllocateMessage.msg:1-11`）。
- **VERIFIED logger** 默认加入所有 hydro topic，CADRC 为不限最小间隔（`logged_topics.cpp:46-65`）；`actuator_outputs` 最小间隔 100 ms（`:161-165`），不足以观察 400 Hz 单拍行为。
- **VERIFIED perf**：hydro rate、att、allocator 与 control_allocator 都有 `PC_ELAPSED cycle`（各构造器与 Run 首尾）；gyro filter 有 cycle/reset/selection counters（`VehicleAngularVelocity.hpp:180-182`）；EKF 有 update/missed IMU（`EKF2.hpp:279-280`）；mixer 有端到端 `control latency`（`mixer_module.cpp:648-655`）。
- **VERIFIED 调度**：rate controller 在 `wq:nav_and_controllers`（栈 8000、相对优先级 -13），allocator、vehicle angular velocity、control_allocator 与 output driver 在最高优先 `wq:rate_ctrl`（栈 5000、优先级 0）（`WorkQueueManager.hpp:51,68`；各构造器）。这意味着 CADRC 和 allocator 不在同一线程，uORB callback 串接。
- **LIKELY 预算**：400 Hz 周期只有 2.5 ms。新增控制器的工程目标应是正常 <0.25 ms、WCET <0.5 ms，整条 gyro→rate→allocator→CA→PWM WCET 保留至少 1 ms 余量；这是安全设计目标，不是已测值。现有 perf 没有保存进池试 ULog，当前真实 WCET 为 **UNKNOWN**，必须在目标板用 `perf`/`top`/work_queue status 测量。

## 6. 主要安全风险与人工确认清单

1. **高/VERIFIED** CADRC 固定 `tau=2.5 ms` 可与真实回调周期偏离；dt 异常检测逻辑失效。新控制器必须使用单调 sample dt、检测跳变并复位。
2. **高/VERIFIED** 无传感器新鲜度/estimator validity 与 allocator 消息 age gate；发布者卡死可保留末值。
3. **高/VERIFIED** Newton 逆矩阵/收敛/NaN 无检查，且无水翼速率限制。通用 allocator 的 `applySlewRateLimit()` 在 hydro 值相加前执行（`ControlAllocator.cpp:434-448,795-853`），不能限制 hydro 水翼命令。
4. **高/VERIFIED** hydro 输出是“加到”通用 control allocator 输出（`ControlAllocator.cpp:798-853`），不是覆盖；同一 Servo4/5 若通用固定翼 effectiveness 也非零，会叠加并饱和。
5. **中/VERIFIED** CADRC 仅 STAB-entry reset，arm/disarm 与模式内故障没有 reset；日志 timestamp 为零。
6. **中/LIKELY** 历史 params 不包含 2026 CADRC 参数，实机依赖 firmware defaults；缺少版本锁定的 CADRC 实验参数包。
7. **必须人工确认** MAIN11/12 的实际左右插线、两侧正角方向、机械角/速率/死区、PWM 800/2200 是否安全、通用 allocator 对 Servo4/5 的叠加量、RC AUX failsafe 值、当前实机 firmware hash、400 Hz perf/WCET。

结论：legacy CADRC 能作为新 safe-roll controller 的接入模板，但不能原封不动作为安全框架；必须保留 mode 0 原路径，并在传感器有效性、状态复位、bumpless transfer、allocator 查询与求解状态方面加外围保护。实施设计见 `SAFE_ROLL_IMPLEMENTATION_PLAN.md`。
