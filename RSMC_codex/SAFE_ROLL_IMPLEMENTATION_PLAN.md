# HAAR safe-roll 最小侵入实施计划（设计，不实现）

本文只描述下一阶段方案。本阶段没有修改任何源码、参数默认值、构建文件或原始日志。

## 1. 接入原则与最小位置

- **VERIFIED 接入点**：`HydroRateControl::Run()` 在 `u_roll=constrain(_cadrc_roll.getUc()*u_scale)` 之后、`_hydro_torque_setpoint.xyz[0]=u_roll` 之前（`src/modules/hydro_rate_control/HydroRateControl.cpp:277-300`）。此处已具备 `roll_sp,phi,p,dt,legacy_u_roll`，下游接口仍是 normalized `u_phi`，侵入最小。
- **VERIFIED 不应接入**：不要在 PWM、`actuator_servos.control[3/4]` 或通用 `ControlAllocator` 后端插入 barrier；那会丢失虚拟 roll 意义并绕过分配器耦合。
- mode 0 必须 byte-for-byte 等价地选择 `legacy_u_roll`。新控制器只生成 `u_roll_selected`，pitch/yaw/depth 保持现状。

建议调用形态（仅设计）：

```text
legacy_u = existing CADRC result
safe_result = SafeRollController::update(state, reference, legacy_u,
                                         allocation_context, dt)
selected_u = mode==0 ? legacy_u : safe_result.u
if invalid/stale/overrun: selected_u = bumplessFallback(legacy_u or 0 increment)
```

## 2. 运行模式

新增整型参数建议名 `HY_SR_MODE`，默认 **0**：

| 值 | 模式 | 行为 | 允许条件 |
|---:|---|---|---|
| 0 | legacy CADRC | 完全复用现有 roll CADRC 输出 | 默认、任何回退终点 |
| 1 | nominal no DE-HOCBF | 新 nominal controller，但不施加 barrier 修正 | 先台架/系留 |
| 2 | worst-case fixed DE-HOCBF | 固定最坏界、固定模型区间 | 参数校核后受控试验 |
| 3 | adaptive safe | nominal + adaptive DE-HOCBF + allocation-aware interval | 完整验证后目标模式 |
| 4 | adaptive without allocation-aware interval | 仅消融试验，禁止普通飞行默认选择 | 独立解锁条件/测试开关 |

- 参数定义放 `hydro_rate_control_params.c`，绑定放 `HydroRateControl.hpp:167-263`。
- 可选 RC 选择使用现有 `manual_control_setpoint.aux1..6` 模式（已有范例 `HydroAllocator.cpp:537-579`），建议另设 `HY_SR_AUX=0..6`；`0` 表示禁用 RC 切换。不要用单阈值直接编码 5 模式，采用离散档位 + debounce，并在 RC invalid/loss 时强制 mode 0。
- 模式 4 需第二道 test-enable 参数，默认 false；armed 时禁止从 mode 0 直接跨到 4。
- 切换只在 STAB 且传感器健康时生效；MANUAL/ACRO 保持当前路径。参数更新沿现有 1 s `parameter_update` 机制（`HydroRateControl.cpp:136-148`）。

## 3. 状态、复位与无跳变回退

### 3.1 必须监控

- `vehicle_angular_velocity.timestamp_sample` 单调且 age 在门限内；`xyz` finite。
- `vehicle_attitude.timestamp_sample/timestamp` 新鲜，quaternion finite/normalized；检测 `quat_reset_counter` 变化。
- `dt` 来自 sample timestamp，未经过先 constrain 掩盖异常；建议有效窗约 `[1,10] ms`，越界先 reset 再用安全有限 dt。
- vehicle status/nav state/arming edge；RC valid、allocator context 有效；solver valid、残差、饱和、计算时间。

### 3.2 reset 条件

进入/退出 STAB、arm/disarm 任一边沿、estimator reset、timestamp 非单调或 gap、sensor stale/NaN、模式变化、新参数影响内部状态、solver invalid、连续 deadline miss。legacy CADRC 也应在这些边沿同步 reset，但为了保证 mode 0 行为不变，首次实施可把新增 reset 限于新控制器，并把 legacy reset 改进列为独立受控补丁。

### 3.3 bumpless transfer

- 所有新模式进入时初始化 nominal/adaptive 内部状态，使第一拍 `u_new = legacy_u`；barrier 修正从 0 开始。
- 输出选择器保存上一拍 `u_selected`，用独立的“虚拟 roll 增量 slew”在若干拍内从新输出回到实时 legacy，而不是冻结旧 legacy。限速值必须由台架翼角速率反推。
- 如果 legacy 自身无效，则回退目标不是 legacy，而是 `u_phi_increment=0`（保留已验证 trim，禁止 NaN）。若 allocator/context 失效，发送零 hydro roll 增量，并置 fault latch。
- 故障恢复必须满足连续健康 N 拍；不自动重入 mode 2/3/4，保持 mode 0 直到操作者重新选择。

## 4. 分配器共享接口

### 4.1 重构目标

将 `HydroAllocator::func/J_func/optim`（`HydroAllocator.cpp:130-191`）抽到无状态库，例如：

```text
src/lib/hydrofoil_allocation/HydrofoilAllocationModel.hpp/.cpp
```

公共类型：`Context`、`VirtualControl`、`Solution`、`Limits`、`Status`、`Linearization`。公共方法：

- `solve(context,u)`：得到左右角/推力、残差、迭代数、饱和 mask、valid；
- `forward(context,actuators)`：由左右状态算有效力及 `u_phi^e`；
- `linearize(context,solution)`：复用解析 Jacobian 得 `g_L/g_R` 与局部可行输入区间；
- `checkCandidate(context,candidate_u_phi)`：纯查询，不发 topic、不改变对象状态。

`HydroAllocator` 与 safe-roll 都依赖该库。保留当前 solver 的回归测试向量，先证明重构前后 mode 0 请求一致，再使用新状态字段。不要在 controller 复制升力公式或 Newton 循环。

### 4.2 可实现区间

用当前 solution 的 `J_i` 与 roll force column算 `g_i=-[1,0]J_i^{-1}a_i`；结合角度余量、每拍角速率余量、推力余量得到各侧对 `Delta u_phi` 的线性区间，取交集。最终候选仍以一次完整 solver 验证 residual/limits。Jacobian 奇异、`|g|` 太小或线性预测越界时缩到零增量/legacy，不外推。

## 5. 新 uORB 状态消息

新增 `msg/HaarSafeRollStatus.msg`（生成 topic `haar_safe_roll_status`），并加入 `msg/CMakeLists.txt`。建议字段：

```text
uint64 timestamp
uint64 timestamp_sample
uint8 requested_mode
uint8 active_mode
uint32 reset_count
uint32 fault_flags
bool healthy
bool fallback_active
float32 dt
float32 phi
float32 p
float32 phi_sp
float32 u_legacy
float32 u_nominal
float32 u_barrier
float32 u_selected
float32 u_effective
float32 u_feasible_min
float32 u_feasible_max
float32 delta_l_req
float32 delta_r_req
float32 thrust_l_req
float32 thrust_r_req
float32 g_l
float32 g_r
float32 barrier_h
float32 barrier_hdot
float32 model_bound
float32 solver_residual
uint16 solver_iterations
uint32 cycle_time_us
```

fault bits至少覆盖 stale gyro/attitude、bad dt、estimator reset、nonfinite、allocator invalid/unsaturated assumption failed、Jacobian singular、solver nonconvergence、deadline miss、RC loss、mode rejected。消息必须每拍设置真实 timestamp；logger 在 `logged_topics.cpp:46-65` 附近加入，400 Hz 全量记录需先评估 SD 带宽，可按 2.5–10 ms 间隔记录核心字段。

## 6. 实时性与验证门

1. 在 `SafeRollController::update()` 单独加 `PC_ELAPSED`，并保留模块 cycle；allocator solver再加 solve/iteration/invalid counters。现有位置见 `HydroRateControl.cpp:45-56,131,536` 和 `HydroAllocator.cpp:41-44,203,648`。
2. 目标板 400 Hz：周期 2500 us；建议新 controller 正常 <250 us、WCET <500 us，整链 WCET <1500 us。这是验收门，不是当前实测结论。
3. 单元测试：mode 0 数值同一性；solver forward/inverse；解析 sensitivity 对中心有限差分；饱和、奇异、NaN、timestamp jump、RC loss、mode transition。
4. SITL/HIL：记录每种 fault 注入后的首拍输出、最大跳变量、回退时间、deadline miss。
5. 断桨台架：确认 MAIN11=右/MAIN12=左、正方向、REV、实际角度曲线/速率/死区；据此设置机械与 slew 限制。
6. 系留水池按 mode 0→1→2→3；mode 4 仅单独试验许可。任何阶段 mode 0 回归不一致即停止。
7. 每次试验前保存并核对 `HY_FOIL_AUX/HY_FOIL_THR/HY_ALLOCATE_MODE`；当前历史 params 不含这些后加字段，而默认 gate 会关闭水翼（`hydro_allocator/module.yaml:201-245`）。

## 7. 最小修改文件清单（下一阶段，当前未修改）

必需：

- `src/modules/hydro_rate_control/HydroRateControl.cpp`
- `src/modules/hydro_rate_control/HydroRateControl.hpp`
- `src/modules/hydro_rate_control/hydro_rate_control_params.c`
- `src/modules/hydro_rate_control/CMakeLists.txt`
- `src/modules/hydro_rate_control/SafeRollController.hpp/.cpp`（新）
- `src/lib/hydrofoil_allocation/HydrofoilAllocationModel.hpp/.cpp`（新共享模型）
- `src/lib/hydrofoil_allocation/CMakeLists.txt`（新）
- `src/modules/hydro_allocator/HydroAllocator.cpp/.hpp`（改为调用共享模型并发布状态）
- `src/modules/hydro_allocator/CMakeLists.txt`
- `msg/HaarSafeRollStatus.msg`（新）
- `msg/CMakeLists.txt`
- `src/modules/logger/logged_topics.cpp`

建议但应独立评审：

- `src/modules/control_allocator/ControlAllocator.cpp/.hpp`：为 hydro topic 加 age/valid gate，明确 generic+hydro 叠加策略；
- `src/modules/hydro_allocator/module.yaml`：机械角、速率、solver residual/iteration 等参数；
- `boards/matek/h743/dummy.px4board` 仅在新增库/测试需要显式 Kconfig 时改；启动脚本原则上无需改，因为仍在现模块内。

## 8. 上线前阻断项

- **UNKNOWN** 物理左右接线、角度正方向与机械限位未签字确认；
- **UNKNOWN** 当前 HEAD 的 clean `matek_h743_dummy` 编译与目标板 WCET；
- **UNKNOWN** 实物验证 CADRC 的精确 firmware hash/参数包；
- **VERIFIED 当前缺陷** 无水翼反馈，故“actual angle”只能估计；安全控制不得把 PWM command 当测量真值；
- **VERIFIED 当前缺陷** allocator solver 不报告失败，generic 与 hydro 输出相加且可能饱和。

这些阻断项关闭前，只允许 mode 0 和断桨/系留测试；参数默认始终保持 mode 0。
