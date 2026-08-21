# HAAR SafeRoll 持续 HydroAllocator 不可行根因审计

日期：2026-07-24  
范围：根因定位、watch 扩展、状态机审计和最小修复建议。  
限制：没有放宽 81 拍阈值、PHIS、DMAX、DRATE 或 residual tolerance；没有屏蔽
`AllocatorInvalidFault`；没有执行 flash 或实机测试。

## 1. 结论

### 1.1 已确认不是 SafeRoll 姿态误差本身

Mode 0 已经出现长时间 external HydroAllocator invalid，而 Mode 0 不运行 SafeRoll candidate
数学。这证明持续 invalid 的起点在 `HydroAllocator` 输入 context/command 或 nonlinear
solver，不是 HOCBF、certificate、PHIS 或 Mode 1 tracking error。

相邻监视帧中：

| 字段 | 173996123 us | 174097387 us | 变化 |
|---|---:|---:|---:|
| phi | 1.8006 deg | 1.7327 deg | -0.0679 deg |
| p | 0.0302 deg/s | 1.4825 deg/s | +1.4523 deg/s |
| u_selected | -0.419621 | -0.419621 | 0 |
| valid | true | false | 翻转 |
| residual | 0.004512 | 3.517896 | +3.513384 |
| mask | 0x02 | 0x1b | 多个约束新增 |

phi 和 `u_selected` 不能解释该跳变。没有显示的 `force_x`、`force_z`、available thrust、
body pitch、speed²、`u_theta/u_psi` 至少有一项发生变化。

### 1.2 源码中发现的首要条件性根因

`HydroAllocator::Run()` 每拍创建零初始化的局部 context：

```cpp
hydrofoil::AllocationContext allocation_context{};
```

实速分支仅在 `debug_vect` 本拍有新消息时赋值：

```cpp
if (_debug_vect_sub.update(&debug_vect_msg)) {
    ...
    allocation_context.speed_squared = real_v2;
}
```

源码：`src/modules/hydro_allocator/HydroAllocator.cpp:251-274`。

因此当 `HY_SPEED_SELECT != 0` 时：

- 有新 `debug_vect` 的回调：`speed_squared=max(debug_vect.z,1)`；
- 没有新消息的绝大多数 400 Hz 回调：`speed_squared=0`；
- context 不缓存上一有效速度；
- `finiteContext()` 允许 `speed_squared==0`，所以不会报 InvalidInput，而是继续 Newton；
- 水翼升力项瞬间消失，solver 只能依靠受限推进器/翼角匹配原 force target，容易得到
  0x1a/0x1b 和大 residual；
- 下一次速度消息恰好到达时又可能突然 valid。

这与“姿态和 u 几乎不变，持续 invalid 后突然恢复”高度一致。

**目标板确认条件：**

```text
param show HY_SPEED_SELECT
hydro_rate_control safe_roll_watch full 10
```

若 `HY_SPEED_SELECT=1` 且 full watch 的 `speed_squared` 在 0 与非零之间跳变，该根因即
**VERIFIED ON TARGET**。

参数元数据目前把 1 标为 `Measurement speed (Unsupported)`、默认 0，见
`src/modules/hydro_allocator/module.yaml:83-91`。当前证据没有提供目标板该参数值，因此
本报告把它标为“源码已验证、目标配置待确认”，而不是虚构成已经读取的实机参数。

### 1.3 若 HY_SPEED_SELECT=0

固定替代速度分支每拍写入 `HY_ALT_SPEED²`，上述零速度问题不生效。此时最可能的实际根因
依次是：

1. depth/thrust controller 的 `command_force_z` 改变；
2. manual throttle 改变 `available_thruster_thrust`；
3. body pitch 改变；
4. `command_force_x/u_theta/u_psi` 改变；
5. 这些量的组合使左右一侧要求负推力、另一侧要求超过 available thrust。

必须用新版 ULog 或 full watch 才能给出数值排序。

## 2. 新版 ULog 可用性

工作区内所有 `.ulg` 均已搜索；没有任何文件包含：

```text
haar_safe_roll_status
haar_safe_roll_estimator_status
```

现有“已有ulg”目录是旧 CADRC 消息格式。它没有新版 allocator context、valid、
residual、iterations 或 saturation mask。

因此本轮无法诚实完成：

- 首次持续 invalid 前后 ±1 s 每拍 context 提取；
- 相邻拍 context 最大变化量的数值排序；
- 使用该真实窗口做四种 initial guess 的逐拍重放。

这是数据缺失，不是通过旧日志猜测可以补足的。将新版 ULog 放入工作区后，可直接按
`timestamp_sample` 对齐 `hydro_allocate_message`、`haar_safe_roll_status` 和 estimator
status 完成上述重放。

## 3. Solver 初值审计

### 3.1 当前运行 solver 没有 warm start

`HydrofoilAllocationModel::solve()` 是无状态函数。每拍左右初值固定为：

```cpp
Vector2f left(0.f, command.force_x * 0.5f);
Vector2f right(0.f, command.force_x * 0.5f);
```

见 `HydrofoilAllocationModel.cpp:149-192`。

这意味着：

- delta 每拍 cold start 为 0；
- thrust 每拍 cold start 为 `force_x/2`；
- 不读取上一拍解；
- 没有“当前 warm start”；
- 没有上一有效解 warm start；
- 没有镜像初值；
- SafeRoll 的 `solve/checkCandidate()` 查询也使用同一固定 cold start。

所以在完全相同的 float input/context 下，当前运行函数是确定性的，不能因内部历史状态
在 valid/invalid 间随机切换。实机翻转必然来自输入/context 变化，或调用前未正确缓存的
输入（`speed_squared` 正是一个源码实例）。

四初值比较仍有研究价值，但需要在离线工具中显式增加 initial-state 参数；不能把当前
runtime solver 称为 warm-start solver。

### 3.2 residual=0.004512 仍 valid 的原因

`solveOneSide()` 有两种 converged 判定：

1. Newton step norm² `<1e-6` 时立即 `converged=true`；
2. 最终 residual `<=1e-3`。

见 `HydrofoilAllocationModel.cpp:92-145`。因此 step 足够小时，即使最终 residual
大于 `1e-3`，status 仍可 valid。这解释了实机 `valid=true,residual=0.004512`，不代表
logger 或监视命令矛盾。

本轮没有改变该语义。后续应分别记录 `step_converged` 与 `residual_accepted`，而不是简单
放宽 residual tolerance。

## 4. 0x1a / 0x1b 精确解码

定义见 `HydrofoilAllocationModel.hpp:24-32`：

| Bit | Hex | 含义 |
|---:|---:|---|
| 0 | 0x01 | DeltaLeft |
| 1 | 0x02 | DeltaRight |
| 2 | 0x04 | ThrustLeftLow |
| 3 | 0x08 | ThrustLeftHigh |
| 4 | 0x10 | ThrustRightLow |
| 5 | 0x20 | ThrustRightHigh |
| 6 | 0x40 | ForceXRedistributed |

因此：

```text
0x1a = 0x02 + 0x08 + 0x10
     = DeltaRight + ThrustLeftHigh + ThrustRightLow

0x1b = 0x01 + 0x02 + 0x08 + 0x10
     = DeltaLeft + DeltaRight + ThrustLeftHigh + ThrustRightLow
```

0x1b 表示两侧翼角都曾越界、左推进器需要超过 available 上界、右推进器需要小于 0。
这不是单一 DeltaRight 问题，而是左右目标力与有限推力/翼角组合发生强冲突。

## 5. residual 的实际定义与分解

虚拟控制先形成四个目标力：

```text
[Fx_L, Fz_L, Fx_R, Fz_R]
 =
M * [u_phi, u_theta, u_psi]
 +
[force_x/2, force_z/2, force_x/2, force_z/2]
```

见 `HydrofoilAllocationModel.cpp:159-163`。

每侧 residual 是二维力残差：

```text
r_x = Fx_target
      - KL*v²*(delta+pitch)*sin(pitch)
      - T*cos(delta)

r_z = Fz_target
      + KL*v²*(delta+pitch)*cos(pitch)
      + T*sin(delta)
```

见 `HydrofoilAllocationModel.cpp:49-62`。

最终发布的 scalar residual 是：

```text
max(norm([rL_x,rL_z]), norm([rR_x,rR_z]))
```

见 `HydrofoilAllocationModel.cpp:194-205`。

当前接口不存在独立的“roll residual / pitch residual / yaw residual”。roll、pitch、yaw
只是通过矩阵叠加到四个 target force；非线性求解后不能从一个 scalar residual 唯一反解
各虚拟控制分量的责任。

正确的离线分解方法是：

1. 保存完整 M、force_x/z、u_phi/theta/psi；
2. 分别计算每个命令分量对四个 target force 的线性贡献；
3. 重放得到左右 `r_x/r_z`；
4. 做 leave-one-component-out counterfactual，而不是把 scalar residual 随意分摊。

## 6. 35 deg 限制下是否本来不可行

从 0x1a/0x1b 可以确认 constrained Newton 已同时碰到多个物理边界，但仅凭 mask 不能严格
证明全局无解；Newton 仍可能受初值和局部 Jacobian 影响。

严格判定每侧应检查：

```text
delta in [-35,+35] deg
T in [0, available_thruster_thrust]
```

下上述两维方程是否存在解。需要的缺失量是：

- KL、speed²、body pitch；
- available thrust；
- 左右 target Fx/Fz。

扩展后的 full watch 已包含所有构造这些量所需的字段。若观测到
`speed_squared=0`，水翼项完全消失，此时大负 `force_z` 与受限 thrust 的组合很容易物理
不可行。

## 7. last_feasible / recovery 审计

### 7.1 Stage A 进入与退出

- 已激活 Modes 1/2/3 遇到 `valid=false || converged=false` 时进入；
- 第 1～20 拍 `hold_last_feasible=true`；
- 第 21～80 拍 temporary legacy；
- 连续健康 20 拍后才退出 recovery 并调用 `beginBlend()`。

源码：`SafeRollController.cpp:1111-1179`。

### 7.2 last_feasible 更新

只有候选完成全部 nonlinear check、实际成为 selected output、且不处于 degraded/temp
状态时才更新，见 `SafeRollController.cpp:1524-1529`。

单个偶发 healthy allocator sample：

- 会把 invalid streak 清零；
- 只把 recovery streak 加 1；
- 不执行 candidate；
- 不更新 last feasible；
- 下一拍再次 invalid 时 recovery streak 又清零。

因此如果 external allocator 始终不能连续健康 20 拍，`u_selected` 长时间保持旧值是当前
状态机的预期结果，不是 U_rate 自己锁死。实机从 +15 deg 越过到 -9.6 deg 仍保持约
-0.42，说明 candidate 很可能一直没有获得连续 20 拍执行窗口。

### 7.3 真正恢复后的行为

第 20 个连续健康样本：

- 清除 degraded/temp；
- `beginBlend()` 从旧 selected output 初始化；
- 同拍计算新 candidate；
- blend 的首拍 lambda=0，所以输出仍等于旧值；
- 下一拍开始按 blend 和 DUMAX 朝新 candidate 移动。

新增回归：

```text
RecoveredNominalMovesAwayFromOldLastFeasibleAfterErrorReverses
```

位置：`SafeRollControllerTest.cpp:995`。

测试先建立负 last-feasible，invalid 进入 temporary fallback，再将 phi 反向；恢复连续
20 拍后确认 `u_nominal>0`，下一拍 `u_selected` 离开旧负值，且单拍变化不超过 DUMAX。
测试通过。

结论：没有发现“恢复后 U_rate 永久围绕旧符号锁死”的代码错误；真正问题是 allocator
输入抖动使 recovery streak 无法达到 20。

## 8. fallback latch 清除路径

`_fallback_latched=false` 只有两类函数：

1. `SafeRollController::reset()`，`SafeRollController.cpp:97-130`；
2. `resetNewController()`，`:132-179`。

### 8.1 显式模块 reset

HydroRateControl 在 STAB entry 或 exit 调用
`_safe_roll_controller.reset()`，见 `HydroRateControl.cpp:187-204`。

### 8.2 update 内 transition reset

以下任一变化都会调用 `resetNewController(input,true)`，并清除 latch：

- controller 尚未 initialized；
- requested mode 变化；
- `HY_SR_ENABLE` 变化；
- `HY_SR_RST_ID` 变化；
- attitude `quat_reset_counter` 变化；
- armed 状态变化；
- in-STAB 状态变化；
- reset-sensitive configuration CRC 变化。

条件见 `SafeRollController.cpp:956-975`。reset-sensitive CRC 包含 A0/B0/BMIN/BMAX、
PHIS、RD0、DDB/DBB、increment thresholds、RB0、DMAX/DRATE/DREM 和 reset ID，见
`:293-312`。

`HY_SR_SHADOW` 变化只重新 blend，不清 latch。

所以：

```text
t=174302369 latch=1
t=174709911 requested_code仍为1但latch=0
```

并不要求 requested mode 必须改变。attitude reset、arm/STAB edge、ENABLE、RST_ID 或
reset-sensitive 参数变化任一都能清锁存。

如果实机确认这些外部状态均没有变化，则这是意外自动清锁存。现实现也违反此前“失败后
除非 mode 参数再次变化或 RST_ID 变化，不自动重新进入”的要求。

最小修复应把“reset estimator/reference/history”与“clear fatal latch”拆开：

- attitude reset、arm/STAB edge、内部 config reset：重置新控制器状态但保留 latch；
- 仅 requested mode 明确变化或 `HY_SR_RST_ID` 变化允许清 latch；
- 记录 latch-clear reason 数字码；
- `reset_count`、configuration CRC、arming/nav state 和 quat reset counter 已加入 full
  watch，便于确认本次是哪条路径。

本轮没有修改 latch 逻辑。

## 9. safe_roll_watch 扩展

实现位于 `HydroRateControl.cpp:888-1022`。没有新增 uORB topic；shell command context
独立订阅已有：

- `haar_safe_roll_status`
- `haar_safe_roll_estimator_status`
- `hydro_allocate_message`
- `vehicle_status`
- `vehicle_attitude`

命令：

```text
hydro_rate_control safe_roll_watch
hydro_rate_control safe_roll_watch 10
hydro_rate_control safe_roll_watch compact 10
hydro_rate_control safe_roll_watch full 10
```

默认 compact、5 Hz；允许 1～20 Hz；Ctrl+C 结束。

compact 保持原 16 列：

```text
timestamp requested_code active_code phi_deg p_deg_s u_nominal u_selected
allocator_valid allocator_converged allocator_residual allocator_saturation_mask
allocator_invalid_streak allocation_interval_degraded temporary_fallback_active
first_fault_code fallback_latched
```

full 在后面追加：

```text
allocator_timestamp
command_force_x command_force_z available_thruster_thrust body_pitch_deg
lift_coefficient speed_squared context_max_hydrofoil_angle_deg
command_u_phi command_u_theta command_u_psi
thrust_l_commanded thrust_r_commanded
delta_l_commanded_deg delta_r_commanded_deg
allocator_iterations allocator_solve_status
last_feasible_u candidate_valid allocator_degraded_active allocator_recovery_streak
reset_count configuration_crc32 arming_state nav_state quat_reset_counter
```

`allocator_solve_status` 数字定义：

```text
0 Success
1 InvalidInput
2 JacobianSingular
3 NonFinite
4 NotConverged
```

## 10. 最小修复建议

### P0：修复实速 context 缓存，不改 allocator 数学

仅在目标确认 `HY_SPEED_SELECT=1` 后实施：

1. 将最后有效 `speed_squared` 和 timestamp 保存为 HydroAllocator 成员；
2. 有新 finite `debug_vect.z` 时更新；
3. 两条消息之间复用缓存，而不是恢复到 0；
4. 初始化使用 `HY_ALT_SPEED²`；
5. 超过明确 freshness 阈值时设置独立 speed-context invalid/stale 状态；
6. 不静默用 0，不修改 Newton tolerance。

需增加回归：

- 400 Hz allocator、10 Hz speed topic，中间 39 拍 speed²保持上一值；
- 同一 context/command 每拍得到完全相同 status；
- speed stale 时产生明确 context fault，不产生周期性 valid/invalid。

### P0：分离 latch reset 与 estimator reset

保留所有必要的 estimator/reference reset，但 fatal latch 只允许 mode change 或 RST_ID
显式清除。增加 `latch_clear_reason` 数字日志。

### P1：拿到新版 ULog 后做精确重放

对首次 invalid ±1 s：

1. 用原始 cold start逐拍重放；
2. 用 delta/thrust 当前解、上一有效解、镜像解重放；
3. 比较同一 input 的 status/residual；
4. 计算左右 `r_x/r_z`；
5. 对 force_x/z、u_phi/theta/psi 分量做 counterfactual；
6. 用有界网格或多初值确认是局部收敛问题还是 35 deg/available thrust 下真实无解。

如果 cold/warm 对同一输入结果不同，再考虑显式、无副作用的 warm-start API；在修复
context 缺陷前引入 warm start只会掩盖输入抖动。

## 11. 验证结果

- SafeRollController：45/45 PASS；
- HydrofoilAllocationModel：10/10 PASS；
- 总计：55/55 PASS；
- 新增“误差反向后恢复输出离开旧 last-feasible”回归通过；
- H743 增量 `-Werror` 编译、链接和 PX4 打包通过；
- FLASH：1,587,152 bytes / 1,966,080 bytes，80.73%；
- AXI SRAM：44,716 / 524,288 bytes，8.53%。

生成固件仅用于编译验证，本轮未刷写：

```text
/home/linyi666/PX4-Autopilot-water-air/build/matek_h743_dummy/matek_h743_dummy.px4
size: 1,525,105 bytes
SHA256: 1f0d8153e206a6e13c3dee00822b72f4d7e913dbdc041b4e0fdd9156cebc51b3
```

## 12. 停止点

本轮已完成源码根因定位、watch 扩展、recovery/latch 审计和回归验证。没有增加 81 拍
阈值，没有放宽任何边界或 residual，没有修改 HydroAllocator 数学，也没有执行 flash。

精确相邻拍 context 差异和四初值重放仍需要用户把本次新版 ULog 放入工作区。当前最优先
的现场确认是：

```text
param show HY_SPEED_SELECT
hydro_rate_control safe_roll_watch full 10
```

观察 `speed_squared` 是否在 0/非零间跳变；若不是，再比较 `command_force_z` 和
`available_thruster_thrust`。
