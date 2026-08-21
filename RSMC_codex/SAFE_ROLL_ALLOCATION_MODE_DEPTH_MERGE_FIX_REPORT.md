# HAAR SafeRoll：HY_ALLOCATE_MODE 深度共同分量修复报告

日期：2026-07-27  
基线 Git HEAD：`5ae2fd95ae`（工作区包含此前 Phase 1/1.5/Final 的未提交修改）  
结论状态：**软件端到端路径、单元测试和 H743 clean build 已验证；未执行 flash、actuator test 或实机测试。**

## 1. 根因

### VERIFIED：HY_ALLOCATE_MODE 的历史定义

参数元数据位于 `src/modules/hydro_allocator/module.yaml:236-251`。

| 值 | 历史名称 | HEAD 5ae2fd95 的真实输出语义 |
|---:|---|---|
| 0 | Normal Mode | 发布 nonlinear allocator 的完整左右翼角和左右推力 |
| 1 | Fixed Mode | 左右翼使用 `HY_SV_L_FM/HY_SV_R_FM` 固定角，推力使用分配后的左右纵向力 |
| 2 | Manual Mode | 水翼共同分量来自 RC pitch，差动来自 `u_phi * HY_ALCT_ROL_MAN`；推进器共同分量来自 RC throttle，差动来自 yaw |

Mode 1 由提交 `18ab8b0b8d` 引入，用于“固定舵机角度的对比实验”。Mode 2
由提交 `d4142b3ecc` 引入，提交说明为“新增控制分配的手动模式，用于特殊情景”。

因此，历史 Mode 2 **不是闭环深度分配模式**。HEAD 5ae2fd95
`HydroAllocator::Run()` 的 Mode 2 分支位于历史文件第 610–618 行：

```text
motor L/R = manual throttle ± yaw manual delta
servo L/R = manual pitch ± roll manual delta
```

该分支不读取 nonlinear solver 已求得的 `delta_left/right`，也不读取其完整
`thrust_left/right`。

### VERIFIED：当前故障的第一源码断点

修复前流程为：

```text
force_z/u_phi/u_theta/u_psi
  -> HydrofoilAllocationModel::solve()
  -> selectExecutable() 得到完整 finite+bounded best-feasible
  -> x 保存完整左右翼角/推力
  -> HY_ALLOCATE_MODE == 2
  -> 丢弃 x
  -> 用 RC pitch ± u_phi*HY_ALCT_ROL_MAN 重建水翼输出
```

第一断点就是修复前 `HydroAllocator::Run()` 的 allocation-mode 发布分支。
它位于当前修复后的对应区域
`src/modules/hydro_allocator/HydroAllocator.cpp:658-746`；旧分支原本在该区域
直接按 Mode 0/1/2 分别写 `hydro_servos` 和 `hydro_motors`。

这解释了实机 A/B：

- `HY_ALLOCATE_MODE=0`：完整 executable solution 被发布，
  `realized_symmetric=±0.61086 rad`；
- `HY_ALLOCATE_MODE=2`：完整解被手动分支覆盖，共同分量取 RC pitch；当 RC
  pitch 为零时 `realized_symmetric=0`。

`full_best_feasible_solution_used=false` 的直接原因是旧诊断只在
`executable.best_feasible && foil_gate_enabled && allocate_mode == 0`
时置位。Mode 2 即使存在完整可执行 best-feasible，也被硬编码排除。

### requested differential 为零、realized differential 非零

`requested_differential_foil_component` 来自 nonlinear solver candidate：

```text
(allocation_solution.delta_left - allocation_solution.delta_right) / 2
```

在两个翼角都被共同深度请求推到同一侧极限时，该值可以为零。

旧 Mode 2 的 realized differential 来自另一条公式：

```text
man_delta_servo = allocation_command.u_phi * HY_ALCT_ROL_MAN
servo L/R = manual_pitch ± man_delta_servo
```

源码对应当前 `HydroAllocator.cpp:658-661`。因此实机中的
`requested differential=0` 与 `realized differential=-0.12867` 不是
solver 自己改变了解，而是 Mode 2 使用了不同的输出源。

### Mode 1 是否同样受影响

是。修复前 Mode 1 用固定左右翼角替换完整解，也会丢弃闭环深度共同分量。
只是本次实机正式保存值为 Mode 2，所以 Mode 2 先暴露。

SafeRoll codes 1/2/3 也受同一问题影响：旧 allocation-mode 分支位于
HydroAllocator 下游，只检查 `HY_ALLOCATE_MODE`，不检查 SafeRoll
控制器的实际 code，因此所有 code 都可能被 Mode 1/2 分支覆盖。

## 2. suppression_reason 与 saturation mask

### suppression_reason = 5

精确定义是：

```text
SUPPRESSION_ALLOCATION_SATURATION = 5
```

定义：`msg/HaarDepthControlStatus.msg:4-11`。  
设置位置：`ControlAllocator::Run()`，
`src/modules/control_allocator/ControlAllocator.cpp:1129-1131`。  
完整条件是：

```text
前面的 controller-disabled、nonfinite、foil-gate、
allocator-output-suppressed、bounded-best-feasible 条件均未命中
AND allocator_saturation_mask != SaturationNone
```

它表示“受分配饱和限制”，不是“深度共同分量已保留”。旧诊断没有描述
allocation-mode 分支覆盖，所以会同时出现：

```text
realized symmetric = 0
depth_command_suppressed = false
depth_degradation_active = false
allocator_output_suppressed = false
suppression_reason = 5
```

这是诊断缺口，不是这些布尔量证明输出正常。

### saturation_mask 43 / 107 / 59

位定义位于
`src/lib/hydrofoil_allocation/HydrofoilAllocationModel.hpp:24-33`。

| 十进制 | 十六进制 | 解码 |
|---:|---:|---|
| 43 | `0x2b` | DeltaLeft、DeltaRight、ThrustLeftHigh、ThrustRightHigh |
| 107 | `0x6b` | `0x2b` 的全部位，再加 ForceXRedistributed |
| 59 | `0x3b` | DeltaLeft、DeltaRight、ThrustLeftHigh、ThrustRightLow、ThrustRightHigh |

mask 是 Newton 迭代期间出现过的饱和位的累积 OR。因而同一侧推力 low/high
可以同时出现，表示不同迭代步分别越过了上下界，不表示最终推力同时处于两个
边界。

`command_force_z` 约 ±15 至 ±80 N 时共同角精确为
`±0.61086 rad`，是因为：

- `HY_WING_MAX_A=35 deg`；
- `35*pi/180 = 0.610865... rad`；
- `solveOneSide()` 在每次 Newton 更新后于
  `HydrofoilAllocationModel.cpp:122-137` 将左右角分别限制到
  `[-max_hydrofoil_angle,+max_hydrofoil_angle]`；
- 岸上 `HY_SPEED_SELECT=0` 且无推进器输出时，大 force-z 请求超过当前
  水动力/推力上下文的严格可实现能力，最终得到有限、有界、但
  NotConverged 的边界解。

本轮没有放宽 residual tolerance，也没有伪造 `allocator_valid=true`。

## 3. 历史路径与当前路径差异

### HEAD 5ae2fd95

HEAD 已包含三个输出分支：

- Normal 发布 nonlinear `x`；
- Fixed 覆盖左右翼为固定参数；
- Manual 覆盖为 RC pitch/roll 与 throttle/yaw 的直接组合。

因此 Mode 2 丢失闭环深度并非 Phase 1 新引入；它是历史特殊实验模式的设计
语义。用户历史岸上按压压力计能获得共同动作，与 Normal 分支或当时实际使用
的非 Manual 输出配置一致。保存参数后来为 2，才会让当前正式配置进入特殊
手动分支。

### Phase 1 共享模型重构

Phase 1 将方程抽取到 `HydrofoilAllocationModel`，但保留了历史
allocation-mode 发布分支。solver 与 Mode 0 回归等价，但 Mode 1/2 仍会在
solver 之后覆盖完整解。

### bounded best-feasible 修复

`selectExecutable()` 现位于
`HydrofoilAllocationModel.cpp:366-405`：

- finite + bounded + NotConverged，在 SafeRoll codes 0–3 可选为完整
  best-feasible；
- 左右翼角各自按 `HY_SR_DRATE` 限速；
- thrust_left/right 原样保留；
- NaN/Inf、JacobianSingular、InvalidInput、hard-unbounded 仍不可执行；
- code 4 保持严格禁止 NotConverged。

此前该修复只解决了“完整解是否可执行”，没有解决 Mode 1/2 在之后重新覆盖
完整解的问题。

### 本轮修复后

新增统一选择函数
`HydrofoilAllocationModel::selectAllocationModeOutput()`：

- 声明：`HydrofoilAllocationModel.hpp:88-100,155-158`；
- 实现：`HydrofoilAllocationModel.cpp:407-446`。

输出语义：

```text
SafeRoll code 0/1/2/3:
    selected_solution = complete executable nonlinear solution
    （HY_ALLOCATE_MODE 0/1/2 均不得覆盖）

SafeRoll code 4:
    HY_ALLOCATE_MODE=0 -> complete executable
    HY_ALLOCATE_MODE=1 -> legacy fixed comparison
    HY_ALLOCATE_MODE=2 -> legacy manual comparison
    NotConverged -> strict suppression
```

HydroAllocator 每拍仍只求解一次完整 source command：

```text
force_x, force_z, selected u_phi, u_theta, u_psi
    -> solve()
    -> selectExecutable()
    -> selectAllocationModeOutput()
    -> normalize complete selected solution
    -> hydro_servos/hydro_motors
```

关键位置：

- 完整 command 和 solve：`HydroAllocator.cpp:284-305`；
- finite/bounded best-feasible 与独立左右速率限制：
  `HydroAllocator.cpp:296-344`；
- legacy special solutions 的显式构造：
  `HydroAllocator.cpp:658-687`；
- 统一 output-source 选择：`HydroAllocator.cpp:689-702`；
- gate 后发布完整左右翼角和推力：`HydroAllocator.cpp:733-746`；
- published solution 重建：`HydroAllocator.cpp:765-790`；
- best-feasible 采用状态：`HydroAllocator.cpp:837-850`。

这比在最终 servo 层拼接 symmetric/differential 更一致：SafeRoll 已在上游只
替换 `u_phi`，其余 `force_z/u_theta/u_psi/force_x` 保持在同一个 source
command 中，因此一次完整 nonlinear solve 的左右翼角与左右推力彼此一致。

数学上：

```text
delta_symmetric = (delta_left + delta_right)/2
delta_differential = (delta_left - delta_right)/2

delta_left  = delta_symmetric + delta_differential
delta_right = delta_symmetric - delta_differential
```

本轮不改变符号、REV、逻辑左右索引、generic+hydro 合成或 PWM 映射。

## 4. 诊断修复

扩展 `msg/HaarDepthAllocatorStatus.msg:4-55`，新增：

- `allocate_mode`；
- `executable_solution_selected`；
- `selected_solution_source`：
  0 suppressed、1 complete executable、2 legacy fixed、3 legacy manual；
- `mode_branch_overrode_solution`；
- branch 前后的 symmetric/differential；
- selected thrust left/right；
- published hydro left/right；
- `symmetric_depth_lost`；
- `symmetric_loss_reason`：
  none、rate-limit transient、mode-branch override、foil gate、allocator
  suppression。

诊断分层现在是：

```text
solver candidate
  -> requested symmetric/differential
selectExecutable
  -> mode_branch_*_before
allocation-mode selection
  -> mode_branch_*_after + selected_solution_source
gate/publication
  -> published_hydro_left/right + realized components
ControlAllocator
  -> existing generic/hydro/final commanded diagnostics
```

丢失判断比较的是 rate-limit **之后**的 executable solution 与 gate 后实际
发布值，所以实机所示“左翼按单拍速率限制、右翼保持”的正常瞬态不会被误报为
mode-branch symmetric loss。

QGC 参数元数据也已更新并在生成的
`build/matek_h743_dummy/parameters.xml:8244-8252` 验证：
codes 0–3 始终保留完整 nonlinear allocation，Fixed/Manual 仅为 code 4
严格对比支路。

## 5. 修改文件

本轮修改：

1. `src/lib/hydrofoil_allocation/HydrofoilAllocationModel.hpp`
   - 增加输出源枚举和无状态 mode-output 选择接口。
2. `src/lib/hydrofoil_allocation/HydrofoilAllocationModel.cpp`
   - 实现 codes 0–3 完整解选择、code 4 严格特殊分支。
3. `src/lib/hydrofoil_allocation/HydrofoilAllocationModelTest.cpp`
   - 增加 allocation-mode/SafeRoll-mode 矩阵、真实 solve 的深度+滚转合成、
     Mode 4 及实机数值速率限制回归。
4. `src/modules/hydro_allocator/HydroAllocator.cpp`
   - 删除 codes 0–3 对完整解的 Mode 1/2 发布覆盖；
   - 从一个物理 selected solution 同时发布左右翼和左右推力；
   - 修正 best-feasible 采用状态与分层诊断。
5. `src/modules/hydro_allocator/module.yaml`
   - 澄清三个参数值以及其与 SafeRoll code 的关系。
6. `msg/HaarDepthAllocatorStatus.msg`
   - 增加 output-source、branch 前后、published solution 和 symmetric-loss
     数字诊断。

未修改：

- `CADRC.cpp/.h/.hpp`；
- 深度控制器数学和增益；
- `HY_DC_SP`、`HY_DC_FF` 默认值；
- SafeRoll HOCBF、estimator、certificate；
- 左右翼映射、REV、trim；
- generic+hydro 下游叠加；
- residual tolerance。

## 6. 测试

### 新增确定性测试

`HydrofoilAllocationModelTest.cpp:324-492`：

1. `AllocationModeMatrixPreservesCompleteDepthForModesZeroThroughThree`
   - 4 个 SafeRoll code × 3 个 allocation mode；
   - finite+bounded+NotConverged；
   - 完整左右翼、symmetric/differential、左右推力全部保持；
   - `status.valid` 仍诚实为 false。
2. `AllocationModeMatrixPreservesSolvedDepthAndRollComposition`
   - 输入真实 nonzero force-z + nonzero u-phi；
   - 12 个组合均保持 solve 后的共同/差动分量和左右推力。
3. `ModeFourRetainsStrictAllocationModeBranches`
   - code 4 保留 Normal/Fixed/Manual 分支；
   - executable 不可用时三个 allocation mode 均禁止输出。
4. `BestFeasibleRateLimitPreservesCompleteVectorAndReconstruction`
   - previous L/R=`0.61086/0.61086`；
   - requested L/R=`0.31533/0.61085`；
   - 左右翼独立限速；
   - symmetric/differential 重建回左右实际值；
   - 推力不被清零。

既有测试继续覆盖：

- NaN/Inf、Jacobian singular、InvalidInput、hard-unbounded 禁止；
- finite+bounded NotConverged 的完整 best-feasible；
- 正负 force-z 的共同分量符号；
- 10 N feed-forward 变化；
- generic+hydro 合成；
- Mode 0 legacy CADRC 数值回归；
- AUX gate、FIELD recovery、Modes 1/2/3 和 Mode 4。

### 结果

```text
HydrofoilAllocationModel: 22/22 PASS
SafeRollController:       63/63 PASS
总计:                     85/85 PASS
git diff --check:          PASS
```

Host 性能参考（不是 H743 WCET）：

```text
allocation solve+forward+linearize average: 315 ns
observed maximum: 9483 ns
SafeRoll candidate average: 2762 ns
observed maximum: 68493 ns
```

## 7. H743 clean -Werror 构建

执行：

```text
ninja -C build/matek_h743_dummy clean
ninja -C build/matek_h743_dummy
```

clean 后第一次启动遇到已有 NuttX 并行生成竞态：
`jlink-nuttx.so` 在 `.config` 生成完成前启动；同一轮随后生成了 `.config`。
在该干净配置状态重跑后完整 1057 项构建通过。编译命令确认包含
`-Wall -Wextra -Werror`。

最终链接：

```text
FLASH used:    1,599,224 bytes / 1,920 KiB (81.34%)
AXI_SRAM used: 44,716 bytes / 512 KiB (8.53%)
```

固件：

```text
路径: /home/linyi666/PX4-Autopilot-water-air/build/matek_h743_dummy/matek_h743_dummy.px4
大小: 1,537,829 bytes
SHA-256: cb376aee888faa6f8ed47fb817f709229fba28fe7a50f126d12729d9b42ed46f
```

## 8. 放行边界

已完成：

- HY_ALLOCATE_MODE=0/1/2 × SafeRoll code 0/1/2/3 的软件选择矩阵；
- finite+bounded NotConverged 完整解、共同/差动合成和左右推力保持；
- 发布路径 H743 编译；
- 数字诊断分层；
- clean `-Werror` 固件生成。

尚未完成且本轮明确未执行：

- flash；
- actuator test；
- 岸上压力计复测；
- 水池/室外实机复测。

因此本报告确认“源码路径和构建产物已修复并通过软件回归”，不宣称目标板物理
响应已经复验。建议实机复测时保持参数不自动迁移，首先只读确认
`selected_solution_source=1`、`mode_branch_overrode_solution=false`，
再比较 solver、executable、published 和 final commanded 四层共同分量。
