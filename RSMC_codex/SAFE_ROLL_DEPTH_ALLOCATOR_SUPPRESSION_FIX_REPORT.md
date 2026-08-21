# HAAR 深度 allocator 抑制故障修复报告

日期：2026-07-24  
源码基线：`5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`  
目标板：`matek_h743_dummy`

## 1. 根因结论

实机诊断已经准确定位到 HydroAllocator 输出选择，而不是深度传感器、
深度控制器或 SafeRoll 的 `force_z` 传递。

修复前路径为：

```text
depth controller
  -> finite command_force_z
  -> nonlinear solver gives finite, bounded, saturated iterate
  -> allocator_valid=false
  -> all four allocator outputs forced to zero
  -> symmetric depth foil component = 0
```

修复前的清零分支位于当时
`src/modules/hydro_allocator/HydroAllocator.cpp:318-324`：

```cpp
if (!allocation_solution.status.valid
    && !(field_output_allowed && allocator_bounded)) {
    x[0][0] = 0.f;
    x[1][0] = 0.f;
    x[0][1] = 0.f;
    x[1][1] = 0.f;
    allocator_output_suppressed = true;
}
```

Mode 0 不满足 `field_output_allowed`，所以即便
`finite=true && bounded=true`，只要 residual 使 `valid=false`，完整左右翼
与左右推进器解都会被清零。这与实机的 `suppression_reason=3`、
requested symmetric 为 ±35°、realized symmetric 为 0 完全对应。

## 2. `suppression_reason=3`

精确定义：

```text
SUPPRESSION_ALLOCATOR_INVALID = 3
```

定义在 `msg/HaarDepthControlStatus.msg:7`。

唯一设置位置是
`src/modules/control_allocator/ControlAllocator.cpp:1119-1122`，完整条件：

```text
hydro_allocate_message.allocator_output_suppressed == true
AND abs(original_depth_controller_output) > 1e-5
```

该字段是诊断分类，不是直接清零执行点。真正清零发生在上述修复前
HydroAllocator 分支；ControlAllocator 随后收到 hydro servo 为零，故
最终共同分量为零。

修复后，有限 bounded `NotConverged` 解使用数字原因：

```text
SUPPRESSION_BOUNDED_BEST_FEASIBLE = 7
depth_command_suppressed = false
depth_command_limited = true
```

见 `HaarDepthControlStatus.msg:11` 和
`ControlAllocator.cpp:1124-1127`。

## 3. requested 与 realized 的准确语义

修复前 `requested_symmetric_foil_component` 来自：

```text
(allocation_solution.delta_left_commanded
 + allocation_solution.delta_right_commanded) / 2
```

其中 `allocation_solution` 是 nonlinear Newton solver 的最终 constrained
iterate，已经经过 solver 内角度/推力约束，但尚未经过 invalid 输出清零、
归一化、foil gate、generic+hydro 合成和最终 servo saturation。

因此它不是未处理的解析力请求，也不是传感器测量，而是
**solver 最终 commanded candidate**。

`realized_symmetric_foil_component` 来自最终 pre-PWM commanded 左右翼：

```text
(final_delta_left + final_delta_right) / 2
```

修复前 HydroAllocator 把左右 hydro 输出清零后，最终仅剩滚转差动，所以
该值为 0。

本轮新增 `haar_depth_allocator_status`，分别记录 solver requested、
HydroAllocator 实际 published、最终 ControlAllocator commanded 三个层级，
避免混淆。

## 4. allocator invalid 的精确原因

源码定义：

```text
valid = finite && converged
```

位于
`src/lib/hydrofoil_allocation/HydrofoilAllocationModel.cpp:140-145`。

实机同时给出：

```text
allocator_valid = false
allocator_bounded = true
```

而 `bounded=true` 的必要条件包含 `solution.status.finite=true`
（`HydrofoilAllocationModel.cpp:346-363`）。因此可以严格推出：

```text
finite = true
converged = false
solve_status = NotConverged
```

不是 hard interval、CBF interval、projection 或 SafeRoll nonlinear
candidate fault；当前样本处于 Mode 0，这些 SafeRoll candidate 阶段没有
参与 HydroAllocator 的 `valid` 判定。

`converged=false` 的直接数值原因是：Newton 最多执行 10 次，翼角和推力在
每次迭代中被硬约束；最终 residual 没有降到 `1e-3` 以下
（`HydrofoilAllocationModel.cpp:96-145`）。用户提供的数据没有包含该几拍
具体 residual/iterations/speed_squared 数值，不能虚构这些数值；新 topic
已经逐拍记录它们。

## 5. 为什么共同翼角固定为 ±0.61086 rad

`0.61086 rad = 35°`。Newton 每次迭代执行：

```text
delta = constrain(delta, -max_hydrofoil_angle, +max_hydrofoil_angle)
thrust = constrain(thrust, 0, available_thruster_thrust)
```

源码：`HydrofoilAllocationModel.cpp:122-137`。

岸上无推进器输出时，可用推力接近 0；`HY_SPEED_SELECT=0` 时
`speed_squared = HY_ALT_SPEED²`。当 `command_force_z` 达到约
±15 至 ±80 N，替代速度下的水翼升力和零推进器推力无法严格满足目标力，
Newton 会持续要求更大的同向翼角，最终被夹在 ±35°。所以：

- candidate 仍 finite、bounded；
- 共同分量符号随 `force_z` 改变；
- residual 仍大于收敛阈值；
- `converged/valid=false`；
- saturation mask 包含左右翼角限制。

饱和是物理可达性信息，不应导致完整深度命令被软件清零。

## 6. saturation mask 解码

枚举定义在
`src/lib/hydrofoil_allocation/HydrofoilAllocationModel.hpp:24-33`：

| Bit | Hex | 含义 |
|---:|---:|---|
| 0 | `0x01` | DeltaLeft，左翼角迭代触及限制 |
| 1 | `0x02` | DeltaRight，右翼角迭代触及限制 |
| 2 | `0x04` | ThrustLeftLow，左推力下限 |
| 3 | `0x08` | ThrustLeftHigh，左推力上限 |
| 4 | `0x10` | ThrustRightLow，右推力下限 |
| 5 | `0x20` | ThrustRightHigh，右推力上限 |
| 6 | `0x40` | ForceXRedistributed，负水平力被重分配 |

实机 mask：

| Decimal | Hex | 置位 |
|---:|---:|---|
| 43 | `0x2B` | DeltaLeft、DeltaRight、ThrustLeftHigh、ThrustRightHigh |
| 107 | `0x6B` | `0x2B` 加 ForceXRedistributed |
| 59 | `0x3B` | DeltaLeft、DeltaRight、ThrustLeftHigh、ThrustRightLow、ThrustRightHigh |

mask 在 Newton 多次迭代间累计，是“曾触发”粘滞位；`0x3B` 中右推力上下限
同时出现不表示最终推力同时处于两个边界，而表示不同迭代曾越过两侧。

## 7. 历史生产路径与当前错误路径

HEAD `5ae2fd95` 的历史代码：

1. `optim()` 固定最多执行 10 次；
2. 每次约束翼角及推力；
3. 无 `valid/converged/residual` 输出门槛；
4. `optim()` 返回的有限 constrained `x` 总是继续归一化；
5. foil gate 开启、`HY_ALLOCATE_MODE=0` 时，完整
   `x[left delta/thrust, right delta/thrust]` 直接写入 hydro 输出。

对应基线位置：

- Newton 与硬约束：旧 `HydroAllocator.cpp:160-192`
- 完整解求取：旧 `:391-402`
- 归一化：旧 `:404-408`
- 左右完整解下发：旧 `:591-600`

Phase 1 共享模型重构保留了 Newton 方程和硬约束，但在当前未提交工作区
新增了 `!valid -> all zero` 输出策略。该变化不属于 HEAD 中某个 Git
commit；它是相对 `5ae2fd95` 的 Phase 1 工作区修改。因此可以精确定位到
代码位置，但不存在可引用的新提交哈希。

这就是历史岸上按压深度计时共同翼会动作，而当前 Mode 0 完全不动作的
逐行行为差异。

## 8. 最小修复

采用候选 A：输出完整 bounded best-feasible allocator 解。

新增
`HydrofoilAllocationModel::selectExecutable()`
（`HydrofoilAllocationModel.cpp:366-405`）：

```text
strict valid:
    使用完整解

finite && bounded && solve_status == NotConverged
AND mode in 0..3:
    使用完整 best-feasible 解
    左右 commanded angle 按 HY_SR_DRATE 限速
    保留完整左右 thrust
    valid 仍诚实保持 false

nonfinite / JacobianSingular / InvalidInput / hard unbounded:
    禁止输出

mode 4:
    保持严格语义
```

HydroAllocator 调用位置：
`src/modules/hydro_allocator/HydroAllocator.cpp:285-330`。

安全依据：

- 只有 `isFiniteBounded()` 通过才允许 best-feasible；
- 左右翼绝对角度仍受 `HY_WING_MAX_A` 限制；
- 左右推力仍在 `[0, min(max thrust, available thrust)]`；
- 降级翼角变化使用现有 `HY_SR_DRATE`；
- 完整 solution 同时包含 symmetric depth、differential roll 和 thrust，
  不再只保留差动；
- 不修改 `allocator_valid`；
- Jacobian singular、NaN/Inf、结构性 solve error 和硬越界仍抑制；
- gate 和 logical mapping 既有安全语义不变；
- Modes 1/2/3 仍只替换 `u_phi`；
- Mode 4 保持严格行为。

这里“Mode 0 bit-exact”需分层表述：

- legacy CADRC `u_roll` 数值路径仍 bit-exact；
- 当前错误版本的“invalid 时深度全清零”被有意修复，不保持错误的执行器
  bit-exact；
- 修复恢复的是 HEAD `5ae2fd95` 历史生产 allocator 对有限 constrained
  iterate 的执行语义，并额外加入明确 commanded angle rate 保护。

## 9. 诊断增强

新增小型 topic `haar_depth_allocator_status`
（`msg/HaarDepthAllocatorStatus.msg:1-29`），记录：

- valid/converged/finite/bounded、residual、iterations、mask；
- speed_squared、speed source、stale；
- requested/realized force_z；
- requested/realized symmetric 与 differential；
- last strict-feasible symmetric/differential；
- full best-feasible 是否真正发布；
- rate limit、depth degradation、累计时间；
- allocator output 是否被抑制。

实际发布结果在 foil gate 和 `HY_ALLOCATE_MODE` 分支之后重构，源码：
`HydroAllocator.cpp:735-780`。这些仍是 commanded/model-based 状态，不是
位置传感器测量。

`haar_depth_control_status` 同步扩展，并增加 reason 7。logger 注册：
`src/modules/logger/logged_topics.cpp:70`。

## 10. 测试

顺序、无构建目录竞争执行：

- `unit-HydrofoilAllocationModel`：18/18 PASS
- `unit-SafeRollController`：63/63 PASS
- 总计：81/81 PASS

新增回归：

1. finite+bounded+NotConverged 完整解在 Modes 0/1/2/3 可执行；
2. best-feasible 左右翼角遵守 rate limit，左右 thrust 不被清零；
3. Mode 4 对相同 candidate 保持严格禁止；
4. NaN/Inf、hard angle 越界、JacobianSingular 均禁止；
5. `force_z` 正负切换时 symmetric 符号按现有模型反向切换；
6. ±80 N、零可用推进器时明确达到 ±35° bounded saturation；
7. 10 N force_z 变化完整进入 source command，输出非零或明确记录饱和；
8. 既有 SafeRoll Modes 1/2/3 非滚转分量保留测试通过；
9. Mode 0 legacy CADRC bit-exact、AUX gate、FIELD recovery、Mode 4 等既有
   63 项全部通过。

测试位置：
`src/lib/hydrofoil_allocation/HydrofoilAllocationModelTest.cpp:294-414`。

`git diff --check`：PASS。

## 11. H743 clean build 与固件

clean `matek_h743_dummy` `-Werror` 构建通过。clean 后前两次启动遇到工程
既有 NuttX `.config`/`jlink-nuttx` 并行依赖竞态；`.config` 生成后使用同一
配置完成全部 1051 个目标，无编译错误或 `-Werror` 警告。

```text
Firmware:
  build/matek_h743_dummy/matek_h743_dummy.px4

Size:
  1,536,749 bytes

SHA-256:
  359bc3455d4bfe36b1067c49d11bf2ce1431d21d0a87d97e19bb9c96d7a4255d

ELF:
  text = 1,594,720
  data =     3,544
  bss  =    41,324
```

链接器 FLASH：1,598,264 / 1,966,080 B，81.29%。

## 12. 修改文件

- `src/lib/hydrofoil_allocation/HydrofoilAllocationModel.hpp`
- `src/lib/hydrofoil_allocation/HydrofoilAllocationModel.cpp`
- `src/lib/hydrofoil_allocation/HydrofoilAllocationModelTest.cpp`
- `src/modules/hydro_allocator/HydroAllocator.hpp`
- `src/modules/hydro_allocator/HydroAllocator.cpp`
- `src/modules/control_allocator/ControlAllocator.hpp`
- `src/modules/control_allocator/ControlAllocator.cpp`
- `msg/HydroAllocateMessage.msg`
- `msg/HaarDepthControlStatus.msg`
- `msg/HaarDepthAllocatorStatus.msg`
- `msg/CMakeLists.txt`
- `src/modules/logger/logged_topics.cpp`
- `RSMC_codex/SAFE_ROLL_DEPTH_ALLOCATOR_SUPPRESSION_FIX_REPORT.md`

未修改深度控制器、`HY_DC_SP`、`HY_DC_FF`、左右翼方向、CADRC 数学、
SafeRoll/HOCBF 数学或执行器映射。未执行 flash、actuator test 或任何
实机操作。
