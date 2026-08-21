# HAAR SafeRoll Mode 3 `0x00000100` 岸上故障审计

日期：2026-07-24  
基线 HEAD：`5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`  
范围：根因定位和 first-fault 只读诊断增强。未修改控制数学、fault
分类或 gate 语义；未执行 flash、actuator test 或实机测试。

## 1. 精确故障定义

**源码已验证：**

```cpp
FoilGateDisabledFault = 1u << 8
```

位置：`src/modules/hydro_rate_control/SafeRollController.hpp:52`。

因此：

```text
256 = 0x00000100 = FoilGateDisabledFault
```

它不是 estimator、certificate、allocator-invalid 或 nonlinear fault。

## 2. 全部置位位置

### 2.1 Modes 1/2/3 已激活路径

位置：`SafeRollController.cpp:1252-1262`。

完整控制条件：

```text
activation_managed
AND !_activation_pending
AND _mode_activated
AND input.allocator_finite
AND allocator_timestamp_valid
AND !input.foil_gate_enabled
```

其中：

```text
activation_managed =
requested_mode in {1,2,3}

allocator_timestamp_valid =
allocator_timestamp != 0
AND now >= allocator_timestamp
AND now - allocator_timestamp <= 100000 us
```

代码使用有序 `if/else if`：

```text
if allocator nonfinite or timestamp invalid -> 0x80
else if foil gate false                   -> 0x100
else allocator best-feasible/invalid/recovery handling
```

实机 `requested_code=3` 且故障前 `active_code=3`，因此本次生产路径精确
命中这里。它还证明该拍：

- allocator finite；
- allocator message timestamp 合法且年龄不超过 100 ms；
- mode 已完成激活；
- 不在 activation pending。

`allocator_valid=false, allocator_bounded=true` 原本可进入 FIELD
best-feasible，但 gate 检查在该分支之前，且 gate 按既有安全要求是立即
fatal，所以 `0x100` 优先。

### 2.2 非 activation-managed 路径

位置：`SafeRollController.cpp:1340-1347`。

条件：

```text
!activation_managed
AND !input.foil_gate_enabled
```

该路径用于非 Modes 1/2/3 的严格分支。生产环境 requested=3 时
`activation_managed=true`，故不是本次置位点。

## 3. gate 的全部来源变量

`HydroRateControl` 从最新缓存的 `hydro_allocate_message` 复制 gate：

- `HydroRateControl.cpp:320-337`；
- `input.foil_gate_enabled =
  hydro_allocate_message.foil_gate_enabled` 位于 `:332`。

HydroAllocator 的实际 gate 公式位于
`src/modules/hydro_allocator/HydroAllocator.cpp:483-540`：

```text
foil_aux = source(HY_FOIL_AUX) * HY_FOIL_AUXGAIN

foil_gate_enabled =
    foil_aux > HY_FOIL_THR
    AND nav_state in {STAB, ACRO, MANUAL}
```

`HY_FOIL_AUX` source：

```text
0       -> 0
1..6    -> manual_control_setpoint.aux1..aux6
7       -> -1
8       -> +1
other   -> 0
```

所以当拍 gate=false 只有两类直接来源：

1. `foil_aux <= HY_FOIL_THR`；
2. HydroAllocator 当拍缓存的 `vehicle_status.nav_state` 不属于
   `{STAB, ACRO, MANUAL}`。

当前提供的故障后冻结值没有保存首次故障当拍的 AUX、threshold 或
HydroAllocator gate 组成项，故这两类来源之间仍为 **UNKNOWN**，不能
凭后续快照猜测。

`HY_SPEED_SELECT=0`、零推进器、solver residual、saturation 和
best-feasible 状态均不参与上述 gate 布尔表达式。

## 4. 为什么 stage mask 为 0、candidate time 为 4 us

candidate 计时从 `SafeRollController.cpp:1131` 开始。随后依次执行：

1. timestamp/dt/sensor/config 检查；
2. allocator timestamp、finite 和 foil gate 前置检查；
3. 只有所有 prerequisite fault 均为零，才从 `:1384` 开始 reference
   filter，并在 `:1388` 调用 estimator。

gate fault 在 `:1262` 已置位。它随后包含在 prerequisite mask
(`:1370-1373`)，因此 candidate 主阶段条件失败：

```text
reference filter          未执行
estimator                 未执行
certificate               未执行
allocation query          未执行
nominal/CBF/nonlinear     未执行
```

故 `execution_stage_mask=0`，所有分阶段计时为零。约 4 us 只是从
candidate timer 起点经过前置布尔检查、fault 汇总和 fallback 处理到
`total_candidate_time_us` 采样点 (`:1644`) 的耗时。

`first_fault_cycle_time_us=54` 在 HydroRateControl 外层计时
(`HydroRateControl.cpp:420-439`)，还包含 SafeRoll 调用外围和该模块当拍
工作；它与 4 us candidate 子计时不是同一范围。

## 5. Mode 3 专属状态逐项审计

下列变量均在 gate fault 当拍的 `updateEstimators()` 之前，因此没有一个
能设置 `0x100`。

| 项目 | 源码行为 | 对本次 `0x100` 的结论 |
|---|---|---|
| estimator initialized | `_have_previous_p` 只在 `updateEstimators():570-588` 使用 | 当拍未访问，不是触发条件 |
| `b_hat/lower/upper/rho_b` | estimator/certificate 数值检查在 `:633-725` | 当拍未检查；所给值全部 finite、lower≤hat≤upper，且 `hat-rho=8.33495>0` |
| `d_hat/rho_d` | 数值检查在 `:633-634` | 当拍未检查；所给值 finite 且 `rho_d>=0` |
| filtered u/r/innovation | 更新在 `:592-617` | 当拍未运行；所给 u/r finite，innovation 当拍没有重新计算 |
| history/lag ring | push/read 在 `:521-540,570-690` | 当拍未访问，不可能导致 stage 0 gate fault |
| sample age/timestamp | gyro/attitude 检查在 `:1135-1170`；allocator age 在 `:1195-1201` | 若低位 timestamp fault 存在，first set bit 会早于 `0x100`；且已激活 gate 分支要求 allocator timestamp valid |
| accepted/conflict lag | 每次 estimator 输出在 `:646-689` | 当拍未生成；故障后消息中的零值不能解释为触发原因 |
| certificate interval | 检查在 `:712-725` | 当拍未执行；故障前 ready=true 仅说明上一健康拍正常 |
| previous effective input | 只在 innovation sample `:574` 使用 | 当拍未读取 |
| reset counter | transition/reset 在 candidate timer 之前 `:1072-1094` | reset 后 Modes 1/2/3 会回 activation pending，不会直接进入已激活 gate 分支；无证据表明 reset 触发本故障 |

故障后显示的 `b_hat/d_hat/filtered_*` 是 controller 内部此前状态在输出
初始化阶段复制出的冻结值，不是 first-fault 条件值，也不是 fault 来源。

## 6. 根因类别判断

用户给出的 A/B/C/D 四类中，没有一项精确描述本故障：

- **A nonfinite/结构性错误：排除。** fault 位不是 numeric fault，且该
  分支要求 allocator finite。
- **B 零推力造成的有限估计器不一致：排除。** estimator 当拍未执行。
- **C 初始化/历史缓存状态机错误：排除为当前直接原因。** history 当拍
  未访问，且 mode 已激活。
- **D stale/timestamp：排除 allocator stale。** 命中 gate 分支前必须
  先满足 allocator timestamp valid；低位 timestamp fault 也未成为
  first fault。

精确分类是独立的 **foil-gate interlock event**。若必须归入最接近的
大类，它属于“candidate 前置外部状态事件”，但不能把它误写成 timestamp
stale。

目前能够确认的软件根因是：

```text
Mode 3 active
-> allocator finite and fresh
-> hydro_allocate_message.foil_gate_enabled became false
-> immediate FoilGateDisabledFault
-> severe latch
```

造成 gate=false 的底层原因（AUX/threshold 还是 HydroAllocator nav-state
样本）尚需 first-fault 当拍证据。

Mode 2 在相同岸上物理条件下不回退，不反证上述结论：gate 是 Modes
1/2/3 共用条件，只有发生 gate=false 的具体拍才会触发。它不是 Mode 3
估计器独有路径。

## 7. 新增 first-fault 快照

未改变 fault 或控制行为，只增加锁存诊断：

- controller 快照结构：
  `SafeRollController.hpp:174-205,230,436`；
- 首次 fatal 当拍锁存：
  `SafeRollController.cpp:1735-1774`；
- internal/attitude reset 在不允许清 severe latch 时同时保留快照：
  `SafeRollController.cpp:97-156`；
- 独立消息：
  `msg/HaarSafeRollFaultSnapshot.msg`；
- 发布：
  `HydroRateControl.cpp:640-677`；
- logger 默认记录：
  `src/modules/logger/logged_topics.cpp:67`。

关键字段包括：

```text
first_fault_code
prerequisite_fault_flags
requested_mode
activation_pending
mode_activated
allocator_timestamp / allocator_age_us / timestamp_valid
allocator valid / converged / finite / bounded
foil_gate_enabled
foil_aux_parameter / foil_aux_value / foil_threshold_parameter
foil_aux_above_threshold
in_stab / logical_mapping_valid
reset_count / attitude_reset_counter
estimator_initialized / history_count
previous_u_effective_model
b_hat / b_lower / b_upper / rho_b
d_hat / rho_d
filtered_u / filtered_r
```

实机读取命令：

```text
listener haar_safe_roll_fault_snapshot 1
```

判别规则：

- `foil_aux_above_threshold=false`：AUX/gain/threshold 一侧直接关闭 gate；
- `foil_aux_above_threshold=true` 且 `foil_gate_enabled=false`：
  按当前唯一 gate 公式，可确定 HydroAllocator 当拍 nav-state 条件为
  false。

快照在 severe latch 存续期间保持首次值，不会被后续冻结状态覆盖。

## 8. 测试与构建

新增测试：

`SafeRollController.FoilGateFaultCapturesAndPreservesFirstFaultPrerequisiteSnapshot`
(`SafeRollControllerTest.cpp:1211-1266`)。

覆盖：

- Mode 3 已激活；
- allocator finite/bounded、valid/converged=false；
- gate=false 精确产生 `0x100`、stage mask=0；
- 完整 prerequisite/AUX/estimator 快照正确；
- attitude/internal reset 不覆盖首次快照。

结果：

```text
SafeRollController           60/60 PASS
HydrofoilAllocationModel     11/11 PASS
总计                          71/71 PASS
git diff --check             PASS
matek_h743_dummy -Werror      PASS
```

H743 链接结果：FLASH 1,592,416 bytes（80.99%）。

诊断固件：

```text
/home/linyi666/PX4-Autopilot-water-air/build/matek_h743_dummy/matek_h743_dummy.px4
size: 1,530,077 bytes
SHA-256:
2485343563d7c502c6bc12ea4474fef59bb3fd7e67007433f745af2fac444606
```

`CADRC.cpp/.h` 未修改。

## 9. 最小修复建议（未实施）

在取得新快照前，不建议修改 gate、阈值、estimator 或 severe 分类。

1. 若 `foil_aux_above_threshold=false`：
   检查所选 AUX 的当拍值、RC failsafe/抖动和 threshold 裕量，再决定是否
   需要明确的 gate debounce；不要直接放宽 estimator 或 allocator。
2. 若 `foil_aux_above_threshold=true`：
   检查 HydroAllocator 与 HydroRateControl 在不同 work queue 中读取的
   `vehicle_status.nav_state` 时序，确认是否存在一次旧/过渡 nav-state
   样本；若证实，最小修复应是统一 gate 的 mode-state 时序来源或对明确
   的状态转换做同步处理。
3. 若快照显示 `allocator_timestamp_valid=false`，则与当前 `0x100`
   生产分支矛盾，应优先检查固件/日志版本是否对应；当前源码会先产生
   `0x80`。

本轮没有足够证据确认 AUX 抖动或 nav-state 跨线程瞬态，故没有实施
行为修复，也没有改变 foil gate 关闭时立即 fatal 的现有安全策略。
