# HAAR SafeRoll Mode 1 第二次岸上故障定位与修复报告

日期：2026-07-24  
Git HEAD：`5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`  
版本：`v1.15.0-beta1-346-g5ae2fd95ae-dirty`

## 结论

本次实机故障不是 `RuntimeOverrunFault`。

当前源码的准确枚举是：

```text
0x00020000 = EmptyHardIntervalFault = 1u << 17
0x00200000 = RuntimeOverrunFault    = 1u << 21
```

定义位于：

- `src/modules/hydro_rate_control/SafeRollController.hpp:59`
- `src/modules/hydro_rate_control/SafeRollController.hpp:63`

实机的：

```text
first_fault=0x00020000
stages=0x0000000f
```

表示参考滤波、估计器、证书状态处理和 allocation query 已完成，
随后 `U_hard` 为空，在 nominal control 之前置位
`EmptyHardIntervalFault`。这与源码唯一对应路径完全一致。

启动时的 `0x10` 是 `AttitudeStale`，解除锁定后的 `0x80` 是
`AllocatorInvalidFault`。它们是独立 reset 周期中的前置条件故障，
没有与本次 `0x20000` 合并分析。

## 为什么不可能是 RuntimeOverrunFault

修复前 `RuntimeOverrunFault` 只有一个置位位置：

```text
if (total_candidate_time_us > 2000)
    fault_flags |= RuntimeOverrunFault;
```

它位于 `SafeRollController::evaluate()` 的候选链末尾，在：

- reference filter；
- estimator/certificate；
- allocation query；
- nominal；
- CBF（适用模式）；
- nonlinear candidate check；
- blend 与输出有效性检查

全部结束之后才比较。

因此，如果是 runtime fault，故障值应为 `0x00200000`，而且比较发生时
nominal/nonlinear 等已执行阶段不会因为 runtime 比较本身而缺失。
`0x00020000 + stage 0x0f` 只能对应 allocation 后的 hard interval 空交。

当前源码中的 runtime 唯一置位位置仍为：

- 文件：`src/modules/hydro_rate_control/SafeRollController.cpp`
- 函数：`SafeRollController::evaluate()`
- 行号：`:1241-1244`
- 条件：

```text
!_fallback_latched
AND runtimeBudgetExceeded(
        total_candidate_time_us,
        transition,
        consecutive_runtime_overruns)
```

`runtimeBudgetExceeded()` 位于 `SafeRollController.cpp:365-384`。

## 原实时性计时审计

### 原预算

原源码使用硬编码：

```text
kCandidateRuntimeLimitUs = 2000
```

单位是微秒。源码没有注释或证据表明它来自 host benchmark；
不能声称 2.67 us 主机结果是该阈值的依据。

此前目标板 hydro allocator 实测最大约 136 us，小于原 2000 us 阈值约
14.7 倍。因此原预算并不低于一次正常 allocation query，更不能解释
`0x00020000`。

需要注意，SafeRoll allocation-aware 路径可能包含 solve、linearize、
候选 solve 及固定次数回缩；下游 HydroAllocator 的单次统计不能直接等同于
SafeRoll 整个 candidate WCET。

### 原计时范围

修复前起点位于 transition/reset 之前，因此包括：

- reset-sensitive configuration CRC；
- mode/enable/reset/arming/STAB transition 检测；
- `resetNewController()`；
- blend 初始化；
- 输入与时间有效性检查；
- 全部 candidate 阶段。

它不包括：

- 最外层 `configurationCrc32()`；
- PX4 参数订阅和 `parameter_update` 处理；
- uORB publication；
- 整个 `HydroRateControl` callback；
- 下游 `ControlAllocator`；
- 下游 actuator/PWM。

`hrt_absolute_time()` 和 `hrt_elapsed_time()` 均使用微秒；
timestamp 转秒时显式乘 `1e-6f`，控制 `dt` 使用秒。未发现 us/ms/s
比较单位错配。

## 实际根因

前一版 Mode 1 修复只处理了：

```text
U_alloc 本身 invalid
```

却遗漏了：

```text
U_alloc valid
AND U_amp ∩ U_rate valid
AND (U_amp ∩ U_rate ∩ U_alloc) empty
```

Mode 0→1 或运行中的 commanded-state/rate-window 漂移可使局部
allocation interval 与很窄的 `U_rate` 不相交。旧逻辑先覆盖
`output.u_hard`，然后在 allocation query 后立即设置
`EmptyHardIntervalFault`，所以首次 stage mask 正好为 `0x0f`。

修复在 `SafeRollController::combineAllocationInterval()` 中统一处理
invalid 和 disjoint 两种情形，源码为
`SafeRollController.cpp:311-331`，调用位于 `:1069-1071`：

- Mode 1：记录 `allocation_interval_degraded=true`，保留
  `U_amp ∩ U_rate`，继续执行完整 nonlinear allocator check；
- Mode 2/3：allocation interval invalid 或空交仍立即判 invalid；
- Mode 4：保持既有 allocation ablation 语义。

没有强制 `candidate_valid=true`，也没有关闭 nonlinear solve、
finite、normalized command、幅值或输入增量约束。

## Mode 1 不必要计算

实机旧 stage bit2 并不表示执行了完整 Mode 3 多 lag certificate：
前一版本已用 `adaptive_mode` 保护多 lag 循环，但 Mode 1 仍执行证书边界
打包/状态检查并无条件设置 `StageCertificate`，导致 stage mask 有误导性。

本次修改后，Mode 1 在 finite point-estimator 检查完成后立即返回：

- 不进入 certificate stage；
- 不运行多 lag intersection；
- `certificate_time_us=0`；
- 不设置 stage bit2；
- 不进入 CBF stage，也不设置 stage bit5。

实现位于 `SafeRollController::updateEstimators()`：
`SafeRollController.cpp:506-648`。Mode 2 使用固定证书状态，
Mode 3 才执行在线证书。

## 新实时性状态机

预算采用固定、明确且写入日志的数值：

```text
runtime_soft_budget_us = 1000 us
runtime_hard_budget_us = 2500 us
soft consecutive limit = 5 samples
```

定义于 `SafeRollController.cpp:22-24`。

选择 1000 us 软预算的依据是：

- 目标板已知 allocator 最大约 136 us；
- 用户要求的正常模拟范围 150--500 us 不应累计；
- 1000 us 对 500 us 留 2 倍余量；
- 仍给 2.5 ms 控制周期保留 1.5 ms；
- 没有采用 host 2.67 us 作为目标板预算。

状态机：

```text
elapsed > 2500 us
    -> 单拍立即 RuntimeOverrunFault 并锁存

1000 us < elapsed <= 2500 us
    -> consecutive count + 1
    -> 连续第 5 拍 RuntimeOverrunFault 并锁存

elapsed <= 1000 us
    -> consecutive count 清零

mode/config transition first sample
    -> 不累计软超时
    -> 但 >2500 us 的硬超时仍立即生效
```

Mode 1/2/3 共用同一 `runtimeBudgetExceeded()`，没有模式特例。

### 修正后的计时边界

计时起点移动到以下工作完成之后：

- configuration CRC；
- transition detection；
- `resetNewController()`；
- blend initialization；
-初始诊断状态拷贝。

起点位于 `SafeRollController.cpp:953-957`，终点位于 `:1217`。

现在计入：

- prerequisite/time/sensor/config validation；
- reference filter；
- estimator；
- Mode 2/3 certificate；
- allocation query；
- nominal；
- Mode 2/3 CBF；
- nonlinear candidate and blend check。

仍不计入：

- 参数订阅/更新；
- uORB publish；
- 完整 HydroRateControl callback；
- 下游 ControlAllocator；
- actuator/PWM。

所有预算和阶段时间均为 `uint32` 微秒。

## 首次故障持久诊断

下列字段在首次 fatal fault 时锁存，直到 SafeRoll reset：

```text
first_fault_stage_mask
first_fault_total_candidate_time_us
first_fault_reference_filter_time_us
first_fault_estimator_time_us
first_fault_certificate_time_us
first_fault_allocation_query_time_us
first_fault_nominal_time_us
first_fault_cbf_time_us
first_fault_nonlinear_check_time_us
first_fault_cycle_time_us
runtime_soft_budget_us
runtime_hard_budget_us
consecutive_runtime_overrun_count
```

控制器锁存实现位于 `SafeRollController.cpp:1251-1264`，
持续回填位于 `:1284-1292`。外层 callback cycle time 在
`HydroRateControl.cpp:406-412` 首次锁存。

字段发布在：

- 内部传递：`msg/HydroCadrcControlMessage.msg:24-36`
- 持久 ULog：`msg/HaarSafeRollEstimatorStatus.msg:10-22`
- 当前 nominal timing：`HaarSafeRollEstimatorStatus.msg:79`
- 发布赋值：`HydroRateControl.cpp:447-466,518-538,589`

`haar_safe_roll_status` 保留当前 `first_fault_code` 和
`execution_stage_mask`；完整首次耗时快照放在
`haar_safe_roll_estimator_status`，避免继续扩大主状态消息。

扩展 estimator 消息后最大未 tokenized ULog 格式为 1783 bytes。
logger format 缓冲由上一版 1800 调整为 1900 bytes，并继续接受原静态断言；
见 `src/modules/logger/messages.h:115-121`。链接报告 AXI RAM 使用量未增加。

## 修复前后流程

修复前：

```text
Mode 1 estimator
 -> certificate status bit
 -> U_amp ∩ U_rate
 -> intersect U_alloc
 -> empty => 0x00020000, stage=0x0f
 -> fallback latch
```

修复后：

```text
Mode 1 point estimator only
 -> U_amp ∩ U_rate
 -> query U_alloc
 -> U_alloc invalid/disjoint:
      mark degraded
      retain U_amp ∩ U_rate
 -> nominal
 -> exact nonlinear allocator check
 -> selected output

runtime:
  >1 ms increments soft counter
  healthy sample clears counter
  5 consecutive soft overruns or one >2.5 ms => 0x00200000
```

## 自动测试

结果：

- SafeRollController：36/36 PASS
- HydrofoilAllocationModel：9/9 PASS
- 合计：45/45 PASS
- 原 40 项全部通过
- 新增 5 项测试全部通过

新增覆盖：

1. `0x00020000`、`0x00200000`、`0x10`、`0x80` 枚举回归；
2. Mode 1 对 invalid allocation interval 的退化；
3. Mode 1 对 valid-but-disjoint allocation/rate intervals 的退化；
4. 150/300/500 us 正常、单次软超限、连续 5 次软超限、2501 us 硬超限；
5. Mode 1/2/3 共用 runtime 规则且 transition 首拍不累计软超限；
6. 首次故障 stage 和各阶段耗时持续到 reset；
7. Mode 1 不设置 certificate/CBF stage；
8. Mode 0 CADRC 逐样本 `EXPECT_FLOAT_EQ` 继续通过。

最终 host benchmark 约：

```text
average 2.55 us
observed max 86.07 us
```

这只作为主机回归结果，不用于宣称 H743 WCET。

## H743 clean `-Werror` 构建

执行了构建目录 clean（清除 1751 个文件），随后显式先生成 NuttX `.config`
以规避本分支已知的 clean-build dependency race，再完成 1063 个
`matek_h743_dummy` 构建步骤。ARM 编译使用项目默认 `-Werror`。

链接结果：

```text
FLASH:   1,583,600 / 1,966,080 bytes = 80.55%
AXI RAM:    44,716 /   524,288 bytes = 8.53%
```

固件：

```text
/home/linyi666/PX4-Autopilot-water-air/build/matek_h743_dummy/matek_h743_dummy.px4
size: 1,522,133 bytes
SHA256: 3f187fce5912884842c583a6f8e1a98f6990d63aae1cbafc075418be344a9d5b
```

## 本次修改文件

- `src/modules/hydro_rate_control/SafeRollController.hpp`
- `src/modules/hydro_rate_control/SafeRollController.cpp`
- `src/modules/hydro_rate_control/SafeRollControllerTest.cpp`
- `src/modules/hydro_rate_control/HydroRateControl.hpp`
- `src/modules/hydro_rate_control/HydroRateControl.cpp`
- `msg/HydroCadrcControlMessage.msg`
- `msg/HaarSafeRollEstimatorStatus.msg`
- `src/modules/logger/messages.h`
- `RSMC_codex/SAFE_ROLL_MODE1_INTERVAL_RUNTIME_FIX_REPORT.md`

没有修改 `CADRC.cpp/.h`、manual mapping、generic+hydro 叠加、trim 或通道映射。
没有执行 flash、actuator test 或实机测试。
