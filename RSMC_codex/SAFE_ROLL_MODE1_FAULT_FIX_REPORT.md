# HAAR SafeRoll Mode 1 岸上故障定位与修复报告

日期：2026-07-24  
源码基线：`5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`（分支 `RSMC`）  
固件版本标识：`v1.15.0-beta1-346-g5ae2fd95ae-dirty`

## 结论

本次实机快照中的 `0x400000` 准确对应
`SafeRollController::FallbackLatchedFault`，定义为 `1u << 22`
（`src/modules/hydro_rate_control/SafeRollController.hpp:64`）。

它不是最初导致候选失败的故障，而是“此前已有 fatal fault，控制器已经锁存回退”的状态位。
五秒后的快照实际命中的布尔条件是 `_fallback_latched == true`：

```text
(current_fault_flags & fatal_faults) != 0 || _fallback_latched
```

随后：

```text
if (_fallback_latched)
    fault_flags |= FallbackLatchedFault
```

对应 `SafeRollController::evaluate()` 的
`src/modules/hydro_rate_control/SafeRollController.cpp:1120-1134`。

锁存后候选入口要求 `!_fallback_latched`
（`SafeRollController.cpp:929`），因此参考滤波、估计器、证书、分配查询和非线性检查全部被跳过。
这与实机的全部子阶段计时为零、`total_candidate_time_us=10` 完全一致。

旧实现没有保存首次 fatal fault。因而仅凭提供的五秒后快照，无法反推出锁存前那一拍究竟是
`InvalidCertificateFault`、`EmptyHardIntervalFault`、`NonlinearCandidateInvalidFault`
还是其他 fatal fault；把其中任意一个宣称为实机首次致因都不具备证据。
这也是本次新增 `first_fault_code` 的直接原因。

## 0x400000 的全部源码路径

全工程只有一个枚举定义和一个置位语句：

- 定义：`SafeRollController.hpp:64`，
  `FallbackLatchedFault = 1u << 22`。
- 置位：`SafeRollController::evaluate()`，
  `SafeRollController.cpp:1132-1134`；完整条件为 `_fallback_latched == true`。

`_fallback_latched` 只有一个置 true 的位置：

```text
if ((fault_flags & fatal_faults) != 0 && !first_sample)
    _fallback_latched = true
```

位于 `SafeRollController.cpp:1123-1129`。其中：

```text
fatal_faults =
    TimestampNonMonotonic
  | DtTooSmall
  | DtTooLarge
  | GyroStale
  | AttitudeStale
  | QuaternionNonFinite
  | AngularVelocityNonFinite
  | AllocatorInvalidFault
  | FoilGateDisabledFault
  | ModeOutOfRange
  | LogicalMappingInvalidFault
  | InvalidConfigurationFault
  | ControllerDisabledFault
  | InvalidCertificateFault
  | EmptyHardIntervalFault
  | NonlinearCandidateInvalidFault
  | ProjectionInvalidFault
  | RuntimeOverrunFault
```

`prerequisite_faults` 定义在 `SafeRollController.cpp:922-926`，
完整 fatal 合并在 `:1120-1121`。锁存仅由 `reset()`、
模式/配置 reset 或新控制器 reset 清除（`:95-106`、`:109-133`）。

## 修改前的结构性问题

1. `updateEstimators()` 的统一返回条件包含在线证书区间有效和
   `b_hat-rho_b>0`。`evaluate()` 用这个单一返回值门控所有 Mode 1--4，
   所以 Mode 1 被错误耦合到自适应证书状态。
2. Mode 2 虽使用固定 `b0/[b_min,b_max]`，仍经过与自适应模式共用的门控接口，
   模式语义不够明确。
3. Mode 1 的局部 allocation-aware 区间一旦无效或为空，会直接产生 fatal fault；
   即使共享非线性 allocator 对候选仍可正常求解，也没有退化路径。
4. 首次 fatal fault 锁存后被下一拍的 `FallbackLatchedFault` 覆盖性地隐藏；
   延迟读取只能看到 `0x400000`。
5. 计时字段为零不能区分“阶段未执行”和“阶段执行时间小于 1 us”。

模式切换 reset 本身不是零估计状态的根因。`HY_SR_MODE: 0 -> 1` 会增加
`reset_count`，并在 `resetNewController()` 中将：

- `b_hat` 初始化为受 `[b_min,b_max]` 限制的 `b0`；
- `b_lower/b_upper` 初始化为 `b_hat ± HY_SR_RB0`；
- `d_hat=0`、`rho_d=HY_SR_RD0`；
- 环形历史、滤波器和新控制器锁存状态清空；
- 上一实际 selected output 保留以实现无跳变融合。

源码为 `SafeRollController.cpp:109-133`。它不会把 `b_hat/bounds` 清零，
也不会 reset legacy CADRC。

`HY_SR_ENABLE` 语义原本正确且保持不变：

```text
enable > 0 && (requested_mode <= 3 || enable == 2)
```

位于 `SafeRollController.cpp:915-919`。因此 `ENABLE=1` 允许 1、2、3，
`ENABLE=2` 额外允许 4。

## 最小修复

### 按模式拆分有效性依赖

`SafeRollController::updateEstimators()` 现在显式区分
`fixed_mode` 与 `adaptive_mode`（`SafeRollController.cpp:429-435`）：

- Mode 1：只要求点估计、扰动估计和扰动界为 finite，且 `rho_d>=0`；
  不要求历史已准备、不要求在线证书、不要求 `accepted_lag_mask!=0`。
- Mode 2：使用配置的 `b0/[b_min,b_max]` 固定区间；
  不运行多滞后在线证书，不依赖 Mode 3 历史。
- Mode 3/4：要求在线区间 finite、上下界有序且 `b_hat-rho_b>0`。
  历史尚不足最大 lag 32 时设置 `certificate_warmup=true`，使用配置先验区间；
  不要求必须接受某个 lag。
- 任一模式出现非 finite、非法上下界或 Mode 3/4 非正鲁棒增益下界时仍回退，
  没有屏蔽 fatal fault。

实现与返回条件在 `SafeRollController.cpp:540-575`。

### Mode 1 的分配退化策略

Mode 1 仍调用共享 `HydrofoilAllocationModel`，仍要求：

- allocator solve 有效；
- generic + hydro 最终 normalized command 为 finite 且位于 `[-1,1]`；
- forward model 有效；
- 输入幅值区间和上一实际执行 `u` 的增量区间有效；
- 投影和输出为 finite。

但局部 Jacobian/commanded-foil interval 不可用时，不再把它当作
Mode 1 的证书前置条件。此时设置 `allocation_interval_degraded=true`，
使用 `U_amp ∩ U_rate`，随后仍执行完整、固定次数、无动态内存的
`HydrofoilAllocationModel::checkCandidate()`。

Mode 2/3 仍严格要求 commanded foil angle/rate allocation-aware interval；
Mode 4 仍是既定的 no-allocation ablation。实现位于
`SafeRollController.cpp:939-971` 和 `:1024-1053`。

### 可定位日志

新增数字字段：

- `first_fault_code`：首次 fatal fault 的最低置位 bit；锁存期间持续保留，
  且同时保留在 `fault_flags` 中。
- `execution_stage_mask`：
  bit0 reference filter、bit1 estimator、bit2 certificate、bit3 allocation query、
  bit4 nominal、bit5 CBF、bit6 nonlinear check、bit7 output selected。
- `estimator_ready`、`certificate_ready`、`certificate_warmup`、
  `allocation_interval_degraded`。

字段加入：

- `msg/HydroCadrcControlMessage.msg:22-23,60-63`
- `msg/HaarSafeRollStatus.msg:12-13,83-86`
- `msg/HaarSafeRollEstimatorStatus.msg:8-9,34-37`

发布链位于：

- `HydroRateControl.cpp:418-425,438-439,475-513`
- `ControlAllocator.cpp:893-903,1011-1017`

首次故障还会输出一次控制台告警：

```text
SafeRoll fallback first_fault=0x........ stages=0x........
```

模式切换/reset 会清除首次故障码（`SafeRollController.cpp:95-133`）。

扩展后的 `haar_safe_roll_status` ULog 格式描述为 1709 字节，超过本分支
logger 原 1600 字节缓冲。为保留既有字段及新诊断，固定格式缓冲调整为
1800 字节，并继续由原静态断言校验所有消息，见
`src/modules/logger/messages.h:115-121`。静态缓冲增加 200 字节；
控制算法和执行器输出不受影响。

## 修改前后状态机

修改前：

```text
prerequisite valid
 -> unified estimator+certificate valid for every mode
 -> U_amp ∩ U_rate ∩ U_alloc valid
 -> nominal/CBF
 -> nonlinear check
 -> output

any fatal -> latch
next samples -> skip all stages -> only 0x400000 remains visible
```

修改后：

```text
prerequisite valid
 -> reference filter
 -> mode-specific validity
      1: finite nominal estimator state
      2: configured fixed certificate
      3/4: online certificate, prior-only warm-up allowed
 -> mode-specific hard interval
      1: local allocation interval if available; otherwise marked degradation
      2/3: allocation-aware interval required
      4: allocation ablation
 -> nominal/CBF as selected
 -> exact nonlinear allocator check
 -> blend/output

any fatal -> latch first fatal bit + 0x400000 + executed-stage mask
```

## 回归测试

`src/modules/hydro_rate_control/SafeRollControllerTest.cpp:121-238`
新增了六项针对性覆盖：

1. Mode 0 + ENABLE=1 仍直接输出 legacy、无 fault。
2. 与实机相符的 `phi=2 deg`、`dt=2.47 ms`、generic trim `-0.1`、
   Mode 0→1：`reset_count` 增加，Mode 1 当拍进入 active，
   candidate/output 有效，参考滤波、nominal、allocation query、
   nonlinear check 和 output-select 阶段均执行。
3. Mode 1 在 estimator history/online certificate 未准备且
   `accepted_lag_mask=0` 时正常运行。
4. Mode 1 在局部 allocation interval 无效但非线性 allocator 可解时正常运行，
   并明确记录 degraded 状态。
5. Mode 2 在没有自适应历史时使用固定证书正常运行。
6. Mode 3 在多 lag 历史未齐时使用明确的先验 warm-up。

原 fallback 测试增加了 `0x400000` 条件回归：首次 allocator fault 被保存，
后续锁存样本同时包含首次 fault 与 `FallbackLatchedFault`，且
`execution_stage_mask=0`。Mode 1/2/3 返回 Mode 0 的测试均验证平滑过渡后恢复
逐样本 legacy 直通。

结果：

- SafeRollController：31/31 PASS；
- HydrofoilAllocationModel：9/9 PASS；
- 总计：40/40 PASS；
- 原有 34 项全部保留并通过，新增 6 项通过；
- Mode 0 CADRC 选择器逐样本 `EXPECT_FLOAT_EQ` 回归通过；
- allocator 原有数值回归、forward/linearize/异常路径测试全部通过；
- host candidate benchmark：平均约 2.67 us，观测最大 71.10 us
  （主机参考值，不是 H743 WCET）。

执行命令：

```text
CCACHE_DIR=/tmp/codex-ccache cmake --build build/px4_sitl_test_phase1 \
  --target unit-SafeRollController unit-HydrofoilAllocationModel -j2
build/px4_sitl_test_phase1/unit-SafeRollController --gtest_color=no
build/px4_sitl_test_phase1/unit-HydrofoilAllocationModel --gtest_color=no
```

## H743 clean `-Werror` 构建

目标：`matek_h743_dummy`。最终从已清理的 1751 个构建文件重新生成并完成
1052 个构建步骤，ARM 编译保持项目默认 `-Werror`。

该分支的 NuttX build graph 存在既有生成顺序缺陷：clean 后首次调用会先构建
`jlink-nuttx.so`，而 `.config` 尚未生成，因而首次调用停止；同一次调用随后已生成
`.config`，第二次调用从该 clean tree 完成全部 1052 步。此问题与 SafeRoll 源码无关，
但为保证可复现性在此如实记录。

最终链接结果：

```text
FLASH    1,582,640 / 1,966,080 bytes (80.50%)
AXI_RAM     44,716 /   524,288 bytes (8.53%)
```

构建成功产物：

```text
/home/linyi666/PX4-Autopilot-water-air/build/matek_h743_dummy/matek_h743_dummy.px4
size: 1,520,845 bytes
SHA256: 9659f95b82efd5f9c96962d4bf4d24995b6decb1473c4c631c3b841564b4bf28
```

`parameters.xml` 已重新生成；`HY_SR_ENABLE` 默认仍为 0，
`HY_SR_MODE` 默认仍为 0。没有修改 `CADRC.cpp` 或 `CADRC.h`，
没有改变 manual reference、generic+hydro 叠加、trim、通道映射或 Mode 0 输出。

## 修改文件

本次故障修复涉及：

- `src/modules/hydro_rate_control/SafeRollController.hpp`
- `src/modules/hydro_rate_control/SafeRollController.cpp`
- `src/modules/hydro_rate_control/SafeRollControllerTest.cpp`
- `src/modules/hydro_rate_control/HydroRateControl.hpp`
- `src/modules/hydro_rate_control/HydroRateControl.cpp`
- `src/modules/control_allocator/ControlAllocator.cpp`
- `msg/HydroCadrcControlMessage.msg`
- `msg/HaarSafeRollStatus.msg`
- `msg/HaarSafeRollEstimatorStatus.msg`
- `src/modules/logger/messages.h`
- `RSMC_codex/SAFE_ROLL_MODE1_FAULT_FIX_REPORT.md`

未执行 flash、actuator test 或水池测试。
