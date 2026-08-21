# HAAR SafeRoll allocator 三级容错修复报告

日期：2026-07-24  
源码基线：`5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`  
状态：软件修改、主机回归和 `matek_h743_dummy` clean `-Werror` 构建完成；未执行 flash、actuator test 或实机测试。

## 1. 定位结论

### 1.1 实机证据能够证明的事项

**VERIFIED（实机日志）**

- SafeRoll Mode 1 能够激活并持续运行。
- 首个异常样本的 allocator 为 `valid=false`、`converged=false`、`residual=0.01235`、`saturation_mask=2`。
- 后续锁存的 `first_fault_code=0x00000080` 是 `AllocatorInvalidFault`；枚举位于
  `src/modules/hydro_rate_control/SafeRollController.hpp`。
- 旧策略连续 5 个无效样本即锁存；400 Hz 标称周期下仅约 12.5 ms，不能区分短暂 Newton
  掉线和持续 allocator 故障。

**VERIFIED（源码）**

- `saturation_mask=2` 精确对应 `hydrofoil::DeltaRight`，即逻辑右翼水翼角在 Newton
  迭代中被夹到 `[-max_hydrofoil_angle, +max_hydrofoil_angle]`。定义见
  `src/lib/hydrofoil_allocation/HydrofoilAllocationModel.hpp:24-32`；夹紧和置位见
  `HydrofoilAllocationModel.cpp:122-136`；右翼调用见 `:189-192`。
- bit 2 不是推力饱和。推力位从 bit 2 开始，即 `ThrustLeftLow=4`、
  `ThrustLeftHigh=8`、`ThrustRightLow=16`、`ThrustRightHigh=32`。
- allocator 无效后发布的 `thrust_l_commanded=0`、`thrust_r_commanded=0` 不能用来判断
  Newton 内部推力解为零；运行 allocator 在无效解时会将下游命令安全清零。

### 1.2 根因边界

**VERIFIED**

直接触发锁存的根因是：运行 allocator 短时 `valid/converged=false`，旧控制器用过短的
5 拍去抖窗口将其升级为永久 `AllocatorInvalidFault`。

**LIKELY**

异常样本中的 `DeltaRight` 饱和和残差超限说明当时逻辑右翼方程在该 allocation context
下没有在容差内收敛。这与正滚转方向的实机现象一致，但仅凭当前快照不能判断是瞬时
可行域边缘、上一拍状态/速率约束、推力/姿态 context，还是实际左右不对称造成。

**UNKNOWN**

未提供异常拍完整的 `body_pitch`、`speed_squared`、`force_x`、`force_z`、`u_theta`、
`u_psi`、available thrust、generic Servo4/5 和上一拍 final commanded foil state，
因此不能离线逐样本重现 `residual=0.01235`。本次没有改动 allocator 数学、角度限制、
HOCBF、证书、trim 或执行器映射。

## 2. 三级容错状态机

实现位于 `SafeRollController.cpp:1076-1223,1435-1529`，阈值常量位于
`SafeRollController.hpp:301-304`。

### 2.1 适用范围

- Mode 0：不进入该状态机，继续 legacy direct path，逐样本 bit-exact。
- Modes 1/2/3：使用完整三级容错。
- Mode 4：保持原来的 allocator invalid 立即故障行为。
- 仅对 `allocator_valid=false` 或 `allocator_converged=false` 进行短时容错。

### 2.2 阶段 A：第 1～20 拍

- 不置 fatal fault，不锁存，不永久改变请求模式。
- `active_code` 保持 requested code。
- 输出最后一个已通过完整 nonlinear check 的 `last_feasible_u`。
- 保持值仍通过现有 `HY_SR_DUMAX × dt/0.0025` 和 `HY_SR_UMAX` 限速/限幅，函数见
  `SafeRollController.cpp:899-907`。
- `allocator_degraded_active=true`。
- Modes 2/3 同时置 `safety_degraded_active=true`，明确该拍没有重新完成 HOCBF 的
  allocator 非线性认证。

在标称 400 Hz 下 20 拍为 50 ms。实际持续时间另由
`allocator_invalid_duration_us` 按 gyro sample timestamp 记录。

### 2.3 阶段 B：第 21～80 拍

- 使用现有 `fallbackOutput()` 向实时 legacy CADRC 平滑、按 DUMAX 限速靠拢。
- `requested_code` 保持不变，`active_code=0`，不锁存。
- `temporary_fallback_active=true`，`fallback_active=true`。
- Modes 2/3 同时置 `safety_degraded_active=true`。
- 连续 20 拍 allocator 健康后清除临时状态，调用 `beginBlend()`，自动重新融合进入
  requested mode，无需修改模式参数或人工 reset。

在标称 400 Hz 下第 21～80 拍覆盖约 50～200 ms。

### 2.4 阶段 C：连续第 81 拍

- 置 `AllocatorInvalidFault (0x00000080)`。
- 锁存 fallback，继续使用原有平滑、限速的 legacy 回退。

### 2.5 仍立即 fatal 的条件

以下没有进入三级去抖：

- allocator 非有限；
- allocator timestamp 为 0、来自未来，或距当前超过现有 100,000 us 严重阈值；
- foil gate 关闭；
- logical mapping 无效；
- SafeRoll 配置非法；
- candidate、投影或最终输出非有限/非法；
- 既有 timestamp、传感器、硬区间、runtime 等严重故障。

对应前置检查见 `SafeRollController.cpp:1026-1074,1076-1081,1111-1118,1216-1223`。

## 3. last feasible 语义

`last_feasible_u` 只在以下条件全部成立时更新，见
`SafeRollController.cpp:1524-1529`：

- 本拍 `candidate_valid=true`；
- `active_code` 等于 requested mode；
- 不处于 allocator degraded 或 temporary fallback；
- 无 fatal fault；
- 最终 `u_selected` 有限。

候选值先经 `validatedCandidate()`，融合后的实际输出还会再次通过
`checkNonlinearCandidate()`，见 `SafeRollController.cpp:1313-1351`。因此保存的是最后一个
真正选择且完成 nonlinear allocator 校核的输出，不是未经执行的 nominal candidate。

## 4. 精简实时监视命令

实现位于 `HydroRateControl.cpp:888-935`：

```text
hydro_rate_control safe_roll_watch
hydro_rate_control safe_roll_watch 10
```

- 默认 5 Hz，允许范围 1～20 Hz；非法值直接报错。
- Ctrl+C 结束。
- 命令在调用它的 shell command context 内创建独立
  `haar_safe_roll_status` subscription 并休眠限频；没有向控制 work queue 增加循环打印，
  没有新增 uORB topic。
- 每行固定 16 列，无标题：

```text
timestamp requested_code active_code phi_deg p_deg_s u_nominal u_selected
allocator_valid allocator_converged allocator_residual allocator_saturation_mask
allocator_invalid_streak allocation_interval_degraded temporary_fallback_active
first_fault_code fallback_latched
```

`fallback_latched` 由现有 `FallbackLatchedFault` 数字位推导。命令只读取现有
`haar_safe_roll_status`。

## 5. 新增诊断

控制器输出结构定义见 `SafeRollController.hpp:194-209`，发布见
`HydroRateControl.cpp:562-568`。

为避免扩大高速主状态消息导致 logger frame/format 上限超标，诊断分布在已有消息中，
没有新增 topic：

- `haar_safe_roll_status`
  - `temporary_fallback_active`
  - 继续保留 activation/invalid streak 和实时监视所需字段
- `haar_safe_roll_estimator_status`
  - `allocator_recovery_streak`
  - `allocator_invalid_duration_us`
  - `allocator_degraded_active`
  - `temporary_fallback_active`
  - `safety_degraded_active`
  - `last_feasible_u`
- `hydro_cadrc_control_message`
  - `temporary_fallback_active`，供 ControlAllocator 转发到主状态消息

## 6. 正负方向离线扫描

测试位于
`src/lib/hydrofoil_allocation/HydrofoilAllocationModelTest.cpp:276-355`。

### 6.1 扫描设置

- 使用历史软件量级的对称 context：`speed_squared=1`、body pitch 3 deg、
  `force_x=24`、左右相同 angle/thrust 上界和反号 roll effectiveness。
- `u_phi` 以 0.01 步长扫描 `[-1,+1]`。
- 对每对 `+u/-u` 比较 valid、converged、iterations、镜像 saturation mask、
  residual、左右 delta、左右 thrust 和局部 `g_left/g_right`。
- 额外遍历 `phi=-10…+10 deg` 和 `p={0,±2,±5} deg/s`。

`phi` 和 `p` 并不是 `HydrofoilAllocationModel::AllocationContext/AllocationCommand` 的输入，
所以在固定 allocation context 下它们不会改变 solver 结果；测试明确验证了这一点，
没有伪造模型中不存在的依赖。

### 6.2 结果

```text
MIRROR_SCAN valid_each=101 feasible=[-1.00,1.00]
delta_err=0 thrust_err=0 residual_err=0 mask2=DeltaRight
```

**VERIFIED**

- 对称配置下正负 `u` 全扫描 valid/converged 结果一致。
- `delta_left(+u)=delta_right(-u)`，`delta_right(+u)=delta_left(-u)`。
- 左右 thrust 同样镜像。
- saturation mask 的 DeltaLeft/DeltaRight 和左右 thrust 位正确交换。
- 局部灵敏度满足 `g_left(+u)=-g_right(-u)`、
  `g_right(+u)=-g_left(-u)`。
- solver 对左右水翼使用相同 `[-max,+max]` 角度边界，对左右推进器使用相同
  `[0,available]` 边界，见 `HydrofoilAllocationModel.cpp:122-136,189-205`。
- 未发现负 `u` 的符号反转、边界交换或错误夹紧。

**参数/执行链说明**

- 历史 CS3/CS4 generic trim 均为 `-0.1`，相同 trim 本身保持逻辑镜像。
- PWM REV 位于共享模型下游；本次按要求没有修改。
- 实机当前 allocation context、generic contribution、上一 final commanded state 和速率区间
  可以形成局部非对称可行域；需用异常段 ULog 的完整字段才能量化。此次没有为了制造
  对称而改动真实 trim 或几何参数。

## 7. 自动测试

执行：

```text
CCACHE_DIR=/tmp/px4_ccache ninja -C build/px4_sitl_test_phase1 \
  unit-SafeRollController unit-HydrofoilAllocationModel
build/px4_sitl_test_phase1/unit-SafeRollController --gtest_color=no
build/px4_sitl_test_phase1/unit-HydrofoilAllocationModel --gtest_color=no
```

结果：

- SafeRollController：44/44 PASS。
- HydrofoilAllocationModel：10/10 PASS。
- 总计：54/54 PASS。
- 原 50 项全部保留并通过；新增 4 个测试组，其中循环覆盖 Modes 1/2/3。
- Mode 0 的 legacy CADRC 状态及输出 `EXPECT_FLOAT_EQ` 逐样本回归继续通过。
- 本次 host benchmark：
  - SafeRoll candidate average 2.578 us，observed max 34.045 us；
  - allocator solve/forward/linearize average 0.311 us，observed max 38.011 us。
  这些值仅用于主机回归，不作为 H743 WCET。

新增/扩展覆盖：

- `SafeRollControllerTest.cpp:893-921`：第 1～20 拍保持 last feasible，不锁存；
- `:922-956`：第 21～80 拍 temporary legacy，第 81 拍准确锁存；
- `:957-994`：连续健康 20 拍自动重新 blend；
- `:995-1053`：nonfinite、stale、gate、mapping 继续立即 fatal；
- `:1054-1072`：Mode 4 行为不变；
- `HydrofoilAllocationModelTest.cpp:276-355`：正负方向镜像扫描。

## 8. H743 clean `-Werror` 构建

执行：

```text
CCACHE_DIR=/tmp/px4_ccache cmake --build build/matek_h743_dummy --target clean
CCACHE_DIR=/tmp/px4_ccache ninja -C build/matek_h743_dummy \
  /home/linyi666/PX4-Autopilot-water-air/platforms/nuttx/NuttX/nuttx/.config
CCACHE_DIR=/tmp/px4_ccache make matek_h743_dummy -j2
```

结果：PASS，1063 个构建步骤完成，项目默认 `-Werror` 生效。

```text
FLASH:    1,586,216 / 1,966,080 bytes = 80.68%
AXI SRAM:    44,716 /   524,288 bytes = 8.53%
```

固件：

```text
/home/linyi666/PX4-Autopilot-water-air/build/matek_h743_dummy/matek_h743_dummy.px4
size: 1,524,417 bytes
SHA256: 9eae28ca4c396dad059133f4451f0bf8a19a68565116535dd96da2cc52e8ee0b
```

## 9. 本次修改范围

- `src/modules/hydro_rate_control/SafeRollController.hpp`
- `src/modules/hydro_rate_control/SafeRollController.cpp`
- `src/modules/hydro_rate_control/SafeRollControllerTest.cpp`
- `src/modules/hydro_rate_control/HydroRateControl.cpp`
- `src/lib/hydrofoil_allocation/HydrofoilAllocationModelTest.cpp`
- `msg/HaarSafeRollStatus.msg`
- `msg/HaarSafeRollEstimatorStatus.msg`
- `msg/HydroCadrcControlMessage.msg`
- `src/modules/control_allocator/ControlAllocator.cpp`
- `src/modules/logger/messages.h`
- 本报告

没有修改 `CADRC.cpp/.h`、manual mapping、PHIS、DMAX、DRATE、HOCBF、证书数学、
nonlinear allocator 方程、generic+hydro 叠加、trim 或通道映射。

## 10. 停止点

软件准备和构建已经完成。没有自动刷写固件，没有执行 actuator test，也没有进行实机或
水池测试。正滚转异常的完整物理/上下文根因仍需下一次实机记录完整 allocation context
后才能逐样本复现；本修复只改变瞬时 `valid/converged` 掉线的时间容错和可观测性，
没有用放宽 solver 有效阈值掩盖方向性问题。
