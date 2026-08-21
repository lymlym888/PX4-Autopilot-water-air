# HAAR SafeRoll 实机运行包络审计

日期：2026-07-24  
源码基线：`5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2` 加当前 SafeRoll 软件修改  
性质：只读源码/ULog/离线模型审计。除本报告外，没有修改控制数学、参数默认值、故障逻辑或构建文件。

## 1. 本轮采用的运行包络

本审计按以下需求解释所有结果：

- 实际滚转角允许约 `[-30,+30] deg`；
- 平稳滑行通常在 `±5 deg` 内，但超过 `5 deg` 本身不是异常；
- raw RC roll reference 允许 `±40 deg`；
- raw reference 可在 0.5 s 内从 0 到 `±40 deg`，即约 `80 deg/s`；
- `|phi_ref-phi|=30~40 deg` 是合法瞬态，不能单独触发 fault/fallback；
- 控制幅值/速率饱和与 best-feasible allocation 是正常降级，不是软件 fatal。

结论先行：

1. **VERIFIED：**当前没有直接按 `|phi|`、tracking error、`|u_nominal|` 或有限
   reference slew 设置 fault 的代码。
2. **VERIFIED：**有限的 `u_nominal` 会投影到硬区间；reference rate/acceleration
   会被限制但不置 fault。
3. **NOT COMPLIANT：**当前 Modes 2/3 的空 allocation/hard interval，以及完整 nonlinear
   candidate check 的某些“有限但不可行”结果，仍可能被升级为 fatal。这是大误差或饱和
   **间接**导致永久 fallback 的路径。
4. **VERIFIED：**当前三级 allocator 短时降级期间 `healthy` 保持 true；第 81 个连续
   invalid 才 fatal。短时降级本身不永久锁存。
5. **DATA LIMITATION：**所选历史室内 ULog 来自旧 CADRC/allocator，消息中没有新版
   `allocator_valid/converged/residual`，因此不能从这些日志直接统计真实 invalid streak。

## 2. Fault 条件完整审计

Fault 位定义在
`src/modules/hydro_rate_control/SafeRollController.hpp:42-64`。最终 fatal mask 在
`SafeRollController.cpp:1432-1433`，`healthy` 在 `:1522`。

### 2.1 前置 fatal 条件

| Fault | 完整源码条件 | 位置 | 与新包络关系 |
|---|---|---|---|
| `TimestampNonMonotonic` | gyro sample timestamp 不大于上一拍 | `SafeRollController.cpp:1016-1018` | 数字/时序 fault，合理 |
| `DtTooSmall` | 首拍或 `dt<1 ms` | `:1016-1032` | 时序 fault；首拍有既有特殊处理 |
| `DtTooLarge` | `dt>10 ms` | `:1021-1028` | 时序 fault，合理 |
| `GyroStale` | timestamp 0、来自未来或 age `>20 ms` | `:1034-1036` | 传感器 fault，合理 |
| `AttitudeStale` | timestamp 0、来自未来或 age `>50 ms` | `:1039-1041` | 传感器 fault，合理 |
| `QuaternionNonFinite` | quaternion finite 标志 false | `:1044-1045` | 数值 fault，合理 |
| `AngularVelocityNonFinite` | rates、phi、p、phi_cmd 或 legacy_u 任一非有限 | `:1048-1050` | 只检查 finite；不检查幅值，合理 |
| `ModeOutOfRange` | requested mode 不在 1～4 | `:1053-1054` | 配置 fault |
| `InvalidConfigurationFault` | SafeRoll 参数非有限、上下界/范围非法 | `:1057-1058`，校验 `:197-235` | 配置 fault |
| `ControllerDisabledFault` | 非 STAB，或 ENABLE 不允许 requested mode | `:1061-1073` | 模式/配置 fault |
| `LogicalMappingInvalidFault` | logical mapping false | `:1065-1066` | 映射 fault |
| `FoilGateDisabledFault` | 已激活后 gate false | `:1115-1116` | 执行链不可用，立即 fatal |
| `AllocatorInvalidFault` | allocator nonfinite 或 timestamp age `>100 ms` 立即；valid/converged 连续第 81 拍 | `:1076-1081,1111-1142` | 短时非 fatal，持续/严重错误 fatal |

Mode 4 仍使用立即 allocator fault 路径，见
`SafeRollController.cpp:1192-1200`。

### 2.2 候选计算 fatal 条件

| Fault | 条件 | 位置 | 包络审计 |
|---|---|---|---|
| `InvalidCertificateFault` | estimator/certificate 返回 false | `:1231-1233` | 数值/证书结构错误；Mode 1 不要求 adaptive certificate |
| `EmptyHardIntervalFault` | `U_amp ∩ U_rate ∩ U_alloc` 非有限/为空 | `:1235-1259` | **可能由正常饱和/可行域收缩触发，当前不符合新语义** |
| `ProjectionInvalidFault` | nominal 非有限、safe interval 无效、融合/psi 非有限或 nonlinear fault 已出现 | `:1272-1273,1307-1310,1376-1378` | 非有限合理；由有限不可行间接触发时需降级分类 |
| `NonlinearCandidateInvalidFault` | projected candidate 与回缩基点均不能通过 nonlinear solve/final angle/rate check，或 blended candidate 失败 | `:1313-1341`；check 条件 `:762-821` | **有限但饱和/不可行也会 fatal，当前不符合 best-feasible 正常降级语义** |
| `RuntimeOverrunFault` | 单拍 `>2500 us`，或连续 5 拍 `>1000 us` | `:378-407,1425-1428` | 实时性 fault，与 tracking error 无关 |

`CertificateConflictFault` 会记录，但不在 fatal mask 内，见
`SafeRollController.cpp:1397-1399,1432-1433`。

### 2.3 A 项逐条结论

#### Tracking error 过大

**没有直接 fault 阈值。**误差仅进入

```text
e1 = phi_ref - phi
e2 = phi_ref_dot - p
```

并用于 nominal formula，见 `SafeRollController.cpp:1262-1268`。
源码中没有 `abs(e1)>...` 或 `abs(e2)>...`。

但是大误差可能产生较大的有限 `u_nominal`，再把 candidate 推向 allocation 边界；若最终
hard interval 为空或 nonlinear check 找不到可行基点，当前会**间接** fatal。因此不能给出
“完全没有间接依赖”的确认。

#### Nominal command 过大

**有限的大 `u_nominal` 不直接 fault。**它由
`project(u_nominal,u_safe)` 投影，见 `SafeRollController.cpp:1313`。只有
`u_nominal` 非有限才置 `ProjectionInvalidFault`，见 `:1272-1273`。

#### Command saturation

- `U_amp` 和 `U_rate` 投影是正常约束；
- CBF 与 hard interval 不相交时，仅松弛 CBF，见 `:1291-1301`；
- Mode 1 的 local allocation interval 无效或与 amp/rate 不相交时，会设置
  `allocation_interval_degraded=true` 并退回 amp/rate interval，见
  `SafeRollController.cpp:334-354`；
- Modes 2/3 的相同 allocation 空交集目前会产生 `EmptyHardIntervalFault`；
- 完整 allocator 饱和导致 solve invalid，或 final angle/rate 不满足时，
  `checkNonlinearCandidate()` 返回 false，见 `:762-800`。若上一执行点也失败，
  `validatedCandidate()` 返回 invalid，见 `:824-864`，随后成为 fatal。

所以“所有饱和均只记录状态”的目标目前**尚未完全满足**。

#### Reference 变化快但有限

reference filter 将 `phi_ref_dot` 限到 `HY_SR_RFPMAX`，将 acceleration 限到
`HY_SR_RFAMAX`，见 `SafeRollController.cpp:514-526`；限幅本身不置 fault。
默认分别为 `120 deg/s` 和 `500 deg/s²`，参数见
`hydro_rate_control_params.c:283-301`。合法的 80 deg/s 在默认 rate limit 内。

## 3. ULog 选择与方法

按用户建议，每类选择 2 条、共 6 条：

### 静态

- `已有ulg/室内静态实验/TII已用(CADRC)/log_360_2026-1-28-11-32-58.ulg`
- `已有ulg/室内静态实验/TII已用(CADRC)/log_355_2026-1-28-11-29-00.ulg`

### 动态

- `已有ulg/室内动态实验/TII已用(CADRC)/log_367_2026-1-28-11-35-10.ulg`
- `已有ulg/室内动态实验/TII已用(CADRC)/log_370_2026-1-28-11-37-14.ulg`

### 扰动

- `已有ulg/室内扰动实验/TII已用(CADRC)/log_371_2026-1-28-11-40-02.ulg`
- `已有ulg/室内扰动实验/TII已用(CADRC)/log_374_2026-1-28-11-42-22.ulg`

统计只使用 `armed=true` 且历史 foil AUX gate 开启的样本。

- phi：由 `vehicle_attitude.q` 转 Euler roll；
- p：`vehicle_angular_velocity.xyz[0]`；
- reference：用带有效 timestamp 的 `manual_control_setpoint.roll × HY_ROLL_MAX`；
- reference rate：相邻有效 manual samples 的有限差分；
- tracking error：将 attitude 按 timestamp 插值到 manual sample；
- 分位值均对绝对值统计。

这些旧日志的 `hydro_cadrc_control_message.timestamp` 全为 0，不能用该 topic 计算 reference
slew；这也是改用 manual topic 的原因。

所选日志的 `HY_ROLL_MAX=30 deg`，因此它们不能实证当前要求的 raw `±40 deg` 全包络。

## 4. ULog 分位统计

格式为 `P50 / P95 / P99 / P99.5 / max`。

### 4.1 |phi|，deg

| 实验类别 | P50 | P95 | P99 | P99.5 | max |
|---|---:|---:|---:|---:|---:|
| 静态 | 1.775 | 3.391 | 4.622 | 5.035 | 5.856 |
| 动态 | 2.338 | 22.345 | 25.224 | 25.372 | 25.735 |
| 扰动 | 1.575 | 3.152 | 12.856 | 20.732 | 32.733 |
| 合并 | 1.713 | 7.754 | 21.322 | 24.428 | 32.733 |

结论：

- 静态/平稳样本主要在 ±5 deg，符合经验；
- 动态和扰动样本明确证明超过 5 deg 不是软件异常；
- 扰动日志最大 32.733 deg，已经超过候选 `±30 deg`。因此 `±30 deg` 目前只能称
  “physical hard boundary candidate/operational target”，不能宣称是已验证绝对物理边界。

### 4.2 |p|，deg/s

| 实验类别 | P50 | P95 | P99 | P99.5 | max |
|---|---:|---:|---:|---:|---:|
| 静态 | 4.494 | 19.293 | 29.349 | 31.416 | 43.088 |
| 动态 | 4.878 | 46.037 | 70.803 | 76.320 | 82.069 |
| 扰动 | 3.370 | 17.861 | 75.178 | 108.809 | 181.382 |
| 合并 | 4.000 | 20.912 | 53.192 | **74.943** | 181.382 |

扩展 allocation 扫描采用合并 P99.5，即 `|p|=74.943 deg/s`。需要注意：p 不是当前
`HydrofoilAllocationModel` 的直接输入，它影响 controller candidate，而不是 solver 方程。

### 4.3 |raw reference|，deg

| 实验类别 | P50 | P95 | P99 | P99.5 | max |
|---|---:|---:|---:|---:|---:|
| 静态 | 0.000 | 0.000 | 0.245 | 0.398 | 0.798 |
| 动态 | 0.000 | 29.632 | 29.632 | 29.652 | 29.693 |
| 扰动 | 0.000 | 2.201 | 3.429 | 3.672 | 4.898 |
| 合并 | 0.000 | 4.228 | 29.571 | 29.632 | 29.693 |

### 4.4 |raw reference finite-difference rate|，deg/s

| 实验类别 | P50 | P95 | P99 | P99.5 | max |
|---|---:|---:|---:|---:|---:|
| 静态 | 0.000 | 0.000 | 1.230 | 1.309 | 3.323 |
| 动态 | 0.000 | 37.542 | 130.927 | 159.054 | 186.137 |
| 扰动 | 0.000 | 0.772 | 4.283 | 9.492 | 23.294 |
| 合并 | 0.000 | 1.405 | 31.682 | 38.660 | 186.137 |

最大值和动态高分位包含 manual topic 的离散更新效应，不等同于连续摇杆速度，但它说明
reference governor 必须把有限快速更新视为正常输入。`80 deg/s` 不应触发 fault；默认
`HY_SR_RFPMAX=120 deg/s` 可以支持目标连续 slew，超过 rate limit 时也只能记录 limiting。

### 4.5 |tracking error|，deg

| 实验类别 | P50 | P95 | P99 | P99.5 | max |
|---|---:|---:|---:|---:|---:|
| 静态 | 1.807 | 3.423 | 4.675 | 5.080 | 5.677 |
| 动态 | 1.881 | 15.359 | 28.008 | 29.684 | 29.903 |
| 扰动 | 1.587 | 3.166 | 14.956 | 20.043 | 28.543 |
| 合并 | 1.690 | 4.089 | 15.913 | 23.630 | 29.903 |

这些只覆盖历史 `HY_ROLL_MAX=30 deg`。新的 30～40 deg 合法 transient 必须通过设计输入
明确覆盖，不能因为旧数据最大约 30 deg 而设置 fault threshold。

### 4.6 软件 throttle

所选 armed+gate 样本的 signed throttle：

```text
min=-1.000
P50=-1.000
P95=0.000
P99=0.129
P99.5=0.146
max=0.180
```

历史 `HY_TH_MAX_GAIN=3`，所以 available thrust 在 throttle `>=-1/3` 时已经夹到最大 30。

## 5. 历史日志中的 allocator invalid 限制

### 5.1 直接统计

**UNKNOWN / 不可从所选 ULog 得出。**

旧 `hydro_allocate_message` 仅有 `thrust_xz_control`、`thrust_xz_limited` 和
`actuator_optimed`，没有：

- allocator_valid；
- allocator_converged；
- allocator_residual；
- saturation_mask；
- iterations。

旧 `HydroAllocator::optim()` 固定最多运行 10 次，夹紧后无论残差如何都发布结果；基线源码
见 Git HEAD 版本的 `HydroAllocator.cpp` 旧 `optim()` 和 `Run()`。因此不能把当时发布的
饱和命令反推成“当时固件记录的 invalid”。

### 5.2 用当前严格模型重放旧输入

作为兼容性检查，使用当前共享模型的 10 次 Newton、`1e-3` residual tolerance 和旧日志
参数重放。结果几乎所有 armed+gate 样本都会被当前严格语义判 invalid：

| 类别 | 重放 invalid 比例 | 最长连续段 |
|---|---:|---:|
| 静态 | 100.000% | 58.185 s |
| 动态 | 99.920% | 11.874 s |
| 扰动 | 99.996% | 72.814 s |

这不是历史实机真的连续掉线，而是明确的版本/语义不兼容：旧控制经常要求约
`Fz=-18…-36`，在 `KL=2`、`v²=1`、35 deg foil limit 和相应 available thrust 下，旧 solver
通过持续夹紧仍发布命令；当前模型则要求 residual 收敛才认为 valid。

因此这些重放持续时间**不能用于调整当前 20/80/81 拍容错阈值**。

### 5.3 正负方向重放差异

在上述旧输入重放中两侧几乎都 invalid，无法定义可信的“有效可行域”。但 residual 显示
负 `u_phi` 一侧更不利：

| 类别 | +u residual P50/P95 | -u residual P50/P95 |
|---|---:|---:|
| 静态 | 5.351 / 16.239 | 16.810 / 19.015 |
| 动态 | 4.540 / 16.289 | 16.627 / 23.603 |
| 扰动 | 5.322 / 17.601 | 17.224 / 18.595 |

这与实机负 `u_selected` 时 `DeltaRight` 饱和一致，但不能证明符号错误。历史 command 的
`Fz` 强烈负偏置；负 roll command 会进一步把逻辑右翼推向更负的 vertical-force 要求，
所以先触及 `DeltaRight` 是方程符号下的预期方向。

## 6. 扩展离线 allocation 扫描

### 6.1 扫描轴

- phi：`-30…+30 deg`；
- reference error：`-40…+40 deg`；
- p：`{-74.943,0,+74.943} deg/s`；
- throttle：`{-1,-0.75,-0.5,-1/3,0,+0.18}`，覆盖所选实验实际 signed 范围；
- u：`-1…+1`，步长 0.01。

重要接口事实：`HydrofoilAllocationModel` 的输入只有 allocation context 与
`force_x/force_z/u_phi/u_theta/u_psi`，见
`HydrofoilAllocationModel.hpp:35-51`。phi、tracking error 和 p 不直接进入 allocator。
在固定 allocation command/context 下，扩展这三个轴只会重复相同 solver 结果；它们应在
controller/golden time-series 扫描中用于生成 u，而不应伪装成 allocator 参数。

### 6.2 对称 shore-like context

为隔离左右符号，采用：

```text
KL=2
speed_squared=1
body_pitch=3 deg
max foil angle=35 deg
max thrust=30
force_x=24
force_z=0
u_theta=u_psi=0
left/right roll effectiveness=-10/+10
available thrust=clip(3*(throttle+1)/2,0,1)*30
```

结果：

| throttle | available thrust | valid u 范围 | 正负差异 | 主要边界 |
|---:|---:|---:|---|---|
| -1.00 | 0.00 | 无 | 正负一致 | thrust/foil saturation |
| -0.75 | 11.25 | 无 | 正负一致 | thrust high，随后 foil |
| -0.50 | 22.50 | `[-0.95,+0.95]` | 严格镜像 | 接近 ±1 时相应 foil angle |
| -0.333 | 30.00 | `[-0.95,+0.95]` | 严格镜像 | 同上 |
| 0.00 | 30.00 | `[-0.95,+0.95]` | 严格镜像 | 同上 |
| +0.18 | 30.00 | `[-0.95,+0.95]` | 严格镜像 | 同上 |

在可行 throttle 上：

- `valid/converged` 对 `+u/-u` 一致；
- residual、iterations 镜像一致；
- `DeltaLeft/DeltaRight` saturation bits 互换；
- thrust 左右互换；
- 没有发现负方向边界交换、错误 clamp 或符号错误。

当前 shore-like 对称扫描不能解释完整实机不对称；需要新版 ULog 中的
`command_force_x/z`、body pitch、available thrust、generic/final foil states 才能重放
首次 `DeltaRight` 异常拍。

## 7. 建议的三层安全范围

本节是评估，不修改参数。

### 7.1 Transparent operating region：建议约 ±20 deg

- 含义：控制器不因角度本身产生 fault；正常跟踪与约束投影应尽量透明。
- 合并 ULog P99 `|phi|=21.322 deg`，所以 ±20 deg 不是异常边界，只是“通常无需强安全
  干预”的近似区域。
- 动态日志 P95 已达 22.345 deg，超过 transparent region 必须是合法状态。

### 7.2 Formal safe boundary candidate：建议约 ±28 deg

- 含义：HOCBF/governor 使用的证书边界候选；
- 不是 fault threshold；
- 接近/越过时允许安全控制饱和、minimum relaxation 和 best-feasible command；
- 扰动日志 P99 `|phi|=28.777 deg`，说明约 ±28 deg 会在真实扰动中被触及，控制器必须
  把它视为需要干预的动态状态，而不是软件异常。

当前 `HY_SR_PHIS` 默认 20 deg，见
`hydro_rate_control_params.c:340-348`。是否改到约 28 deg 需要独立数学和实机审查，本轮
没有改参数。

### 7.3 Physical hard boundary candidate：约 ±30 deg

- 含义：实验/机械/姿态安全管理的候选极限，不应简单等同为软件 finite/fault 检查；
- 所选日志最大 32.733 deg，已超出 30 deg；
- 正式采用前必须核对姿态估计误差、结构干涉、翼面/推进器能力和人工终止策略；
- 在证据关闭前建议称“约 ±30 deg operational hard target”，不要宣称已证明的绝对
  physical hard boundary。

## 8. Reference governor 设计建议

本轮只设计，不实现。

### 8.1 信号层次

```text
manual roll
  -> phi_raw = manual_roll * radians(HY_ROLL_MAX)      # 允许并记录 ±40 deg
  -> feasible amplitude governor                       # 候选约 ±28 deg
  -> rate/acceleration governor                        # 至少支持 80 deg/s
  -> existing second-order reference state
  -> phi_ref / phi_ref_dot / phi_ref_ddot
```

Mode 0 必须继续使用 raw `roll_sp`，保持 bit-exact。

### 8.2 建议规则

- raw RC reference 始终记录，不因超过 ±28 deg 修改 `healthy`；
- internal feasible goal 可取
  `clamp(phi_raw,-phi_governor,+phi_governor)`，`phi_governor` 候选 28 deg；
- governor rate 下限不得低于 80 deg/s；当前默认 120 deg/s 满足；
- acceleration limit 保持显式并记录；
- amplitude/rate/acceleration limiting 只设置数字状态，例如：
  - `reference_amplitude_limited`
  - `reference_rate_limited`
  - `reference_limit_amount`
- limiter active 不置 fault、不锁存、不要求人工 reset；
- reset/mode switch 以当前执行 reference state 初始化，避免跳变；
- 如果 raw reference 回到可行域，自动平滑退出 limiting；
- 不根据试验结果动态改变数字 mode code。

当前源码默认 `HY_ROLL_MAX=30 deg`，但参数元数据允许到 80 deg，见
`hydro_rate_control_params.c:117-124`。实机若要求 raw ±40 deg，需要人工确认目标参数为
40；本轮没有修改默认值或实机参数。

## 9. 后续故障语义整改建议

这不是本轮实现清单，而是审计发现：

1. 将“数学/数据非法”与“有限但不可行”分开：
   - NaN、singular、stale、mapping/config invalid：fatal；
   - finite saturation、empty demand intersection、best-feasible boundary：degraded。
2. Modes 2/3 的 empty allocation interval 不应仅有 `EmptyHardIntervalFault` 一条路径；
   应保留 last feasible、按 DUMAX 退向实时 legacy，并记录 certificate unavailable。
3. nonlinear solver：
   - nonfinite/Jacobian structural failure 可 fatal；
   - finite saturated/nonconverged 应进入现有三级 allocator degradation；
   - 不得仅通过放宽 residual threshold 掩盖。
4. 建议增加非 fatal 数字状态：
   - `tracking_error_large`（诊断，不是阈值 fault）；
   - `u_amplitude_limited`；
   - `u_rate_limited`；
   - `foil_angle_limited`；
   - `foil_rate_limited`；
   - `best_feasible_active`；
   - `reference_amplitude_limited`；
   - `reference_rate_limited`。
5. `healthy` 应只反映软件、时序、传感器和结构性数值完整性；正常限制器 active 时保持
   true。
6. Modes 2/3 在 degradation 段继续置 `safety_degraded_active=true`，明确该拍不宣称
   HOCBF 已重新认证。

## 10. 最终审计结论

- 新给定的 `±30 deg` 姿态、`±40 deg` raw reference 和 30～40 deg tracking transient
  与源码中的 finite 检查不存在直接冲突。
- reference filter 的默认 120 deg/s rate limit 支持 80 deg/s 需求，发生有限限速不会直接
  fault。
- 当前实现仍存在“合法大误差/饱和通过 allocation 空区间或 nonlinear invalid 间接升级
  fatal”的路径，不能宣称 F 项完全满足。
- 历史 ULog 支持“±5 deg 是常态但不是边界”；动态/扰动样本到 25～33 deg。
- 历史 ULog 不含新版 allocator validity，不能用它们给当前 invalid streak 做直接统计；
  严格模型重放揭示旧 allocator 长期饱和语义与当前 valid 定义不兼容。
- 对称扩展扫描没有发现负 u 符号或左右 clamp 错误；真实方向不对称仍需新版完整
  allocation context ULog。
- 本轮没有修改数学、fault 逻辑、参数默认值或固件，没有执行 flash、actuator test 或实机
  测试。
