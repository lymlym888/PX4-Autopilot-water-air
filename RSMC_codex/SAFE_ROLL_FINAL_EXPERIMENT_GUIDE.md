# HAAR SafeRoll 数字试验指南

本指南只使用数字 code。不要把 ULog 数字字段替换为文字标签。

## 每次试验前

1. 记录固件 SHA256、参数导出和操作者。
2. 确认 `HY_CADRC_EN=1`、STAB、foil gate 和 allocator 状态。
3. 设置唯一 `HY_SR_TRIAL`。
4. 记录两条状态消息的 `trial_id`、`requested_code`、`active_code`、`configuration_crc32`。
5. 确认 `fault_flags=0`、`fallback_active=false`；新 code 还应有 `candidate_valid=true`、`interval_valid=true`。
6. 先使用小 roll stick、小速度、短时窗口；逐级扩大，不直接到边界或全行程。

## 建议顺序

```text
code 0
code 1
code 2
code 3
code 4
```

- code 0：`HY_SR_ENABLE=0` 或 `HY_SR_MODE=0`。
- code 1–3：`HY_SR_ENABLE=1`。
- code 4：仅 `HY_SR_ENABLE=2`，仅受控消融。

每次只改变一个数字 code 或一个参数组；改变后等待 blend 完成再开始有效采样。

## Shadow

需要只计算时设置：

```text
HY_SR_SHADOW=1
```

需要所选 code 驱动时设置：

```text
HY_SR_SHADOW=0
```

ULog 判断依据是 `shadow_active`、`output_enabled`、`active_code`，而不是字符串。

## ULog 必查字段

两条 topic 必须用相同：

```text
timestamp_sample
trial_id
active_code
configuration_crc32
```

同时检查：

```text
fault_flags
fallback_active
output_enabled
dt_raw
dt_used
u_legacy
u_candidate
u_blended
u_selected
u_effective_model
b_hat
b_lower
b_upper
rho_b
d_hat
rho_d
u_hard_lower
u_hard_upper
u_cbf_lower
u_cbf_upper
u_safe_lower
u_safe_upper
epsilon_relax
psi_min
generic_servo4
generic_servo5
hydro_servo4
hydro_servo5
final_servo4
final_servo5
delta_left_commanded
delta_right_commanded
foil_rate_left_commanded
foil_rate_right_commanded
total_candidate_time_us
```

这些 delta/rate 是 commanded；`u_effective_model` 是 reconstructed，不是 measured。

## 立即回到 code 0

正常退出：

```text
HY_SR_MODE=0
```

总禁用：

```text
HY_SR_ENABLE=0
```

两者都会平滑回到实时 code 0。发生锁存 fault 后，先结束试验并排查；不要用连续修改参数强行清故障。准备重新尝试时改变 `HY_SR_RST_ID` 或重新选择数字 code。

## 停止条件

出现以下任一项立即终止本次有效数据段并回到 code 0：

- `fallback_active=true`；
- fatal `fault_flags`；
- `candidate_valid=false` 或 `interval_valid=false`；
- 持续 dt 异常；
- `total_candidate_time_us` 接近/超过 2000；
- commanded angle/rate 接近人工批准范围；
- 执行器异常噪声、卡滞、振荡或机体响应不可预期。

本指南不执行 flash、actuator test 或水池动作；所有实机步骤由人工安全流程授权和执行。

