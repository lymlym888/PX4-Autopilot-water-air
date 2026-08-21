# HAAR SafeRoll QGC 使用指南

## 前提

固件默认保持旧控制链：

```text
HY_SR_MODE   = 0
HY_SR_ENABLE = 0
HY_SR_SHADOW = 0
```

新控制输出只允许在 STAB、`HY_CADRC_EN=1`、foil gate 开启、传感器/时间戳/allocator/逻辑索引有效时工作。MANUAL、ACRO 和其他模式不进入 SafeRoll。

## 1. 启用与数字模式

在 QGC 参数页面搜索 `HAAR Safe Roll`：

1. 设置 `HY_SR_TRIAL` 为本次试验编号。
2. 选择 `HY_SR_MODE`：
   - 0：Legacy CADRC
   - 1：Nominal control
   - 2：WC fixed DE-HOCBF
   - 3：Adaptive safe control
   - 4：No-allocation ablation
3. 设置总使能：
   - `HY_SR_ENABLE=0`：任何 MODE 都由 code 0 驱动；
   - `HY_SR_ENABLE=1`：允许 code 1–3；
   - `HY_SR_ENABLE=2`：允许 code 1–4。

这些参数通过 `parameter_update` 在线读取，不需要重新编译或重启飞控。

## 2. Shadow

- `HY_SR_SHADOW=0`：合法 candidate 经 blend 后驱动输出。
- `HY_SR_SHADOW=1`：仍计算所选 code 和全部证书/区间，但旧控制输出驱动执行链。

Shadow 是可选诊断，不是启用前强制阶段。日志中用 `shadow_active` 和 `output_enabled` 区分计算与实际驱动。

## 3. 在线调参

所有 `HY_SR_*` 参数均可在线修改。建议一次只改变一个参数，并保存修改前后 `configuration_crc32`。

- 增益、滤波器、DUMAX、BLEND 等在下一拍生效。
- 模型、安全边界、先验/变化界、证书阈值和翼角约束变化会 reset 新控制器，并从上一执行输出重新 blend。
- `HY_SR_RST_ID` 每次改变都会 reset 参考滤波、b/d estimator、证书 history 和 blend；不会 reset legacy CADRC。
- 非法值或非法 `BMIN/B0/BMAX` 关系会拒绝 candidate 并回退。

## 4. 确认运行状态

在 MAVLink Console：

```text
listener haar_safe_roll_status 5
listener haar_safe_roll_estimator_status 5
```

检查两条消息：

- `timestamp_sample`、`trial_id`、`active_code`、`configuration_crc32` 必须一致；
- `requested_code` 是参数请求；
- `active_code` 是正在有效计算的数字 code；shadow 时还必须结合 `output_enabled=false` 判断实际仍由旧输出驱动；
- `fault_flags=0`、`healthy=true`、`candidate_valid=true`、`interval_valid=true` 才表示新 candidate 健康；
- `foil_position_feedback_available=false` 与 `u_effective_is_model_based=true` 是当前硬件的预期值。

`configuration_crc32` 覆盖影响 SafeRoll 结果的参数和 requested code，不包括 trial id。相同试验配置应得到相同 CRC。

## 5. 切回旧控制

优先操作：

```text
HY_SR_MODE = 0
```

也可设置：

```text
HY_SR_ENABLE = 0
```

MODE 切回 0 时先在 `HY_SR_BLEND` 时间内平滑回到实时 legacy，之后恢复 mode-0 直接路径。故障回退会锁存；修复原因后，需要再次改变 `HY_SR_MODE` 或 `HY_SR_RST_ID` 才会尝试重新进入新 code。

## 6. Code 4

只有 `HY_SR_ENABLE=2` 才允许 code 4 输出。它不使用 allocation-aware interval，但仍要求 allocator/context/foil gate/数值有效，并做有限的共享非线性 candidate 校核。该 code 仅用于受控消融试验。

## 7. 实验前最小检查

```text
param show HY_SR_*
param show HY_CADRC_EN
param show HY_FOIL_*
param show HY_ALLOCATE_MODE
hydro_allocator status
listener haar_safe_roll_status 5
listener haar_safe_roll_estimator_status 5
```

本指南不授权自动刷写、actuator test 或水池动作。固件刷写与实验仍按组织的硬件安全流程人工执行。

