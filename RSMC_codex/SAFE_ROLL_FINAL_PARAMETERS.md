# HAAR SafeRoll 最终参数

所有参数位于 QGC 分组 `HAAR Safe Roll`，源码 `hydro_rate_control_params.c:137-537`。生成的 `parameters.xml` 通过 XML 检查：41 个参数、非布尔参数均有 min/max、没有 reboot-required 标志。

## Operation

| 参数 | 默认 | 范围/枚举 | 在线行为 |
|---|---:|---|---|
| HY_SR_MODE | 0 | 0..4 | 请求数字 code；变化后 reset/blend |
| HY_SR_ENABLE | 0 | 0 legacy only；1 allow 1–3；2 allow 1–4 | 变化后 reset/blend |
| HY_SR_SHADOW | 0 | 0 output；1 calculate only | 在线 blend |
| HY_SR_TRIAL | 0 | 0..65535 | 仅日志，不控制 |
| HY_SR_BLEND | 0.30 s | 0.05..2.0 | 下一拍生效 |
| HY_SR_RST_ID | 0 | 0..2147483647 | 值变化 reset 新控制器 |
| HY_SR_MAP_OK | 0 | boolean | deprecated diagnostic，不阻断 |
| HY_SR_OWN_OK | 0 | boolean | deprecated diagnostic，不阻断 |

## Reference filter

| 参数 | 默认 | 范围 | 单位 |
|---|---:|---:|---|
| HY_SR_RFWN | 4.0 | 0.5..20 | rad/s |
| HY_SR_RFZ | 1.0 | 0.5..2.0 | 1 |
| HY_SR_RFPMAX | 120 | 10..360 | deg/s |
| HY_SR_RFAMAX | 500 | 50..2000 | deg/s² |

## Model, nominal and hard input

| 参数 | 默认 | 范围 | 说明 |
|---|---:|---:|---|
| HY_SR_A0 | -0.50 | -20..20 | rate model coefficient, 1/s |
| HY_SR_B0 | 10.1746 | 0.1..100 | nominal effectiveness |
| HY_SR_BMIN | 6.50 | 0.1..100 | certified prior lower |
| HY_SR_BMAX | 13.00 | 0.1..100 | certified prior upper |
| HY_SR_K1 | 15.0 | 0..1000 | angle error gain |
| HY_SR_K2 | 7.0 | 0..1000 | rate error gain |
| HY_SR_UMAX | 1.0 | 0.1..1.0 | normalized amplitude |
| HY_SR_DUMAX | 0.009 | 0.0001..0.2 | nominal 2.5 ms increment |

运行时还要求 `BMIN <= B0 <= BMAX`，见 `SafeRollController.cpp:145-193`。

## Estimator and certificate

| 参数 | 默认 | 范围 |
|---|---:|---:|
| HY_SR_FLAM | 0.982 | 0.8..1 |
| HY_SR_GB | 0.055 | 0..1 |
| HY_SR_EPSB | 0.010 | 1e-6..10 |
| HY_SR_UFMIN | 0.015 | 0..1 |
| HY_SR_IGATE | 0.75 | 0.001..100 |
| HY_SR_BDMAX | 0.006 | 0..1 |
| HY_SR_LD | 0.18 | 0.001..1 |
| HY_SR_RD0 | 0.06 | 0..100 |
| HY_SR_DDB | 0.0060 | 0..1 |
| HY_SR_DBB | 0.00405 | 0..1 |
| HY_SR_IUMIN | 0.018 | 0..2 |
| HY_SR_IRES | 0.0015 | 0..10 |
| HY_SR_IGFAC | 1.25 | 1..100 |
| HY_SR_RB0 | 0.35 | 0..100 |

## HOCBF

| 参数 | 默认 | 范围 | 单位 |
|---|---:|---:|---|
| HY_SR_PHIS | 20 | 10..35 | deg |
| HY_SR_G1 | 0.011 | 1e-6..10 | 1 |
| HY_SR_G2 | 0.011 | 1e-6..10 | 1 |
| HY_SR_BFLOOR | 0.50 | 0.01..100 | effectiveness |

## Allocation

| 参数 | 默认 | 范围 | 单位 |
|---|---:|---:|---|
| HY_SR_DMAX | 35 | 1..65 | deg commanded |
| HY_SR_DRATE | 90 | 1..720 | deg/s commanded |
| HY_SR_DREM | 0.05 | 0..2 | deg commanded |

`HY_SR_DREM` 已从 Phase 1.5 的错误 rad 语义修正为 deg；内部在 `HydroRateControl.cpp:371` 转成 rad。默认内部值为 `0.00087266463 rad`，单元测试见 `SafeRollControllerTest.cpp:380-385`。

## 在线生效与 reset

`parameter_update` 在循环中读取全部参数：`HydroRateControl.cpp:137-148`。

- 下一拍直接生效：K1/K2、G1/G2、reference gains/limits、point/disturbance estimator gains、DUMAX、BLEND、ENABLE/SHADOW。
- 触发新控制器 reset 并重新 blend：RST_ID、A0/B0/BMIN/BMAX、PHIS、DDB/DBB、RD0/RB0、incremental thresholds、DMAX/DRATE/DREM，以及 requested code 改变。
- TRIAL 仅改变日志标记。
- 所有参数先做 finite、范围和上下界关系校验；非法配置置 fault 并平滑回退。

