# HAAR SafeRoll Golden 测试

## 参考来源

工作区中没有找到：

```text
RSMC_codex/reference/HAAR_TII_SectionIV_Simulation_v7.m
```

因此不能宣称与 MATLAB V7 逐样本交叉验证。本阶段根据用户提供的完整公式建立了两类 golden：

1. estimator/HOCBF 的 400 样本固定合成序列；
2. PX4 共享真实非线性 allocator 的独立回归向量。

## 完整合成序列

测试位于 `SafeRollControllerTest.cpp:448-521`。输入包含固定 dt=2.5 ms 的 phi、p、manual reference、generic contribution、allocator context 和前一执行状态；在样本 50、100、200、399 固定比较：

- b_hat；
- b_lower/b_upper；
- rho_b；
- d_hat/rho_d；
- u_nominal；
- CBF lower/upper；
- hard lower/upper；
- u_candidate；
- epsilon_relax；
- psi_min。

容差为 `2e-6`，400 个样本回归通过。

## 公式级测试映射

- causal point/disturbance estimator：`SafeRollControllerTest.cpp:249-279`
- nominal formula：`:280-286`
- fixed/adaptive CBF：`:287-300`
- minimum relaxation/project：`:301-313`
- single/multi-lag：`:314-335`
- conflict discard 与 variable-dt scaling：`:336-347`
- allocation-aware 与 code 4：`:348-364`
- generic+hydro commanded state：`:365-379`
- DREM deg→rad：`:380-385`
- CRC/reset policy：`:386-411`
- nonfinite/empty interval：`:412-430`

## Shared allocator golden

`HydrofoilAllocationModelTest.cpp:124-227` 覆盖：

- 正常区 legacy solver 逐量比较；
- 正负 roll、pitch、throttle 与 saturation；
- solve/forward 一致性；
- analytic sensitivity 对中心有限差分；
- singular、NaN、nonconvergence；
- commanded angle/rate interval。

此部分以 PX4 共享模型为唯一真值，没有复制 MATLAB 简化映射。

## 限制

这些 golden 可防止软件公式和重构回归，但不能替代：

- MATLAB V7 独立实现交叉核对；
- H743 浮点/调度时序实测；
- 植物参数与水池实物辨识。

获得 V7 文件后，应新增离线向量生成与逐字段导入测试，而不是修改现有 golden 去追随运行结果。

