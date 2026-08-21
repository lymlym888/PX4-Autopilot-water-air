# HAAR SafeRoll 最终实现摘要

## 结论

状态：**软件实现与主机/构建验证完成；未烧写、未执行 actuator test、未进行水池试验。**

本实现基于 Git `5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`（分支 `RSMC`，PX4 `v1.15.0-beta1-346-g5ae2fd95ae`）。默认 `HY_SR_MODE=0`、`HY_SR_ENABLE=0`，因此安装固件后仍直接使用旧 CADRC。旧 `CADRC.cpp`、`CADRC.h` 未修改。

默认工具链 clean build 已通过，固件为：

```text
build/matek_h743_dummy/matek_h743_dummy.px4
SHA256 a2a3c900442ff08ef417d63a528a36a7ff833244af760c1883ae3d6432104345
```

首次 clean 并行构建曾出现 NuttX `jlink-nuttx` 先于 `.config` 生成的竞态；原命令重跑后从 clean 状态完整链接成功。GCC 13 的既有 SPI `-Waddress` 问题通过有条件的指针空检查修正，见 `platforms/common/spi.cpp:88-96`。

## 已实现

- 数字 code 0–4、总使能、shadow、trial id、在线 reset 和配置 CRC32：`SafeRollController.hpp:17-95`、`SafeRollController.cpp:196-277`。
- mode 0 不计算新控制数学，直接返回旧 CADRC；从新模式切回 0 时先连续融合，融合完成后恢复逐样本直通：`SafeRollController.cpp:756-786`、`:1103-1151`。
- 二阶人工滚转参考滤波器：`SafeRollController.cpp:403-420`。输入仍是 `manual_control_setpoint.roll * radians(HY_ROLL_MAX)`：`HydroRateControl.cpp:254-258`。
- 因果创新、点估计、扰动估计、五个固定 lag 的集合证书：`SafeRollController.cpp:422-561`。
- nominal、固定最坏情况与自适应鲁棒 DE-HOCBF、最小松弛、闭式投影：`SafeRollController.cpp:287-385`、`:921-1039`。
- generic+hydro 的 allocation-aware commanded 翼角/速率区间，以及共享非线性模型最终校核和固定 8 次二分：`SafeRollController.cpp:583-723`。
- 异常锁存、增量受限且按 `HY_SR_BLEND` 平滑回退旧 CADRC：`SafeRollController.cpp:741-754`、`:1049-1099`。
- QGC 可在线更新的 41 个 `HAAR Safe Roll` 参数：`hydro_rate_control_params.c:137-537`；`parameters.xml` 验证结果为 41/41 均有范围、默认值，无 reboot 标志。
- 两条同步数字日志：`HaarSafeRollStatus.msg:1-80`、`HaarSafeRollEstimatorStatus.msg:1-61`。消息中没有控制器名称字符串。

## 保持不变的链路

- roll/pitch/yaw/throttle 的人工输入映射未改变；STAB roll command 仍在 `HydroRateControl.cpp:254-258` 生成。
- pitch、yaw、throttle 和 MANUAL/ACRO 路径不使用新控制器。
- legacy CADRC 始终先按原顺序计算，mode 0 直接使用其归一化输出：`HydroRateControl.cpp:269-303`、`:396-403`。
- generic trim、hydro 叠加和最终 `[-1,1]` constrain 保留：`ControlAllocator.cpp:850-876`。
- 逻辑映射仍为 Servo5/control[4] 左、Servo4/control[3] 右；MAP_OK/OWN_OK 仅记录，不再阻断输出：`SafeRollController.cpp:140-143`。

## 验证摘要

- SafeRoll 主机测试：25/25。
- 共享分配器测试：9/9。
- 总计：34/34。
- mode 0：旧 CADRC 状态及输出逐样本 bit-exact 测试通过，见 `SafeRollControllerTest.cpp:79-119`。
- 共享 allocator：legacy 重构、饱和区、forward/solve、Jacobian、异常求解测试通过，见 `HydrofoilAllocationModelTest.cpp:124-227`。
- 400 点完整合成序列 golden 回归通过：`SafeRollControllerTest.cpp:448-521`。
- 独立主机基准：candidate 平均 2.579 us、最大 47.242 us；allocator solve+forward+linearize 平均 0.302 us、最大 9.756 us。它们不是 H743 WCET。
- 默认 H743 clean build：通过。最终 ELF `text=1,578,128`、`data=3,544`、`bss=41,324`。
- 相对 Phase 1.5 ELF：text `+16,176 B`，data/bss 无变化。

## 尚未由本阶段证明

- 未找到 `RSMC_codex/reference/HAAR_TII_SectionIV_Simulation_v7.m`。因此 estimator/HOCBF golden 是依据提示公式生成的 PX4 合成基准，不是 MATLAB V7 逐点交叉验证。
- 没有目标 H743 的实测 WCET、work queue 负载、栈 high-water、logger 开关对比或 gyro-to-output 延迟。静态栈使用最大值见运行安全报告。
- 没有水翼位置传感器。所有翼角均为 commanded/reconstructed；`u_effective_model` 是模型重构，不是测量。
- SafeRoll 上游只能读取上一条 `haar_safe_roll_status` 的 generic/final 命令；因此 allocation query 使用最近完成的下游样本。最终下游状态由 ControlAllocator 用同拍 pre-PWM final command 再重构并记录。
- 生成固件不等同于完成实机放行。刷写、台架、目标板性能和受控水池步骤必须由人工执行。

