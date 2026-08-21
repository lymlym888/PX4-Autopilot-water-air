# HAAR safe-roll Phase 1.5 总结

日期：2026-07-23。源码基线为分支 `RSMC`、Git HEAD `5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`，PX4 版本 `v1.15.0-beta1-346-g5ae2fd95ae`。Phase 1 与 Phase 1.5 都是该 HEAD 上的未提交工作树变更。

## 已完成的软件准备

- **源码已验证：** `HY_SR_MODE` 仍默认 `0`；新增 `HY_SR_MAP_OK=0`、`HY_SR_OWN_OK=0`，代码不会自动置 1（`src/modules/hydro_rate_control/hydro_rate_control_params.c:135-207`）。
- **源码已验证：** modes 1–4 必须同时通过 sensor freshness/finite、allocator valid、foil gate、逻辑映射、人工物理映射确认和人工 ownership 确认；任一失败均 `active_mode=0`、`u_selected=u_legacy`（`SafeRollController.cpp:99-224`）。四个候选接口仍只返回 `ModeNotImplemented`（`:61-79`）。
- **源码已验证：** `safe_roll_hardware_ready = physical_mapping_verified && servo_ownership_verified && foil_gate_enabled && logical_mapping_valid`（`SafeRollController.cpp:55-59`）。精确状态字段已加入 `HaarSafeRollStatus.msg`。
- **源码已验证：** HydroAllocator 的 `status` 新增 AUX 选择、当前增益后 AUX 值、阈值、gate 结果、allocation mode、逻辑索引和 cycle perf 输出（`HydroAllocator.cpp:588-598`）；uORB 同时记录 `foil_aux_value`。
- **源码已验证：** Servo4/5 继续保持原有 `final=constrain(generic+hydro)`，没有清零或改 ownership；新增 generic/hydro/final 与一致性诊断（`ControlAllocator.cpp:843-963`）。
- **源码已验证：** 增加纯数学 trim 迁移比较工具，不接入运行控制路径。相同 legacy trim 与显式 neutral calibration 的归一化最终命令逐样本相等（`HydrofoilAllocationModel.cpp:337-374` 的 `composeNormalizedCommand()`、`compareTrimMigration()`）。

## Phase 1 基线保护

- checkpoint 前 `git status --short` 的 tracked 修改为：三个 uORB/CMake 文件、`src/lib/CMakeLists.txt`、ControlAllocator 两文件、HydroAllocator 三文件、HydroRateControl 四文件和 logger topic；untracked 为 `RSMC_codex/`、`HaarSafeRollStatus.msg`、`src/lib/hydrofoil_allocation/`、SafeRollController 两源码及测试。`CADRC.cpp/.h` 不在修改列表。
- checkpoint 前 tracked `git diff --stat`：14 files changed，374 insertions，170 deletions；完整 binary-capable diff 已保存在 checkpoint patch，untracked Phase 1 文件和报告保存在配套 tar.gz。
- Phase 1 固件 SHA-256：PX4 包 `b2eac5a779534321ca28c7457c80824b4a3ebe4c25254c8258bbbd7d981ce3b4`；ELF `81ef712fd144553d7b2df5457674a3aa0a83f8edd4f4ba4c361501b8a60792ae`；BIN `067d8cf72219156eb8ea1bdf3607910bd51ef4fa1307d085bc2450337973ef06`。
- Phase 1 本地 checkpoint 位于 `RSMC_codex/checkpoints/`：tracked patch SHA-256 `9c2bf08d5f9d960daa1d34addd92e4dd96aa623802fbfaee37746907c16f5665`；untracked archive SHA-256 `125737a922a0bd3360178a710be37c21808ebe96227a9318d9cab7d2754d3743`。未提交远端。
- Phase 1 默认值：`HY_SR_MODE=0`、`HY_SR_DMAX=35 deg`、`HY_SR_DRATE=90 deg/s`、`HY_SR_DREM=0.05 rad`。
- Phase 1 的 7 个 allocator 与 8 个 SafeRoll 测试在修改前重新执行，15/15 通过；Phase 1 H743 clean build 产物及 hash 保留。

## 回归与构建结论

- **源码已验证：mode 0 bit-exact。** 6 组 `phi/p/roll_sp/dt` 序列的两套 CADRC 状态与 legacy 选择器输出继续逐样本 `EXPECT_FLOAT_EQ`。mode 0 直接调用 `observeLegacy()`，不会运行候选框架（`HydroRateControl.cpp:327-335`）。
- **源码已验证：allocator 重构等价。** 原有正常域、正负 roll、pitch/throttle、饱和覆盖回归仍通过；`delta_L/delta_R/T_L/T_R` 容差未变。
- **源码已验证：21/21 单元测试通过。** allocator 9 项，SafeRoll 12 项；新增人工确认门、hardware-ready、诊断组合和 trim 迁移测试。
- **源码已验证：clean `matek_h743_dummy` build 通过。** 独立目录 `build/matek_h743_dummy_phase15/`；只对仓库既有 `platforms/common/spi.cpp:90` 使用构建参数 `-Wno-error=address`，没有修改该源码。固件 PX4 SHA-256 为 `78c7996331312845e992cb8e32a5d265b1552ed2d9bee5536820b6426c8f4d4a`。
- Phase 1.5 ELF：text `1,561,952 B`、data `3,544 B`、bss `41,324 B`；相对 Phase 1 text `+1,344 B`、data/bss `0 B`。FLASH 链接占用 `79.63%`，AXI SRAM `8.53%`。

## 结论分级

- **源码已验证：** 软件门、状态字段、status 输出、测试、clean build、无位置反馈语义均完成。
- **参数推断：** 历史 `CS3/CS4 trim=-0.1` 会成为 Servo4/5 的 generic 常量贡献；按历史 MIN/MAX/REV 推算，hydro=0 时 MAIN11/MAIN12 分别约为 1430/1570 us。该数值是配置数学推算，不是机械零位测量。
- **尚待人工台架确认：** MAIN11/12 的物理左右、正方向、机械中位、回差、死区、速率和安全小范围；是否将 trim 按方案 B 迁移。
- **尚待目标板实测：** H743 cycle/perf、400 Hz dt 分布、工作队列负载、stack high-water、gyro-to-output latency 和 logger 影响。

本阶段没有实现 nominal、WC fixed DE-HOCBF 或 proposed 数学；没有烧写、没有执行 actuator test、没有修改实机参数、没有水池测试。B1、B2、B3、B5 均未宣称关闭。
