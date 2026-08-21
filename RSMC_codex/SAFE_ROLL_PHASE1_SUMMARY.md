# HAAR safe-roll Phase 1 总结

日期：2026-07-22。基线：分支 `RSMC`，Git HEAD `5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`，PX4 `v1.15.0-beta1-346-g5ae2fd95ae`。开始实施前工作区只有三份未跟踪审计报告；本阶段未改写旧 `CADRC.cpp/.h`，未改旧参数默认值，未烧写飞控，未执行 actuator test 或水池测试。

## 结论

- **VERIFIED — mode 0 逐样本数值等价。** `HY_SR_MODE` 默认 0（`hydro_rate_control_params.c:135-147`）。旧 CADRC 仍先按原顺序执行、缩放、限幅和 trim（`HydroRateControl.cpp:264-302`）；mode 0 随后只调用 `observeLegacy()`，不会调用任何候选控制器（`:324-332`）。两套相同 CADRC 状态在 6 组 `phi/p/roll_sp/dt` 序列中，CADRC 状态、legacy 输出和选择器输出均通过 `EXPECT_FLOAT_EQ`。
- **VERIFIED — modes 1–4 没有控制数学。** 四个占位接口只返回 `ModeNotImplemented`（`SafeRollController.cpp:55-73`）；任何非零请求都保持 `active_mode=0`、`u_selected=u_legacy` 和 `fallback_active=true`（`:154-203`）。没有自动参考发生器。
- **VERIFIED — 操作者映射未变。** STAB 的 `roll_sp=manual_control_setpoint.roll*radians(HY_ROLL_MAX)`、pitch/yaw/throttle 仍沿用原路径（`HydroRateControl.cpp:252-258,295-301,373-378`）；MANUAL/ACRO 分支未接入 SafeRoll（`:225-237,264-265`）。
- **VERIFIED — 分配方程只有一份生产实现。** `HydrofoilAllocationModel::solve/forward/linearize/checkCandidate` 位于 `src/lib/hydrofoil_allocation`（`HydrofoilAllocationModel.cpp:61-335`），`HydroAllocator` 调用该库（`HydroAllocator.cpp:251-323`）。查询接口无 uORB、无静态或 warm-start 状态。
- **VERIFIED — 分配器正常域和覆盖样本数值等价。** 5 个正常样本及 45 个 pitch/throttle/正负滚转/饱和组合中，`delta_L/delta_R/T_L/T_R` 与测试内冻结的旧方程参考实现均在 `1e-6..1e-4` 容差内。奇异 Jacobian、NaN 和不收敛明确返回 invalid（`HydrofoilAllocationModel.cpp:101-145`）。
- **VERIFIED — 有效输入是模型重构，不是测量。** commanded `delta/T` 经 forward 方程重构并对滚转列做最小二乘投影得到 `u_phi_effective_model`（`HydrofoilAllocationModel.cpp:214-254`）；状态固定记录 `foil_position_feedback_available=false`、`u_effective_is_model_based=true`（`HydroAllocator.cpp:317-319`；`ControlAllocator.cpp:909-915`）。未把 PWM 或 `actuator_outputs` 命名为实际翼角。
- **VERIFIED — 逻辑映射被显式检查和记录但物理线束仍未知。** 历史参数 `HY_SV0_IDX=5`、`HY_SV1_IDX=4`，所以当前约定是 `control[4]=逻辑左`、`control[3]=逻辑右`；运行时检查必须为 index 4/3（`HydroAllocator.cpp:322-326`），不匹配时拒绝 modes 1–4（`SafeRollController.cpp:147-166`）。状态永久标记 `physical_left_right_mapping_verified=false`。默认参数仍为 1，未被自动修改（`hydro_allocator/module.yaml:16-23`）。
- **VERIFIED — clean `matek_h743_dummy` 构建通过。** 输出位于 `build/matek_h743_dummy_phase1/matek_h743_dummy.px4`。GCC 13 对仓库既有 `platforms/common/spi.cpp:90` 报 `-Waddress`，独立构建目录仅用 `-Wno-error=address` 将这一条既有诊断降为 warning；未修改源码或板配置。本阶段新增对象在 ARM 编译中无警告。

## 安全外围

原始 dt 由相邻 `vehicle_angular_velocity.timestamp_sample` 计算，先检查非单调和 `[1,10] ms` 范围，再生成 `dt_used`（`SafeRollController.cpp:105-119,200`）。gyro/attitude stale 阈值分别为 20/50 ms，四元数和角速度检查 finite（`:121-137`）。arm/disarm、STAB entry/exit、mode change、attitude reset counter、时间跳变、stale 和 allocator invalid 都只 reset 新控制器（`:75-90,198-205`）；旧 CADRC 的 reset 语义没有混入本补丁。

`HY_SR_DMAX=35 deg`、`HY_SR_DRATE=90 deg/s`、`HY_SR_DREM=0.05 rad` 已建立，明确只约束未来 modes 1–4 的 commanded/predicted 状态（`hydro_rate_control_params.c:149-183`）。commanded angle/rate interval 工具在 `HydrofoilAllocationModel.cpp:314-334`，本阶段 mode 0 不使用这些新限制。

## Servo4/5 双重贡献

**VERIFIED — 软件始终执行 `final=constrain(generic+hydro)`。** 加法在 `ControlAllocator.cpp:841-870`；新增诊断分别记录 generic、hydro 和 final，并在 generic 非零时置 conflict fault（`:891-946`）。本阶段没有清零 generic，也没有把最终饱和当作解决方案。

**LIKELY/高风险 — 历史实机参数会产生非零 generic。** 历史配置为 Custom airframe、3 motors、5 control surfaces；CS3 和 CS4 的 torque 均为 0，但 trim 均为 `-0.1`（`release/1.0/hydro3-250110.params:95,218,249-262,284`）。Custom effectiveness 把 control surfaces 加入矩阵（`ActuatorEffectivenessCustom.cpp:51-75`），其 trim 写入 allocator（`ActuatorEffectivenessControlSurfaces.cpp:154-165`）。因此 Servo4/5 在该配置下很可能各带 `-0.1` generic 偏置。必须用新增状态消息在实机参数集上确认；在确认和修复前 modes 1–4 保持禁用。

最小后续修复方案：经人工确认 Servo4/5 只归 hydro 所有后，在对应实机参数中将 CS3/CS4 从通用效果矩阵移除或把其 generic trim/效应显式置零；不得在 `publish_actuator_controls()` 中暗中覆盖。该方案本阶段未实施。

## 资源与实时性

- 主机参考（非 H743 WCET，三次/两次运行汇总）：mode 0 observer 平均 `40–45 ns`、观测最大 `15.592 us`；共享模型 `solve+forward+linearize` 平均 `302–328 ns`、观测最大 `79.057 us`。400 Hz 周期预算为 `2500 us`。最大值含非实时主机调度抖动。目标上 `safe_roll: cycle` perf counter 和 `cycle_time_us` 日志已就绪（`SafeRollController.cpp:21-35,93-97,216-218`；`HydroRateControl.cpp:322-376`），但没有硬件运行数据，因此目标平均/最大值为 **UNKNOWN**。
- ARM `-fstack-usage`：`SafeRollController::evaluate` 静态 frame 48 B；共享模型 `solve` 152 B、`solveOneSide` 136 B、`forward` 152 B、`linearize` 112 B。最深新模型路径静态 frame 合计约 288 B（不含库内部调用）。旧匹配基线没有 `.su`，所以精确 stack delta 为 **UNKNOWN**。
- 新固件 `text=1,560,608 B, data=3,544 B, bss=41,324 B`。旧现存 ELF 为另一提交 `914ec24d...`，表面差值 `text -10,408 B, data -4 B, bss 0 B` 不可归因于 Phase 1；精确同提交尺寸 delta 为 **UNKNOWN**。FLASH 链接占用 `1,564,152/1,966,080 B (79.56%)`，AXI SRAM `44,716/524,288 B (8.53%)`。

## 当前运行约束

`HY_FOIL_AUX` 默认 0、`HY_FOIL_THR` 默认 0，gate 使用严格 `>`，所以纯默认配置 gate 关闭（`hydro_allocator/module.yaml:201-245`；`HydroAllocator.cpp:515-525`）。非零 SafeRoll mode 在 gate 关闭、输入无效、allocator invalid 或逻辑映射不符时拒绝并回退 legacy（`SafeRollController.cpp:151-203`），控制台只在模式请求边沿警告（`HydroRateControl.cpp:336-353`）。本阶段 mode 1–4 无论 armed/disarmed 都不输出新控制。
