# HAAR SafeRoll 现场连续运行模式实现报告

日期：2026-07-24  
基线 HEAD：`5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`

## 结论

已实现 `HY_SR_FIELD`。默认值为 `0`，因此现有严格语义、Mode 0 的 legacy
CADRC 直接路径以及执行器映射均保持不变。`HY_SR_FIELD=1` 只作用于 Modes
1/2/3；Mode 4 保持原严格行为。

现场模式将有限物理不可行分成两类：

1. allocator 或 nonlinear check 给出 finite 且 bounded 的执行器解时，继续输出
   有界 best-feasible 结果，设置 `best_feasible_active`；Modes 2/3 同时设置
   `safety_degraded_active`。
2. 没有 bounded finite 解时，前 20 拍按 `HY_SR_DUMAX` 保持/靠近
   `last_feasible_u`，之后进入平滑 Legacy 临时回退。`requested_code` 不变，
   临时回退期间 `active_code=0`；allocator 连续健康 20 拍后自动重新 blend。
   现场模式不因这种普通有限不可行创建永久 `AllocatorInvalidFault`。

NaN/Inf、时间戳/传感器严重 stale、逻辑映射错误、非法配置、foil gate
关闭、单拍运行时间超过 2500 us、candidate/output 非有限仍进入严重故障路径。

## 关键实现

- 参数及 QGC 元数据：`hydro_rate_control_params.c:161-174`。
- 参数只在 Mode 0 接受：`HydroRateControl.cpp:369-380`；HydroAllocator 的执行
  输出策略采用相同规则：`HydroAllocator.cpp:281-299`。
- `HY_SR_FIELD` 已进入 configuration CRC：
  `SafeRollController.cpp:278-285`。
- finite/bounded 解判定：
  `HydrofoilAllocationModel.cpp` 的 `isFiniteBounded()`。
- best-feasible nonlinear 校核及最终 normalized/角度/速率有界处理：
  `SafeRollController.cpp:795-883`。
- 现场三级状态机与自动恢复：
  `SafeRollController.cpp:1144-1272`。
- 严格与现场 fatal 汇合、严重锁存及累计时间：
  `SafeRollController.cpp:1580-1674`。
- attitude、armed、STAB 和内部状态 reset 不再清除 severe latch；只有
  requested mode 明确变化或 `HY_SR_RST_ID` 变化允许清除：
  `SafeRollController.cpp:1021-1043`。
- `HY_SPEED_SELECT=1` 的 speed² 缓存：
  `SpeedContextCache.hpp:15-69` 和 `HydroAllocator.cpp:260-267`。启动或超过
  500 ms stale 时使用 `HY_ALT_SPEED²`，低频 `debug_vect` 消息之间复用最后有效值。

## 日志

为避免已有大型状态消息超过 PX4 logger 的 ULog format 栈上限，现场累计诊断
放入小型数值 topic `haar_safe_roll_field_status`，由 logger 默认记录：

- `field_mode_active`
- `best_feasible_active`
- `temporary_fallback_active`
- `safety_degraded_active`
- `severe_fault_code`
- 三个 cumulative time 字段
- 同步的 timestamp、timestamp_sample、requested/active code、trial id 和 CRC

`hydro_allocate_message` 记录 `allocator_bounded`、
`speed_context_cached`、`speed_context_stale` 和 `speed_context_age_us`。

## 测试

主机单元测试：**63/63 通过**。

- SafeRollController：52/52；
- HydrofoilAllocationModel/SpeedContextCache：11/11；
- 原 55 项全部保留并通过；
- 新增覆盖 Modes 1/2/3 数秒 finite invalid 连续运行、无 bounded 解的长期
  临时回退和 20 拍恢复、finite saturation、严重 fault、锁存清除边界、
  CRC 变化及 400 Hz speed² 缓存。

Mode 0 bit-exact 回归继续通过；`CADRC.cpp` 和 `CADRC.h` 未修改。

## H743 构建与固件

`matek_h743_dummy` 全量 clean `-Werror` 构建通过。首次并行启动遇到 NuttX
`.config` 生成次序竞争，继续同一 clean build 后完整 1043 个目标编译并链接成功；
SafeRoll、HydroAllocator、消息、logger 和参数元数据均由 ARM 工具链重新编译。

- FLASH：1,589,920 / 1,966,080 bytes（80.87%）
- AXI SRAM：44,716 / 524,288 bytes（8.53%）
- 固件：`build/matek_h743_dummy/matek_h743_dummy.px4`
- SHA-256：`cc78ab51a7fd506465948c053b1424bd43111c734869d04c3925cc2cc635d909`

构建配置阶段仍打印工程既有 SymForce circular-import traceback 和 Python
`SyntaxWarning`，两者不影响配置、`-Werror` 编译、链接或固件生成。

未执行 flash、actuator test 或任何实机动作。
