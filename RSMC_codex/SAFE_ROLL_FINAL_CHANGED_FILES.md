# HAAR SafeRoll 最终修改文件

基线：`5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`。旧 `src/modules/hydro_rate_control/CADRC.cpp`、`CADRC.h` 未修改。

## 新增

- `src/modules/hydro_rate_control/SafeRollController.hpp`：固定内存配置、输入、输出、状态与 fault 定义。
- `src/modules/hydro_rate_control/SafeRollController.cpp`：完整 code 0–4、估计器、证书、HOCBF、allocation interval、blend/fallback。
- `src/modules/hydro_rate_control/SafeRollControllerTest.cpp`：25 个 SafeRoll 测试。
- `src/lib/hydrofoil_allocation/HydrofoilAllocationModel.hpp`
- `src/lib/hydrofoil_allocation/HydrofoilAllocationModel.cpp`
- `src/lib/hydrofoil_allocation/HydrofoilAllocationModelTest.cpp`
- `src/lib/hydrofoil_allocation/CMakeLists.txt`
- `msg/HaarSafeRollStatus.msg`
- `msg/HaarSafeRollEstimatorStatus.msg`

## 修改

- `src/modules/hydro_rate_control/HydroRateControl.cpp/.hpp`：在线参数、输入 freshness/context、selector、两条状态发布；人工映射未变。
- `src/modules/hydro_rate_control/hydro_rate_control_params.c`：41 个 QGC 参数，统一 `HAAR Safe Roll` 分组。
- `src/modules/hydro_rate_control/CMakeLists.txt`：SafeRoll/shared model 链接。
- `src/modules/hydro_allocator/HydroAllocator.cpp/.hpp`：调用共享模型并发布 context、command、gate、solver 状态；输出顺序不变。
- `src/modules/hydro_allocator/CMakeLists.txt`：链接共享模型。
- `src/modules/control_allocator/ControlAllocator.cpp/.hpp`：记录 generic/hydro/final；按 final pre-PWM command 重构 model-based effective input。
- `src/modules/control_allocator/CMakeLists.txt`：链接共享模型。
- `msg/HydroAllocateMessage.msg`：增加共享 context/command、gate、solver 诊断。
- `msg/HydroCadrcControlMessage.msg`：增加 SafeRoll 内部传输字段和有效 timestamp。
- `msg/CMakeLists.txt`：注册两条 SafeRoll uORB。
- `src/modules/logger/logged_topics.cpp:64-65`：默认记录两条 SafeRoll topic。
- `src/lib/CMakeLists.txt`：加入共享模型目录。
- `platforms/common/spi.cpp:88-96`：GCC 13 `-Waddress` clean-build 兼容修复；多硬件版本检查指针，单版本数组直接遍历。

## 报告

本阶段新增九份 `RSMC_codex/SAFE_ROLL_FINAL_*.md`。Phase 1/1.5 报告和 checkpoint 未删除。

## 明确未改

- legacy CADRC 数学及状态；
- manual_control_setpoint roll/pitch/yaw/throttle 映射；
- generic trim、REV、PWM 和下游 `generic+hydro` 叠加；
- 旧 HydroAllocator topic 的执行器顺序；
- 实机参数值。

没有提交到远端，没有 flash，没有 actuator test，没有水池测试。

