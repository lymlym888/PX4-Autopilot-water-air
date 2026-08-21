# HAAR safe-roll Phase 1 修改文件

所有修改基于 HEAD `5ae2fd95ae`。旧 `src/modules/hydro_rate_control/CADRC.cpp` 和 `CADRC.h` 未修改；旧 CADRC 参数默认值和 `HY_WING_MAX_A` 默认值未修改；原始日志和历史 `.params` 未修改。

## 新增

- `src/modules/hydro_rate_control/SafeRollController.hpp/.cpp`：输入/输出、mode 状态机、freshness/finite 检查、reset/fallback、bumpless 工具、perf counter和四个未实现占位接口。
- `src/modules/hydro_rate_control/SafeRollControllerTest.cpp`：mode 0 状态/输出逐样本等价、时间戳、freshness、reset、gate、allocator、fallback、bumpless 与主机性能测试。
- `src/lib/hydrofoil_allocation/HydrofoilAllocationModel.hpp/.cpp`：无状态共享分配模型，含 `solve/forward/linearize/checkCandidate/commandedAngleInterval`。
- `src/lib/hydrofoil_allocation/HydrofoilAllocationModelTest.cpp`：旧实现回归、forward/solve、灵敏度、异常求解、区间与性能测试。
- `src/lib/hydrofoil_allocation/CMakeLists.txt`：共享库和测试目标。
- `msg/HaarSafeRollStatus.msg`：SafeRoll、commanded/model-based allocator、Servo4/5 叠加、gate、逻辑映射合规性和物理映射未验证诊断。
- `RSMC_codex/SAFE_ROLL_PHASE1_SUMMARY.md`、`SAFE_ROLL_PHASE1_CHANGED_FILES.md`、`SAFE_ROLL_PHASE1_TEST_RESULTS.md`、`SAFE_ROLL_PHASE1_BLOCKERS.md`。

## 修改

- `src/modules/hydro_rate_control/HydroRateControl.cpp/.hpp`：只在 CADRC-enabled STAB 路径接入选择器；mode 0 direct legacy；订阅 allocator/status，修复 CADRC 消息时间戳和字段。
- `src/modules/hydro_rate_control/hydro_rate_control_params.c`：新增 `HY_SR_MODE/DMAX/DRATE/DREM`；mode 默认 0。
- `src/modules/hydro_rate_control/CMakeLists.txt`：构建 SafeRoll library；生产模块链接该库。
- `src/modules/hydro_allocator/HydroAllocator.cpp/.hpp`：删除生产中的重复方程/Jacobian/Newton 实现，改用共享模型；topic 和输出顺序保留；增加求解、gate、commanded/model-based 与映射诊断。
- `src/modules/hydro_allocator/CMakeLists.txt`：链接共享模型。
- `src/modules/control_allocator/ControlAllocator.cpp/.hpp`：只读采集 Servo4/5 generic/hydro/final 贡献，汇总并发布 `haar_safe_roll_status`；控制加法逻辑保持原样。
- `msg/HydroAllocateMessage.msg`：增加 timestamp_sample、求解状态、模型重构、灵敏度、gate 和逻辑映射字段。
- `msg/HydroCadrcControlMessage.msg`：增加 timestamp_sample、SafeRoll 状态和已赋值诊断字段。
- `msg/CMakeLists.txt`：生成 `HaarSafeRollStatus`。
- `src/modules/logger/logged_topics.cpp`：默认记录 `haar_safe_roll_status`。
- `src/lib/CMakeLists.txt`：注册 `hydrofoil_allocation` 子目录。

## 报告归档

原三份审计报告已放到用户指定的 `RSMC_codex/`：`SAFE_ROLL_PX4_AUDIT.md`、`SAFE_ROLL_DATAFLOW.md`、`SAFE_ROLL_IMPLEMENTATION_PLAN.md`。未改变其审计结论。
