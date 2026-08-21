# HAAR safe-roll Phase 1 阻断项

Phase 1 基础设施已完成，但下列项目阻止 modes 1–4 进入可输出状态。代码本身也强制这些模式回退到 mode 0。

## B1 — Servo4/5 generic + hydro 双重贡献

状态：**VERIFIED 软件叠加；LIKELY 历史配置非零；实机值 UNKNOWN**。

`ControlAllocator` 先取 generic，再加 hydro，最后才限幅（`ControlAllocator.cpp:841-870`）。历史参数中 Servo4/5 对应 CS3/CS4，二者 trim 均为 `-0.1`（`release/1.0/hydro3-250110.params:249-262`），而 control-surface trim 会写入分配器（`ActuatorEffectivenessControlSurfaces.cpp:154-165`）。这意味着历史参数下即使力矩效应为零，也很可能存在 `-0.1 + hydro`。

处置：用 `haar_safe_roll_status.generic_servo4/5` 在目标参数集和 STAB 水面模式确认。人工确认专属 ownership 后，从通用效果矩阵移除这两个 surface 或清除其 generic trim/effect；不得依赖最终 saturation，不得在发布处偷偷清零。

## B2 — 物理左右线束未验证

状态：**UNKNOWN**。

历史软件参数证明 `HY_SV0_IDX=5`（逻辑左→`control[4]`）、`HY_SV1_IDX=4`（逻辑右→`control[3]`），但没有硬件证据。运行时若参数不满足这一逻辑映射，modes 1–4 会拒绝并回退；日志只记录逻辑 commanded 状态，`physical_left_right_mapping_verified` 固定 false。

处置：后续经批准的断桨/安全台架，用低幅单通道动作和人工目视确认；本阶段禁止 actuator test，未执行。

## B3 — 默认 foil gate 关闭

状态：**VERIFIED 默认关闭；当前实机参数 UNKNOWN**。

`HY_FOIL_AUX=0` 产生 0，`HY_FOIL_THR=0`，gate 为严格 `0>0`，因此默认 false（`hydro_allocator/module.yaml:201-245`；`HydroAllocator.cpp:512-522`）。历史参数文件没有后加入的 gate 字段。

处置：上电前只读核对 `HY_FOIL_AUX/HY_FOIL_THR/HY_ALLOCATE_MODE/HY_WING_MAX_A`。本阶段不自动修改。非零 SafeRoll 请求在 gate false 时已有 fault、控制台警告和 legacy fallback。

## B4 — 无水翼位置反馈

状态：**VERIFIED**。

没有位置传感器 topic 或驱动。`delta_l/r_commanded` 是 allocator 请求；`u_phi_effective_model` 是同一模型从 commanded delta/T 和 context 重构，绝非 measured。PWM/`actuator_outputs` 不能用于证明实际翼角。

处置：安全证明必须保留 command/model uncertainty；如未来增加传感器，另行设计校准、freshness、故障检测和 measured 字段，不能复用当前 commanded 名义替代。

## B5 — H743 实机 WCET/stack 未测

状态：**UNKNOWN**。

主机 benchmark 和 ARM 静态 frame 均满足继续开发的数量级判断，但不是目标板 WCET 或 stack high-water。400 Hz 周期为 2500 us；运行 perf/log 接口已经存在。

处置：在任何新数学 armed 启用前，先做不驱动执行器的目标板监测，记录平均/最大 cycle、调度抖动、stack high-water 和 logger 负载。需要留出显著于观测最大值的裕量。

## B6 — 严格同提交尺寸基线缺失

状态：**UNKNOWN**。

工作区已有旧 ELF 内嵌 commit `914ec24d...`，不是 `5ae2fd95ae`；因此不能准确拆分 Phase 1 flash/RAM/stack 增量。当前 Phase 1 clean build通过且容量有余，但 delta 不可认证。

处置：若发布流程要求严格 delta，在独立、未修改的 `5ae2fd95ae` worktree 用同一 GCC/CMake 重新构建，再与 Phase 1 相同工具链比较。

## 非阻断但需修复的环境项

- GCC 13 将仓库既有 `platforms/common/spi.cpp:90` 的恒真地址判断报为 `-Waddress`；默认 `-Werror` 阻断。Phase 1 clean build只在构建参数中降级该单一类别，没有改源码。
- 本机 SymForce Python 安装存在 circular import traceback；当前生成步骤继续完成，但应在可复现构建容器中修复依赖。

## 放行条件

modes 1–4 继续保持禁用，直到至少 B1、B2、B3、B5 完成人工确认/实测；随后仍需独立评审 nominal、WC fixed DE-HOCBF、proposed 控制数学。本阶段没有实现这些数学，也没有改变默认 mode 0。
