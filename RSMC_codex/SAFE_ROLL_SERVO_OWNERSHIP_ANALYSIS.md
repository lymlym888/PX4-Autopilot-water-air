# Servo4/5 所有权与 trim 语义分析

## 结论

**源码已验证：** 历史 `CA_SV_CS3_TRIM=-0.1`、`CA_SV_CS4_TRIM=-0.1` 属于 generic ControlAllocator 的 actuator neutral trim，不是 hydro allocator 输出，也不是 PWM 层独立 trim。在对应 torque/flap/spoiler 均为零时，它仍使 generic Servo4/5 为 `-0.1`。运行发布路径随后再加 hydro contribution，最后限幅。因此历史配置是 `final_servo4/5=constrain(-0.1+hydro_servo4/5)`。

**参数推断：** 这两个相同偏置很可能承担过舵机/连杆中位用途，但提交没有解释。不能仅凭配置证明其物理用途，也不能直接清零。

## A/B/C/D 四层路径

### A. Generic control effectiveness contribution

1. `ActuatorEffectivenessControlSurfaces` 查找并读取 `CA_SV_CS{i}_TRIM`（`src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessControlSurfaces.cpp:45-64,68-92`）。
2. `addActuators()` 将 trim 写入 `configuration.trim`（`:154-162`）。
3. `ControlAllocator` 将 configuration trim 传给各 allocation 实例（`src/modules/control_allocator/ControlAllocator.cpp:577-605`）。
4. `ControlAllocation::setEffectivenessMatrix()` 保存 `_actuator_trim`（`ControlAllocation/ControlAllocation.cpp:52-63`）。
5. pseudo-inverse 分配为 `_actuator_sp=_actuator_trim+mix*(control_sp-control_trim)`（`ControlAllocation/ControlAllocationPseudoInverse.cpp:182-185`）。因此控制请求为零且 CS3/CS4 effectiveness 为零时，输出仍是 `-0.1`。

历史参数证据：`CA_SV_CS_COUNT=5`，CS3/CS4 torque、flap、spoiler 均为 0，trim 均为 `-0.10000000149`（`release/1.0/hydro3-250110.params:249-262,284`）。CS 索引从 0 开始，CS3/CS4 对应 Servo4/Servo5。

### B. Actuator neutral / linkage calibration

当前代码没有专用 `HY_FOIL_L_TRIM/HY_FOIL_R_TRIM`。历史配置唯一显式的这两个归一化偏置是 CS3/CS4 generic trim。是否用于机械中位属于 **尚待人工台架确认**。

### C. Hydro allocator requested contribution

历史 `HY_SV0_IDX=5`、`HY_SV1_IDX=4`（`release/1.0/hydro3-250110.params:727-728`）。生产代码使用 `index-1` 写 `hydro_servos`（`src/modules/hydro_allocator/HydroAllocator.cpp:528-556`），所以：

- 逻辑左：`hydro_servos.control[4]` = Servo5；
- 逻辑右：`hydro_servos.control[3]` = Servo4。

ControlAllocator 在发布前执行 `actuator_sp += hydro_servos.control[servos_idx]`（`ControlAllocator.cpp:843-872`）。历史 PWM functions 为 MAIN11=204/Servo4、MAIN12=205/Servo5（参数文件 `:883-884`），故软件逻辑为 MAIN11=逻辑右、MAIN12=逻辑左；物理线束仍 **UNKNOWN**。

### D. Final PWM mapping and REV

本 PX4 版本没有 `PWM_MAIN_TRIM` 参数。`MixingOutput` 只读取每通道 FUNC/DIS/MIN/MAX 和统一 REV mask（`src/lib/mixer_module/mixer_module.cpp:111-130,150-179`）。归一化值先按 REV 取反，再线性映射 `[-1,1]→[MIN,MAX]`（`:528-542`）。

历史 MAIN11/12 均为 MIN=800、MAX=2200 us；`PWM_MAIN_REV=6273`。位掩码显示 MAIN11 不反向、MAIN12 反向。因此在 hydro=0、generic=-0.1 时：

- MAIN11：`800 + 0.45*(2200-800) = 1430 us`；
- MAIN12：REV 后为 `+0.1`，`800 + 0.55*(2200-800) = 1570 us`。

若直接清除 CS3/CS4 trim，两通道命令均变为 0，PWM 都变为约 1500 us，即分别变化 `+70 us/-70 us`。这可能改变物理水翼零位，故禁止无台架证据清零。

## 历史来源

`git blame` 将上述参数定位到提交 `86b521fd2b5`（2025-01-10，作者 Cao Yue，提交标题 `1.0.0`）。该提交一次性加入完整参数文件、readme 和照片，没有对 `-0.1` 的单独注释；`git log -S` 未找到后续调整。结论是 **提交来源已验证，设计意图 UNKNOWN**。

## 三个候选方案

### 方案 A：确认无物理用途后清除 generic trim/effect

优点：Servo4/5 由 hydro 唯一拥有，模型和最终命令简单一致。缺点：从历史 `-0.1` 变为 0 会改变 MAIN11/12 PWM 中位；只有台架证明 1430/1570 并非所需物理零位后才可采用。

### 方案 B：迁移为明确的左右 hydrofoil neutral calibration

建议未来增加 `HY_FOIL_L_TRIM/HY_FOIL_R_TRIM`，从 CS3/CS4 移除 trim/effect，并在统一 hydro 输出映射处执行 `final=constrain(neutral+hydro_commanded)`。迁移初值应按逻辑左右和 REV 之前的归一化语义确定，不能按物理通道名字猜测。

优点：控制所有权唯一；保留物理中位；safe-roll forward model可使用与最终命令一致的 commanded delta；neutral 语义清晰。缺点：需要台架测得左右/方向/中位，并需要单独受审补丁验证 mode 0 物理输出逐点不变。

**推荐：方案 B，但仅在台架确认 `-0.1` 确实承担中位后执行。** 本阶段只提供 `compareTrimMigration()` 纯数学工具和回归测试（`src/lib/hydrofoil_allocation/HydrofoilAllocationModel.cpp:337-374`），没有增加/启用上述未来参数，也没有改变运行输出。

### 方案 C：保留 generic trim并纳入 forward model

优点：无需参数迁移，历史 PWM 中位自然不变。缺点：Servo4/5 所有权仍分裂，模型必须精确包含 generic、hydro、REV 前饱和及可能的 aileron override，容易发生时序/配置不一致。仅当 A/B 均不可行时考虑。

## 验收条件

只有以下全部完成后才允许人工将 `HY_SR_OWN_OK=1`：

1. 目标板记录 generic/hydro/final，验证 `generic_hydro_final_consistent=true`；
2. 台架测得 MAIN11/12 的物理左右、正方向和机械中位；
3. 选择 A/B/C 并由独立补丁证明 mode 0 的物理中位和逐点 PWM 行为保持预期；
4. safe-roll 使用的 commanded delta 与最终 REV 前执行命令语义一致；
5. 不依赖最终 `[-1,1]` saturation 掩盖叠加。

B1 当前仍为 **尚待人工台架确认**，未关闭。
