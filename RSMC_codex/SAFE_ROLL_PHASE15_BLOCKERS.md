# HAAR safe-roll Phase 1.5 阻断项

Phase 1.5 已完成软件准备，但没有人工台架结果或目标板实测。B1、B2、B3、B5 均保持开放；modes 1–4 仍只回退 legacy。

## B1 — Servo4/5 ownership 与 trim 未决

状态：**源码已验证双重贡献；参数用途推断；尚待人工台架确认。**

历史 CS3/CS4 torque/flap/spoiler 均为零但 trim 均为 `-0.1`，所以无姿态请求时 generic Servo4/5 仍为 `-0.1`；随后再加 hydro（`ControlAllocator.cpp:843-872`）。本 PX4 版本没有独立 `PWM_MAIN_TRIM`，历史 MIN/MAX/REV 推算 hydro=0 时 MAIN11/12 约为 1430/1570 us，但机械零位未知。

关闭条件：完成台架中位/方向测量，选择并实现受审的 A/B/C 方案，证明 mode 0 物理中位和逐点 PWM 行为符合预期，最后才可人工设置 `HY_SR_OWN_OK=1`。当前默认 0。

## B2 — 物理左右与符号未确认

状态：**源码逻辑已验证；物理映射 UNKNOWN。**

软件保持 `control[4]=逻辑左`、`control[3]=逻辑右`，历史 functions 推出 MAIN12=逻辑左、MAIN11=逻辑右；不能证明线束物理左右或正命令方向。

关闭条件：按断桨清单分别完成 MAIN11/12 五点小幅测试，结果模板由操作/观察/审核三方签字；随后才可人工设置 `HY_SR_MAP_OK=1`。当前默认 0。

## B3 — foil gate 正式参数未批准

状态：**源码与默认行为已验证；目标参数尚待人工只读采集和审核。**

gate 逻辑为：

```text
selected_aux_value = source(HY_FOIL_AUX) * HY_FOIL_AUXGAIN
gate = selected_aux_value > HY_FOIL_THR
       AND nav_state in {STAB, ACRO, MANUAL}
```

源码位于 `HydroAllocator.cpp:461-528`。参数影响：

- `HY_FOIL_AUX` 选择 AUX1..6、常量 -1/1 或默认常量 0；
- `HY_FOIL_THR` 是严格 `>` 阈值；
- `HY_ALLOCATE_MODE` 不决定 gate 开关，只决定 gate 开启后的 Normal/Fixed/Manual 输出分支（`:530-556`）；
- `HY_WING_MAX_A` 不决定 gate 开关，只限制共享求解模型的 commanded 水翼角。

默认 `HY_FOIL_AUX=0`、gain=1、threshold=0，故 `0>0` 为 false。推荐正式方案是选择一个有明确物理标签、失联后落到 disabled 侧的 AUX，阈值与 gain 留有抖动裕量，`HY_ALLOCATE_MODE=0` 用于正常模型分配，`HY_WING_MAX_A` 保持经机械验证的 legacy 值；这些均是 **人工审核项**，本阶段不自动修改。

关闭条件：保存目标只读参数和 `hydro_allocator status`，审批 AUX fail-safe/阈值/allocation mode/角度限制并做受控验证。gate false 时 modes 1–4 继续拒绝。

## B4 — 无水翼位置反馈

状态：**源码已验证，结构性限制。**

仍无位置传感器。`delta_l/r_commanded`、final normalized 和 PWM 都是 commanded；`u_phi_effective_model` 是 forward-model reconstructed，绝非 measured。该项不是靠 MAP_OK/OWN_OK 可消除的。

## B5 — H743 实时性未实测

状态：**尚待目标板实测。**

clean build 和主机 benchmark 通过，但不能证明 H743 WCET、stack、2.5 ms 裕量或 logger 影响。关闭条件见 `SAFE_ROLL_PHASE15_PERF_PLAN.md` 和结果模板。

## 工具链环境项

- GCC 13 的既有 `platforms/common/spi.cpp:90` `-Waddress` 在默认 `-Werror` 下阻断，需要发布构建环境修复或明确批准单项降级。
- 本机 SymForce 安装存在 circular import traceback，虽然当前配置/构建完成，仍不满足完全干净的可复现环境要求。

## 放行状态

`HY_SR_MAP_OK=0`、`HY_SR_OWN_OK=0`、默认 foil gate=false，且候选接口仍返回 `ModeNotImplemented`。即使人工错误请求 mode 1–4，控制器也返回 legacy `u_selected`。在 B1/B2/B3/B5 有经签字的证据之前，不得宣称任何一项关闭，不得启用新控制输出。
