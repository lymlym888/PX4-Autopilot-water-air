# HAAR SafeRoll 深度控制保留审计与修复报告

日期：2026-07-24  
源码基线：`5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`  
目标板：`matek_h743_dummy`

## 1. 结论

### 1.1 已由源码验证

SafeRoll 没有独立的深度控制器，也没有在 Modes 1/2/3 中接管完整
hydro control vector。当前生产链中：

1. 原深度控制器在 `HydroRateControl::Run()` 内运行，并且位于 SafeRoll
   滚转选择之后、STAB/ACRO/MANUAL 共用路径中
   （`src/modules/hydro_rate_control/HydroRateControl.cpp:844-972`）。
2. SafeRoll 只选择 `_hydro_torque_setpoint.xyz[0]` 的 `u_roll`
   （`:448-460`, `:751`）；同一拍的 `u_pitch` 和 `u_yaw` 分别原样写入
   `xyz[1]`、`xyz[2]`（`:752-753`）。
3. HydroAllocator 每拍同时接收 `force_x`、`force_z`、`u_phi`、
   `u_theta`、`u_psi`
   （`src/modules/hydro_allocator/HydroAllocator.cpp:283-290`）。
4. 共享模型按

   ```text
   [FxL, FzL, FxR, FzR]^T
     = A [u_phi, u_theta, u_psi]^T
       + [force_x/2, force_z/2, force_x/2, force_z/2]^T
   ```

   合成完整请求
   （`src/lib/hydrofoil_allocation/HydrofoilAllocationModel.cpp:149-163`）。
5. 最终执行器仍为既有
   `final = constrain(generic + hydro, -1, 1)`
   （`src/modules/control_allocator/ControlAllocator.cpp:844-891`）。
   本轮没有修改 trim、REV、通道、所有权或此合成规则。

因此，源码不支持“SafeRoll 用差动指令直接覆盖完整左右翼命令”这一假设。
用户提供的某拍
`delta_left=+0.03084, delta_right=-0.03084` 确实说明该拍共同分量为零，
但消失点位于 allocator 输入的深度/垂向力形成处或更上游，而不是最终
servo 合成处。

### 1.2 最强根因证据与尚未验证项

原深度控制的选择参数 `HY_DC_MODE` 当前源码默认值为 **0 Disabled**
（`src/modules/hydro_rate_control/hydro_rate_control_params.c:1006-1016`）。
当它为 0 时，代码明确执行：

```text
hydro_vertical_thrust_setpoint = 0
```

（`HydroRateControl.cpp:934-946`）。在对称配置下，这会准确产生“只有
SafeRoll 差动分量、没有深度共同分量”的现象。

作为对照，历史已成功水池 ULog
`已有ulg/室内动态实验/TII已用(CADRC)/log_370_2026-1-28-11-37-14.ulg`
保存的是：

```text
HY_DC_MODE = 1
HY_DC_SP = 0.30000001 m
HY_DC_FF = -20 N
HY_DC_MAX_THR = 0.25
```

该 ULog 中 `hydro_thrust_setpoint.xyz[2]` 有 3669 个样本，范围
`[-35.3468, -17.8132] N`，均值 `-26.9986 N`，证明历史滑行深度链依靠
非零 `force_z` 工作。

但是，当前实机的 `HY_DC_MODE`、`hydro_depth_control_message` 和
`command_force_z` 快照没有随本次问题提供，工作区也没有该次水上 ULog。
因此：

- **VERIFIED：** 深度分量在 allocator 前已经为零时，Modes 1/2/3
  只能产生所观察到的纯差动输出。
- **LIKELY：** 当前目标参数为默认 `HY_DC_MODE=0`，或没有恢复历史深度
  参数集。这是与现象完全一致、且证据最强的触发原因。
- **UNKNOWN：** 当前实机是否确实为 0；在没有目标参数或本次 ULog 前，
  不能把它冒充为已实机确认的事实。

本轮没有自动修改 `HY_DC_MODE` 或任何深度参数。代码库的 ROMFS、board
startup 和工具脚本中也没有找到设置 `HY_DC_MODE` 的启动项，故深度模式
是否启用完全依赖目标参数存储/人工参数集。

## 2. 原深度/俯仰控制完整链路

| 级别 | 生产信号与源码 |
|---|---|
| 深度测量 | uORB `depth_estimated.depth_estimated`；订阅在 `HydroRateControl.hpp:130`，读取在 `HydroRateControl.cpp:862,869`；向下为正 |
| 深度参考 | 参数 `HY_DC_SP`，读取在 `HydroRateControl.cpp:873`；定义在 `hydro_rate_control_params.c:1029-1036` |
| 深度控制器 | `HydroRateControl::Run()` 内联实现；NSF `:889-910`，HSMC `:912-932`，由 `HY_DC_MODE` 在 `:934-946` 选择 |
| 原控制输出 | `hydro_vertical_thrust_setpoint`，单位 N；mode 1 为 `nsf_u_limited + HY_DC_FF`，mode 2 为 `_hsmc_x3 + HY_DC_FF` |
| 坐标转换 | 与水平推力按实测 body pitch 旋转为 `hydro_thrust_setpoint.xyz[0/2]`，`HydroRateControl.cpp:960-968` |
| 俯仰姿态控制 | 旧 pitch CADRC 产生 `u_pitch`，`HydroRateControl.cpp:277-315`；SafeRoll 不改它 |
| allocator 输入 | `force_x=hydro_thrust.xyz[0]`、`force_z=xyz[2]`、`u_phi=torque[0]`、`u_theta=torque[1]`、`u_psi=torque[2]`，`HydroAllocator.cpp:283-288` |
| nonlinear allocation | 完整向量进入 `HydrofoilAllocationModel::solve()`，`HydroAllocator.cpp:289-296` |
| hydro 输出 | 左/右翼及左右推进器写入 `hydro_servos`/`hydro_motors`，`HydroAllocator.cpp:636-664` |
| 最终输出 | generic 与 hydro 相加并限幅，`ControlAllocator.cpp:844-891` |

历史水池参数中四个 pitch allocator 系数
`HY_VXRT0/1_PIT_R`、`HY_VZRT0/1_PIT_R` 均为 0。因此该历史配置里的
水面深度控制主要通过 `command_force_z` 产生左右翼共同分量，而不是通过
`u_theta`。当前生产代码没有一个可由源码可靠识别的独立 tail/elevator
输出通道；本轮没有猜测或虚构一个执行器索引。

## 3. Mode 0 与 Modes 1/2/3 逐级比较

| 信号/级别 | Mode 0 | Mode 1 | Mode 2 | Mode 3 | 审计结果 |
|---|---|---|---|---|---|
| depth measurement/reference | 原 `depth_estimated` / `HY_DC_SP` | 相同 | 相同 | 相同 | 未清零、未跳过 |
| depth controller update | 原 NSF/HSMC/manual/FF | 相同 | 相同 | 相同 | 位于 SafeRoll 模式之外 |
| `hydro_vertical_thrust_setpoint` | 由 `HY_DC_MODE` 选择 | 相同 | 相同 | 相同 | 只受深度参数影响 |
| `command_force_x/z` | 完整发布 | 完整发布 | 完整发布 | 完整发布 | SafeRoll 不重建 |
| `u_theta/u_psi` | 旧 pitch/yaw | 相同 | 相同 | 相同 | 未覆盖 |
| `u_phi` | legacy CADRC | nominal | fixed HOCBF | adaptive | 唯一被替换的分量 |
| allocator command | 五分量完整向量 | 仅替换 `u_phi` | 仅替换 `u_phi` | 仅替换 `u_phi` | 新 bit-exact invariant 已测试 |
| common foil component | 由 force/pitch 请求决定 | 保留 | 保留 | 保留 | 若输入为零则输出也为零 |
| differential foil component | legacy roll | SafeRoll roll | SafeRoll roll | SafeRoll roll | 正常差异 |
| generic+hydro/final servo | 既有合成 | 相同 | 相同 | 相同 | 未修改 |
| dedicated tail/elevator | 未发现明确生产映射 | 不触碰 | 不触碰 | 不触碰 | UNKNOWN/N/A，不猜通道 |

`HY_FOIL_AUX=8` 是固定高 gate 源；它不解释本次“只有共同分量消失”。
gate 若关闭会抑制整个 hydro 输出，而不是只选择性清除深度。本轮没有把
问题重新归因于 AUX6。

## 4. 最小代码修复

虽然未发现现有 SafeRoll 覆盖完整向量的执行路径，本轮仍把“只替换滚转”
从隐含约定提升为可测试接口：

```text
candidate = original_allocation_command
candidate.u_phi = safe_roll_u
```

实现：

- `HydrofoilAllocationModel::withRollCommand()`：
  `HydrofoilAllocationModel.cpp:313-318`
- `checkCandidate()` 统一使用该接口：`:306-310`
- SafeRoll allocation interval 的 base command 也使用同一接口：
  `SafeRollController.cpp:773-775`

这保证 `force_x`、`force_z`、`u_theta`、`u_psi` 逐 bit 保留，不在最终
servo 层重新拼接或覆盖深度输出。Mode 0 仍直接调用 legacy observer，
没有运行候选数学。

同时加入共同/差动的明确数学工具：

```text
delta_symmetric    = (delta_left + delta_right) / 2
delta_differential = (delta_left - delta_right) / 2
delta_left         = delta_symmetric + delta_differential
delta_right        = delta_symmetric - delta_differential
```

源码：`HydrofoilAllocationModel.cpp:320-344`。该工具仅用于测试和诊断，
不改变生产 allocator 数学或物理符号。

## 5. 新增端到端诊断

新增日志 topic `haar_depth_control_status`
（`msg/HaarDepthControlStatus.msg:1-45`），记录：

- `depth_reference/measured_depth/depth_error`
- `depth_controller_mode/enabled/output_finite`
- `original_depth_controller_output`
- `pitch_reference/measured_pitch/u_theta`
- `command_force_x/command_force_z`
- requested/realized symmetric foil component
- SafeRoll differential component及最终左右翼
- allocator valid/bounded/saturation、foil gate
- `depth_command_suppressed/limited` 和数字 suppression reason
- requested/active SafeRoll code
- tail/elevator availability；当前明确记录 unavailable，而非猜测通道

辅助诊断修改：

- `HydroDepthControlMessage.msg:1-7,31-32` 增加 sample timestamp、深度
  模式、enable/finite 及最终选择的垂向力；
- `HydroRateControl.cpp:948-958` 发布原生产深度输出，不合成伪输出；
- `HydroAllocateMessage.msg:8-12` 和
  `HydroAllocator.cpp:308-324` 保留 solver 原始请求，并标明旧逻辑是否
  因 invalid 而将实际输出抑制为零；
- `ControlAllocator.cpp:1026-1109` 在最终 generic+hydro 合成后计算实际
  common/differential 分量并分类 suppressed/limited；
- logger 注册位于 `src/modules/logger/logged_topics.cpp:69`。

下一份 ULog 可以直接区分：

```text
controller disabled
controller output nonfinite
foil gate suppression
allocator invalid suppression
allocator saturation
final generic+hydro saturation
```

不会再把 PWM 或 actuator output 描述成测得翼角；所有翼角仍是
commanded/model-reconstructed。

## 6. 确定性测试

无竞争顺序执行结果：

- `unit-HydrofoilAllocationModel`：14/14 PASS
- `unit-SafeRollController`：63/63 PASS
- 合计：77/77 PASS

本轮新增：

1. `RollReplacementPreservesEveryNonRollCommandBitExactly`
   （`HydrofoilAllocationModelTest.cpp:200-217`）：
   替换 roll 后 force_x/z、u_theta/u_psi 逐值不变。
2. `SymmetricDepthAndDifferentialRollComponentsRoundTrip`
   （`:219-237`）：共同与差动分量合成/分解及 nonfinite 防护。
3. `NonzeroDepthForceProducesCommonFoilComponentWithZeroRoll`
   （`:239-251`）：零 roll、非零深度力产生非零同向水翼和零差动。
4. `ModesOneTwoThreePreserveDepthForceAndOnlyReplaceRollCommand`
   （`SafeRollControllerTest.cpp:233-280`）：Modes 1/2/3 分别验证完整
   非滚转请求保留、共同深度分量存在、最终左右解与共享 allocator 一致。

既有测试同时覆盖：

- Mode 0 样本级 bit-exact；
- Mode 0 不运行 candidate；
- Modes 1/2/3 FIELD best-feasible、temporary fallback、自动恢复；
- Mode 4 严格语义；
- generic+hydro 最终合成；
- nonfinite、gate、mapping、runtime 等 severe fault。

当前配置没有独立 tail/elevator 生产映射，故不能构造一个虚假的通道测试；
`u_theta` 作为真实 pitch virtual command 已在 Modes 1/2/3 测试中逐值验证
不变。`CADRC.cpp/.h` 和原深度控制数学、增益均未修改。

## 7. 构建与固件

`matek_h743_dummy` clean `-Werror` 构建通过。clean 后首次并行启动遇到
NuttX `jlink-nuttx` 在 `.config` 生成前运行的既有依赖竞态；`.config`
生成后以相同配置续跑，完整 1050 个目标全部成功，无编译警告或
`-Werror` 失败。

```text
firmware:
  build/matek_h743_dummy/matek_h743_dummy.px4
size:
  1,534,081 bytes
SHA-256:
  635d54a8ec886e3f7d3960f3435bd93fbf2b781045d772b6d0ac7416074d2293

ELF:
  text 1,592,424
  data     3,544
  bss     41,324
```

链接器 FLASH 报告为 1,595,968 / 1,966,080 B（81.18%）。
`git diff --check` 通过。

## 8. 修改文件

本轮功能性修改：

- `src/lib/hydrofoil_allocation/HydrofoilAllocationModel.hpp`
- `src/lib/hydrofoil_allocation/HydrofoilAllocationModel.cpp`
- `src/lib/hydrofoil_allocation/HydrofoilAllocationModelTest.cpp`
- `src/modules/hydro_rate_control/SafeRollController.cpp`
- `src/modules/hydro_rate_control/SafeRollControllerTest.cpp`
- `src/modules/hydro_rate_control/HydroRateControl.cpp`
- `src/modules/hydro_allocator/HydroAllocator.cpp`
- `src/modules/control_allocator/ControlAllocator.hpp`
- `src/modules/control_allocator/ControlAllocator.cpp`
- `msg/HydroDepthControlMessage.msg`
- `msg/HydroAllocateMessage.msg`
- `msg/HaarDepthControlStatus.msg`
- `msg/CMakeLists.txt`
- `src/modules/logger/logged_topics.cpp`

报告：

- `RSMC_codex/SAFE_ROLL_DEPTH_CONTROL_PRESERVATION_FIX_REPORT.md`

## 9. 安全边界与停止条件

- 未修改 `CADRC.cpp/.h`。
- 未修改深度控制公式、增益、默认参数、遥控映射。
- 未修改左右水翼映射、trim、REV、generic+hydro 叠加。
- 未修改 Mode 4 严格语义。
- 未自动设置任何实机参数。
- 未 flash、未执行 actuator test、未执行实机或水池测试。

在读取当前目标的 `HY_DC_MODE` 和新
`haar_depth_control_status`/`hydro_allocate_message` 前，不宣称当前实机
触发原因已经由硬件数据最终闭环确认。
