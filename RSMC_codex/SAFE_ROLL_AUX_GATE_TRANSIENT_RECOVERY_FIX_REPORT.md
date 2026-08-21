# HAAR SafeRoll AUX 门控瞬态恢复修复报告

日期：2026-07-24  
工程：`/home/linyi666/PX4-Autopilot-water-air`  
结论状态：软件修复、主机测试和 H743 clean build 已完成；未刷写、未执行 actuator test、未进行实机动作。

## 1. 结论

实机首次故障 `0x00000100` 已由快照确定为 `FoilGateDisabledFault`。触发链为：

```text
HY_FOIL_AUX=6
  -> manual_control_setpoint.aux6=-1
  -> foil_aux=-1 * HY_FOIL_AUXGAIN
  -> foil_aux > HY_FOIL_THR(-0.5) 为 false
  -> AUX gate false
  -> combined foil gate false
  -> 原严格路径置 FoilGateDisabledFault
  -> severe latch，active_code=0
```

本轮把 AUX gate 和 nav-state gate 显式分开，并只改变 `FIELD=1`、请求 Modes 1/2/3 时“有限 AUX 阈值关闭”的处理：

- 水翼门关闭后，`HydroAllocator` 当拍即停止 hydro 输出，不继续发送 SafeRoll 水翼命令；
- SafeRoll 保持 `requested_code`，令 `active_code=0`，进入 `aux_gate_inhibit_active` 和 temporary fallback；
- 不设置 severe fault，不锁存 fallback；
- AUX gate 恢复后，必须通过连续 20 拍完整健康 candidate 检查，随后自动 `beginBlend()` 回请求模式；
- AUX 持续关闭可以无限期 inhibit，不转为永久软件故障；
- `FIELD=0`、Mode 4、nav-state 关闭、AUX 非有限/失效、allocator 严重错误继续沿用严格 severe 语义。

未修改 SafeRoll 控制数学、CBF、estimator、certificate、allocation solver、左右映射、Mode 0、Mode 4 或 FIELD 物理不可行处理。

## 2. AUX6 数据源审计

### 2.1 完整生产链

1. `RC_MAP_AUX6` 决定逻辑 AUX6 使用哪个物理 RC 通道。参数默认值为 0（未映射）：
   - `src/modules/rc_update/params.c:1764`
   - `src/modules/rc_update/rc_update.cpp:250-255`
2. 原始 `input_rc.values[]` 按该通道的 MIN/TRIM/DZ/MAX 做分段线性校准，输出 `[-1, 1]`，随后应用 REV：
   - `src/modules/rc_update/rc_update.cpp:476-497`
3. `get_rc_value()` 返回映射通道的校准值并约束到 `[-1,1]`；未映射时返回 `0.f`：
   - `src/modules/rc_update/rc_update.cpp:304-310`
4. `UpdateManualControlInput()` 把逻辑 AUX6 写入 `manual_control_setpoint.aux6`，同时写入 `timestamp_sample`、`SOURCE_RC` 和 `valid`：
   - `src/modules/rc_update/rc_update.cpp:679-701`
5. ManualControl selector 使用 `timestamp_sample`、`valid`、`data_source` 和 `COM_RC_LOSS_T` 判断输入是否可选，并保留原始 sample timestamp：
   - `src/modules/manual_control/ManualControlSelector.cpp:36-87`
   - `src/modules/manual_control/ManualControl.cpp:100-148,299`
6. HydroAllocator 通过 subscription `copy()` 更新缓存；没有新消息时继续使用最后一条缓存，不会每拍构造一个默认消息：
   - `src/modules/hydro_allocator/HydroAllocator.cpp:163`
7. `HY_FOIL_AUX=6` 选择缓存的 `.aux6`，乘以 `HY_FOIL_AUXGAIN`，再与 `HY_FOIL_THR` 严格比较：
   - `src/modules/hydro_allocator/HydroAllocator.cpp:524-539,571-577`

### 2.2 `-1` 的准确语义

`manual_control_setpoint.aux6=-1` 是已归一化 AUX 通道的合法最低端值。它不是：

- 未映射值：未映射由 `get_rc_value()` 返回 `0`；
- 默认构造占位值：零初始化消息的该字段是 `0`；
- PX4 为“无数据”主动注入的 sentinel；
- RC loss/failsafe 的固定替代值。

因此，本次 `-1` 说明一个被选中的有限输入样本在 AUX6 上到达了校准下端。根据现有故障快照，不能进一步唯一判断它是：

- 操作者真实把开关置于低端；
- 接收机在链路异常时继续输出低端，但未置 `rc_failsafe`；
- 某一物理通道的单帧瞬态；
- 输入源切换后另一有效 source 给出的低端值。

`InputRc.msg:29-30` 明确指出：`rc_failsafe=true` 可信，但部分接收机会在失联后继续发帧而不报告 failsafe。因此不能仅凭 `-1` 把事件称为“RC 抖动”或“RC failsafe”。

### 2.3 丢失、failsafe、重连和更新频率

- `input_rc.rc_lost`、`input_rc.rc_failsafe`、通道数不足或配置的 failsafe threshold 会使 RC signal-lost 判定为真：
  - `src/modules/rc_update/rc_update.cpp:427-474`
- RCUpdate 即使信号无效仍发布 `rc_channels` 用于诊断，但只有输入 source/channel 稳定且 signal-lost hysteresis 为 false 时才发布有效 manual input：
  - `src/modules/rc_update/rc_update.cpp:500-517`
- 重连后有 100 ms signal-regain hysteresis，避免把重启瞬态当作操作：
  - `src/modules/rc_update/rc_update.cpp:500-505`
- 通道变化时发布 manual input；通道不变时最迟每 300 ms 发布一次：
  - `src/modules/rc_update/rc_update.cpp:519-537`
- selector 超时后不再把旧输入视为有效，不会合成 `-1`：
  - `src/modules/manual_control/ManualControlSelector.cpp:69-87`
- 模式切换本身不改写 AUX6；它只可能改变所选 manual input source。

### 2.4 当前物理 RC 映射

源码确认 `HY_FOIL_AUX=6` 表示“逻辑 AUX6”，而实际物理通道由目标机当前 `RC_MAP_AUX6` 决定。现有实机快照没有保存该参数，因此物理通道在本报告中仍标记为 **尚待目标只读确认**，不能从 `HY_FOIL_AUX=6` 推断。

新增诊断直接发布：

- `foil_aux_rc_channel`；
- `foil_aux_mapping_valid`；
- manual input `data_source`、timestamp 和 age；
- `rc_signal_lost`、`rc_failsafe`。

下一次 ULog 可区分 RC mapping、source 切换、数据 stale、显式 failsafe 与单纯阈值关闭。

## 3. 门控实现

HydroAllocator 现明确计算：

```text
aux_gate_enabled_raw =
    config_valid
    AND aux_finite
    AND manual_data_fresh/mapped
    AND foil_aux > HY_FOIL_THR

nav_gate_enabled_raw =
    nav_state in {STAB, ACRO, MANUAL}

foil_gate_enabled =
    aux_gate_enabled_raw AND nav_gate_enabled_raw
```

关键位置：

- gate 原因枚举：`src/modules/hydro_allocator/HydroAllocator.cpp:43-49`
- AUX、配置、timestamp、mapping 检查：`HydroAllocator.cpp:510-570`
- raw AUX/nav/combined gate：`HydroAllocator.cpp:571-577`
- 原因分类及诊断发布：`HydroAllocator.cpp:578-620`

关闭原因数字定义：

| code | 含义 |
|---:|---|
| 0 | none |
| 1 | aux threshold |
| 2 | nav state |
| 3 | aux nonfinite |
| 4 | aux stale/mapping invalid |
| 5 | gate config invalid |

`HY_FOIL_AUX=8` 仍按既有语义提供固定 `+1`；没有把选择器硬编码成 8，也没有删除 AUX gate。

## 4. SafeRoll 状态机修改

### 修改前

```text
任何 foil_gate_enabled=false
  -> FoilGateDisabledFault
  -> severe latch
  -> active_code=0
  -> 必须 mode change 或 RST_ID 才能恢复
```

### 修改后

只在以下完整条件成立时进入可恢复 AUX inhibit：

```text
FIELD=1
AND requested mode in {1,2,3}
AND AUX gate data/config/数值有效
AND nav gate=true
AND raw AUX gate=false
```

处理路径：

```text
finite AUX threshold closure
  -> HydroAllocator 当拍 gate hydro output
  -> active_code=0
  -> aux_gate_inhibit_active=true
  -> temporary_fallback_active=true
  -> severe_fault_code=0
  -> fallback_latched=false

AUX raw gate 恢复
  -> 每拍运行完整 allocator/candidate/mapping 检查
  -> 连续20拍严格健康
  -> clear inhibit
  -> beginBlend(requested_code)
  -> 自动恢复 active_code
```

关键位置：

- FIELD AUX inhibit 分类：`src/modules/hydro_rate_control/SafeRollController.cpp:1207-1213`
- activation pending 时不把有限 AUX 关闭记为 fatal：`SafeRollController.cpp:1246-1279`
- 已激活模式的 immediate inhibit/strict fault 分流：`SafeRollController.cpp:1282-1305`
- 完整 candidate recovery 与连续 20 拍计数：`SafeRollController.cpp:1617-1659`
- severe 状态清除互斥诊断：`SafeRollController.cpp:1847-1865`
- 最终 streak/inhibit/safety 字段：`SafeRollController.cpp:1883-1904`

下列情况没有降级为可恢复门控：

- AUX 值 NaN/Inf；
- AUX 参数、mapping、manual sample 有效性或 freshness 非法；
- nav-state gate 关闭；
- allocator nonfinite 或严重 timestamp stale；
- 其他既有 severe fault。

`FIELD=0` 的 `FoilGateDisabledFault 0x00000100` 行为保持不变。Mode 4 不进入 FIELD 可恢复 AUX inhibit。

## 5. 诊断增强

新增小型 uORB topic：

```text
haar_safe_roll_gate_status
```

定义：`msg/HaarSafeRollGateStatus.msg:1-36`  
发布：`src/modules/hydro_rate_control/HydroRateControl.cpp:702-730`  
logger 注册：`src/modules/logger/logged_topics.cpp:68`

记录字段覆盖：

- raw AUX gate、raw nav gate 和 combined foil gate；
- AUX value、threshold、selector；
- manual-control timestamp、sample timestamp、age、source、valid；
- RC signal-lost/failsafe；
- RC channel mapping 与有效性；
- AUX low/high streak；
- inhibit active；
- 数字 gate-close reason。

同时扩展 `hydro_allocate_message` 传递同一组原始 gate 诊断：

- `msg/HydroAllocateMessage.msg:34-50`

没有增加字符串算法名称；诊断 topic 不参与控制反馈。

## 6. 修改文件

本轮功能修改：

- `msg/HydroAllocateMessage.msg`
- `msg/HaarSafeRollGateStatus.msg`（新增）
- `msg/CMakeLists.txt`
- `src/modules/hydro_allocator/HydroAllocator.hpp`
- `src/modules/hydro_allocator/HydroAllocator.cpp`
- `src/modules/hydro_rate_control/SafeRollController.hpp`
- `src/modules/hydro_rate_control/SafeRollController.cpp`
- `src/modules/hydro_rate_control/HydroRateControl.hpp`
- `src/modules/hydro_rate_control/HydroRateControl.cpp`
- `src/modules/hydro_rate_control/SafeRollControllerTest.cpp`
- `src/modules/logger/logged_topics.cpp`

未修改：

- `src/modules/hydro_rate_control/CADRC.cpp`
- `src/modules/hydro_rate_control/CADRC.h`
- HydrofoilAllocationModel 求解数学；
- CBF、estimator、certificate；
- actuator mapping、trim、generic+hydro 叠加；
- Mode 0 直接输出路径；
- Mode 4 严格语义。

CADRC 文件 SHA-256：

```text
CADRC.cpp a48a62f8a87c120eff482487acad5d065ace0899add4978d6c5c02a08ced7201
CADRC.h   ec8cd46017e0efefaae60e78fa6d2b7bf7f8eff3530926b8ac9d98b7862c8a54
```

## 7. 测试

### 主机单元测试

```text
SafeRollController          62/62 PASS
HydrofoilAllocationModel    11/11 PASS
合计                        73/73 PASS
```

原 71 项全部保留通过。新增确定性覆盖：

1. Mode 3、FIELD=1、AUX 单拍降到 `-1`：无 severe/latch，当拍 inhibit；
2. AUX 恢复后连续 20 拍自动 blend 回 Mode 3，不改 mode/RST_ID；
3. Mode 2 重复同一过程并自动恢复；
4. AUX 持续关闭 1000 拍（约 2.5 s）：持续禁止输出且不锁存，恢复后自动重入；
5. FIELD=0 同一关闭事件保持 `0x00000100` strict fault；
6. FIELD=1 下 AUX NaN/Inf 立即 severe；
7. AUX data stale/invalid 立即 severe；
8. nav-state 不允许仍立即 severe；
9. Mode 0 既有 bit-exact 回归通过；
10. Mode 4 既有严格行为回归通过。

测试位置：

- `src/modules/hydro_rate_control/SafeRollControllerTest.cpp:1277-1346`
- `src/modules/hydro_rate_control/SafeRollControllerTest.cpp:1348-1400`

### 静态检查

```text
git diff --check: PASS
```

### H743 clean `-Werror` build

目标：`matek_h743_dummy`  
结果：`1049/1049 PASS`

构建配置阶段仍打印本机既有 SymForce circular-import traceback，但配置和完整固件链接成功；没有新增编译警告或 `-Werror` 失败。

内存结果：

```text
FLASH image: 1,594,328 bytes (81.09%)
text:        1,590,784 bytes
data:            3,544 bytes
bss:            41,324 bytes
```

固件：

```text
路径:   build/matek_h743_dummy/matek_h743_dummy.px4
大小:   1,532,489 bytes
SHA256: 1abe3e939c67186a05a37dbc33bdf1535b1861980398951f1a3081add1b44c59
```

## 8. 实机复测建议（仅说明，未执行）

刷写及实机操作不属于本轮已执行范围。若人工后续复测，应同时记录：

- `listener haar_safe_roll_gate_status`
- `listener hydro_allocate_message`
- `listener haar_safe_roll_fault_snapshot`
- `param show RC_MAP_AUX6`

重点确认发生低端事件时：

- `manual_control_data_source`；
- `foil_aux_rc_channel` 和 mapping validity；
- `manual_control_age_us`；
- `rc_signal_lost` / `rc_failsafe`；
- `gate_close_reason=1` 是否与实际操作者开关动作一致；
- 恢复后 `aux_gate_high_streak` 到 20 并自动重新 blend。

本报告没有宣称单拍 `-1` 的物理来源已经通过实机确认；源码只能确定其数值语义和允许它进入控制链的条件。
