# HAAR 2026-07-27 室内水面滑行 ULog 快速准入审计

审计日期：2026-07-27  
日志目录：`/home/linyi666/PX4-Autopilot-water-air/已有ulg/log_26.7.27新`  
审计范围：递归检查全部 `*.ulg`；共 59 条。  
最终结论：**C. RETEST_SPECIFIC_GROUPS_BEFORE_LEAVING**

本审计只判断日志完整性、实验动作、分组数量和关键控制链健康状态。没有生成
图片、没有批量导出 CSV、没有计算控制性能排名或论文指标，也没有修改任何
ULog、PX4 源码、参数或固件。

## 快速判据

- Mode 仅使用日志中的 `active_code/requested_code`；不使用文件名或时间顺序。
- dynamic 要求实际参考同时出现持续的正、负变化并有回零数据。
- impact 要求参考基本不变，存在位于日志内部且前后均有数据的孤立高滚转速率
  事件。
- static 要求参考基本为零、具有连续滑行段，且不符合 dynamic/impact。
- “有效段”快速判据为：至少约 5 s、foil gate 开启、Servo4/5 数据有限且有
  推进/滑行输入。它是准入判据，不是性能指标。
- 下表中的可用数量采用 `USABLE + USABLE_WITH_CAUTION`。本批没有无条件
  `USABLE`，原因是所有可解码日志均有同一种 logger/格式注意项；详见后文。

实验类型阈值只用于本次快速内容分类：

- dynamic：正、负参考各持续至少约 0.25 s，且存在回零；
- impact：参考基本为零，孤立 `|p|` 峰值至少约 80 deg/s，峰值前后各至少
  1 s；
- 参考动作不完整或处在 static/impact 边界时标为 `TYPE_UNCERTAIN`。

## 逐条判定

`PASS` 表示对 codes 0–3 检查到完整 executable allocation 正常；旧 schema
无法直接读取新字段时，用 requested/realized symmetric、gate、finite、
bounded 和 suppression 信号交叉检查。

| 文件名 | 可解析 | 推断Mode | 实验类型 | 有效段 | 新固件检查 | 判定 | 原因 |
|---|---:|---:|---|---|---|---|---|
| log_461_2026-7-27-11-46-36.ulg | 是 | 0 | static | 否 | FAIL | UNUSABLE | 旧 schema，存在持续 requested-symmetric 非零而 realized 为零，且无滑行输入 |
| log_462_2026-7-27-11-47-04.ulg | 是 | 0 | static | 否 | FAIL | UNUSABLE | 旧 schema，存在持续 requested-symmetric 非零而 realized 为零，且时长仅 3.8 s |
| log_463_2026-7-27-14-56-40.ulg | 否 | MODE_UNCERTAIN | TYPE_UNCERTAIN | 否 | FAIL | UNUSABLE | 文件开头为零数据而非 ULog header，无法解析 |
| log_464_2026-7-27-14-56-44.ulg | 是 | 0 | static | 否 | PASS | UNUSABLE | 仅 4.1 s，无足够长有效滑行段 |
| log_465_2026-7-27-14-56-58.ulg | 是 | 0 | static | 否 | PASS | UNUSABLE | 仅 4.3 s且 throttle 近关闭，无有效滑行段 |
| log_466_2026-7-27-14-57-10.ulg | 是 | 0 | static | 是（6.3s） | PASS | USABLE_WITH_CAUTION | static 可辨；有效段偏短且有间歇日志缺口 |
| log_467_2026-7-27-14-57-32.ulg | 是 | 0 | dynamic | 是（11.0s） | PASS | USABLE_WITH_CAUTION | 正负参考均出现但正侧持续偏短；有间歇日志缺口 |
| log_468_2026-7-27-14-58-00.ulg | 是 | 0 | static | 是（6.5s） | PASS | USABLE_WITH_CAUTION | static 可辨；有效段偏短且有间歇日志缺口 |
| log_469_2026-7-27-14-58-10.ulg | 是 | 0 | static | 是（5.8s） | PASS | USABLE_WITH_CAUTION | static 可辨；有效段偏短且有间歇日志缺口 |
| log_470_2026-7-27-14-58-32.ulg | 是 | 0 | TYPE_UNCERTAIN | 是（11.2s） | PASS | USABLE_WITH_CAUTION | 只有明显负参考、没有完整正参考动作；有间歇日志缺口 |
| log_471_2026-7-27-15-00-50.ulg | 是 | 0 | static | 是（12.1s） | PASS | USABLE_WITH_CAUTION | static 动作完整；有间歇日志缺口 |
| log_472_2026-7-27-15-01-36.ulg | 是 | 0 | static | 是（10.5s） | PASS | USABLE_WITH_CAUTION | 参考基本为零，仅有短小杆量；有间歇日志缺口 |
| log_473_2026-7-27-15-02-20.ulg | 是 | 0 | TYPE_UNCERTAIN | 是（10.5s） | PASS | USABLE_WITH_CAUTION | 零参考但扰动介于普通滑行与明确 impact 之间；有间歇日志缺口 |
| log_474_2026-7-27-15-03-02.ulg | 是 | 0 | dynamic | 是（10.7s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_475_2026-7-27-15-03-46.ulg | 是 | 0 | dynamic | 是（11.5s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_476_2026-7-27-15-04-24.ulg | 是 | 0 | dynamic | 是（9.9s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_477_2026-7-27-15-05-08.ulg | 是 | 0 | dynamic | 是（10.4s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_478_2026-7-27-15-06-06.ulg | 是 | 0 | dynamic | 是（10.9s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_479_2026-7-27-15-07-24.ulg | 是 | 1 | static | 是（11.7s） | PASS | USABLE_WITH_CAUTION | static 动作完整；有间歇日志缺口 |
| log_480_2026-7-27-15-08-12.ulg | 是 | 1 | static | 是（9.7s） | PASS | USABLE_WITH_CAUTION | static 动作完整；有间歇日志缺口 |
| log_481_2026-7-27-15-08-50.ulg | 是 | 1 | static | 否 | PASS | UNUSABLE | throttle 持续为关闭侧，不能确认有效水面滑行段 |
| log_482_2026-7-27-15-09-00.ulg | 是 | 1 | static | 是（9.2s） | PASS | USABLE_WITH_CAUTION | static 动作完整；有间歇日志缺口 |
| log_483_2026-7-27-15-09-42.ulg | 是 | 1 | dynamic | 是（10.6s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_484_2026-7-27-15-10-20.ulg | 是 | 1 | dynamic | 是（9.7s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_485_2026-7-27-15-10-58.ulg | 是 | 1 | dynamic | 是（11.3s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_486_2026-7-27-15-11-38.ulg | 是 | 1 | dynamic | 是（10.9s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_487_2026-7-27-15-12-26.ulg | 是 | 1 | dynamic | 是（10.2s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_488_2026-7-27-15-13-10.ulg | 是 | 2 | static | 是（10.6s） | PASS | USABLE_WITH_CAUTION | static 动作完整；有间歇日志缺口 |
| log_489_2026-7-27-15-13-44.ulg | 是 | 2 | static | 是（10.0s） | PASS | USABLE_WITH_CAUTION | static 动作完整；有间歇日志缺口 |
| log_490_2026-7-27-15-14-20.ulg | 是 | 2 | static | 是（10.4s） | PASS | USABLE_WITH_CAUTION | static 动作完整；有间歇日志缺口 |
| log_491_2026-7-27-15-14-56.ulg | 是 | 2 | dynamic | 是（10.5s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_492_2026-7-27-15-15-34.ulg | 是 | 2 | dynamic | 是（10.7s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_493_2026-7-27-15-16-10.ulg | 是 | 2 | dynamic | 是（9.9s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_494_2026-7-27-15-16-46.ulg | 是 | 2 | dynamic | 是（10.9s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_495_2026-7-27-15-17-22.ulg | 是 | 2 | dynamic | 是（10.3s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_496_2026-7-27-15-18-06.ulg | 是 | 3 | static | 是（10.4s） | PASS | USABLE_WITH_CAUTION | static 动作完整；有间歇日志缺口 |
| log_497_2026-7-27-15-18-44.ulg | 是 | 3 | static | 是（9.9s） | PASS | USABLE_WITH_CAUTION | static 动作完整；有间歇日志缺口 |
| log_498_2026-7-27-15-19-20.ulg | 是 | 3 | dynamic | 是（11.4s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_499_2026-7-27-15-19-56.ulg | 是 | 3 | dynamic | 是（10.2s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_500_2026-7-27-15-20-32.ulg | 是 | 3 | dynamic | 是（10.8s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_501_2026-7-27-15-21-06.ulg | 是 | 3 | dynamic | 是（10.5s） | PASS | USABLE_WITH_CAUTION | dynamic 动作完整；有间歇日志缺口 |
| log_502_2026-7-27-15-24-32.ulg | 是 | 0 | impact | 是（10.7s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_503_2026-7-27-15-25-08.ulg | 是 | 0 | impact | 是（10.0s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_504_2026-7-27-15-25-48.ulg | 是 | 0 | impact | 是（9.1s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_505_2026-7-27-15-26-18.ulg | 是 | 0 | impact | 是（10.4s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_506_2026-7-27-15-26-50.ulg | 是 | 1 | impact | 是（9.4s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_507_2026-7-27-15-27-22.ulg | 是 | 1 | impact | 是（10.1s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_508_2026-7-27-15-28-12.ulg | 是 | 1 | impact | 是（11.0s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_509_2026-7-27-15-28-44.ulg | 是 | 1 | impact | 是（9.9s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_510_2026-7-27-15-29-16.ulg | 是 | 1 | impact | 是（9.5s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_511_2026-7-27-15-29-46.ulg | 是 | 1 | impact | 是（9.9s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_512_2026-7-27-15-30-22.ulg | 是 | 2 | impact | 是（9.5s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_513_2026-7-27-15-30-52.ulg | 是 | 2 | impact | 是（9.8s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_514_2026-7-27-15-31-30.ulg | 是 | 2 | impact | 是（10.4s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_515_2026-7-27-15-31-58.ulg | 是 | 2 | impact | 是（10.0s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_516_2026-7-27-15-32-34.ulg | 是 | 3 | impact | 是（9.6s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_517_2026-7-27-15-33-02.ulg | 是 | 3 | impact | 是（10.0s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_518_2026-7-27-15-33-38.ulg | 是 | 3 | impact | 是（9.9s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |
| log_519_2026-7-27-15-34-08.ulg | 是 | 3 | impact | 是（10.4s） | PASS | USABLE_WITH_CAUTION | impact 动作完整且前后有数据；有间歇日志缺口 |

## 数量汇总

### 文件和准入等级

| 项目 | 数量 |
|---|---:|
| ULog 总数 | 59 |
| 可恢复解析 | 58 |
| 标准 pyulog 1.2.2 无需兼容处理即可解析 | 0 |
| USABLE | 0 |
| USABLE_WITH_CAUTION | 53 |
| UNUSABLE | 6 |
| exact SHA-256 重复 | 0 |
| 明显损坏 | 1 |

“可恢复解析”是指只跳过缺少 FORMAT 的非关键
`haar_safe_roll_estimator_status` 后，其余 topic 均能由 pyulog 正常解析。
58 个有 ULog header 的文件都存在该缺失 FORMAT 问题；`log_463` 还缺少有效
ULog header。审计脚本保留于
`RSMC_codex/quick_ulg_audit_tmp/quick_ulg_audit.py`，不修改原始文件。

### 按 Mode 的可准入日志

这里的“可准入”包含 `USABLE_WITH_CAUTION`，TYPE_UNCERTAIN 仍计入 Mode
总数，但不计入三类实验矩阵。

| Mode | 可准入数量 |
|---:|---:|
| 0 | 17 |
| 1 | 14 |
| 2 | 12 |
| 3 | 10 |
| 4 | 0 |

### 按实验类型的可准入日志

| 类型 | 数量 |
|---|---:|
| static | 13 |
| dynamic | 20 |
| impact | 18 |
| TYPE_UNCERTAIN | 2 |

### Mode × 实验类型矩阵

每个数字均为 `USABLE + USABLE_WITH_CAUTION`；本批实际均为 CAUTION。

| Mode | static | dynamic | impact | 每组是否至少3条 |
|---:|---:|---:|---:|---|
| 0 | 5 | 6 | 4 | 是 |
| 1 | 3 | 5 | 6 | 是 |
| 2 | 3 | 5 | 4 | 是 |
| 3 | **2** | 4 | 4 | **否：static 缺1条** |
| 4 | 0 | 0 | 0 | 否；但为非核心消融组 |

Mode 0–3 是核心矩阵。唯一数量不足的核心单元格为：

```text
Mode 3 / static：现有2条，目标至少3条，必须补1次。
```

Mode 4 完全缺失，但按任务定义仅是严格消融组，不能单独阻止离场；需要在论文
消融计划中另行注明。

## Mode、动作和故障审计

### Mode 识别

- 58 个可解析日志中，Mode 均能从日志内容识别。
- 可准入日志的 `requested_code == active_code` 比例最低仍为 99.15%，差异仅
  出现在启动/激活过渡，没有长期模式错配。
- 没有按文件创建时间推断 Mode。
- `MODE_UNCERTAIN` 仅有损坏的 `log_463`。

### 实验类型

- dynamic 参考实际出现了正负变化并有回零，未按预设角度硬匹配。
- 明确 impact 日志的峰值位于约 4–6 s，峰值前后均有数据，不是日志开头
  启动或结尾收油事件。
- `log_470` 只有明显负参考、没有完整正参考动作；
  `log_473` 的零参考扰动强度处于 static 与明确 impact 之间。因此两者保持
  `TYPE_UNCERTAIN`，未强行计入数量矩阵。

### RC、failsafe、重启和严重故障

在 58 个可解析日志中未发现：

- RC lost 或 RC failsafe；
- `vehicle_status.failsafe`；
- manual-control-signal lost；
- SafeRoll severe fault；
- topic timestamp 倒退；
- 同一日志内的飞控重启证据。

`log_463` 是损坏文件，无法对其内部故障状态作判断。

## 新固件完整分配检查

对具有新字段且可准入的 codes 0–3 日志，全部满足：

```text
selected_solution_source == 1
mode_branch_overrode_solution == false
symmetric_depth_lost == false
allocator_output_suppressed == false
foil_gate_enabled == true
allocator_finite == true
allocator_bounded == true
```

同时：

- 没有 accepted 日志出现持续 requested/active code 不一致；
- 没有左右水翼 commanded 信号缺失或非有限；
- `allocator_valid/converged=false` 与
  `full_best_feasible_solution_used/depth_degradation_active=true` 的组合被按
  允许的 finite+bounded best-feasible 处理，没有误判为失败；
- 没有 accepted 日志出现 `symmetric_depth_lost`；
- 没有新 schema 日志出现旧 Mode 2 manual branch override 复发。

`log_461`、`log_462` 是早期旧 schema/无滑行输入的 checkout 日志，确实出现
持续的 requested symmetric 非零而 realized symmetric 为零，已判
UNUSABLE。它们是修复前证据，不是后续新固件中的复发。

## 全批次注意项

### 1. 标准 pyulog FORMAT 缺失

58 个有效 header 的日志用原版 pyulog 1.2.2 均会因：

```text
KeyError: haar_safe_roll_estimator_status
```

而中止。原因是该 logged topic 没有对应 FORMAT 定义。审计脚本只跳过这个
无法解码的非关键 topic，其余所需 topic 可正常解析。

这不影响本次列出的姿态、参考、Mode、allocator、depth 和 actuator 准入
结论，但正式论文脚本必须复用该兼容读取逻辑或先修复日志格式工具链。

### 2. 间歇日志缺口

所有可准入日志的 `haar_safe_roll_status` 都存在间歇数据缺口：

- 采样时间间隔 p99 约 54–72 ms；
- 单个最大间隔约 161–227 ms。

因此本批全部标为 `USABLE_WITH_CAUTION`。重复次数较多，且 dynamic/impact
动作和前后段仍清晰可辨，所以该问题本身不要求把全部实验现场重做；但后续
分析不能把这些日志当作无缺口的严格 400 Hz 序列，尤其是 impact 瞬时峰值。

## 离场判断

### 必须立即补做

只补以下核心单元格：

| Mode | 实验类型 | 当前可准入 | 至少需要 | 缺少 |
|---:|---|---:|---:|---:|
| 3 | static | 2 | 3 | **1次** |

补做时应确认：

- requested/active code 持续为 3；
- 滚转参考基本为零；
- 至少约 8–10 s 连续有效滑行；
- foil gate 开启；
- `selected_solution_source=1`；
- 无 branch override、symmetric-depth-lost、suppression、RC/failsafe；
- 日志文件能在停止记录后实际保存。

不要求因为 Mode 4 缺失而继续留场，也不要求重做其他 Mode 0–3
dynamic/impact/static 组。

## 最终结论

**C. RETEST_SPECIFIC_GROUPS_BEFORE_LEAVING**

核心控制链和新完整分配修复在可准入日志中没有发现故障，绝大多数组数量已经
足够。但 Mode 3/static 只有 2 条，少于明确要求的 3 条。完成 1 次新的
Mode 3 static 并当场确认日志保存与关键字段后，可以离开试验场地。
