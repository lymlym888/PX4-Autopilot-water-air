# HAAR SafeRoll Mode 2/3 FIELD 连续恢复修复报告

日期：2026-07-24  
基线 HEAD：`5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`  
范围：软件定位、最小修复、主机测试和 `matek_h743_dummy` clean `-Werror` 构建。未执行 flash、actuator test 或实机测试。

## 1. 结论与根因

### 1.1 `0x00080000` 的精确定义

**源码已验证：**

- `0x00080000 = 1u << 19 = ProjectionInvalidFault`，定义于
  `src/modules/hydro_rate_control/SafeRollController.hpp:61`。
- `NonlinearCandidateInvalidFault = 1u << 18 = 0x00040000`，定义于同文件
  `:60`。

因此 Mode 3 岸测的 `severe_fault_code=524288` 是
`ProjectionInvalidFault`，不是此前推断的
`NonlinearCandidateInvalidFault`。

### 1.2 Mode 3 的已证实触发路径

Mode 2/3 共用的计算路径为：

```text
finite nominal
  -> robust CBF interval
  -> CBF ∩ hard interval
  -> minimum relaxation
  -> relaxed CBF ∩ hard interval
  -> projection/nonlinear validation
```

修复前，两个有限区间在最小松弛后仍可能因物理冲突或浮点边界的数个
ULP 而保持空交集。代码随即写入 `ProjectionInvalidFault`，然后统一
`fatal_faults` 汇总将其变为 severe latch。FIELD 的 best-feasible
分类尚未接管这个出口。

本轮构造了确定性复现：`phi=-30 deg`、`p=0`、人工参考 `+40 deg`。
修复前 Mode 2 和 Mode 3 都得到：

```text
first/severe fault = 0x00080000
active_code = 0
fallback_latched = true
```

该复现与 Mode 3 实机故障码一致，证明现场 Mode 3 进入了这一
Mode 2/3 共用投影出口，而不是 Mode 3 独有的自适应估计器出口。

### 1.3 Mode 2 的结论边界

Mode 2 现场没有保存首次故障码，所以**无法以现有实机证据断言其现场
故障也一定是 `0x00080000`**。源码审计和确定性测试证明：

- Mode 2 使用相同的 CBF/硬区间、投影、blend 和 nonlinear validation
  路径；
- 同一有限动态冲突可在 Mode 2 复现 `0x00080000`；
- Mode 2 还可能在共用 nonlinear validation 严格路径产生
  `0x00040000`；
- Mode 2 的固定最坏情况 `b0/[b_min,b_max]` 不依赖 Mode 3 在线集合
  证书。

因此修复覆盖了 Mode 2/3 的共用故障出口，但没有把未知的 Mode 2
现场首次故障码写成已验证事实。

### 1.4 FIELD 分类为何此前没有接管

旧顺序实质上是：

```text
finite dynamic infeasibility
  -> fault bit 写入
  -> fatal mask 汇总
  -> severe latch
  -> FIELD best-feasible 状态仍可能残留
```

修复后，在写 fault bit 之前先区分：

```text
nonfinite / structural / absolute-bound violation -> severe
finite + bounded                            -> best feasible
finite + no bounded candidate               -> hold / temporary Legacy
```

只有第一类进入 severe mask。有限物理不可行不再先写 fatal bit。

### 1.5 自动恢复的第二个根因

此前 `allocator_best_feasible` 对严格健康 allocator 也可能为真，且该
分支在 recovery 分支之前执行，导致恢复 pending/streak 被健康样本反复
清除。恢复计数还没有要求整条 candidate 路径重新成功。

修复后：

- `allocator_best_feasible` 只在 bounded finite 且
  `!valid || !converged` 时成立
  (`SafeRollController.cpp:1197-1201`)；
- recovery probe 会重新执行 reference、estimator、certificate、
  allocation、projection、nonlinear 和 blended validation
  (`:1372-1588`)；
- 只有严格健康 allocator 和完整 bounded candidate 连续 20 拍成功，
  才清除 temporary fallback 并 `beginBlend()` 回 requested mode
  (`:1559-1583`)；
- 不修改 requested mode，也不修改 `HY_SR_RST_ID`。

## 2. 共用故障出口审计

下表中的“FIELD 修复后”适用于 Modes 1/2/3；Mode 4 维持严格语义。

| 故障/出口 | 关键源码位置 | 全部输入可 finite？ | FIELD 分类是否早于 fault bit | severe / active / temporary 结果 | 模式范围 |
|---|---|---:|---:|---|---|
| `EmptyHardIntervalFault` | `SafeRollController.cpp:1420-1430` | 是 | 是 | finite：hold→temporary，无 latch；nonfinite/strict：severe、active 0 | M1/2/3 共用；M4 严格 |
| `ProjectionInvalidFault`：nominal 非有限 | `:1433-1445` | 否 | 不适用 | 立即 severe、active 0 | 共用 |
| `ProjectionInvalidFault`：CBF/hard 有限冲突 | `:1462-1497` | 是 | 是 | 使用 immutable `U_hard` 作为 bounded best-feasible；M2/3 标记 safety degraded | M2/3 共用 |
| `ProjectionInvalidFault`：最终 blend/psi 非有限 | `:1590-1615` | 否 | 不适用 | 立即 severe、active 0 | 共用 |
| `NonlinearCandidateInvalidFault`：projected/base/retracted | `:1500-1518`；验证与固定次数回缩在 `:795-965` | 是 | 是 | finite bounded：best-feasible；finite 无 bounded：hold→temporary；nonfinite/structural：severe | 共用 |
| `NonlinearCandidateInvalidFault`：blended candidate | `:1527-1545` | 是 | 是 | 分类同上 | 共用 |
| `InvalidCertificateFault` | `:1383-1393` | 是 | 是 | finite state：hold→temporary；非有限：severe | M3 在线；M2 使用固定 `b0`/范围，不要求 M3 在线证书 |
| `RuntimeOverrunFault` | `:1663-1674` | 是 | 不属于物理可行性 | FIELD 仅单拍 `>2500 us` severe；soft pressure 只计数；strict 保持既有五拍规则 | 共用 |
| allocator input/output | `:1190-1205`，状态机 `:1226-1353` | 是/否 | 是 | finite bounded invalid：best-feasible；finite 无结果：hold→temporary；nonfinite/stale/gate：立即 severe | M1/2/3；M4 严格 |
| final normalized output | `:829-846` | 是 | 是 | finite 越普通 `[-1,1]`：FIELD clamp 并 best-feasible；非有限：severe | 共用 |
| final foil angle/rate | `:848-877` | 是 | 是 | finite 达限：FIELD 在不可变角度/速率边界内约束并 best-feasible；非有限/结构错误：severe | M1/2/3；M4 严格 |
| full forward-model result | `:880-903` | 可能否 | 是 | forward invalid 或 effective nonfinite：severe | 共用 |
| fatal 汇总与 latch | `:1677-1755` | — | 所有 FIELD finite 分类均已在此前完成 | severe 清 active 并锁存；普通 degradation 不进入 | 共用 |
| CBF 与 hard 不相交 | `:1462-1497` | 是 | 是 | 先最小松弛；仍为空时 FIELD 保留 hard constraints 并明确“不宣称本拍重新认证” | M2/3 |

`ProjectionInvalidFault` 和 `NonlinearCandidateInvalidFault` 均被保留；
修复不是屏蔽 fault，也没有强制 `candidate_valid=true`。它们仍用于
nonfinite、结构性 solver/forward 损坏以及无法信任的最终输出。

## 3. 修复后的状态机

### 3.1 bounded best-feasible

条件：输入、solver 结果和最终命令 finite，且最终命令可保持在既有硬
边界内，但严格 CBF/nonlinear/收敛目标暂时不能满足。

状态：

```text
severe_fault_code = 0
fallback_latched = false
active_code = requested_code
best_feasible_active = true
safety_degraded_active = true  (Modes 2/3)
```

### 3.2 无 bounded candidate

前 20 拍通过既有 `DUMAX` 限速保持/靠近 last feasible；随后无限期
平滑 temporary Legacy：

```text
requested_code = 2 or 3
active_code = 0
temporary_fallback_active = true
safety_degraded_active = true
severe_fault_code = 0
fallback_latched = false
```

完整 candidate 条件连续健康 20 拍后自动 `beginBlend()` 返回 requested
mode，无需 `2->0->2`、`3->0->3` 或修改 reset id。

### 3.3 severe latch

nonfinite、严重 timestamp/sensor stale、配置/映射/gate 错误、结构性
solver/forward 损坏、最终非有限输出或单拍硬超时仍 severe。状态被强制
互斥：

```text
active_code = 0
severe_fault_code != 0
fallback_latched = true
best_feasible_active = false
temporary_fallback_active = false
safety_degraded_active = false
```

实现位于 `SafeRollController.cpp:1717-1749`。

## 4. latch 清除路径

源码审计结果：

- requested mode 明确变化可清 severe latch
  (`SafeRollController.cpp:1067-1086`)；
- `HY_SR_RST_ID` 变化可清 severe latch（同上）；
- 显式切入 Mode 0 也属于 requested-mode change，并走 legacy direct
  (`:1034-1064`)；
- attitude reset、arming/STAB edge、内部 estimator/config reset 会
  reset 新控制器状态，但 `explicit_latch_clear` 不包含这些条件，不能
  意外清除 severe latch；
- FIELD 普通 degradation 根本不创建 severe latch，因此自动恢复不是
  “自动清 latch”。

## 5. 修改文件

本轮只修改：

1. `src/modules/hydro_rate_control/SafeRollController.hpp`
   - nonlinear validation 返回明确的 severe/non-severe 分类
     (`:350-354`)；
   - 增加仅在 host test 编译路径存在的 deterministic failure seam。
2. `src/modules/hydro_rate_control/SafeRollController.cpp`
   - bounded/nonfinite nonlinear 分类与固定次数回缩 (`:795-965`)；
   - FIELD candidate-unavailable 状态入口 (`:1190-1224`)；
   - finite certificate/hard interval/CBF projection 的提前分类
     (`:1377-1518`)；
   - blended candidate 分类与 20 拍完整恢复 probe (`:1527-1588`)；
   - FIELD hard runtime 与互斥诊断状态 (`:1663-1783`)。
3. `src/modules/hydro_rate_control/SafeRollControllerTest.cpp`
   - 新增 FIELD/strict、Mode 2/3、自动恢复和互斥状态回归
     (`:1298-1492`)。

未修改：

- `CADRC.cpp` SHA-256：
  `a48a62f8a87c120eff482487acad5d065ace0899add4978d6c5c02a08ced7201`
- `CADRC.h` SHA-256：
  `ec8cd46017e0efefaae60e78fa6d2b7bf7f8eff3530926b8ac9d98b7862c8a54`

也未修改 PHIS、DMAX、DRATE、DUMAX、20/80/81 时序、HOCBF/estimator/
certificate 数学、allocator residual tolerance、左右映射、Mode 0 direct
路径或 Mode 4 严格语义。

## 6. 新增测试与结果

新增 7 项：

1. FIELD soft runtime pressure 不锁存，单拍硬超时仍 severe；
2. Mode 2/3 finite CBF projection conflict 使用 bounded best-feasible；
3. FIELD=0 相同 projection conflict 保持严格 `0x00080000`；
4. Mode 2/3 finite nonlinear failure：20 拍 hold、随后 temporary
   fallback、20 拍完整健康后自动恢复，且不改变 mode/reset；
5. FIELD=0 finite nonlinear failure 保持严格
   `NonlinearCandidateInvalidFault`；
6. finite empty hard interval 进入 degradation；
7. nonfinite nonlinear candidate 立即 severe，并清除互相矛盾的
   degradation 状态。

主机测试结果：

```text
SafeRollController             59/59 PASS
HydrofoilAllocationModel       11/11 PASS
总计                            70/70 PASS
```

原 63 项全部保留并通过。现有测试继续覆盖 Mode 0 bit-exact、
`ENABLE=0` legacy、Mode 4 严格行为、allocator 模型和 speed-context
缓存。`git diff --check` 通过。

## 7. H743 clean `-Werror` 构建

目标：`matek_h743_dummy`  
结果：**PASS**，1047/1047，链接器报告 FLASH 80.90%。

产物：

| 文件 | 大小 |
|---|---:|
| `build/matek_h743_dummy/matek_h743_dummy.px4` | 1,527,825 bytes |
| `build/matek_h743_dummy/matek_h743_dummy.bin` | 1,590,608 bytes |
| `build/matek_h743_dummy/matek_h743_dummy.elf` | 34,334,216 bytes |

ELF size：

```text
text    1,587,064
data        3,544
bss        41,324
dec     1,631,932
```

可刷固件：

```text
/home/linyi666/PX4-Autopilot-water-air/build/matek_h743_dummy/matek_h743_dummy.px4
SHA-256 ed254a1519beff5dad8ce4f49645956215492cfd4102cf5e2266c458c831f3fa
```

该固件仅完成软件构建验证；本轮没有执行 flash、actuator test 或任何
实机/水池测试。
