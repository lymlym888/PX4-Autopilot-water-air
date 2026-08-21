# HAAR SafeRoll allocator 激活去抖最小修复报告

日期：2026-07-24  
源码基线 HEAD：`5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`  
结论：软件修复、主机测试和 H743 clean `-Werror` 构建已完成；未 flash，未执行 actuator test，未进行实机测试。

## 1. 实机证据与根因

`0x00000080` 的准确枚举是：

```cpp
AllocatorInvalidFault = 1u << 7
```

定义位置：`src/modules/hydro_rate_control/SafeRollController.hpp:51`。

实机首次故障的 `first_fault_stage_mask=0`、`first_fault_total_candidate_time_us=4` 和
`first_fault_cycle_time_us=32` 说明故障发生在 reference filter、estimator、allocation query
等候选阶段之前。修改前，`SafeRollController::evaluate()` 在候选入口前对单拍
`allocator_valid=false` 或 allocator 消息 timestamp 无效/超时直接置
`AllocatorInvalidFault`，随后第一次非初始采样立即永久锁存。五秒后的
`allocator_valid=true` 只能说明 allocator 已恢复，不能解除已经锁存的 fallback。

根因是：Mode 0 到新模式切换时没有 allocator 就绪等待窗口，也没有区分
`valid/converged` 的瞬时掉线与 nonfinite/freshness 等严重错误。

## 2. 修复前后状态机

修改前：

```text
request mode 1/2/3
 -> reset/blend initialization
 -> one sample allocator_valid=false or allocator timestamp invalid
 -> 0x80
 -> first fault latch
 -> permanent legacy fallback until reset/mode transition
```

修改后：

```text
request mode 1/2/3
 -> activation_pending=true, active_code=0
 -> keep legacy fallback output
 -> require 20 consecutive samples:
      allocator_valid
      AND allocator_converged
      AND allocator_finite
      AND allocator timestamp fresh
      AND foil_gate_enabled
 -> any failed readiness sample resets valid streak, but does not set fatal fault
 -> 20th consecutive healthy sample starts the existing bumpless blend
 -> active_code=requested mode

after activation:
 -> allocator nonfinite/stale: immediate AllocatorInvalidFault
 -> foil gate closed: immediate FoilGateDisabledFault
 -> logical mapping invalid: immediate LogicalMappingInvalidFault
 -> allocator valid/converged false:
      current sample uses the existing rate-limited live legacy fallback path
      increment allocator_invalid_streak
      samples 1..4 do not latch
      a healthy sample clears the invalid streak and re-blends
      sample 5 sets AllocatorInvalidFault and latches fallback
```

Mode 0 continues to call the direct `observeLegacy()` path and does not use this state machine.
Mode 4 behavior was deliberately not changed because the requested preparation policy is limited
to modes 1/2/3.

## 3. 代码修改

### Controller 状态与判断

- `src/modules/hydro_rate_control/SafeRollController.hpp:131-164`
  - 输入增加独立 `allocator_converged` 和 `allocator_finite`。
- `src/modules/hydro_rate_control/SafeRollController.hpp:192-195`
  - 输出增加 streak 和 pending 数字状态。
- `src/modules/hydro_rate_control/SafeRollController.hpp:293-294`
  - 固定常量：激活连续健康 20 拍、激活后连续无效 5 拍。
- `src/modules/hydro_rate_control/SafeRollController.hpp:359-373`
  - 固定大小状态变量，无 heap、无动态分配。
- `src/modules/hydro_rate_control/SafeRollController.cpp:933-949`
  - modes 1/2/3 的 transition 进入 pending，blend target 保持 mode 0。
- `src/modules/hydro_rate_control/SafeRollController.cpp:1045-1071`
  - 20 拍连续健康资格判断；准备阶段的瞬时无效只清零 valid streak。
- `src/modules/hydro_rate_control/SafeRollController.cpp:1073-1095`
  - 激活后严重错误立即 fatal；valid/converged 掉线使用 5 拍去抖。
- `src/modules/hydro_rate_control/SafeRollController.cpp:1120-1122`
  - pending 或瞬时掉线时不运行候选数学。
- `src/modules/hydro_rate_control/SafeRollController.cpp:1332-1341`
  - pending/非致命瞬时掉线保持 `active_code=0`，走现有平滑 legacy fallback。
- `src/modules/hydro_rate_control/SafeRollController.cpp:1385-1388`
  - 每拍输出数字诊断状态。

### 运行数据接入与日志

- `src/modules/hydro_rate_control/HydroRateControl.cpp:307-318`
  - 从 `hydro_allocate_message` 独立读取 valid、converged、finite、gate 和 logical mapping。
- `src/modules/hydro_rate_control/HydroRateControl.cpp:471-477`
  - 写入 `hydro_cadrc_control_message`。
- `src/modules/hydro_rate_control/HydroRateControl.cpp:548-551`
  - 写入 `haar_safe_roll_estimator_status`。
- `src/modules/control_allocator/ControlAllocator.cpp:895-905`
  - 将同一诊断透传到常用的 `haar_safe_roll_status`。
- `msg/HydroCadrcControlMessage.msg:37-40`
- `msg/HaarSafeRollEstimatorStatus.msg:23-26`
- `msg/HaarSafeRollStatus.msg:14-17`
  - 三处均记录：
    - `activation_pending`
    - `allocator_valid_streak`
    - `allocator_invalid_streak`
    - `activation_required_streak`
- `src/modules/logger/messages.h:119-121`
  - uORB 最长未 token 化定义增至 1909 字符后，将 ULog format 缓冲从 1900
    调整为 2000 字节；logger 编译期 static assertion 通过。

未修改：

- `CADRC.cpp` / `CADRC.h`
- PHIS、DMAX、DRATE
- HOCBF、证书、nominal 或 nonlinear allocator 数学
- manual roll 映射
- generic + hydro 叠加、trim、通道映射
- Mode 0 输出选择逻辑

## 4. 自动测试

命令：

```text
CCACHE_DIR=/tmp/px4_ccache ninja -C build/px4_sitl_test_phase1 \
  unit-SafeRollController unit-HydrofoilAllocationModel
build/px4_sitl_test_phase1/unit-SafeRollController --gtest_color=no
build/px4_sitl_test_phase1/unit-HydrofoilAllocationModel --gtest_color=no
```

结果：

- SafeRollController：41/41 PASS
- HydrofoilAllocationModel：9/9 PASS
- 总计：50/50 PASS
- 原有 45 项：45/45 PASS
- 新增：5 项测试组 PASS；循环覆盖 modes 1/2/3
- Mode 0 CADRC 回归继续使用 `EXPECT_FLOAT_EQ`，逐样本 bit-exact PASS
- host benchmark 本次观测：候选平均约 2.706 us，最大约 84.786 us

新增测试位于 `SafeRollControllerTest.cpp:833-970`：

1. Mode 0→1/2/3 首拍 allocator invalid，随后连续 20 拍健康，成功激活；
2. pending 中偶发 `converged=false` 清零 streak、不置 `0x80`、不锁存；
3. 激活后单拍 `valid=false` 当拍回到 legacy 路径、不锁存，下一健康拍恢复；
4. 激活后连续第 5 拍无效准确锁存 `AllocatorInvalidFault`；
5. 激活后 `allocator_finite=false` 和 gate 关闭仍分别立即 fatal；
6. 原有 enable、shadow、runtime、Mode 0 bit-exact、数学 golden 和分配器回归均通过。

## 5. H743 clean `-Werror` 构建

执行：

```text
CCACHE_DIR=/tmp/px4_ccache cmake --build build/matek_h743_dummy --target clean
CCACHE_DIR=/tmp/px4_ccache ninja -C build/matek_h743_dummy \
  /home/linyi666/PX4-Autopilot-water-air/platforms/nuttx/NuttX/nuttx/.config
CCACHE_DIR=/tmp/px4_ccache make matek_h743_dummy -j2
```

结果：PASS，所有本次源码以 `-Werror` 编译并成功链接、打包。

最终链接资源：

```text
FLASH    1,584,376 / 1,966,080 bytes = 80.59%
AXI SRAM    44,716 /   524,288 bytes = 8.53%
```

## 6. 新固件

可刷固件（本阶段未刷写）：

```text
/home/linyi666/PX4-Autopilot-water-air/build/matek_h743_dummy/matek_h743_dummy.px4
size: 1,522,989 bytes
SHA256: bf223f102e4812918ecd9abaea906994757d94cb98f91a2da4aa6798234c0471
```

辅助产物：

```text
matek_h743_dummy.bin
SHA256: 7fe3e66171e86442c5afa7bcd5e6f7de7d9b98795e843e1afe090d227cea23d7

matek_h743_dummy.elf
SHA256: 76253340bf957c9634e51708e7fd50a171ea40003c1505aa093f5ca9fc58f3a6
```

## 7. 实机观察预期

切换到 code 1/2/3 后，正常的首次状态应为：

```text
requested_code = requested mode
active_code = 0
activation_pending = true
allocator_valid_streak = 0..19
allocator_invalid_streak = 0
activation_required_streak = 20
first_fault_code = 0
```

第 20 个连续健康 allocator 样本到达后：

```text
activation_pending = false
allocator_valid_streak = 20
active_code = requested mode
```

激活后偶发单拍 valid/converged 掉线：

```text
active_code = 0
allocator_invalid_streak = 1
first_fault_code = 0
```

连续第 5 拍仍无效时才应出现：

```text
first_fault_code = 0x00000080
fallback latched
```

本报告只描述软件预期；没有宣称完成新固件实机验证。
