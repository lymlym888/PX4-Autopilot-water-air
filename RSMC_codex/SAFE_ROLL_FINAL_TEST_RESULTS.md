# HAAR SafeRoll 最终测试结果

## 主机单元测试

执行：

```text
cmake --build build/px4_sitl_test_phase1 \
  --target unit-SafeRollController unit-HydrofoilAllocationModel -j4
build/px4_sitl_test_phase1/unit-SafeRollController
build/px4_sitl_test_phase1/unit-HydrofoilAllocationModel
```

结果：

```text
SafeRollController        25/25 PASS
HydrofoilAllocationModel   9/9 PASS
Total                     34/34 PASS
```

用户列出的 30 类要求由若干组合测试覆盖。例如 enable policy 在一个测试中遍历 code 1–4，fixed/adaptive CBF 在一个测试中覆盖两种策略，full synthetic regression 在单个测试中逐样本检查多字段。

关键结论：

- mode 0 legacy CADRC 状态和输出逐样本 bit-exact：PASS。
- ENABLE=0/1/2 和 code 4 gating：PASS。
- shadow 输出 legacy：PASS。
- 在线切换、进入 blend、切回 0：PASS。
- 故障 fallback 连续 10 拍均满足 DUMAX：PASS。
- reference filter、b/d estimator、single/multi-lag、conflict discard、variable dt：PASS。
- nominal、fixed/adaptive CBF、relaxation、projection：PASS。
- generic+hydro allocation interval、exact nonlinear check、code 4：PASS。
- DREM 0.05 deg 转换：PASS。
- CRC、reset-sensitive 参数、nonfinite/empty interval、固定存储：PASS。
- 400 样本完整合成序列：PASS。
- allocator legacy 重构和异常求解：PASS。

## Host benchmark

独立、无并行构建负载的一次结果：

```text
safe_roll_candidate:
  average 2.579 us
  maximum 47.242 us

allocation solve+forward+linearize:
  average 0.302 us
  maximum 9.756 us
```

这是 x86 host 参考，不是 H743 WCET。固件日志已提供 reference filter、estimator、certificate、CBF、allocation query、nonlinear check 和 total candidate 分项计时，见 `HaarSafeRollEstimatorStatus.msg:55-61`。

## 参数和 logger

- `parameters.xml`：xmllint PASS。
- `HAAR Safe Roll` 参数数：41。
- 缺 min：0；缺 max：0；reboot_required：0。
- 参数名最长 12 字符，满足 PX4 限制。
- QGC enum：MODE、ENABLE、SHADOW 均已生成。
- logger message-format 静态断言：PASS。
- uORB payload：main 184 B，estimator 194 B。
- 若两条均以 400 Hz 记录，原始 payload 约 `151,200 B/s`；加 ULog framing 后约 154 kB/s，尚未包含其他 topics，必须在目标 SD 卡验证。

## H743 构建

默认 `-Werror` clean build：

```text
cmake --build build/matek_h743_dummy --target clean
CCACHE_DISABLE=1 cmake --build build/matek_h743_dummy -- -j4
```

首次调用遇到 NuttX jlink/.config 并行生成竞态；原命令重跑后完整 PASS。没有 SafeRoll 编译警告。最终：

```text
FLASH used 1,581,672 / 1,966,080 B = 80.45%
AXI SRAM used 44,716 / 524,288 B = 8.53%

ELF text 1,578,128
ELF data     3,544
ELF bss     41,324
```

相对 Phase 1.5：

```text
text +16,176 B
data      +0 B
bss       +0 B
```

固件：

```text
build/matek_h743_dummy/matek_h743_dummy.px4
SHA256 a2a3c900442ff08ef417d63a528a36a7ff833244af760c1883ae3d6432104345
```

## ARM stack-usage 静态结果

使用 `-fstack-usage` 的兼容构建：

```text
SafeRollController::validatedCandidate  656 B
SafeRollController::evaluate            544 B
HydroAllocator::Run                     904 B
HydroRateControl::Run                  1288 B
ControlAllocator::Run                   640 B
```

ControlAllocator 文件中最大的非 Run 函数为既有 `update_effectiveness_matrix_if_needed` 1592 B。静态值不等于任务栈 high-water。

## 未执行

- 目标 H743 在线 perf/stack high-water；
- logger on/off deadline 对比；
- flash；
- actuator test；
- 水池试验。

