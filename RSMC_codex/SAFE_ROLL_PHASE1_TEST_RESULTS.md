# HAAR safe-roll Phase 1 测试结果

## 自动测试

测试配置：`px4_sitl_test`，GoogleTest 1.13 固定版本，GCC 13.3。命令：

```sh
env CCACHE_DISABLE=1 ninja -C build/px4_sitl_test_phase1 unit-HydrofoilAllocationModel unit-SafeRollController
./build/px4_sitl_test_phase1/unit-HydrofoilAllocationModel --gtest_color=no
./build/px4_sitl_test_phase1/unit-SafeRollController --gtest_color=no
```

结果：**VERIFIED，15/15 PASS**。

| Suite | 覆盖 | 结果 |
|---|---|---|
| SafeRollController | mode 0 direct path 6 个输出样本 bit-exact | PASS |
| SafeRollController | 6 个 `phi/p/roll_sp/dt` 连续样本的 CADRC Z1/Z2、legacy 输出、selector 输出逐样本 `EXPECT_FLOAT_EQ` | PASS |
| SafeRollController | modes 1–4 `MODE_NOT_IMPLEMENTED`、active mode 0、fallback legacy | PASS |
| SafeRollController | timestamp 非单调、gyro/attitude stale、quaternion/rate nonfinite、reset | PASS |
| SafeRollController | foil gate 和 allocator invalid 拒绝 | PASS |
| SafeRollController | 非 `Servo5=逻辑左/Servo4=逻辑右` 的映射拒绝 | PASS |
| SafeRollController | bumpless rate step | PASS |
| SafeRollController | 主机性能参考 | PASS；avg 40–45 ns，重复运行观测 max 15.592 us |
| AllocationModel | 5 个正常域旧/新回归样本 | PASS；delta 容差 1e-6，thrust 1e-5 |
| AllocationModel | 3 pitch × 3 thrust × 5 roll 的 45 个含饱和样本 | PASS；delta 1e-5，thrust 1e-4 |
| AllocationModel | solve/forward 与 model-based effective input | PASS |
| AllocationModel | 解析 `g_L/g_R` 对中心有限差分 | PASS；容差 2e-3 |
| AllocationModel | singular/NaN/nonconvergence | PASS；明确 invalid |
| AllocationModel | commanded angle/rate/remainder interval | PASS |
| AllocationModel | 主机 solve+forward+linearize 性能参考 | PASS；avg 302–328 ns，重复运行观测 max 79.057 us |

性能数字来自非实时 x86 主机，只用于回归和数量级参考，不是 H743 WCET。目标运行时数据应从 `safe_roll: cycle` perf counter 和 `haar_safe_roll_status.cycle_time_us` 获取。

## 板级构建

干净目录：`build/matek_h743_dummy_phase1`。配置与构建：

```sh
cmake -G Ninja -DCONFIG=matek_h743_dummy \
  -DCMAKE_CXX_FLAGS_MINSIZEREL='-Os -DNDEBUG -Wno-error=address' \
  -S . -B build/matek_h743_dummy_phase1
env CCACHE_DISABLE=1 cmake --build build/matek_h743_dummy_phase1 -j 4
```

结果：**VERIFIED PASS**，生成 ELF/BIN/PX4。新增 `HydrofoilAllocationModel`、`SafeRollController`、hydro allocator、hydro rate controller、ControlAllocator、uORB 和 logger 全部由 ARM GCC 13.2 编译并链接。

警告检查：本阶段新增/修改对象无编译警告。全量构建存在一条仓库既有 `platforms/common/spi.cpp:90 [-Waddress]`，在 GCC 13 默认 `-Werror` 下会阻断；仅在独立构建命令降级该类别后 clean build 通过。另有既有 SymForce Python circular-import traceback 和 Python SyntaxWarning，不影响生成/链接，均非本阶段引入。

## 尺寸与栈

`arm-none-eabi-size`：

```text
text=1560608  data=3544  bss=41324  dec=1605476
FLASH 1564152/1966080 (79.56%)
AXI_SRAM 44716/524288 (8.53%)
```

现存旧 ELF 的 commit 是 `914ec24d...`，不是要求的 HEAD，因此不能作为严格同基线尺寸回归。其表面数据为 `text=1571016,data=3548,bss=41324`；表面差值 `-10408/-4/0 B` 只记录，不宣称是 Phase 1 节省。

ARM `-fstack-usage` 静态 frame：SafeRoll evaluate 48 B；model solve 152 B、solveOneSide 136 B、forward 152 B、linearize 112 B；HydroRateControl::Run 自身 8 B，HydroAllocator::Run 自身 8 B。最深新增 solve 路径约 288 B，不包含库内部 frame。旧匹配基线无 stack-usage 文件，精确增量为 **UNKNOWN**。

## 未执行

- **VERIFIED 未执行**：flash、actuator test、实物台架动作、水池测试、armed 模式输出。
- **UNKNOWN**：H743 实机平均/最大 cycle time、实机任务 stack high-water mark、实机当前参数和左右物理线束。
