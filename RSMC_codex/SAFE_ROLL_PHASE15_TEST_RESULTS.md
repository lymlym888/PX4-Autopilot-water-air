# HAAR safe-roll Phase 1.5 测试结果

日期：2026-07-23；Git HEAD `5ae2fd95aefb8c84028a72dbdeec2dbc7cb314b2`。所有测试均为主机/静态构建测试，没有烧写或驱动实物。

## 单元测试

构建：

```sh
CCACHE_DIR=/tmp/px4_ccache cmake --build build/px4_sitl_test_phase1 \
  --target unit-HydrofoilAllocationModel unit-SafeRollController -j4
```

结果：**21/21 PASS**。

### HydrofoilAllocationModel：9/9

1. legacy normal-region regression：PASS；
2. pitch/throttle/正负 roll/饱和覆盖：PASS；
3. forward 与 model-based effective input：PASS；
4. analytic sensitivity 对中心有限差分：PASS；
5. singular/NaN/nonconvergence：PASS；
6. commanded angle/rate interval：PASS；
7. trim migration pure-math regression：PASS；
8. generic+hydro+neutral final composition：PASS；
9. host performance reference：PASS。

本次主机观测 `solve+forward+linearize average=332 ns, maximum=60.384 us`。它不是 H743 WCET。

### SafeRollController：12/12

1. mode 0 direct selector sample exact：PASS；
2. 两套 CADRC 状态和输出逐样本 bit-exact：PASS；
3. modes 1–4 未实现且 legacy fallback：PASS；
4. freshness/timestamp/reset：PASS；
5. gate/allocator fault：PASS；
6. logical mapping fault：PASS；
7. MAP_OK/OWN_OK 输入默认 false：PASS；
8. MAP_OK false 或 OWN_OK false 分别拒绝 modes 1–4：PASS；
9. 两者 true 但 gate false 仍拒绝：PASS；
10. hardware-ready 四项 AND：PASS；
11. bumpless rate step 工具：PASS；
12. host performance reference：PASS。

本次主机观测 mode-0 observer `average=45 ns, maximum=9.351 us`。它不是 H743 WCET。

## Phase 1 回归

- **mode 0 数值等价：VERIFIED。** 测试继续使用 `EXPECT_FLOAT_EQ` 比较 CADRC `_Z1/_Z2` 状态、legacy before/after selector 和 `u_selected`。
- **allocator 数值等价：VERIFIED。** 原有冻结 legacy 方程回归全部保留并通过。
- **原 15 项：15/15 PASS。** 新增 6 项后总数 21，没有删除或放宽旧断言。
- `git diff --check`：PASS。

## Clean matek_h743_dummy build

独立配置与构建：

```sh
cmake -S . -B build/matek_h743_dummy_phase15 -G Ninja \
  -DCONFIG=matek_h743_dummy \
  '-DCMAKE_CXX_FLAGS_MINSIZEREL=-Os -DNDEBUG -Wno-error=address'
CCACHE_DIR=/tmp/px4_ccache cmake --build build/matek_h743_dummy_phase15 -j4
```

结果：**PASS**，生成：

- `matek_h743_dummy.px4` SHA-256 `78c7996331312845e992cb8e32a5d265b1552ed2d9bee5536820b6426c8f4d4a`；
- ELF `74d39829b9a832bcc5e1cc5a929910508ef686cf2dc346094cb89f1d36444ef5`；
- BIN `35c16dbd0d9b5185cf82f0a1ad68425159733841abf883b3a2cb8c3bee3cdca8`。

尺寸：

| Build | text B | data B | bss B | total B |
|---|---:|---:|---:|---:|
| Phase 1 | 1,560,608 | 3,544 | 41,324 | 1,605,476 |
| Phase 1.5 | 1,561,952 | 3,544 | 41,324 | 1,606,820 |
| delta | +1,344 | 0 | 0 | +1,344 |

链接报告：FLASH `1,565,496/1,966,080 B = 79.63%`；AXI SRAM `44,716/524,288 B = 8.53%`。

## 编译诊断

- 新增/修改的 SafeRoll、HydroAllocator、ControlAllocator、uORB 和共享模型对象没有产生本补丁专属 warning。
- GCC 13 将仓库既有 `platforms/common/spi.cpp:90` 恒真地址判断报为 `-Waddress`；默认 `-Werror` 的第一次 clean build因此停止。与 Phase 1 相同，只在独立构建参数中使用 `-Wno-error=address` 后完成；没有修改该文件。
- 本机 SymForce Python 包在 CMake 配置阶段打印 circular-import traceback；CMake 随后正常完成且固件生成。可复现发布环境仍需修复该依赖。

## 参数生成验证

生成的 `parameters.xml` 已验证：

- `HY_SR_MODE default=0`；
- `HY_SR_MAP_OK default=0, boolean=true`；
- `HY_SR_OWN_OK default=0, boolean=true`；
- Phase 1 的 DMAX/DRATE/DREM 默认值未变。

## 未执行

未 flash、未 actuator test、未修改目标参数、未水池测试。目标板 perf、stack、latency 和 dt 分布均 **尚待目标板实测**。
