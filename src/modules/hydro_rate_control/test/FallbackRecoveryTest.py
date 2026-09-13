#!/usr/bin/env python3
"""Host test of the exact production recovery block + real controller classes.

This is not a full PX4/uORB integration test. Source contracts bind the block
to the STAB/new-controller timing gate and unsuccessful-output latch.
"""
import pathlib
import subprocess
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
MODULE = HERE.parent
source = (MODULE / "HydroRateControl.cpp").read_text()
start = source.index("\t\t\t\tif (_new_controller_recovery_pending &&")
end = source.index("\n\n", source.index("++_roll_controller_reset_counter;", start))
block = source[start:end]
assert block.rstrip().endswith("}")
branch = source.index("fallback_active = true;", source.index("// tau："))
timing = source.index("if (diagnostic_dt_invalid)", branch)
update = source.index("updateNewController(_active_controller, controller_input)", start)
mark = source.index("_new_controller_recovery_pending = !new_roll_output_available;", update)
cadrc = source.index("&& _param_hy_cadrc_en.get()", mark)
assert branch < timing < start < update < mark < cadrc
assert "diagnostic_reference_valid && input_finite && !_new_controller_recovery_pending" in source
assert "controller_input.u_applied_prev = _new_roll_limiter.appliedPrevious();" in source[start:update]
assert "_manual_roll_latch.reset" not in block
assert "_manual_roll_latch.valid()" in block
timing_recovery = source[source.index("if (_new_controller_timing_fault)", timing):
                         source.index("const bool nadrc_active", timing)]
assert "_manual_roll_latch.reset" not in timing_recovery

cpp = r'''
#include "HydroRollController.hpp"
#include "RollReferenceGenerator.hpp"
#include "LADRC.hpp"
#include "MCEO_ADRC.hpp"
#include "AMCEO_ADRC.hpp"
#include "ClassicalNADRC.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#define PX4_ISFINITE std::isfinite
template<typename Controller> struct Fixture {
    Controller controller;
    ManualRollCommandLatch _manual_roll_latch;
    RollReferenceGenerator _roll_reference_generator;
    HydroRollLimiter _new_roll_limiter;
    bool _new_controller_recovery_pending{true};
    bool controller_reset_event{false};
    unsigned _roll_controller_reset_counter{0};
    int _active_controller{2};
    struct { float xyz[3]{0.3f, 0.f, 0.f}; } _hydro_torque_setpoint;
    struct { float get() const { return 1.f; } } _param_hy_rll_u_max;
    HydroRollControllerInput reset_input{};
    void resetNewController(int, const HydroRollControllerInput &input) {
        reset_input = input;
        controller.reset(input);
    }
    void recover(float roll, float p, float dt) {
        struct { uint64_t timestamp_sample{123456}; } vehicle_angular_velocity;
        if (!HydroRollTimingGuard::isValid(dt, .01f)) { return; }
        @BLOCK@
    }
};
template<typename Controller, typename Config>
void check(const Config &config) {
    Fixture<Controller> f;
    assert(f.controller.setParameters(config));
    f._new_roll_limiter.reset(-.5f);
    // An explicitly invalid manual sample waits without observer propagation.
    f._manual_roll_latch.update(true, true, false, 0.f, 1, 1, 1, 1.f, .1f);
    f.recover(.1f, .2f, .0025f);
    assert(f._new_controller_recovery_pending);
    assert(f.controller.resetCounter() == 0 && f.controller.updateCounter() == 0);
    f._manual_roll_latch.update(true, true, true, .2f, 1, 2, 2, 1.f, .1f);
    for (float bad_dt : {0.f, -.001f, .0101f, .04f, NAN, INFINITY}) {
        f.recover(.1f, .2f, bad_dt);
        assert(f._new_controller_recovery_pending);
        assert(f.controller.resetCounter() == 0);
    }
    f.recover(NAN, .2f, .0025f);
    f.recover(.1f, INFINITY, .0025f);
    f._hydro_torque_setpoint.xyz[0] = NAN;
    f.recover(.1f, .2f, .0025f);
    assert(f.controller.resetCounter() == 0);
    f._hydro_torque_setpoint.xyz[0] = .3f;
    f.recover(.1f, .2f, .0025f);
    assert(!f._new_controller_recovery_pending && f.controller_reset_event);
    assert(f._roll_controller_reset_counter == 1);
    assert(f.controller.z1() == .1f && f.controller.z2() == .2f && f.controller.z3() == 0.f);
    assert(f._new_roll_limiter.appliedPrevious() == .3f);
    assert(f.reset_input.u_applied_prev == .3f);
    assert(f.reset_input.timestamp_sample == 123456);
    assert(f._manual_roll_latch.valid() && f._manual_roll_latch.commandRoll() == .2f);
    assert(f._roll_reference_generator.phiRef() == .1f);
    assert(f._roll_reference_generator.phiRefDot() == 0.f);
    auto input = f.reset_input;
    input.u_applied_prev = f._new_roll_limiter.appliedPrevious();
    const auto output = f.controller.update(input);
    assert(output.valid && std::isfinite(output.u_requested));
    auto limited = f._new_roll_limiter.update(output.u_requested, 0.f, .0025f, 1.f, 1.f);
    assert(limited.valid && std::fabs(limited.u_applied - .3f) <= .002501f);
    // No observer/limiter reset during valid operation or publication gaps.
    f._manual_roll_latch.update(true, false, false, 0.f, 0, 0, 2000000, 1.f, .1f);
    f.recover(.2f, .3f, .0025f);
    assert(f.controller.resetCounter() == 1 && f.controller.updateCounter() == 1);
    assert(f._new_roll_limiter.appliedPrevious() == limited.u_applied);
    // A second fallback must use its LAST applied value, not its first one.
    f._new_controller_recovery_pending = true;
    f._hydro_torque_setpoint.xyz[0] = -.2f;
    f.recover(-.1f, -.2f, .0025f);
    assert(f.controller.resetCounter() == 2 && f.controller.updateCounter() == 0);
    assert(f.controller.z1() == -.1f && f.controller.z2() == -.2f);
    assert(f._new_roll_limiter.appliedPrevious() == -.2f);
    // Recovered observer derivative uses the fallback's applied command.
    input = f.reset_input;
    assert(f.controller.update(input).valid);
    const float damping = std::is_same<Controller, MCEOADRC>::value
                       || std::is_same<Controller, AMCEOADRC>::value ? -.5f * -.2f : 0.f;
    const float expected_z2 = -.2f + .0025f * (damping + 10.1746f * -.2f);
    assert(std::fabs(f.controller.z2() - expected_z2) < 1e-6f);
}
int main() {
    // Reproduce the old stale-history jump, then the corrected limiter seed.
    HydroRollLimiter old;
    old.reset(-.5f);
    const auto stale = old.update(0.f, 0.f, .0025f, 1.f, 1.f);
    assert(std::fabs(stale.u_applied - .3f) > .79f);
    old.reset(.3f);
    const auto recovered = old.update(0.f, 0.f, .0025f, 1.f, 1.f);
    assert(std::fabs(recovered.u_applied - .3f) <= .002501f);
    HydroRollControllerConfig common{10.1746f, -.5f, 26.f, 9.8f, 22.f};
    check<LADRC>(common);
    check<MCEOADRC>(common);
    check<AMCEOADRC>(AMCEOControllerConfig{common, 18.f, 46.f, .16f, .7f, .01f, .055f, .0523598776f, .698131701f});
    check<ClassicalNADRC>(ClassicalNADRCConfig{10.1746f, 26.f, 9.8f, 20.f, 20.f, 2.f, .5f, .25f, .01f, .8f, 1.2f, .01f});
    assert(HydroRollTimingGuard::isValid(.0099f, .01f));
    assert(HydroRollTimingGuard::isValid(.0100f, .01f));
    std::puts("FallbackRecoveryTest PASS: IDs 2-5, actual recovery block; no full uORB/target build claim");
}
'''.replace("@BLOCK@", block)
cpp = "#include <initializer_list>\n#include <type_traits>\n" + cpp
with tempfile.TemporaryDirectory(prefix="amceo-fallback-recovery-") as directory:
    path = pathlib.Path(directory)
    (path / "test.cpp").write_text(cpp)
    sources = ["HydroRollController.cpp", "RollReferenceGenerator.cpp", "LADRC.cpp",
               "MCEO_ADRC.cpp", "AMCEO_ADRC.cpp", "ClassicalNADRC.cpp"]
    subprocess.run(["g++", "-std=c++14", "-Wall", "-Wextra", "-Werror", "-I", str(MODULE),
                    str(path / "test.cpp"), *(str(MODULE / s) for s in sources),
                    "-o", str(path / "test")], check=True)
    subprocess.run([str(path / "test")], check=True)
