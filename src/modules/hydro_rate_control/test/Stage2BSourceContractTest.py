#!/usr/bin/env python3

import hashlib
import pathlib
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[4]


def sha256(relative_path: str) -> str:
    return hashlib.sha256((ROOT / relative_path).read_bytes()).hexdigest()


def require_in_order(text: str, snippets: list[str]) -> None:
    position = 0
    for snippet in snippets:
        next_position = text.find(snippet, position)
        assert next_position >= 0, f"missing or reordered source contract: {snippet}"
        position = next_position + len(snippet)


assert sha256("src/modules/hydro_rate_control/CADRC.cpp") == (
    "a48a62f8a87c120eff482487acad5d065ace0899add4978d6c5c02a08ced7201"
)
assert sha256("src/modules/hydro_rate_control/CADRC.h") == (
    "ec8cd46017e0efefaae60e78fa6d2b7bf7f8eff3530926b8ac9d98b7862c8a54"
)

params = (ROOT / "src/modules/hydro_rate_control/hydro_rate_control_params.c").read_text()
assert "PARAM_DEFINE_INT32(HY_RLL_CTRL_SEL, 1);" in params
assert "PARAM_DEFINE_INT32(HY_CADRC_EN, 1);" in params

source = (ROOT / "src/modules/hydro_rate_control/HydroRateControl.cpp").read_text()
require_in_order(
    source,
    [
        "_cadrc_roll.update(roll, roll_sp, p, dt_used);",
        "float u_roll  = math::constrain(_cadrc_roll.getUc()  * u_scale, -1.f, 1.f);",
        "_hydro_cadrc_control_message.u_roll = _cadrc_roll.getUc()  * u_scale;",
        "u_roll  = math::constrain(u_roll  + _param_trim_roll.get(),  -1.f, 1.f);",
        "_hydro_torque_setpoint.xyz[0] = u_roll;",
    ],
)

# Legacy manual behavior remains intentionally publication-edge based and clears
# its stored setpoint on a publication gap. Only the separate new latch may hold.
require_in_order(
    source,
    [
        "const bool got_manual = _manual_control_setpoint_sub.update(&manual_sp);",
        "if (got_manual && manual_sp.valid && manual_sp.timestamp != 0)",
        "_manual_control_setpoint = {};",
        "_manual_control_setpoint.valid = false;",
    ],
)

assert "if (selector_edge && armed && requested_controller != _active_controller)" in source
assert "_roll_controller_switch_blocked = true;" in source
assert "fallback_active = true;" in source
assert "_hydro_roll_controller_status.timestamp = diagnostic_now;" in source
assert "_hydro_roll_controller_status.timestamp_sample = vehicle_angular_velocity.timestamp_sample;" in source
assert "_hydro_roll_controller_status.controller_id = static_cast<uint8_t>(_active_controller);" in source

subprocess.run(
    [
        "git",
        "diff",
        "--exit-code",
        "--",
        "src/modules/hydro_rate_control/CADRC.cpp",
        "src/modules/hydro_rate_control/CADRC.h",
        "src/modules/hydro_allocator",
        "src/modules/control_allocator",
        "boards",
    ],
    cwd=ROOT,
    check=True,
)

print("Stage2BSourceContractTest: PASS")
