#!/usr/bin/env python3

import hashlib
import pathlib
import re
import subprocess
from Z3LimitContract import baseline_bytes, LIMITED_SOURCES


ROOT = pathlib.Path(__file__).resolve().parents[4]


def text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text()


def sha256(relative_path: str) -> str:
    # Freeze all old bytes except the exact user-authorized z3-limit insertion.
    return hashlib.sha256(baseline_bytes(relative_path, (ROOT / relative_path).read_bytes())).hexdigest()


def require_in_order(source: str, snippets: list[str]) -> None:
    position = 0
    for snippet in snippets:
        position = source.find(snippet, position)
        assert position >= 0, f"missing or reordered contract: {snippet}"
        position += len(snippet)


# Immutable CADRC, LADRC and fixed-bandwidth MCEO baselines. Stage 4 is
# intentionally allowed to replace the AMCEO placeholder.
assert sha256("src/modules/hydro_rate_control/CADRC.cpp") == (
    "a48a62f8a87c120eff482487acad5d065ace0899add4978d6c5c02a08ced7201"
)
assert sha256("src/modules/hydro_rate_control/CADRC.h") == (
    "ec8cd46017e0efefaae60e78fa6d2b7bf7f8eff3530926b8ac9d98b7862c8a54"
)
assert sha256("src/modules/hydro_rate_control/LADRC.cpp") == (
    "2bd7ff16ba6614b6ebe5b5dc4369b96b07211e43a7f78f0e79e19d3b70711039"
)
assert sha256("src/modules/hydro_rate_control/LADRC.hpp") == (
    "465d43da0678ccfb0018b3af96011f1a6217f9ff39808ef4c33e649cb1f1c569"
)
assert sha256("src/modules/hydro_rate_control/MCEO_ADRC.cpp") == (
    "763b7faa250249d4c007a8fedb84cb9073c2e88f357b74a2b88bb300f374a128"
)
assert sha256("src/modules/hydro_rate_control/MCEO_ADRC.hpp") == (
    "ec3a4e960abf53145c42b672d07a01d03540d977e3336dfe1816c4c5ea77d87d"
)

params = text("src/modules/hydro_rate_control/hydro_rate_control_params.c")
for definition in (
    "PARAM_DEFINE_FLOAT(HY_RLL_B0, 10.1746f);",
    "PARAM_DEFINE_FLOAT(HY_RLL_APHI, -0.5f);",
    "PARAM_DEFINE_FLOAT(HY_RLL_K1, 26.0f);",
    "PARAM_DEFINE_FLOAT(HY_RLL_K2, 9.8f);",
    "PARAM_DEFINE_FLOAT(HY_LADRC_WO, 25.0f);",
    "PARAM_DEFINE_FLOAT(HY_MCEO_WO, 22.0f);",
):
    assert definition in params

ladrc = text("src/modules/hydro_rate_control/LADRC.cpp")
require_in_order(
    ladrc,
    [
        "const float innovation = input.phi - _z1;",
        "const float z1_dot = _z2 + l1 * innovation;",
        "const float z2_dot = _config.b0 * input.u_applied_prev + _z3 + l2 * innovation;",
        "const float z3_dot = l3 * innovation;",
        "_z1 += input.dt_used * z1_dot;",
        "_z2 += input.dt_used * z2_dot;",
        "_z3 += input.dt_used * z3_dot;",
        "_e1 = _z1 - input.phi_ref;",
        "_e2 = _z2 - input.phi_ref_dot;",
        "input.phi_ref_ddot - _config.k_phi1 * _e1 - _config.k_phi2 * _e2",
        "const float u_requested = (v_phi - _z3) / _config.b0;",
    ],
)
assert len(re.findall(r"input\.p(?![A-Za-z0-9_])", ladrc)) == 3  # validity/reset only
assert "constrain" not in ladrc and "HY_CADRC" not in ladrc

mceo = text("src/modules/hydro_rate_control/MCEO_ADRC.cpp")
require_in_order(
    mceo,
    [
        "const float innovation = input.phi - _z1;",
        "const float z1_dot = _z2 + l1 * innovation;",
        "_config.a_phi * _z2 + _config.b0 * input.u_applied_prev + _z3 + l2 * innovation",
        "const float z3_dot = _z4 + l3 * innovation;",
        "const float z4_dot = l4 * innovation;",
        "_z1 += input.dt_used * z1_dot;",
        "_z2 += input.dt_used * z2_dot;",
        "_z3 += input.dt_used * z3_dot;",
        "_z4 += input.dt_used * z4_dot;",
        "_e1 = _z1 - input.phi_ref;",
        "_e2 = _z2 - input.phi_ref_dot;",
        "input.phi_ref_ddot - _config.k_phi1 * _e1 - _config.k_phi2 * _e2",
        "const float u_requested = (v_phi - _config.a_phi * _z2 - _z3) / _config.b0;",
    ],
)
assert len(re.findall(r"input\.p(?![A-Za-z0-9_])", mceo)) == 3
assert "constrain" not in mceo and "HY_CADRC" not in mceo
assert "rho" not in mceo and "sigma" not in mceo

integration = text("src/modules/hydro_rate_control/HydroRateControl.cpp")
assert "controller == HydroRollControllerId::Ladrc" in integration
assert "controller == HydroRollControllerId::MceoAdrc" in integration
assert "controller_input.u_applied_prev = _new_roll_limiter.appliedPrevious();" in integration
require_in_order(
    integration,
    [
        "const HydroRollControllerOutput output = updateNewController(_active_controller, controller_input);",
        "_new_roll_limiter.update(output.u_requested,",
        "new_roll_applied = limited.u_applied;",
        "_hydro_torque_setpoint.xyz[0] = new_roll_applied;",
    ],
)
assert "armed new roll gain update deferred until disarm" in integration
assert "applyNewControllerParameters(_pending_new_controller_parameters)" in integration
assert "_hydro_roll_controller_status.observer_bandwidth = diagnostic_observer_bandwidth;" in integration
assert "diagnostic_state_3 = _ladrc_roll.z3();" in integration
assert "diagnostic_state_4 = _mceo_adrc_roll.z4();" in integration

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

print("Stage3SourceContractTest: PASS")
