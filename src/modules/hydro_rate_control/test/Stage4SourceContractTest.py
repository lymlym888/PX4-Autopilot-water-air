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
        assert position >= 0, f"missing or reordered Stage 4 contract: {snippet}"
        position += len(snippet)


# Release-blocking immutable mathematical sources.
protected_hashes = {
    "src/modules/hydro_rate_control/CADRC.cpp": "a48a62f8a87c120eff482487acad5d065ace0899add4978d6c5c02a08ced7201",
    "src/modules/hydro_rate_control/CADRC.h": "ec8cd46017e0efefaae60e78fa6d2b7bf7f8eff3530926b8ac9d98b7862c8a54",
    "src/modules/hydro_rate_control/LADRC.cpp": "2bd7ff16ba6614b6ebe5b5dc4369b96b07211e43a7f78f0e79e19d3b70711039",
    "src/modules/hydro_rate_control/LADRC.hpp": "465d43da0678ccfb0018b3af96011f1a6217f9ff39808ef4c33e649cb1f1c569",
    "src/modules/hydro_rate_control/MCEO_ADRC.cpp": "763b7faa250249d4c007a8fedb84cb9073c2e88f357b74a2b88bb300f374a128",
    "src/modules/hydro_rate_control/MCEO_ADRC.hpp": "ec3a4e960abf53145c42b672d07a01d03540d977e3336dfe1816c4c5ea77d87d",
    "src/modules/hydro_rate_control/AMCEO_ADRC.cpp": "c0ea3e0b7dc2af981251ed94253d1e3205343b1f005c7a6e30e694cfbb995ea2",
    "src/modules/hydro_rate_control/AMCEO_ADRC.hpp": "050e3c29a6cf104cafc06dae53a095f4edc148a4d02ab61d5bcb153270076235",
}
for path, expected in protected_hashes.items():
    assert sha256(path) == expected, f"protected source drift: {path}"


params = text("src/modules/hydro_rate_control/hydro_rate_control_params.c")
for definition in (
    "PARAM_DEFINE_FLOAT(HY_RLL_DT_MAX, 0.010f);",
    "PARAM_DEFINE_FLOAT(HY_AM_WMIN, 18.0f);",
    "PARAM_DEFINE_FLOAT(HY_AM_WMAX, 46.0f);",
    "PARAM_DEFINE_FLOAT(HY_AM_RHO0, 0.16f);",
    "PARAM_DEFINE_FLOAT(HY_AM_CE, 0.70f);",
    "PARAM_DEFINE_FLOAT(HY_AM_CU, 0.01f);",
    "PARAM_DEFINE_FLOAT(HY_AM_TRHO, 0.055f);",
    "PARAM_DEFINE_FLOAT(HY_AM_E1BAR, 0.0523598776f);",
    "PARAM_DEFINE_FLOAT(HY_AM_E2BAR, 0.698131701f);",
):
    assert definition in params


amceo = text("src/modules/hydro_rate_control/AMCEO_ADRC.cpp")
require_in_order(
    amceo,
    [
        "const float e1_sched = _z1 - input.phi_ref;",
        "const float e2_sched = _z2 - input.phi_ref_dot;",
        "_sigma_e = saturate01(std::fabs(e1_sched) / _config.e1_bar + std::fabs(e2_sched) / _config.e2_bar);",
        "_sigma_u = saturate01(std::fabs(input.u_applied_prev) / input.u_max);",
        "_rho_star = saturate01(_config.rho0 + _config.ce * _sigma_e - _config.cu * _sigma_u);",
        "_rho = saturate01(_rho + input.dt_used / _config.tau_rho * (-_rho + _rho_star));",
        "_observer_bandwidth = _config.omega_min + (_config.omega_max - _config.omega_min) * _rho;",
        "const float l1 = 4.f * _observer_bandwidth;",
        "const float l2 = 6.f * omega_o2;",
        "const float l3 = 4.f * omega_o2 * _observer_bandwidth;",
        "const float l4 = omega_o2 * omega_o2;",
        "const float innovation = input.phi - _z1;",
        "const float z1_dot = _z2 + l1 * innovation;",
        "_config.common.a_phi * _z2 + _config.common.b0 * input.u_applied_prev",
        "const float z3_dot = _z4 + l3 * innovation;",
        "const float z4_dot = l4 * innovation;",
        "_z1 += input.dt_used * z1_dot;",
        "_z2 += input.dt_used * z2_dot;",
        "_z3 += input.dt_used * z3_dot;",
        "_z4 += input.dt_used * z4_dot;",
        "_e1 = _z1 - input.phi_ref;",
        "_e2 = _z2 - input.phi_ref_dot;",
        "input.phi_ref_ddot - _config.common.k_phi1 * _e1 - _config.common.k_phi2 * _e2",
        "const float u_requested = (v_phi - _config.common.a_phi * _z2 - _z3) / _config.common.b0;",
    ],
)
assert "sigma_o" not in amceo
assert "eo_bar" not in amceo
assert re.search(r"\bc_o\b", amceo) is None
assert "HY_CADRC" not in amceo and "constrain" not in amceo
assert "- _z4" not in amceo
assert "input.u_applied_prev" in amceo


integration = text("src/modules/hydro_rate_control/HydroRateControl.cpp")
require_in_order(
    integration,
    [
        "const bool timing_guard_required = isImplementedNewController(_active_controller);",
        "diagnostic_dt_invalid = timing_guard_required",
        "&& !HydroRollTimingGuard::isValid(dt,",
        "if (diagnostic_dt_invalid)",
        "_new_controller_timing_fault = true;",
        "} else {",
        "if (_new_controller_timing_fault)",
        "_new_controller_timing_fault = false;",
        "if (_new_controller_recovery_pending && _manual_roll_latch.valid()",
        "recovery_input.u_applied_prev = _hydro_torque_setpoint.xyz[0];",
        "_roll_reference_generator.reset(recovery_input.command_roll, roll);",
        "_new_roll_limiter.reset(recovery_input.u_applied_prev);",
        "resetNewController(_active_controller, recovery_input);",
        "const HydroRollControllerOutput output = updateNewController(_active_controller, controller_input);",
        "_new_roll_limiter.update(output.u_requested,",
        "_new_controller_recovery_pending = !new_roll_output_available;",
        "_hydro_torque_setpoint.xyz[0] = new_roll_applied;",
    ],
)
implemented_function = integration[
    integration.index("bool HydroRateControl::isImplementedNewController"):
    integration.index("HydroRateControl::NewControllerParameters")
]
assert "HydroRollControllerId::AmceoAdrc" in implemented_function
assert "HydroRollControllerId::Nadrc" in implemented_function
assert "controller_input.u_applied_prev = _new_roll_limiter.appliedPrevious();" in integration
assert "controller_input.u_max = _param_hy_rll_u_max.get();" in integration
assert "_amceo_adrc_roll.setParameters(amceo_config)" in integration
assert "sizeof(NewControllerParameters) == 24 * sizeof(float)" in integration
assert "armed new roll gain update deferred until disarm" in integration
assert "_hydro_roll_controller_status.dt_invalid = diagnostic_dt_invalid;" in integration
assert "_hydro_roll_controller_status.timing_fault = _new_controller_timing_fault || diagnostic_dt_invalid;" in integration


message = text("msg/HydroRollControllerStatus.msg")
assert "bool dt_invalid" in message
assert "bool timing_fault" in message


subprocess.run(
    [
        "git",
        "diff",
        "--exit-code",
        "--",
        "src/modules/hydro_rate_control/CADRC.cpp",
        "src/modules/hydro_rate_control/CADRC.h",
        "src/modules/hydro_rate_control/LADRC.hpp",
        "src/modules/hydro_rate_control/MCEO_ADRC.hpp",
        "src/modules/hydro_rate_control/AMCEO_ADRC.hpp",
        "src/modules/hydro_allocator",
        "src/modules/control_allocator",
        "boards",
    ],
    cwd=ROOT,
    check=True,
)

print("Stage4SourceContractTest: PASS")
