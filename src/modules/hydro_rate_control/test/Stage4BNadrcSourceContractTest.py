#!/usr/bin/env python3

import hashlib
import pathlib
import re
import subprocess


ROOT = pathlib.Path(__file__).resolve().parents[4]


def text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text()


def sha256(relative_path: str) -> str:
    return hashlib.sha256((ROOT / relative_path).read_bytes()).hexdigest()


def require_in_order(source: str, snippets: list[str]) -> None:
    position = 0
    for snippet in snippets:
        position = source.find(snippet, position)
        assert position >= 0, f"missing or reordered Stage 4B contract: {snippet}"
        position += len(snippet)


# IDs 0-4 are immutable mathematical baselines during Stage 4B.
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
    "PARAM_DEFINE_FLOAT(HY_NA_WO, 20.0f);",
    "PARAM_DEFINE_FLOAT(HY_NA_TDR, 20.0f);",
    "PARAM_DEFINE_FLOAT(HY_NA_TDN0, 2.0f);",
    "PARAM_DEFINE_FLOAT(HY_NA_OA1, 0.5f);",
    "PARAM_DEFINE_FLOAT(HY_NA_OA2, 0.25f);",
    "PARAM_DEFINE_FLOAT(HY_NA_ODEL, 0.01f);",
    "PARAM_DEFINE_FLOAT(HY_NA_CA1, 0.8f);",
    "PARAM_DEFINE_FLOAT(HY_NA_CA2, 1.2f);",
    "PARAM_DEFINE_FLOAT(HY_NA_CDEL, 0.01f);",
):
    assert definition in params


nadrc = text("src/modules/hydro_rate_control/ClassicalNADRC.cpp")

# Canonical fal, including the equality branch and sign(0)=0 behavior.
require_in_order(
    nadrc,
    [
        "const float absolute_error = std::fabs(error);",
        "if (absolute_error <= delta)",
        "const float denominator = std::pow(delta, 1.f - alpha);",
        "result = error / denominator;",
        "result = std::pow(absolute_error, alpha) * sign(error);",
    ],
)

# Canonical Han fhan. The +a2 form is intentional and release-blocking.
require_in_order(
    nadrc,
    [
        "const float d = r_td * h0 * h0;",
        "if (!std::isfinite(d) || d <= FLT_MIN)",
        "const float a0 = h0 * x2;",
        "const float y = x1 + a0;",
        "const float a1 = std::sqrt(radicand);",
        "const float a2 = a0 + sign(y) * (a1 - d) * 0.5f;",
        "const float sy = (sign(y + d) - sign(y - d)) * 0.5f;",
        "const float a = (a0 + y - a2) * sy + a2;",
        "const float sa = (sign(a + d) - sign(a - d)) * 0.5f;",
        "const float fh = -r_td * (a / d - sign(a)) * sa - r_td * sign(a);",
    ],
)

# TD -> nonlinear ESO with u_applied[k-1] -> NLSEF -> disturbance compensation.
require_in_order(
    nadrc,
    [
        "const float h0 = _config.td_n0 * input.dt_used;",
        "const float v1_dot = _v2;",
        "_td_acceleration = fhan(_v1 - input.command_roll, _v2, _config.td_r, h0);",
        "const float beta01 = 3.f * _config.observer_bandwidth;",
        "const float beta02 = 3.f * omega_o2;",
        "const float beta03 = omega_o2 * _config.observer_bandwidth;",
        "const float observer_error = _z1 - input.phi; // estimated minus measured",
        "const float z1_dot = _z2 - beta01 * observer_error;",
        "const float z2_dot = _z3 - beta02 * fal_o1 + _config.b0 * input.u_applied_prev;",
        "const float z3_dot = -beta03 * fal_o2;",
        "_v1 += input.dt_used * v1_dot;",
        "_v2 += input.dt_used * _td_acceleration;",
        "_z1 += input.dt_used * z1_dot;",
        "_z2 += input.dt_used * z2_dot;",
        "_z3 += input.dt_used * z3_dot;",
        "_e1 = _v1 - _z1;",
        "_e2 = _v2 - _z2;",
        "_u0 = _config.k_phi1 * fal_c1 + _config.k_phi2 * fal_c2;",
        "const float u_requested = (_u0 - _z3) / _config.b0;",
    ],
)
assert "a_phi" not in nadrc
assert re.search(r"\brho\b|\bsigma_[a-zA-Z0-9_]*\b", nadrc) is None
assert "HY_CADRC" not in nadrc and "constrain" not in nadrc
assert "u_max" not in nadrc


integration = text("src/modules/hydro_rate_control/HydroRateControl.cpp")
enum_header = text("src/modules/hydro_rate_control/HydroRollController.hpp")
message = text("msg/HydroRollControllerStatus.msg")

assert "Nadrc = 5" in enum_header
assert "CONTROLLER_ID_NADRC = 5" in message
assert "selector_value <= static_cast<int32_t>(HydroRollControllerId::Nadrc)" in integration
assert "controller == HydroRollControllerId::Nadrc" in integration
assert "case HydroRollControllerId::Nadrc:" in integration
assert "return _nadrc_roll.update(input);" in integration
assert "_nadrc_roll.setParameters(nadrc_config)" in integration
assert "sizeof(NewControllerParameters) == 24 * sizeof(float)" in integration

# ID 5 bypasses the common reference model as a control input, exposes its
# private TD via generic diagnostics, and still uses the shared limiter.
assert "if (_manual_roll_latch.valid() && !nadrc_active)" in integration
assert "controller_input.phi_ref = nadrc_active ? controller_input.command_roll : diagnostic_phi_ref;" in integration
assert "diagnostic_phi_ref = _nadrc_roll.v1();" in integration
assert "diagnostic_phi_ref_dot = _nadrc_roll.v2();" in integration
assert "diagnostic_phi_ref_ddot = _nadrc_roll.tdAcceleration();" in integration
require_in_order(
    integration,
    [
        "controller_input.u_applied_prev = _new_roll_limiter.appliedPrevious();",
        "const HydroRollControllerOutput output = updateNewController(_active_controller, controller_input);",
        "_new_roll_limiter.update(output.u_requested,",
        "new_roll_applied = limited.u_applied;",
        "_hydro_torque_setpoint.xyz[0] = new_roll_applied;",
    ],
)
assert "HydroRollTimingGuard::isValid(dt," in integration
assert "armed new roll gain update deferred until disarm" in integration


diff_paths = subprocess.run(
    ["git", "diff", "--name-only"], cwd=ROOT, check=True, capture_output=True, text=True
).stdout.splitlines()
for path in diff_paths:
    assert not path.startswith("src/modules/hydro_allocator/"), path
    assert not path.startswith("src/modules/control_allocator/"), path
    assert not path.startswith("boards/"), path
    assert "pwm" not in path.lower(), path

subprocess.run(
    [
        "git",
        "diff",
        "--exit-code",
        "--",
        *protected_hashes.keys(),
        "src/modules/hydro_allocator",
        "src/modules/control_allocator",
        "boards",
    ],
    cwd=ROOT,
    check=True,
)

print("Stage4BNadrcSourceContractTest: PASS")
