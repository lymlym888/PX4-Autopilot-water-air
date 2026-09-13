"""Allow ONLY the user-authorized 2026-09-12 z3 bound atop frozen source hashes."""
import pathlib

LIMITED_SOURCES = {
    "src/modules/hydro_rate_control/LADRC.cpp",
    "src/modules/hydro_rate_control/MCEO_ADRC.cpp",
    "src/modules/hydro_rate_control/AMCEO_ADRC.cpp",
    "src/modules/hydro_rate_control/ClassicalNADRC.cpp",
}
LIMIT_BLOCK = "\n\t// User-requested bench bound on disturbance state [rad/s^2], not output.\n\t// Reject overflow before clipping so nonfinite state cannot appear valid.\n\tif (!std::isfinite(_z3)) {\n\t\t_valid = false;\n\t\treturn {NAN, false, reset_event};\n\t}\n\n\tif (_z3 > 0.25f) {\n\t\t_z3 = 0.25f;\n\n\t} else if (_z3 < -0.25f) {\n\t\t_z3 = -0.25f;\n\t}\n"

def baseline_bytes(path, data):
    if path not in LIMITED_SOURCES:
        return data
    block = LIMIT_BLOCK.encode()
    # Exact user-authorized +/-0.25 state bound, identical for IDs 2-5.
    assert data.count(block) == 1, f"missing/modified authorized z3 bound: {path}"
    position = data.index(block)
    prior = b"_z4 += input.dt_used * z4_dot;\n" if pathlib.Path(path).name in (
        "MCEO_ADRC.cpp", "AMCEO_ADRC.cpp") else b"_z3 += input.dt_used * z3_dot;\n"
    assert data[:position].endswith(prior), f"z3 bound moved before Euler completion: {path}"
    assert data[position + len(block):].startswith(b"\n\t_e1 ="), path
    return data.replace(block, b"", 1)
