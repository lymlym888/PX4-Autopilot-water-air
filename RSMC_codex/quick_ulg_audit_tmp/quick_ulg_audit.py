#!/usr/bin/env python3
"""Read-only, plot-free admission audit for the 2026-07-27 HAAR ULogs."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
from pathlib import Path

import numpy as np
from pyulog import ULog


WANTED_TOPICS = [
    "vehicle_attitude",
    "vehicle_attitude_setpoint",
    "vehicle_angular_velocity",
    "vehicle_acceleration",
    "manual_control_setpoint",
    "actuator_servos",
    "vehicle_status",
    "failsafe_flags",
    "input_rc",
    "haar_safe_roll_status",
    "haar_safe_roll_field_status",
    "haar_safe_roll_fault_snapshot",
    "haar_depth_control_status",
    "haar_depth_allocator_status",
    "hydro_allocate_message",
]


def install_missing_format_workaround():
    """Skip only logged topics whose FORMAT record is absent from the ULog."""
    original = ULog._MessageAddLogged.__init__
    missing = set()

    def patched(self, data, header, formats):
        try:
            original(self, data, header, formats)
        except KeyError:
            self.multi_id, = struct.unpack("<B", data[0:1])
            self.msg_id, = struct.unpack("<H", data[1:3])
            self.message_name = ULog.parse_string(data[3:])
            missing.add(self.message_name)
            self.field_data = []
            self.timestamp_idx = -1
            self.max_data_size = 0
            self.timestamp_offset = 0
            self.buffer = bytearray()
            self.dtype = np.dtype([])

    ULog._MessageAddLogged.__init__ = patched
    return missing


MISSING_FORMAT_TOPICS = install_missing_format_workaround()


def arr(dataset, name, default=None):
    if dataset is None or name not in dataset.data:
        return np.asarray([] if default is None else default)
    return np.asarray(dataset.data[name])


def finite(a):
    a = np.asarray(a)
    return a[np.isfinite(a)] if a.size else a


def q(a, percentile, default=math.nan):
    a = finite(a)
    return float(np.percentile(a, percentile)) if a.size else default


def dominant(a):
    a = np.asarray(a)
    if not a.size:
        return None, 0.0
    values, counts = np.unique(a.astype(np.int64), return_counts=True)
    index = int(np.argmax(counts))
    return int(values[index]), float(counts[index] / counts.sum())


def bool_fraction(a):
    a = np.asarray(a)
    return float(np.mean(a.astype(bool))) if a.size else math.nan


def max_true_run_seconds(values, timestamps):
    values = np.asarray(values, dtype=bool)
    timestamps = np.asarray(timestamps, dtype=np.int64)
    if values.size == 0 or timestamps.size != values.size:
        return 0.0
    best = 0
    start = None
    for i, value in enumerate(values):
        if value and start is None:
            start = i
        if (not value or i == values.size - 1) and start is not None:
            end = i if value and i == values.size - 1 else i - 1
            best = max(best, int(timestamps[end] - timestamps[start]))
            start = None
    return best / 1e6


def dwell_seconds(values, timestamps, predicate):
    values = np.asarray(values)
    timestamps = np.asarray(timestamps)
    if values.size < 2 or timestamps.size != values.size:
        return 0.0
    dt = np.diff(timestamps, append=timestamps[-1])
    positive_dt = dt[(dt > 0) & (dt < 1_000_000)]
    replacement = np.median(positive_dt) if positive_dt.size else 0
    dt[-1] = replacement
    return float(np.sum(dt[predicate(values)]) / 1e6)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def get_dataset(ulog, name):
    candidates = [dataset for dataset in ulog.data_list if dataset.name == name]
    if not candidates:
        return None
    candidates.sort(key=lambda dataset: (dataset.multi_id, -len(dataset.data.get("timestamp", []))))
    return candidates[0]


def audit_file(path):
    result = {
        "file": path.name,
        "path": str(path),
        "size": path.stat().st_size,
        "sha256": sha256(path),
        "parseable": False,
        "parse_error": "",
    }
    missing_before = set(MISSING_FORMAT_TOPICS)
    try:
        ulog = ULog(
            str(path),
            message_name_filter_list=WANTED_TOPICS,
            disable_str_exceptions=True,
        )
    except Exception as exc:  # one bad file must not stop the batch
        result["parse_error"] = f"{type(exc).__name__}: {exc}"
        return result

    result["parseable"] = True
    result["missing_format_topics"] = sorted(set(MISSING_FORMAT_TOPICS) - missing_before)
    result["duration_s"] = float((ulog.last_timestamp - ulog.start_timestamp) / 1e6)
    result["file_corruption"] = bool(ulog.file_corruption)
    result["dropout_count"] = len(ulog.dropouts)
    result["dropout_total_ms"] = int(sum(item.duration for item in ulog.dropouts))
    result["dropout_max_ms"] = int(max((item.duration for item in ulog.dropouts), default=0))
    result["params"] = {
        key: ulog.initial_parameters.get(key)
        for key in (
            "HY_SR_MODE",
            "HY_SR_ENABLE",
            "HY_SR_FIELD",
            "HY_ALLOCATE_MODE",
            "HY_FOIL_AUX",
            "HY_DC_MODE",
            "HY_ROLL_MAX",
        )
    }

    datasets = {name: get_dataset(ulog, name) for name in WANTED_TOPICS}
    result["topics"] = {
        name: int(len(arr(dataset, "timestamp")))
        for name, dataset in datasets.items()
        if dataset is not None
    }
    result["missing_key_topics"] = [
        name
        for name in (
            "vehicle_attitude",
            "vehicle_angular_velocity",
            "manual_control_setpoint",
            "actuator_servos",
            "haar_safe_roll_status",
        )
        if datasets[name] is None
    ]

    # Time-axis integrity in every selected topic.
    nonmonotonic = []
    for name, dataset in datasets.items():
        timestamps = arr(dataset, "timestamp")
        if timestamps.size > 1 and np.any(np.diff(timestamps.astype(np.int64)) < 0):
            nonmonotonic.append(name)
    result["nonmonotonic_topics"] = nonmonotonic

    safe = datasets["haar_safe_roll_status"]
    safe_t = arr(safe, "timestamp")
    requested = arr(safe, "requested_code")
    if requested.size == 0:
        requested = arr(safe, "requested_mode")
    active = arr(safe, "active_code")
    if active.size == 0:
        active = arr(safe, "active_mode")
    requested_mode, requested_fraction = dominant(requested)
    active_mode, active_fraction = dominant(active)
    result.update(
        requested_mode=requested_mode,
        requested_mode_fraction=requested_fraction,
        active_mode=active_mode,
        active_mode_fraction=active_fraction,
        requested_active_match_fraction=(
            float(np.mean(requested == active))
            if requested.size and requested.size == active.size
            else math.nan
        ),
        mode_switches=(
            int(np.sum(np.diff(requested.astype(np.int64)) != 0)) if requested.size > 1 else 0
        ),
    )

    phi_sp = arr(safe, "phi_sp").astype(float)
    phi = arr(safe, "phi").astype(float)
    p_rate = arr(safe, "p").astype(float)
    if phi_sp.size == 0:
        phi_sp = arr(datasets["vehicle_attitude_setpoint"], "roll_body").astype(float)
    if p_rate.size == 0:
        p_rate = arr(datasets["vehicle_angular_velocity"], "xyz[0]").astype(float)
    rad_to_deg = 180.0 / math.pi
    ref_deg = phi_sp * rad_to_deg
    phi_deg = phi * rad_to_deg
    p_deg_s = p_rate * rad_to_deg
    ref_t = safe_t if safe_t.size == ref_deg.size else arr(datasets["vehicle_attitude_setpoint"], "timestamp")
    p_t = safe_t if safe_t.size == p_deg_s.size else arr(datasets["vehicle_angular_velocity"], "timestamp")
    result.update(
        ref_min_deg=float(np.nanmin(ref_deg)) if ref_deg.size else math.nan,
        ref_max_deg=float(np.nanmax(ref_deg)) if ref_deg.size else math.nan,
        ref_abs_p95_deg=q(np.abs(ref_deg), 95),
        ref_negative_dwell_s=dwell_seconds(ref_deg, ref_t, lambda x: x < -2.0),
        ref_positive_dwell_s=dwell_seconds(ref_deg, ref_t, lambda x: x > 2.0),
        ref_zero_dwell_s=dwell_seconds(ref_deg, ref_t, lambda x: np.abs(x) < 1.0),
        phi_min_deg=float(np.nanmin(phi_deg)) if phi_deg.size else math.nan,
        phi_max_deg=float(np.nanmax(phi_deg)) if phi_deg.size else math.nan,
        phi_abs_p95_deg=q(np.abs(phi_deg), 95),
        p_abs_p95_deg_s=q(np.abs(p_deg_s), 95),
        p_abs_p99_deg_s=q(np.abs(p_deg_s), 99),
        p_abs_max_deg_s=float(np.nanmax(np.abs(p_deg_s))) if p_deg_s.size else math.nan,
    )
    if safe_t.size > 1:
        safe_gaps_ms = np.diff(safe_t.astype(np.int64)) / 1000.0
        result["safe_status_gap_p99_ms"] = q(safe_gaps_ms, 99)
        result["safe_status_gap_max_ms"] = q(safe_gaps_ms, 100)
    else:
        result["safe_status_gap_p99_ms"] = math.nan
        result["safe_status_gap_max_ms"] = math.nan

    # Interior peak provides pre/post data for impact classification.
    if p_deg_s.size and p_t.size == p_deg_s.size:
        peak_index = int(np.nanargmax(np.abs(p_deg_s)))
        result["p_peak_time_from_start_s"] = float((p_t[peak_index] - p_t[0]) / 1e6)
        result["p_peak_time_to_end_s"] = float((p_t[-1] - p_t[peak_index]) / 1e6)
    else:
        result["p_peak_time_from_start_s"] = math.nan
        result["p_peak_time_to_end_s"] = math.nan

    acceleration = datasets["vehicle_acceleration"]
    accel_components = [arr(acceleration, f"xyz[{index}]").astype(float) for index in range(3)]
    if all(component.size for component in accel_components):
        accel_norm = np.sqrt(sum(component * component for component in accel_components))
        accel_deviation = np.abs(accel_norm - np.nanmedian(accel_norm))
        result["accel_dev_p99"] = q(accel_deviation, 99)
        result["accel_dev_max"] = float(np.nanmax(accel_deviation))
    else:
        result["accel_dev_p99"] = math.nan
        result["accel_dev_max"] = math.nan

    manual = datasets["manual_control_setpoint"]
    throttle = arr(manual, "throttle").astype(float)
    result["throttle_median"] = q(throttle, 50)
    result["manual_valid_fraction"] = bool_fraction(arr(manual, "valid"))

    hydro = datasets["hydro_allocate_message"]
    result["available_thrust_median"] = q(arr(hydro, "available_thruster_thrust"), 50)
    result["hydro_force_x_abs_p50"] = q(np.abs(arr(hydro, "command_force_x")), 50)
    result["hydro_sample_count"] = int(len(arr(hydro, "timestamp")))

    servos = datasets["actuator_servos"]
    servo4 = arr(servos, "control[3]").astype(float)
    servo5 = arr(servos, "control[4]").astype(float)
    result["servo45_samples"] = int(min(servo4.size, servo5.size))
    result["servo45_finite_fraction"] = (
        float(np.mean(np.isfinite(servo4) & np.isfinite(servo5)))
        if servo4.size and servo4.size == servo5.size
        else math.nan
    )
    result["servo45_activity"] = (
        float(np.nanmax(np.maximum(np.abs(servo4), np.abs(servo5))))
        if servo4.size and servo5.size
        else math.nan
    )

    gate = arr(safe, "foil_gate_enabled")
    result["foil_gate_fraction"] = bool_fraction(gate)
    result["allocator_valid_fraction"] = bool_fraction(arr(safe, "allocator_valid"))
    result["allocator_bounded_fraction"] = bool_fraction(arr(safe, "allocator_bounded"))
    result["fallback_fraction"] = bool_fraction(arr(safe, "fallback_active"))
    result["fault_nonzero_fraction"] = (
        float(np.mean(arr(safe, "fault_flags") != 0)) if arr(safe, "fault_flags").size else math.nan
    )
    result["first_fault_max"] = int(np.max(arr(safe, "first_fault_code"))) if arr(safe, "first_fault_code").size else 0

    field = datasets["haar_safe_roll_field_status"]
    severe = arr(field, "severe_fault_code")
    result["severe_fault_max"] = int(np.max(severe)) if severe.size else 0
    result["temporary_fallback_fraction"] = bool_fraction(arr(field, "temporary_fallback_active"))

    depth_alloc = datasets["haar_depth_allocator_status"]
    depth_t = arr(depth_alloc, "timestamp")
    source = arr(depth_alloc, "selected_solution_source")
    override = arr(depth_alloc, "mode_branch_overrode_solution")
    symmetric_lost = arr(depth_alloc, "symmetric_depth_lost")
    suppressed = arr(depth_alloc, "allocator_output_suppressed")
    allocator_finite = arr(depth_alloc, "allocator_finite")
    allocator_bounded = arr(depth_alloc, "allocator_bounded")
    result["new_allocation_schema_present"] = bool(source.size)
    result["selected_source_complete_fraction"] = (
        float(np.mean(source == 1)) if source.size else math.nan
    )
    result["mode_branch_override_any"] = bool(np.any(override)) if override.size else False
    result["symmetric_depth_lost_any"] = bool(np.any(symmetric_lost)) if symmetric_lost.size else False
    result["allocator_output_suppressed_fraction"] = bool_fraction(suppressed)
    result["allocator_finite_fraction"] = bool_fraction(allocator_finite)
    result["depth_allocator_bounded_fraction"] = bool_fraction(allocator_bounded)
    result["full_best_feasible_fraction"] = bool_fraction(arr(depth_alloc, "full_best_feasible_solution_used"))

    req_sym = arr(depth_alloc, "requested_symmetric_foil_component").astype(float)
    real_sym = arr(depth_alloc, "realized_symmetric_foil_component").astype(float)
    if req_sym.size and req_sym.size == real_sym.size:
        apparent_loss = (
            np.isfinite(req_sym)
            & np.isfinite(real_sym)
            & (np.abs(req_sym) > math.radians(2.0))
            & (np.abs(real_sym) < math.radians(0.5))
        )
        result["apparent_symmetric_loss_fraction"] = float(np.mean(apparent_loss))
        result["apparent_symmetric_loss_max_run_s"] = max_true_run_seconds(apparent_loss, depth_t)
    else:
        result["apparent_symmetric_loss_fraction"] = math.nan
        result["apparent_symmetric_loss_max_run_s"] = 0.0

    depth_control = datasets["haar_depth_control_status"]
    result["depth_suppressed_fraction"] = bool_fraction(arr(depth_control, "depth_command_suppressed"))
    result["depth_controller_enabled_fraction"] = bool_fraction(arr(depth_control, "depth_controller_enabled"))
    result["suppression_reason_max"] = (
        int(np.max(arr(depth_control, "suppression_reason")))
        if arr(depth_control, "suppression_reason").size
        else 0
    )

    vehicle_status = datasets["vehicle_status"]
    result["vehicle_failsafe_any"] = bool(np.any(arr(vehicle_status, "failsafe")))
    result["gcs_lost_any"] = bool(np.any(arr(vehicle_status, "gcs_connection_lost")))
    result["arming_states"] = sorted(set(int(x) for x in arr(vehicle_status, "arming_state")))
    result["nav_states"] = sorted(set(int(x) for x in arr(vehicle_status, "nav_state")))

    failsafe_flags = datasets["failsafe_flags"]
    result["manual_control_signal_lost_any"] = bool(
        np.any(arr(failsafe_flags, "manual_control_signal_lost"))
    )
    input_rc = datasets["input_rc"]
    result["rc_lost_any"] = bool(np.any(arr(input_rc, "rc_lost")))
    result["rc_failsafe_any"] = bool(np.any(arr(input_rc, "rc_failsafe")))
    result["hydro_rc_signal_lost_any"] = bool(np.any(arr(hydro, "rc_signal_lost")))
    result["hydro_rc_failsafe_any"] = bool(np.any(arr(hydro, "rc_failsafe")))
    return result


def classify(result):
    """Conservative content-based admission classification."""
    if not result["parseable"]:
        return "MODE_UNCERTAIN", "TYPE_UNCERTAIN", False, "FAIL", "UNUSABLE", "ULog损坏，无法解析"
    if result["file_corruption"] or result["nonmonotonic_topics"]:
        return "MODE_UNCERTAIN", "TYPE_UNCERTAIN", False, "FAIL", "UNUSABLE", "文件损坏或时间轴非单调"

    mode = result["active_mode"]
    mode_label = str(mode) if mode in (0, 1, 2, 3, 4) and result["active_mode_fraction"] >= 0.9 else "MODE_UNCERTAIN"
    if mode_label == "MODE_UNCERTAIN":
        return mode_label, "TYPE_UNCERTAIN", False, "FAIL", "UNUSABLE", "有效段Mode无法可靠识别"

    reference_is_zero = (
        result["ref_abs_p95_deg"] < 1.0
        or (
            result["ref_abs_p95_deg"] < 2.5
            and result["ref_min_deg"] > -3.0
            and result["ref_max_deg"] < 3.0
            and result["ref_negative_dwell_s"] < 0.25
            and result["ref_positive_dwell_s"] < 0.25
        )
    )
    signed_reference_complete = (
        result["ref_negative_dwell_s"] >= 0.25
        and result["ref_positive_dwell_s"] >= 0.25
        and result["ref_zero_dwell_s"] >= 0.5
    )
    impact_clear = (
        reference_is_zero
        and result["p_abs_max_deg_s"] >= 80.0
        and result["p_peak_time_from_start_s"] >= 1.0
        and result["p_peak_time_to_end_s"] >= 1.0
    )
    if signed_reference_complete:
        experiment_type = "dynamic"
    elif impact_clear:
        experiment_type = "impact"
    elif reference_is_zero and result["p_abs_max_deg_s"] < 40.0:
        experiment_type = "static"
    else:
        experiment_type = "TYPE_UNCERTAIN"

    valid_segment = (
        result["duration_s"] >= 5.0
        and result["foil_gate_fraction"] >= 0.9
        and result["servo45_samples"] > 0
        and result["servo45_finite_fraction"] >= 0.99
        and result["throttle_median"] > -0.9
    )

    critical_fault = (
        result["vehicle_failsafe_any"]
        or result["rc_lost_any"]
        or result["rc_failsafe_any"]
        or result["hydro_rc_signal_lost_any"]
        or result["hydro_rc_failsafe_any"]
        or result["severe_fault_max"] != 0
        or result["allocator_finite_fraction"] < 0.999
        or result["depth_allocator_bounded_fraction"] < 0.999
    )

    if mode != 4 and result["new_allocation_schema_present"]:
        allocation_ok = (
            result["selected_source_complete_fraction"] >= 0.99
            and not result["mode_branch_override_any"]
            and not result["symmetric_depth_lost_any"]
            and result["allocator_output_suppressed_fraction"] <= 0.01
            and result["foil_gate_fraction"] >= 0.99
        )
        allocation_label = "PASS" if allocation_ok else "FAIL"
    elif mode != 4:
        allocation_ok = (
            result["apparent_symmetric_loss_max_run_s"] <= 0.5
            and result["allocator_output_suppressed_fraction"] <= 0.01
            and result["foil_gate_fraction"] >= 0.99
        )
        allocation_label = "LEGACY_SCHEMA_PASS" if allocation_ok else "FAIL"
    else:
        allocation_ok = True
        allocation_label = "MODE4_STRICT"

    if critical_fault:
        return mode_label, experiment_type, valid_segment, allocation_label, "UNUSABLE", "存在RC/failsafe/severe或allocator非有限/越界"
    if not allocation_ok:
        return mode_label, experiment_type, valid_segment, allocation_label, "UNUSABLE", "完整分配路径检查失败"
    if not valid_segment:
        return mode_label, experiment_type, False, allocation_label, "UNUSABLE", "无足够长且有推进/翼面信号的有效滑行段"

    cautions = []
    if experiment_type == "TYPE_UNCERTAIN":
        cautions.append("实验类型置信度不足")
    if result["missing_format_topics"]:
        cautions.append("标准pyulog缺少estimator FORMAT，需兼容读取")
    if result["safe_status_gap_p99_ms"] > 25.0 or result["safe_status_gap_max_ms"] > 100.0:
        cautions.append(
            f"日志有间歇缺口(p99/max={result['safe_status_gap_p99_ms']:.0f}/{result['safe_status_gap_max_ms']:.0f}ms)"
        )
    if result["duration_s"] < 8.0:
        cautions.append("有效段偏短")
    if experiment_type == "dynamic" and (
        result["ref_negative_dwell_s"] < 0.5 or result["ref_positive_dwell_s"] < 0.5
    ):
        cautions.append("正负参考一侧持续偏短")

    # All current logs require the missing-format compatibility path; retain
    # an honest caution label even when the experiment itself is complete.
    level = "USABLE_WITH_CAUTION" if cautions else "USABLE"
    return mode_label, experiment_type, True, allocation_label, level, "；".join(cautions) or "动作完整且无关键异常"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_directory", type=Path)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    logs = sorted(args.log_directory.rglob("*.ulg"))
    results = [audit_file(path) for path in logs]
    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=2, allow_nan=True))
        return
    header = (
        "file dur mode(req/act) match ref[min,max] pmax phi[min,max] gate src1 ov lost "
        "supp finite bounded best fs severe drop(ms)"
    )
    print(header)
    for r in results:
        if not r["parseable"]:
            print(f"{r['file']} PARSE_ERROR {r['parse_error']}")
            continue
        print(
            f"{r['file']} {r['duration_s']:.1f} "
            f"{r['requested_mode']}/{r['active_mode']} "
            f"{r['requested_active_match_fraction']:.2f} "
            f"[{r['ref_min_deg']:.1f},{r['ref_max_deg']:.1f}] "
            f"{r['p_abs_max_deg_s']:.1f} "
            f"[{r['phi_min_deg']:.1f},{r['phi_max_deg']:.1f}] "
            f"{r['foil_gate_fraction']:.2f} "
            f"{r['selected_source_complete_fraction']:.2f} "
            f"{int(r['mode_branch_override_any'])} {int(r['symmetric_depth_lost_any'])} "
            f"{r['allocator_output_suppressed_fraction']:.2f} "
            f"{r['allocator_finite_fraction']:.2f} "
            f"{r['depth_allocator_bounded_fraction']:.2f} "
            f"{r['full_best_feasible_fraction']:.2f} "
            f"{int(r['vehicle_failsafe_any'] or r['rc_lost_any'] or r['rc_failsafe_any'])} "
            f"{r['severe_fault_max']} {r['dropout_total_ms']}"
        )


if __name__ == "__main__":
    main()
