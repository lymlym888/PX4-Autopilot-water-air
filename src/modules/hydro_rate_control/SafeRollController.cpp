/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "SafeRollController.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

#include <cmath>
#include <cstring>

namespace
{
constexpr float kDtMin = 0.001f;
constexpr float kDtMax = 0.010f;
constexpr float kNominalDt = 0.0025f;
constexpr uint64_t kGyroStaleUs = 20'000;
constexpr uint64_t kAttitudeStaleUs = 50'000;
constexpr uint32_t kRuntimeSoftBudgetUs = 1'000;
constexpr uint32_t kRuntimeHardBudgetUs = 2'500;
constexpr uint32_t kRuntimeSoftConsecutiveLimit = 5;
constexpr int kBisectionIterations = 8;
constexpr int kLags[5] {4, 8, 16, 24, 32};
constexpr float kEpsilon = 1e-6f;
constexpr float kDegToRad = 0.017453292519943295f;

hrt_abstime timingNow()
{
#ifdef SAFE_ROLL_DISABLE_PERF
	return 0;
#else
	return hrt_absolute_time();
#endif
}

uint32_t timingElapsed(const hrt_abstime &start)
{
#ifdef SAFE_ROLL_DISABLE_PERF
	(void)start;
	return 0;
#else
	return hrt_elapsed_time(&start);
#endif
}

bool finite(float value)
{
	return std::isfinite(value);
}

uint32_t firstSetBit(uint32_t value)
{
	return value & (~value + 1u);
}

uint32_t crc32Byte(uint32_t crc, uint8_t byte)
{
	crc ^= byte;

	for (int bit = 0; bit < 8; ++bit) {
		crc = (crc >> 1) ^ (0xEDB88320u & static_cast<uint32_t>(-static_cast<int32_t>(crc & 1u)));
	}

	return crc;
}

template<typename T>
void crc32Add(uint32_t &crc, const T &value)
{
	const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);

	for (unsigned i = 0; i < sizeof(T); ++i) {
		crc = crc32Byte(crc, bytes[i]);
	}
}
}

SafeRollController::SafeRollController() :
#ifndef SAFE_ROLL_DISABLE_PERF
	_cycle_perf(perf_alloc(PC_ELAPSED, "safe_roll: cycle"))
#else
	_cycle_perf(nullptr)
#endif
{
}

SafeRollController::~SafeRollController()
{
#ifndef SAFE_ROLL_DISABLE_PERF
	perf_free(_cycle_perf);
#endif
}

void SafeRollController::reset(bool clear_severe_latch)
{
	const bool preserved_latch = _fallback_latched;
	const uint32_t preserved_fault = _first_fault_code;
	const uint32_t preserved_stage = _first_fault_stage_mask;
	const uint32_t preserved_total = _first_fault_total_candidate_time_us;
	const uint32_t preserved_reference = _first_fault_reference_filter_time_us;
	const uint32_t preserved_estimator = _first_fault_estimator_time_us;
	const uint32_t preserved_certificate = _first_fault_certificate_time_us;
	const uint32_t preserved_allocation = _first_fault_allocation_query_time_us;
	const uint32_t preserved_nominal = _first_fault_nominal_time_us;
	const uint32_t preserved_cbf = _first_fault_cbf_time_us;
	const uint32_t preserved_nonlinear = _first_fault_nonlinear_check_time_us;
	const FirstFaultSnapshot preserved_snapshot = _first_fault_snapshot;
	_initialized = false;
	_reference_initialized = false;
	_have_previous_p = false;
	_fallback_latched = false;
	_activation_pending = false;
	_mode_activated = false;
	_allocator_valid_streak = 0;
	_allocator_invalid_streak = 0;
	_allocator_recovery_streak = 0;
	_aux_gate_low_streak = 0;
	_aux_gate_high_streak = 0;
	_allocator_invalid_start_timestamp = 0;
	_allocator_invalid_duration_us = 0;
	_allocator_recovery_pending = false;
	_allocator_degraded_active = false;
	_temporary_fallback_active = false;
	_best_feasible_active = false;
	_aux_gate_inhibit_active = false;
	_have_last_feasible = false;
	_first_fault_code = 0;
	_first_fault_stage_mask = 0;
	_first_fault_total_candidate_time_us = 0;
	_first_fault_reference_filter_time_us = 0;
	_first_fault_estimator_time_us = 0;
	_first_fault_certificate_time_us = 0;
	_first_fault_allocation_query_time_us = 0;
	_first_fault_nominal_time_us = 0;
	_first_fault_cbf_time_us = 0;
	_first_fault_nonlinear_check_time_us = 0;
	_first_fault_snapshot = {};
	_consecutive_runtime_overruns = 0;
	_returning_to_legacy = false;
	_history_head = -1;
	_history_count = 0;
	_last_sample_timestamp = 0;

	if (!clear_severe_latch) {
		_fallback_latched = preserved_latch;
		_first_fault_code = preserved_fault;
		_first_fault_stage_mask = preserved_stage;
		_first_fault_total_candidate_time_us = preserved_total;
		_first_fault_reference_filter_time_us = preserved_reference;
		_first_fault_estimator_time_us = preserved_estimator;
		_first_fault_certificate_time_us = preserved_certificate;
		_first_fault_allocation_query_time_us = preserved_allocation;
		_first_fault_nominal_time_us = preserved_nominal;
		_first_fault_cbf_time_us = preserved_cbf;
		_first_fault_nonlinear_check_time_us = preserved_nonlinear;
		_first_fault_snapshot = preserved_snapshot;
	}

	++_reset_count;
}

void SafeRollController::resetNewController(const Input &input, bool preserve_selected, bool clear_severe_latch)
{
	const float selected_candidate = preserve_selected ? _last_selected : input.legacy_u_roll;
	const float selected = finite(selected_candidate) ? selected_candidate : 0.f;
	_reference_z1 = finite(input.phi_cmd) ? input.phi_cmd : 0.f;
	_reference_z2 = finite(input.p) ? input.p : 0.f;
	_reference_initialized = true;
	_have_previous_p = false;
	_history_head = -1;
	_history_count = 0;
	_filtered_u = 0.f;
	_filtered_r = 0.f;
	_b_hat = math::constrain(input.config.b0, input.config.b_min, input.config.b_max);
	_b_lower = math::max(input.config.b_min, _b_hat - input.config.b_radius_initial);
	_b_upper = math::min(input.config.b_max, _b_hat + input.config.b_radius_initial);
	_d_hat = 0.f;
	_rho_d = input.config.disturbance_radius_initial;
	_last_selected = selected;
	_blend_start_output = selected;
	_blend_start_timestamp = input.angular_velocity_timestamp_sample;
	_blend_target_code = 0;
	if (clear_severe_latch) {
		_fallback_latched = false;
		_first_fault_code = 0;
		_first_fault_stage_mask = 0;
		_first_fault_total_candidate_time_us = 0;
		_first_fault_reference_filter_time_us = 0;
		_first_fault_estimator_time_us = 0;
		_first_fault_certificate_time_us = 0;
		_first_fault_allocation_query_time_us = 0;
		_first_fault_nominal_time_us = 0;
		_first_fault_cbf_time_us = 0;
		_first_fault_nonlinear_check_time_us = 0;
		_first_fault_snapshot = {};
	}
	_activation_pending = false;
	_mode_activated = false;
	_allocator_valid_streak = 0;
	_allocator_invalid_streak = 0;
	_allocator_recovery_streak = 0;
	_aux_gate_low_streak = 0;
	_aux_gate_high_streak = 0;
	_allocator_invalid_start_timestamp = 0;
	_allocator_invalid_duration_us = 0;
	_allocator_recovery_pending = false;
	_allocator_degraded_active = false;
	_temporary_fallback_active = false;
	_best_feasible_active = false;
	_aux_gate_inhibit_active = false;
	_have_last_feasible = false;
	_last_feasible_u = selected;
	_consecutive_runtime_overruns = 0;
	_returning_to_legacy = false;
	++_reset_count;
}

float SafeRollController::bumplessStep(float current, float target, float max_rate, float dt)
{
	if (!finite(current) || !finite(target) || !finite(max_rate) || !finite(dt)
	    || max_rate < 0.f || dt <= 0.f) {
		return current;
	}

	const float maximum_step = max_rate * dt;
	return current + math::constrain(target - current, -maximum_step, maximum_step);
}

bool SafeRollController::hardwareReady(const Input &input)
{
	return input.foil_gate_enabled && input.logical_mapping_valid;
}

bool SafeRollController::validateConfig(const Config &c)
{
	const bool all_finite =
		finite(c.blend_time) && finite(c.reference_wn) && finite(c.reference_zeta)
		&& finite(c.reference_rate_max) && finite(c.reference_accel_max)
		&& finite(c.a0) && finite(c.b0) && finite(c.b_min) && finite(c.b_max)
		&& finite(c.phi_safe) && finite(c.u_max) && finite(c.du_max_nominal)
		&& finite(c.foil_angle_max) && finite(c.foil_rate_max) && finite(c.foil_remainder)
		&& finite(c.filter_lambda) && finite(c.b_adaptation_gain) && finite(c.b_adaptation_epsilon)
		&& finite(c.filtered_u_min) && finite(c.innovation_gate) && finite(c.b_update_max)
		&& finite(c.disturbance_gain) && finite(c.disturbance_radius_initial)
		&& finite(c.disturbance_variation_nominal) && finite(c.effectiveness_variation_nominal)
		&& finite(c.incremental_u_min) && finite(c.incremental_residual)
		&& finite(c.incremental_gate_factor) && finite(c.b_radius_initial)
		&& finite(c.k1) && finite(c.k2) && finite(c.gamma1) && finite(c.gamma2)
		&& finite(c.robust_gain_floor);

	return all_finite && c.enable <= 2 && c.blend_time >= 0.05f && c.blend_time <= 2.f
	       && c.reference_wn >= 0.5f && c.reference_wn <= 20.f
	       && c.reference_zeta >= 0.5f && c.reference_zeta <= 2.f
	       && c.reference_rate_max >= 10.f * kDegToRad && c.reference_rate_max <= 360.f * kDegToRad
	       && c.reference_accel_max >= 50.f * kDegToRad && c.reference_accel_max <= 2000.f * kDegToRad
	       && c.a0 >= -20.f && c.a0 <= 20.f
	       && c.b_min >= 0.1f && c.b_max <= 100.f && c.b_min <= c.b0 && c.b0 <= c.b_max
	       && c.phi_safe >= 10.f * kDegToRad && c.phi_safe <= 35.f * kDegToRad
	       && c.u_max >= 0.1f && c.u_max <= 1.f
	       && c.du_max_nominal >= 0.0001f && c.du_max_nominal <= 0.2f
	       && c.foil_angle_max >= 1.f * kDegToRad && c.foil_angle_max <= 65.f * kDegToRad
	       && c.foil_rate_max >= 1.f * kDegToRad && c.foil_rate_max <= 720.f * kDegToRad
	       && c.foil_remainder >= 0.f && c.foil_remainder <= 2.f * kDegToRad
	       && c.foil_remainder < c.foil_angle_max
	       && c.filter_lambda >= 0.8f && c.filter_lambda <= 1.f
	       && c.b_adaptation_gain >= 0.f && c.b_adaptation_gain <= 1.f
	       && c.b_adaptation_epsilon >= 1e-6f && c.b_adaptation_epsilon <= 10.f
	       && c.filtered_u_min >= 0.f && c.filtered_u_min <= 1.f
	       && c.innovation_gate >= 0.001f && c.innovation_gate <= 100.f
	       && c.b_update_max >= 0.f && c.b_update_max <= 1.f
	       && c.disturbance_gain >= 0.001f && c.disturbance_gain <= 1.f
	       && c.disturbance_radius_initial >= 0.f && c.disturbance_radius_initial <= 100.f
	       && c.disturbance_variation_nominal >= 0.f && c.disturbance_variation_nominal <= 1.f
	       && c.effectiveness_variation_nominal >= 0.f && c.effectiveness_variation_nominal <= 1.f
	       && c.incremental_u_min >= 0.f && c.incremental_u_min <= 2.f
	       && c.incremental_residual >= 0.f && c.incremental_residual <= 10.f
	       && c.incremental_gate_factor >= 1.f && c.incremental_gate_factor <= 100.f
	       && c.b_radius_initial >= 0.f && c.b_radius_initial <= 100.f
	       && c.k1 >= 0.f && c.k1 <= 1000.f && c.k2 >= 0.f && c.k2 <= 1000.f
	       && c.gamma1 >= 1e-6f && c.gamma1 <= 10.f
	       && c.gamma2 >= 1e-6f && c.gamma2 <= 10.f
	       && c.robust_gain_floor >= 0.01f && c.robust_gain_floor <= 100.f;
}

uint32_t SafeRollController::configurationCrc32(uint8_t requested_mode, const Config &c)
{
	uint32_t crc = 0xFFFFFFFFu;
	crc32Add(crc, requested_mode);
	crc32Add(crc, c.enable);
	const uint8_t field_mode = c.field_mode ? 1u : 0u;
	crc32Add(crc, field_mode);
	const uint8_t shadow = c.shadow ? 1u : 0u;
	crc32Add(crc, shadow);
	crc32Add(crc, c.reset_id);
	crc32Add(crc, c.blend_time);
	crc32Add(crc, c.reference_wn);
	crc32Add(crc, c.reference_zeta);
	crc32Add(crc, c.reference_rate_max);
	crc32Add(crc, c.reference_accel_max);
	crc32Add(crc, c.a0);
	crc32Add(crc, c.b0);
	crc32Add(crc, c.b_min);
	crc32Add(crc, c.b_max);
	crc32Add(crc, c.phi_safe);
	crc32Add(crc, c.u_max);
	crc32Add(crc, c.du_max_nominal);
	crc32Add(crc, c.foil_angle_max);
	crc32Add(crc, c.foil_rate_max);
	crc32Add(crc, c.foil_remainder);
	crc32Add(crc, c.filter_lambda);
	crc32Add(crc, c.b_adaptation_gain);
	crc32Add(crc, c.b_adaptation_epsilon);
	crc32Add(crc, c.filtered_u_min);
	crc32Add(crc, c.innovation_gate);
	crc32Add(crc, c.b_update_max);
	crc32Add(crc, c.disturbance_gain);
	crc32Add(crc, c.disturbance_radius_initial);
	crc32Add(crc, c.disturbance_variation_nominal);
	crc32Add(crc, c.effectiveness_variation_nominal);
	crc32Add(crc, c.incremental_u_min);
	crc32Add(crc, c.incremental_residual);
	crc32Add(crc, c.incremental_gate_factor);
	crc32Add(crc, c.b_radius_initial);
	crc32Add(crc, c.k1);
	crc32Add(crc, c.k2);
	crc32Add(crc, c.gamma1);
	crc32Add(crc, c.gamma2);
	crc32Add(crc, c.robust_gain_floor);
	return crc ^ 0xFFFFFFFFu;
}

uint32_t SafeRollController::resetConfigurationCrc32(const Config &c)
{
	uint32_t crc = 0xFFFFFFFFu;
	crc32Add(crc, c.reset_id);
	crc32Add(crc, c.a0);
	crc32Add(crc, c.b0);
	crc32Add(crc, c.b_min);
	crc32Add(crc, c.b_max);
	crc32Add(crc, c.phi_safe);
	crc32Add(crc, c.disturbance_radius_initial);
	crc32Add(crc, c.disturbance_variation_nominal);
	crc32Add(crc, c.effectiveness_variation_nominal);
	crc32Add(crc, c.incremental_u_min);
	crc32Add(crc, c.incremental_residual);
	crc32Add(crc, c.incremental_gate_factor);
	crc32Add(crc, c.b_radius_initial);
	crc32Add(crc, c.foil_angle_max);
	crc32Add(crc, c.foil_rate_max);
	crc32Add(crc, c.foil_remainder);
	return crc ^ 0xFFFFFFFFu;
}

bool SafeRollController::finiteInterval(const Interval &interval)
{
	return interval.valid && finite(interval.lower) && finite(interval.upper) && interval.lower <= interval.upper;
}

SafeRollController::Interval SafeRollController::intersect(const Interval &a, const Interval &b)
{
	Interval out{};

	if (!finiteInterval(a) || !finiteInterval(b)) {
		return out;
	}

	out.lower = math::max(a.lower, b.lower);
	out.upper = math::min(a.upper, b.upper);
	out.valid = out.lower <= out.upper;
	return out;
}

SafeRollController::Interval SafeRollController::combineAllocationInterval(uint8_t mode,
		const Interval &amplitude_rate, const Interval &allocation, bool use_allocation, bool &degraded,
		bool field_mode)
{
	degraded = false;

	if (!use_allocation) {
		return amplitude_rate;
	}

	const Interval combined = intersect(amplitude_rate, allocation);

	if (finiteInterval(combined)) {
		return combined;
	}

	if ((mode == static_cast<uint8_t>(Mode::Nominal) || field_mode) && finiteInterval(amplitude_rate)) {
		degraded = true;
		return amplitude_rate;
	}

	return {};
}

float SafeRollController::project(float value, const Interval &interval)
{
	return finite(value) && finiteInterval(interval) ? math::constrain(value, interval.lower, interval.upper) : NAN;
}

float SafeRollController::minimumRelaxation(const Interval &cbf, const Interval &hard)
{
	if (!cbf.valid || !finite(cbf.lower) || !finite(cbf.upper) || !finiteInterval(hard)) {
		return NAN;
	}

	return math::max(0.f, math::max(0.5f * (cbf.lower - cbf.upper),
				      math::max(cbf.lower - hard.upper, hard.lower - cbf.upper)));
}

float SafeRollController::scaledVariationBound(float nominal_bound, float dt)
{
	return finite(nominal_bound) && nominal_bound >= 0.f && finite(dt) && dt >= 0.f
	       ? nominal_bound * dt / kNominalDt : NAN;
}

uint32_t SafeRollController::runtimeSoftBudgetUs()
{
	return kRuntimeSoftBudgetUs;
}

uint32_t SafeRollController::runtimeHardBudgetUs()
{
	return kRuntimeHardBudgetUs;
}

bool SafeRollController::runtimeBudgetExceeded(uint32_t elapsed_us, bool transition,
		uint32_t &consecutive_soft_overruns)
{
	if (elapsed_us > kRuntimeHardBudgetUs) {
		return true;
	}

	if (transition) {
		consecutive_soft_overruns = 0;
		return false;
	}

	if (elapsed_us > kRuntimeSoftBudgetUs) {
		++consecutive_soft_overruns;

	} else {
		consecutive_soft_overruns = 0;
	}

	return consecutive_soft_overruns >= kRuntimeSoftConsecutiveLimit;
}

SafeRollController::Interval SafeRollController::refineCertificate(const Interval &prior, float delta_r,
		float delta_u, float omega, bool &conflict)
{
	conflict = false;

	if (!finiteInterval(prior) || !finite(delta_r) || !finite(delta_u) || !finite(omega)
	    || omega < 0.f || std::fabs(delta_u) < kEpsilon) {
		return {};
	}

	float measurement_lower = (delta_r - omega) / delta_u;
	float measurement_upper = (delta_r + omega) / delta_u;

	if (measurement_lower > measurement_upper) {
		const float tmp = measurement_lower;
		measurement_lower = measurement_upper;
		measurement_upper = tmp;
	}

	const Interval measurement{measurement_lower, measurement_upper, true};
	const Interval refined = intersect(prior, measurement);

	if (!finiteInterval(refined)) {
		conflict = true;
		return prior;
	}

	return refined;
}

float SafeRollController::nominalControl(float a0, float p, float d_hat, float phi_ref_ddot,
		float k1, float e1, float k2, float e2, float b_control)
{
	if (!finite(a0) || !finite(p) || !finite(d_hat) || !finite(phi_ref_ddot)
	    || !finite(k1) || !finite(e1) || !finite(k2) || !finite(e2)
	    || !finite(b_control) || b_control <= 0.f) {
		return NAN;
	}

	return (-a0 * p - d_hat + phi_ref_ddot + k1 * e1 + k2 * e2) / b_control;
}

SafeRollController::Interval SafeRollController::robustCbfInterval(float phi_safe, float phi, float p, float dt,
		float a0, float d_hat, float rho_d, float b_hat, float rho_b,
		float gamma1, float gamma2, float robust_gain_floor,
		float &h_upper, float &h_lower)
{
	Interval out{};
	h_upper = phi_safe - phi;
	h_lower = phi_safe + phi;

	if (!finite(phi_safe) || !finite(phi) || !finite(p) || !finite(dt) || dt <= 0.f
	    || !finite(a0) || !finite(d_hat) || !finite(rho_d) || rho_d < 0.f
	    || !finite(b_hat) || !finite(rho_b) || rho_b < 0.f
	    || !finite(gamma1) || !finite(gamma2) || !finite(robust_gain_floor)) {
		return out;
	}

	const float dt2 = dt * dt;
	const float beta_upper = gamma1 * gamma2 * h_upper - (gamma1 + gamma2) * dt * p
				 - dt2 * (a0 * p + d_hat);
	const float beta_lower = gamma1 * gamma2 * h_lower + (gamma1 + gamma2) * dt * p
				 + dt2 * (a0 * p + d_hat);
	const float b_minus = math::max(robust_gain_floor, b_hat - rho_b);
	const float b_plus = b_hat + rho_b;

	if (b_minus <= 0.f || b_plus <= 0.f) {
		return out;
	}

	const float c_lower = beta_lower / dt2 - rho_d;
	const float c_upper = beta_upper / dt2 - rho_d;
	out.lower = c_lower >= 0.f ? -c_lower / b_plus : -c_lower / b_minus;
	out.upper = c_upper >= 0.f ? c_upper / b_plus : c_upper / b_minus;
	out.valid = finite(out.lower) && finite(out.upper);
	return out;
}

void SafeRollController::pushInnovation(const InnovationSample &sample)
{
	_history_head = (_history_head + 1) % HistoryLength;
	_history[_history_head] = sample;
	_history_count = math::min(_history_count + 1, HistoryLength);
}

const SafeRollController::InnovationSample *SafeRollController::historyLag(int lag) const
{
	if (lag < 0 || lag >= _history_count || _history_head < 0) {
		return nullptr;
	}

	int index = _history_head - lag;

	while (index < 0) {
		index += HistoryLength;
	}

	return &_history[index];
}

void SafeRollController::updateReference(const Input &input, Output &output, float dt)
{
	if (!_reference_initialized) {
		_reference_z1 = input.phi_cmd;
		_reference_z2 = math::constrain(input.p, -input.config.reference_rate_max, input.config.reference_rate_max);
		_reference_initialized = true;
	}

	float acceleration = input.config.reference_wn * input.config.reference_wn * (input.phi_cmd - _reference_z1)
			     - 2.f * input.config.reference_zeta * input.config.reference_wn * _reference_z2;
	acceleration = math::constrain(acceleration, -input.config.reference_accel_max, input.config.reference_accel_max);
	_reference_z1 += dt * _reference_z2;
	_reference_z2 = math::constrain(_reference_z2 + dt * acceleration,
					-input.config.reference_rate_max, input.config.reference_rate_max);
	output.phi_ref = _reference_z1;
	output.phi_ref_dot = _reference_z2;
	output.phi_ref_ddot = acceleration;
}

bool SafeRollController::updateEstimators(const Input &input, Output &output, float dt)
{
	const hrt_abstime estimator_start = timingNow();
	const Config &c = input.config;
	const bool fixed_mode = input.requested_mode == static_cast<uint8_t>(Mode::WorstCaseFixed);
	const bool adaptive_mode = input.requested_mode == static_cast<uint8_t>(Mode::Adaptive)
				   || input.requested_mode == static_cast<uint8_t>(Mode::NoAllocationAblation);

	if (_have_previous_p) {
		InnovationSample sample{};
		sample.timestamp = input.angular_velocity_timestamp_sample;
		sample.r = (input.p - _previous_p) / dt - c.a0 * _previous_p;
		sample.u_effective = input.previous_u_effective_model;
		sample.b_lower = _b_lower;
		sample.b_upper = _b_upper;
		sample.final_left = input.previous_final_servo5 * input.allocation_context.max_hydrofoil_angle;
		sample.final_right = input.previous_final_servo4 * input.allocation_context.max_hydrofoil_angle;
		sample.valid = finite(sample.r) && finite(sample.u_effective) && finite(sample.final_left)
			       && finite(sample.final_right);

		if (sample.valid) {
			pushInnovation(sample);
		}
	}

	_previous_p = input.p;
	_have_previous_p = true;
	const InnovationSample *newest = historyLag(0);
	const InnovationSample *previous = historyLag(1);

	if (newest && previous && newest->valid && previous->valid) {
		const float b_prior = _b_hat;
		_filtered_u = c.filter_lambda * (_filtered_u + newest->u_effective - previous->u_effective);
		_filtered_r = c.filter_lambda * (_filtered_r + newest->r - previous->r);
		const float innovation = _filtered_r - b_prior * _filtered_u;
		output.filtered_innovation = innovation;

		if (!fixed_mode && std::fabs(_filtered_u) >= c.filtered_u_min
		    && std::fabs(innovation) <= c.innovation_gate) {
			float delta_b = c.b_adaptation_gain * _filtered_u * innovation
					/ (c.b_adaptation_epsilon + _filtered_u * _filtered_u);
			delta_b = math::constrain(delta_b, -c.b_update_max, c.b_update_max);
			_b_hat = math::constrain(b_prior + delta_b, c.b_min, c.b_max);
			_b_hat = math::constrain(_b_hat, _b_lower, _b_upper);
			output.b_update_active = true;
		}

		const float disturbance_b = fixed_mode ? c.b0 : b_prior;
		_d_hat = (1.f - c.disturbance_gain) * _d_hat
			 + c.disturbance_gain * (newest->r - disturbance_b * newest->u_effective);
		const float rho_b_prior = fixed_mode ? math::max(c.b0 - c.b_min, c.b_max - c.b0)
					    : math::max(std::fabs(b_prior - _b_lower), std::fabs(_b_upper - b_prior));
		_rho_d = (1.f - c.disturbance_gain) * _rho_d
			 + c.disturbance_gain * std::fabs(newest->u_effective) * rho_b_prior
			 + scaledVariationBound(c.disturbance_variation_nominal, dt);
		_rho_d = math::max(0.f, _rho_d);
	}

	output.estimator_time_us = timingElapsed(estimator_start);
	output.execution_stage_mask |= StageEstimator;
	output.estimator_ready = newest && previous && newest->valid && previous->valid;
	output.b_hat = fixed_mode ? c.b0 : _b_hat;
	output.b_lower = fixed_mode ? c.b_min : _b_lower;
	output.b_upper = fixed_mode ? c.b_max : _b_upper;
	output.rho_b = fixed_mode ? math::max(c.b0 - c.b_min, c.b_max - c.b0)
		       : math::max(std::fabs(_b_hat - _b_lower), std::fabs(_b_upper - _b_hat));
	output.certified_width = output.b_upper - output.b_lower;
	output.d_hat = _d_hat;
	output.rho_d = _rho_d;
	output.filtered_u = _filtered_u;
	output.filtered_r = _filtered_r;
	const bool estimator_numeric_valid = finite(output.b_hat) && finite(output.d_hat) && finite(output.rho_d)
					     && output.rho_d >= 0.f;

	if (!fixed_mode && !adaptive_mode) {
		// Mode 1 uses the finite point estimate only. Do not enter the
		// fixed/adaptive certificate stage or charge certificate runtime.
		return estimator_numeric_valid;
	}

	const hrt_abstime certificate_start = timingNow();
	float refined_lower = _b_lower;
	float refined_upper = _b_upper;

	if (newest && newest->valid && adaptive_mode) {
		for (int i = 0; i < LagCount; ++i) {
			const InnovationSample *old = historyLag(kLags[i]);

			if (!old || !old->valid || newest->timestamp <= old->timestamp) {
				continue;
			}

			const float delta_r = newest->r - old->r;
			const float delta_u = newest->u_effective - old->u_effective;
			const float delta_t = static_cast<float>(newest->timestamp - old->timestamp) * 1e-6f;
			const float omega = scaledVariationBound(c.disturbance_variation_nominal, delta_t)
					    + scaledVariationBound(c.effectiveness_variation_nominal, delta_t)
					    * std::fabs(old->u_effective) + c.incremental_residual;

			if (std::fabs(delta_u) < c.incremental_u_min) {
				continue;
			}

			const float rho_b = math::max(std::fabs(_b_hat - refined_lower),
						     std::fabs(refined_upper - _b_hat));
			const float gate = c.incremental_gate_factor
					   * (omega + std::fabs(delta_u) * math::max(rho_b, kEpsilon));

			if (std::fabs(delta_r - _b_hat * delta_u) > gate) {
				continue;
			}

			bool conflict = false;
			const Interval refined = refineCertificate(
						 Interval{refined_lower, refined_upper, true},
						 delta_r, delta_u, omega, conflict);

			if (!conflict && finiteInterval(refined)) {
				refined_lower = refined.lower;
				refined_upper = refined.upper;
				output.accepted_lag_mask |= 1u << i;
				output.certificate_update_active = true;

			} else if (conflict) {
				output.conflict_lag_mask |= 1u << i;
				output.certificate_conflict = true;
			}
		}
	}

	_b_hat = math::constrain(_b_hat, refined_lower, refined_upper);
	output.b_hat = fixed_mode ? c.b0 : _b_hat;
	output.b_lower = fixed_mode ? c.b_min : refined_lower;
	output.b_upper = fixed_mode ? c.b_max : refined_upper;
	output.rho_b = fixed_mode ? math::max(c.b0 - c.b_min, c.b_max - c.b0)
		       : math::max(std::fabs(_b_hat - refined_lower), std::fabs(refined_upper - _b_hat));
	output.certified_width = output.b_upper - output.b_lower;
	output.d_hat = _d_hat;
	output.rho_d = _rho_d;
	output.filtered_u = _filtered_u;
	output.filtered_r = _filtered_r;
	output.certificate_time_us = timingElapsed(certificate_start);
	output.execution_stage_mask |= StageCertificate;

	if (adaptive_mode) {
		const float propagation = scaledVariationBound(c.effectiveness_variation_nominal, dt);
		_b_lower = math::max(c.b_min, refined_lower - propagation);
		_b_upper = math::min(c.b_max, refined_upper + propagation);
	}

	const bool certificate_numeric_valid = finite(output.b_lower) && finite(output.b_upper)
					      && finite(output.rho_b) && output.rho_b >= 0.f
					      && output.b_lower <= output.b_upper;
	output.certificate_ready = fixed_mode ? certificate_numeric_valid
				   : (adaptive_mode && certificate_numeric_valid && output.b_hat - output.rho_b > 0.f);
	output.certificate_warmup = adaptive_mode && _history_count <= kLags[LagCount - 1];

	// Mode 1 needs a finite point estimate only. Mode 2 uses the configured fixed
	// [b_min,b_max] certificate. Only adaptive modes require the online certificate.
	if (adaptive_mode) {
		return estimator_numeric_valid && output.certificate_ready;
	}

	return estimator_numeric_valid && (fixed_mode ? output.certificate_ready : true);
}

SafeRollController::Interval SafeRollController::intervalFromAffineConstraint(float base_u, float base_value,
		float gain, float value_lower, float value_upper)
{
	Interval out{};

	if (!finite(base_u) || !finite(base_value) || !finite(gain)
	    || !finite(value_lower) || !finite(value_upper) || value_lower > value_upper) {
		return out;
	}

	if (std::fabs(gain) < kEpsilon) {
		out.lower = -INFINITY;
		out.upper = INFINITY;
		out.valid = base_value >= value_lower && base_value <= value_upper;
		return out;
	}

	out.lower = base_u + (value_lower - base_value) / gain;
	out.upper = base_u + (value_upper - base_value) / gain;

	if (out.lower > out.upper) {
		const float tmp = out.lower;
		out.lower = out.upper;
		out.upper = tmp;
	}

	out.valid = true;
	return out;
}

SafeRollController::Interval SafeRollController::allocationInterval(const Input &input, const Config &c, float dt,
		hydrofoil::AllocationSolution &base_solution,
		hydrofoil::AllocationLinearization &linearization) const
{
	Interval invalid{};
	const hydrofoil::AllocationCommand base_command =
		hydrofoil::HydrofoilAllocationModel::withRollCommand(input.allocation_command, _last_selected);
	base_solution = hydrofoil::HydrofoilAllocationModel::solve(input.allocation_context, base_command);

	if (!base_solution.status.valid || input.allocation_context.max_hydrofoil_angle <= 0.f) {
		return invalid;
	}

	linearization = hydrofoil::HydrofoilAllocationModel::linearize(input.allocation_context, base_solution);

	if (!linearization.valid) {
		return invalid;
	}

	const float scale = input.allocation_context.max_hydrofoil_angle;
	const float base_left = input.generic_servo5 * scale + base_solution.delta_left_commanded;
	const float base_right = input.generic_servo4 * scale + base_solution.delta_right_commanded;
	const float previous_left = input.previous_final_servo5 * scale;
	const float previous_right = input.previous_final_servo4 * scale;
	const float absolute_limit = math::min(c.foil_angle_max - c.foil_remainder, scale);
	const float rate_step = c.foil_rate_max * dt;
	const float left_lower = math::max(-absolute_limit, previous_left - rate_step);
	const float left_upper = math::min(absolute_limit, previous_left + rate_step);
	const float right_lower = math::max(-absolute_limit, previous_right - rate_step);
	const float right_upper = math::min(absolute_limit, previous_right + rate_step);
	const Interval left = intervalFromAffineConstraint(base_command.u_phi, base_left, linearization.g_left,
				      left_lower, left_upper);
	const Interval right = intervalFromAffineConstraint(base_command.u_phi, base_right, linearization.g_right,
				       right_lower, right_upper);
	return intersect(left, right);
}

bool SafeRollController::checkNonlinearCandidate(const Input &input, float candidate, float dt,
		bool enforce_foil_constraints, Output &output, float &effective_model, bool &severe_failure) const
{
	severe_failure = false;

#ifdef SAFE_ROLL_DISABLE_PERF
	if (input.test_force_finite_nonlinear_failure) {
		return false;
	}
#endif

	if (!finite(candidate) || input.allocation_context.max_hydrofoil_angle <= 0.f) {
		severe_failure = true;
		return false;
	}

	hydrofoil::AllocationCommand command = input.allocation_command;
	const hydrofoil::AllocationSolution solution =
		hydrofoil::HydrofoilAllocationModel::checkCandidate(input.allocation_context, command, candidate);
	const bool bounded_finite =
		hydrofoil::HydrofoilAllocationModel::isFiniteBounded(input.allocation_context, solution);

	if (!bounded_finite) {
		// A finite solve that cannot produce a bounded candidate is a temporary
		// physical infeasibility in FIELD mode. Non-finite/invalid solver state
		// is structural and remains severe.
		severe_failure = !solution.status.finite;
		return false;
	}

	if (!solution.status.valid && !input.config.field_mode) {
		return false;
	}

	const float scale = input.allocation_context.max_hydrofoil_angle;
	float final_left_normalized = input.generic_servo5 + solution.delta_left_commanded / scale;
	float final_right_normalized = input.generic_servo4 + solution.delta_right_commanded / scale;

	if (!finite(final_left_normalized) || !finite(final_right_normalized)) {
		severe_failure = true;
		return false;
	}

	if (std::fabs(final_left_normalized) > 1.f || std::fabs(final_right_normalized) > 1.f) {
		if (!input.config.field_mode) {
			return false;
		}

		final_left_normalized = math::constrain(final_left_normalized, -1.f, 1.f);
		final_right_normalized = math::constrain(final_right_normalized, -1.f, 1.f);
		output.best_feasible_active = true;
	}

	float final_left = final_left_normalized * scale;
	float final_right = final_right_normalized * scale;

	if (enforce_foil_constraints) {
		const float absolute_limit = input.config.foil_angle_max - input.config.foil_remainder;
		const float rate_limit = input.config.foil_rate_max * dt + 1e-5f;
		const float previous_left = input.previous_final_servo5 * scale;
		const float previous_right = input.previous_final_servo4 * scale;

		if (std::fabs(final_left) > absolute_limit || std::fabs(final_right) > absolute_limit
		    || std::fabs(final_left - previous_left) > rate_limit
		    || std::fabs(final_right - previous_right) > rate_limit) {
			if (!input.config.field_mode) {
				return false;
			}

			const float left_lower = math::max(-absolute_limit, previous_left - rate_limit);
			const float left_upper = math::min(absolute_limit, previous_left + rate_limit);
			const float right_lower = math::max(-absolute_limit, previous_right - rate_limit);
			const float right_upper = math::min(absolute_limit, previous_right + rate_limit);
			final_left = left_lower <= left_upper
				     ? math::constrain(final_left, left_lower, left_upper)
				     : math::constrain(final_left, math::max(-scale, previous_left - rate_limit),
						       math::min(scale, previous_left + rate_limit));
			final_right = right_lower <= right_upper
				      ? math::constrain(final_right, right_lower, right_upper)
				      : math::constrain(final_right, math::max(-scale, previous_right - rate_limit),
							math::min(scale, previous_right + rate_limit));
			output.best_feasible_active = true;
		}
	}

	hydrofoil::AllocationSolution final_solution = solution;
	final_solution.delta_left_commanded = final_left;
	final_solution.delta_right_commanded = final_right;
	const hydrofoil::AllocationForwardResult forward =
		hydrofoil::HydrofoilAllocationModel::forward(input.allocation_context, final_solution);

	if (!forward.valid) {
		severe_failure = true;
		return false;
	}

	effective_model = forward.u_phi_effective_model;
	output.delta_left_commanded = final_left;
	output.delta_right_commanded = final_right;
	output.foil_rate_left_commanded =
		(final_left - input.previous_final_servo5 * scale) / dt;
	output.foil_rate_right_commanded =
		(final_right - input.previous_final_servo4 * scale) / dt;
	output.thrust_left_commanded = solution.thrust_left_commanded;
	output.thrust_right_commanded = solution.thrust_right_commanded;
	output.best_feasible_active = output.best_feasible_active || !solution.status.valid
				      || solution.status.saturation_mask != hydrofoil::SaturationNone;
	severe_failure = !finite(effective_model);
	return !severe_failure;
}

float SafeRollController::validatedCandidate(const Input &input, float requested_candidate, float dt,
		bool enforce_foil_constraints, Output &output, bool &valid, bool &severe_failure) const
{
	severe_failure = false;
	float effective = 0.f;
	bool requested_severe = false;

	if (checkNonlinearCandidate(input, requested_candidate, dt, enforce_foil_constraints, output, effective,
				    requested_severe)) {
		output.u_effective_model = effective;
		valid = true;
		return requested_candidate;
	}

	if (requested_severe) {
		severe_failure = true;
		valid = false;
		return _last_selected;
	}

	float low = _last_selected;
	float low_effective = 0.f;
	Output low_output = output;
	bool low_severe = false;

	if (!checkNonlinearCandidate(input, low, dt, enforce_foil_constraints, low_output, low_effective, low_severe)) {
		severe_failure = low_severe;
		valid = false;
		return _last_selected;
	}

	float high = requested_candidate;

	for (int i = 0; i < kBisectionIterations; ++i) {
		const float mid = 0.5f * (low + high);
		float mid_effective = 0.f;
		Output mid_output = output;
		bool mid_severe = false;

		if (checkNonlinearCandidate(input, mid, dt, enforce_foil_constraints, mid_output, mid_effective,
					    mid_severe)) {
			low = mid;
			low_effective = mid_effective;
			low_output = mid_output;

		} else if (mid_severe) {
			severe_failure = true;
			valid = false;
			return _last_selected;

		} else {
			high = mid;
		}
	}

	output = low_output;
	output.u_effective_model = low_effective;
	output.best_feasible_active = true;
	valid = true;
	return low;
}

void SafeRollController::beginBlend(uint8_t target_code, uint64_t timestamp)
{
	_blend_target_code = target_code;
	_blend_start_output = _last_selected;
	_blend_start_timestamp = timestamp;
}

float SafeRollController::blendTo(float target, const Config &config, uint64_t timestamp, const Interval &hard)
{
	const float elapsed = timestamp >= _blend_start_timestamp
			      ? static_cast<float>(timestamp - _blend_start_timestamp) * 1e-6f : 0.f;
	const float lambda = math::constrain(elapsed / config.blend_time, 0.f, 1.f);
	const float blended = (1.f - lambda) * _blend_start_output + lambda * target;
	return finiteInterval(hard) ? project(blended, hard) : blended;
}

float SafeRollController::fallbackOutput(const Input &input, float dt)
{
	const float step = input.config.du_max_nominal * dt / kNominalDt;
	const float current = finite(_last_selected) ? _last_selected : 0.f;
	const float legacy = finite(input.legacy_u_roll) ? input.legacy_u_roll : current;
	const float rate_limited_legacy = math::constrain(legacy, current - step, current + step);

	if (_blend_target_code != 0) {
		beginBlend(0, input.angular_velocity_timestamp_sample);
	}

	const float blended = blendTo(rate_limited_legacy, input.config, input.angular_velocity_timestamp_sample,
				      Interval{-input.config.u_max, input.config.u_max, true});
	return math::constrain(blended, current - step, current + step);
}

float SafeRollController::holdLastFeasibleOutput(const Input &input, float dt) const
{
	const float step = input.config.du_max_nominal * dt / kNominalDt;
	const float current = finite(_last_selected) ? _last_selected : 0.f;
	const float target = _have_last_feasible && finite(_last_feasible_u) ? _last_feasible_u : current;
	return math::constrain(math::constrain(target, -input.config.u_max, input.config.u_max),
			       current - step, current + step);
}

SafeRollController::Output SafeRollController::evaluate(const Input &input, bool direct_legacy)
{
#ifndef SAFE_ROLL_DISABLE_PERF
	perf_begin(_cycle_perf);
#endif
	Output output{};
	output.requested_code = input.requested_mode;
	output.trial_id = input.config.trial_id;
	output.configuration_crc32 = configurationCrc32(input.requested_mode, input.config);
	output.reset_count = _reset_count;
	output.u_legacy = input.legacy_u_roll;
	output.u_selected = input.legacy_u_roll;
	output.u_blended = input.legacy_u_roll;
	output.manual_roll = input.manual_roll;
	output.phi_cmd = input.phi_cmd;
	output.phi = input.phi;
	output.p = input.p;
	output.safe_roll_hardware_ready = hardwareReady(input);
	output.field_mode_active = input.config.field_mode
				   && input.requested_mode >= static_cast<uint8_t>(Mode::Nominal)
				   && input.requested_mode <= static_cast<uint8_t>(Mode::Adaptive);
	output.runtime_soft_budget_us = kRuntimeSoftBudgetUs;
	output.runtime_hard_budget_us = kRuntimeHardBudgetUs;
	output.activation_required_streak = activationRequiredStreak();

	if (direct_legacy) {
		if (_initialized && _last_requested_mode != 0) {
			_fallback_latched = false;
			_first_fault_code = 0;
		_first_fault_stage_mask = 0;
		}

		output.result = Result::LegacyDirect;
		output.active_code = 0;
		output.healthy = true;
		_last_selected = input.legacy_u_roll;
		_last_sample_timestamp = input.angular_velocity_timestamp_sample;
		_last_requested_mode = 0;
		_blend_target_code = 0;
		_activation_pending = false;
		_mode_activated = false;
		_allocator_valid_streak = 0;
		_allocator_invalid_streak = 0;
		_allocator_recovery_streak = 0;
		_aux_gate_low_streak = 0;
		_aux_gate_high_streak = 0;
		_allocator_invalid_start_timestamp = 0;
		_allocator_invalid_duration_us = 0;
		_allocator_recovery_pending = false;
		_allocator_degraded_active = false;
		_temporary_fallback_active = false;
		_best_feasible_active = false;
		_aux_gate_inhibit_active = false;
		_have_last_feasible = false;
		_initialized = true;
#ifndef SAFE_ROLL_DISABLE_PERF
		perf_end(_cycle_perf);
#endif
		return output;
	}

	const uint32_t reset_configuration_crc = resetConfigurationCrc32(input.config);
	const bool transition = !_initialized || input.requested_mode != _last_requested_mode
				|| input.config.enable != _last_enable
				|| input.config.reset_id != _last_reset_id
				|| input.attitude_reset_counter != _last_attitude_reset_counter
				|| input.in_stab != _last_in_stab || input.armed != _last_armed
				|| reset_configuration_crc != _last_reset_configuration_crc;
	const bool explicit_latch_clear = (!_initialized && !_fallback_latched)
					  || input.requested_mode != _last_requested_mode
					  || input.config.reset_id != _last_reset_id;

	bool activation_managed = input.requested_mode >= static_cast<uint8_t>(Mode::Nominal)
				  && input.requested_mode <= static_cast<uint8_t>(Mode::Adaptive);
#ifdef SAFE_ROLL_DISABLE_PERF
	activation_managed = activation_managed && !input.test_activation_bypass;
#endif

	if (transition) {
		resetNewController(input, true, explicit_latch_clear);
		_activation_pending = activation_managed;
		_mode_activated = !activation_managed;
		beginBlend(_activation_pending || input.config.shadow ? 0 : input.requested_mode,
			   input.angular_velocity_timestamp_sample);

	} else if (input.config.shadow != _last_shadow) {
		beginBlend(_activation_pending || input.config.shadow ? 0 : input.requested_mode,
			   input.angular_velocity_timestamp_sample);
	}

	const bool fixed_mode = input.requested_mode == static_cast<uint8_t>(Mode::WorstCaseFixed);
	output.first_fault_code = _first_fault_code;
	output.b_hat = fixed_mode ? input.config.b0 : _b_hat;
	output.b_lower = fixed_mode ? input.config.b_min : _b_lower;
	output.b_upper = fixed_mode ? input.config.b_max : _b_upper;
	output.rho_b = fixed_mode ? math::max(input.config.b0 - input.config.b_min,
					     input.config.b_max - input.config.b0)
		       : math::max(std::fabs(_b_hat - _b_lower), std::fabs(_b_upper - _b_hat));
	output.certified_width = output.b_upper - output.b_lower;
	output.d_hat = _d_hat;
	output.rho_d = _rho_d;
	output.filtered_u = _filtered_u;
	output.filtered_r = _filtered_r;
	output.first_fault_stage_mask = _first_fault_stage_mask;
	output.first_fault_total_candidate_time_us = _first_fault_total_candidate_time_us;
	output.first_fault_reference_filter_time_us = _first_fault_reference_filter_time_us;
	output.first_fault_estimator_time_us = _first_fault_estimator_time_us;
	output.first_fault_certificate_time_us = _first_fault_certificate_time_us;
	output.first_fault_allocation_query_time_us = _first_fault_allocation_query_time_us;
	output.first_fault_nominal_time_us = _first_fault_nominal_time_us;
	output.first_fault_cbf_time_us = _first_fault_cbf_time_us;
	output.first_fault_nonlinear_check_time_us = _first_fault_nonlinear_check_time_us;
	output.first_fault_snapshot = _first_fault_snapshot;
	output.runtime_soft_budget_us = kRuntimeSoftBudgetUs;
	output.runtime_hard_budget_us = kRuntimeHardBudgetUs;
	output.consecutive_runtime_overrun_count = _consecutive_runtime_overruns;
	output.activation_required_streak = activationRequiredStreak();

	// Candidate WCET starts after configuration CRC, transition detection and
	// one-shot reset/blend initialization. It covers validation and all candidate
	// stages, but not uORB publication or downstream ControlAllocator execution.
	const hrt_abstime candidate_start = timingNow();

	const bool first_sample = _last_sample_timestamp == 0;

	if (!first_sample) {
		if (input.angular_velocity_timestamp_sample <= _last_sample_timestamp) {
			output.fault_flags |= TimestampNonMonotonic;

		} else {
			output.dt_raw = static_cast<float>(input.angular_velocity_timestamp_sample - _last_sample_timestamp) * 1e-6f;

			if (output.dt_raw < kDtMin) {
				output.fault_flags |= DtTooSmall;

			} else if (output.dt_raw > kDtMax) {
				output.fault_flags |= DtTooLarge;
			}
		}
	} else {
		output.fault_flags |= DtTooSmall;
	}

	if (input.angular_velocity_timestamp_sample == 0 || input.now < input.angular_velocity_timestamp_sample
	    || input.now - input.angular_velocity_timestamp_sample > kGyroStaleUs) {
		output.fault_flags |= GyroStale;
	}

	if (input.attitude_timestamp == 0 || input.now < input.attitude_timestamp
	    || input.now - input.attitude_timestamp > kAttitudeStaleUs) {
		output.fault_flags |= AttitudeStale;
	}

	if (!input.quaternion_finite) {
		output.fault_flags |= QuaternionNonFinite;
	}

	if (!input.angular_velocity_finite || !finite(input.phi) || !finite(input.p)
	    || !finite(input.phi_cmd) || !finite(input.legacy_u_roll)) {
		output.fault_flags |= AngularVelocityNonFinite;
	}

	if (input.requested_mode < 1 || input.requested_mode > 4) {
		output.fault_flags |= ModeOutOfRange;
	}

	if (!validateConfig(input.config)) {
		output.fault_flags |= InvalidConfigurationFault;
	}

	if (!input.in_stab) {
		output.fault_flags |= ControllerDisabledFault;
	}

	if (!input.logical_mapping_valid) {
		output.fault_flags |= LogicalMappingInvalidFault;
	}

	const bool mode_allowed = input.config.enable > 0
				  && (input.requested_mode <= 3 || input.config.enable == 2);

	if (!mode_allowed) {
		output.fault_flags |= ControllerDisabledFault;
	}

	const bool allocator_timestamp_valid = input.allocator_timestamp != 0
					       && input.now >= input.allocator_timestamp
					       && input.now - input.allocator_timestamp <= 100'000;
	const bool field_mode = input.config.field_mode && activation_managed;
	const bool aux_gate_finite_valid = input.aux_gate_data_valid && input.aux_gate_value_finite
					   && finite(input.foil_aux_value)
					   && finite(input.foil_threshold_parameter);
	const bool field_aux_gate_inhibit = field_mode && aux_gate_finite_valid
					    && input.nav_gate_enabled_raw
					    && !input.aux_gate_enabled_raw;
	const bool allocator_strict_healthy = input.allocator_valid && input.allocator_converged
					      && input.allocator_finite
					      && allocator_timestamp_valid && input.foil_gate_enabled;
	const bool allocator_bounded_finite = field_mode && input.allocator_finite && input.allocator_bounded
					      && allocator_timestamp_valid && input.foil_gate_enabled;
	const bool allocator_best_feasible = allocator_bounded_finite
					     && (!input.allocator_valid || !input.allocator_converged);
	const bool allocator_activation_sample_valid = allocator_strict_healthy || allocator_best_feasible;
	bool hold_last_feasible = false;
	bool use_temporary_legacy = false;
	_best_feasible_active = (allocator_best_feasible && !allocator_strict_healthy)
				|| (field_mode && input.allocator_saturation_mask != hydrofoil::SaturationNone);
	auto enter_field_candidate_unavailable = [&]() {
		_allocator_recovery_pending = true;
		_allocator_recovery_streak = 0;
		_allocator_invalid_streak =
			math::min<uint16_t>(_allocator_invalid_streak + 1, allocatorInvalidStreakLimit());
		_best_feasible_active = false;
		output.best_feasible_active = false;

		if (_allocator_invalid_streak <= allocatorHoldStreakLimit() && !_temporary_fallback_active) {
			_allocator_degraded_active = true;
			hold_last_feasible = true;

		} else {
			_allocator_degraded_active = false;
			_temporary_fallback_active = true;
			hold_last_feasible = false;
			use_temporary_legacy = true;
		}
	};

	if (activation_managed) {
		if (_activation_pending) {
			_allocator_invalid_streak = 0;
			_allocator_recovery_streak = 0;
			_allocator_invalid_duration_us = 0;
			_allocator_invalid_start_timestamp = 0;
			_allocator_recovery_pending = false;
			_allocator_degraded_active = false;
			_temporary_fallback_active = false;
			_aux_gate_inhibit_active = field_aux_gate_inhibit;

			if (field_aux_gate_inhibit) {
				_aux_gate_low_streak =
					math::min<uint16_t>(_aux_gate_low_streak + 1, UINT16_MAX);
				_aux_gate_high_streak = 0;

			} else if (input.aux_gate_enabled_raw) {
				_aux_gate_low_streak = 0;
			}

			if (allocator_activation_sample_valid) {
				if (_allocator_valid_streak < activationRequiredStreak()) {
					++_allocator_valid_streak;
				}

			} else {
				_allocator_valid_streak = 0;
			}

			if (_allocator_valid_streak >= activationRequiredStreak()) {
				_activation_pending = false;
				_mode_activated = true;
				beginBlend(input.config.shadow ? 0 : input.requested_mode,
					   input.angular_velocity_timestamp_sample);
			}

		} else if (_mode_activated) {
			if (!input.allocator_finite || !allocator_timestamp_valid) {
				output.fault_flags |= AllocatorInvalidFault;

			} else if (!aux_gate_finite_valid || !input.nav_gate_enabled_raw) {
				output.fault_flags |= FoilGateDisabledFault;

			} else if (field_aux_gate_inhibit) {
				_aux_gate_inhibit_active = true;
				_aux_gate_low_streak =
					math::min<uint16_t>(_aux_gate_low_streak + 1, UINT16_MAX);
				_aux_gate_high_streak = 0;
				_allocator_recovery_pending = true;
				_allocator_recovery_streak = 0;
				_allocator_degraded_active = false;
				_temporary_fallback_active = true;
				use_temporary_legacy = true;

			} else if (!input.foil_gate_enabled) {
				// FIELD mode only treats a finite AUX threshold closure as a
				// recoverable operator inhibit. Nav closure and all strict-mode
				// gate closures retain the existing severe behavior.
				output.fault_flags |= FoilGateDisabledFault;

			} else if (allocator_best_feasible) {
				_allocator_recovery_streak = 0;
				_allocator_degraded_active = true;
				_allocator_invalid_duration_us = 0;
				_allocator_invalid_start_timestamp = 0;

				if (_allocator_recovery_pending) {
					if (_temporary_fallback_active) {
						use_temporary_legacy = true;

					} else {
						hold_last_feasible = true;
					}

				} else {
					_allocator_invalid_streak = 0;
					_temporary_fallback_active = false;
				}

			} else if (!input.allocator_valid || !input.allocator_converged) {
				_allocator_valid_streak = 0;
				_allocator_recovery_streak = 0;
				_allocator_recovery_pending = true;

				if (_allocator_invalid_streak == 0) {
					_allocator_invalid_start_timestamp = input.angular_velocity_timestamp_sample;
				}

				if (_allocator_invalid_streak < allocatorInvalidStreakLimit()) {
					++_allocator_invalid_streak;
				}

				if (input.angular_velocity_timestamp_sample >= _allocator_invalid_start_timestamp) {
					const uint64_t elapsed = input.angular_velocity_timestamp_sample
							 - _allocator_invalid_start_timestamp;
					const uint64_t sample_us =
						input.angular_velocity_timestamp_sample > _last_sample_timestamp
						? input.angular_velocity_timestamp_sample - _last_sample_timestamp : 0;
					_allocator_invalid_duration_us = static_cast<uint32_t>(
						math::min(elapsed + sample_us, static_cast<uint64_t>(UINT32_MAX)));
				}

				if (!field_mode && _allocator_invalid_streak >= allocatorInvalidStreakLimit()) {
					output.fault_flags |= AllocatorInvalidFault;

				} else if (_allocator_invalid_streak <= allocatorHoldStreakLimit()
					   && !_temporary_fallback_active) {
					_allocator_degraded_active = true;
					hold_last_feasible = true;

				} else {
					_allocator_degraded_active = false;
					_temporary_fallback_active = true;
					use_temporary_legacy = true;
				}

			} else if (_allocator_recovery_pending) {
				_allocator_valid_streak = 0;

				if (_aux_gate_inhibit_active && input.aux_gate_enabled_raw) {
					_aux_gate_low_streak = 0;
				}

				// External allocator health only permits a recovery probe. The
				// streak is advanced below after the full candidate, projection
				// and nonlinear validation chain succeeds.
				if (_temporary_fallback_active) {
					use_temporary_legacy = true;

				} else {
					_allocator_degraded_active = true;
					hold_last_feasible = true;
				}

			} else {
				_allocator_invalid_streak = 0;
				_allocator_recovery_streak = 0;
				_allocator_valid_streak = activationRequiredStreak();
				_allocator_invalid_duration_us = 0;
				_allocator_invalid_start_timestamp = 0;
				_allocator_degraded_active = false;
				_temporary_fallback_active = false;
				_aux_gate_low_streak = 0;
				_aux_gate_high_streak = 0;
				_aux_gate_inhibit_active = false;
			}
		}

	} else {
		if (!input.foil_gate_enabled) {
			output.fault_flags |= FoilGateDisabledFault;
		}

		if (!input.allocator_valid || !input.allocator_converged || !input.allocator_finite
		    || !allocator_timestamp_valid) {
			output.fault_flags |= AllocatorInvalidFault;
		}
	}

	output.activation_pending = _activation_pending;
	output.allocator_valid_streak = _allocator_valid_streak;
	output.allocator_invalid_streak = _allocator_invalid_streak;
	output.allocator_recovery_streak = _allocator_recovery_streak;
	output.allocator_invalid_duration_us = _allocator_invalid_duration_us;
	output.allocator_degraded_active = _allocator_degraded_active;
	output.temporary_fallback_active = _temporary_fallback_active;
	output.aux_gate_inhibit_active = _aux_gate_inhibit_active;
	output.aux_gate_low_streak = _aux_gate_low_streak;
	output.aux_gate_high_streak = _aux_gate_high_streak;
	output.best_feasible_active = _best_feasible_active;
	output.safety_degraded_active =
		(input.requested_mode == static_cast<uint8_t>(Mode::WorstCaseFixed)
		 || input.requested_mode == static_cast<uint8_t>(Mode::Adaptive))
		&& (_allocator_degraded_active || _temporary_fallback_active || _best_feasible_active
		    || _aux_gate_inhibit_active);
	output.last_feasible_u = _last_feasible_u;

	const uint32_t prerequisite_faults = TimestampNonMonotonic | DtTooSmall | DtTooLarge | GyroStale
					     | AttitudeStale | QuaternionNonFinite | AngularVelocityNonFinite
					     | AllocatorInvalidFault | FoilGateDisabledFault | ModeOutOfRange
					     | LogicalMappingInvalidFault | InvalidConfigurationFault
					     | ControllerDisabledFault;
	output.dt_used = output.dt_raw >= kDtMin && output.dt_raw <= kDtMax ? output.dt_raw : 0.f;

	const bool recovery_probe = activation_managed && _allocator_recovery_pending
				    && allocator_activation_sample_valid;
	const bool candidate_permitted = !_activation_pending
					 && ((!hold_last_feasible && !use_temporary_legacy) || recovery_probe);

	if (candidate_permitted && (output.fault_flags & prerequisite_faults) == 0 && !_fallback_latched) {
		const hrt_abstime reference_start = timingNow();
		updateReference(input, output, output.dt_used);
		output.reference_filter_time_us = timingElapsed(reference_start);
		output.execution_stage_mask |= StageReferenceFilter;

		if (!updateEstimators(input, output, output.dt_used)) {
			const bool certificate_state_finite =
				finite(output.b_hat) && finite(output.b_lower) && finite(output.b_upper)
				&& finite(output.rho_b) && finite(output.d_hat) && finite(output.rho_d);

			if (field_mode && certificate_state_finite) {
				enter_field_candidate_unavailable();

			} else {
				output.fault_flags |= InvalidCertificateFault;
			}

		} else {
			output.u_amp = {-input.config.u_max, input.config.u_max, true};
			const float du_step = input.config.du_max_nominal * output.dt_used / kNominalDt;
			output.u_rate = {_last_selected - du_step, _last_selected + du_step, true};
			output.u_hard = intersect(output.u_amp, output.u_rate);
			const bool use_allocation = input.requested_mode != static_cast<uint8_t>(Mode::NoAllocationAblation);
			const bool enforce_foil_constraints =
				input.requested_mode != static_cast<uint8_t>(Mode::Nominal) && use_allocation;
			hydrofoil::AllocationSolution base_solution{};
			hydrofoil::AllocationLinearization linearization{};
			const hrt_abstime allocation_start = timingNow();

			if (use_allocation) {
				output.u_alloc = allocationInterval(input, input.config, output.dt_used, base_solution, linearization);

			} else {
				output.u_alloc = output.u_amp;
			}

			output.u_hard = combineAllocationInterval(input.requested_mode, output.u_hard,
						output.u_alloc, use_allocation, output.allocation_interval_degraded,
						field_mode);
			output.allocation_query_time_us = timingElapsed(allocation_start);
			output.execution_stage_mask |= StageAllocationQuery;

			if (!finiteInterval(output.u_hard)) {
				const bool finite_hard_failure = finite(output.u_hard.lower) && finite(output.u_hard.upper)
							 && finite(output.u_amp.lower) && finite(output.u_amp.upper)
							 && finite(output.u_rate.lower) && finite(output.u_rate.upper);

				if (field_mode && finite_hard_failure) {
					enter_field_candidate_unavailable();

				} else {
					output.fault_flags |= EmptyHardIntervalFault;
				}

			} else {
				const hrt_abstime nominal_start = timingNow();
				const float control_b = fixed_mode ? input.config.b0 : output.b_hat;
				const float e1 = output.phi_ref - input.phi;
				const float e2 = output.phi_ref_dot - input.p;
				output.u_nominal = nominalControl(input.config.a0, input.p, output.d_hat,
								 output.phi_ref_ddot, input.config.k1, e1,
								 input.config.k2, e2, control_b);
				output.nominal_time_us = timingElapsed(nominal_start);
				output.execution_stage_mask |= StageNominalControl;

				if (!finite(output.u_nominal)) {
					output.fault_flags |= ProjectionInvalidFault;

				} else {
					output.u_safe = output.u_hard;
					const hrt_abstime cbf_start = timingNow();

						if (input.requested_mode != static_cast<uint8_t>(Mode::Nominal)) {
						const float cbf_b = fixed_mode ? input.config.b0 : output.b_hat;
						const float cbf_rho = fixed_mode
								      ? math::max(input.config.b0 - input.config.b_min,
										  input.config.b_max - input.config.b0)
								      : output.rho_b;
						output.u_cbf = robustCbfInterval(input.config.phi_safe, input.phi, input.p,
									       output.dt_used, input.config.a0, output.d_hat,
									       output.rho_d, cbf_b, cbf_rho,
									       input.config.gamma1, input.config.gamma2,
									       input.config.robust_gain_floor,
									       output.h_upper, output.h_lower);
						const Interval strict = intersect(output.u_cbf, output.u_hard);

						if (finiteInterval(strict)) {
							output.u_safe = strict;

						} else {
							output.epsilon_relax = minimumRelaxation(output.u_cbf, output.u_hard);
							const Interval relaxed{output.u_cbf.lower - output.epsilon_relax,
									       output.u_cbf.upper + output.epsilon_relax, true};
								output.u_safe = intersect(relaxed, output.u_hard);
							}

							output.cbf_time_us = timingElapsed(cbf_start);
							output.execution_stage_mask |= StageCbf;
						}

						output.interval_valid = finiteInterval(output.u_safe);

					if (!output.interval_valid) {
						const bool finite_interval_conflict =
							output.u_cbf.valid && finite(output.u_cbf.lower)
							&& finite(output.u_cbf.upper) && finite(output.epsilon_relax)
							&& finiteInterval(output.u_hard);

						if (field_mode && finite_interval_conflict) {
							// The CBF and actuator intervals are both finite, but no
							// strictly recertified point survived their intersection.
							// FIELD mode keeps the immutable hard constraints and marks
							// the bounded command as not strictly HOCBF-certified.
							output.u_safe = output.u_hard;
							output.interval_valid = true;
							output.best_feasible_active = true;

						} else {
							output.fault_flags |= ProjectionInvalidFault;
						}
					}

					if (output.interval_valid
					    && (output.fault_flags & ProjectionInvalidFault) == 0) {
						const float projected = project(output.u_nominal, output.u_safe);
						bool nonlinear_valid = false;
						bool nonlinear_severe = false;
						const hrt_abstime nonlinear_start = timingNow();
						output.execution_stage_mask |= StageNonlinearCheck;
						output.u_candidate = validatedCandidate(input, projected, output.dt_used,
										      enforce_foil_constraints, output, nonlinear_valid,
										      nonlinear_severe);
						output.candidate_valid = nonlinear_valid;

						if (!nonlinear_valid) {
							if (field_mode && !nonlinear_severe) {
								enter_field_candidate_unavailable();

							} else {
								output.fault_flags |= NonlinearCandidateInvalidFault;
							}

						} else {
							const uint8_t blend_target_code = input.config.shadow ? 0 : input.requested_mode;

							if (_blend_target_code != blend_target_code) {
								beginBlend(blend_target_code, input.angular_velocity_timestamp_sample);
							}

							const float blend_target = input.config.shadow
										 ? input.legacy_u_roll : output.u_candidate;
							output.u_blended = blendTo(blend_target, input.config,
										input.angular_velocity_timestamp_sample, output.u_hard);
							float blended_effective = 0.f;
							Output blended_output = output;
							bool blended_severe = false;
							const bool blended_valid =
								checkNonlinearCandidate(input, output.u_blended, output.dt_used,
									enforce_foil_constraints, blended_output,
									blended_effective, blended_severe);

							if (!blended_valid) {
								if (field_mode && !blended_severe) {
									enter_field_candidate_unavailable();

								} else {
									output.fault_flags |= NonlinearCandidateInvalidFault;
								}

							} else {
								output.delta_left_commanded = blended_output.delta_left_commanded;
								output.delta_right_commanded = blended_output.delta_right_commanded;
								output.foil_rate_left_commanded = blended_output.foil_rate_left_commanded;
								output.foil_rate_right_commanded = blended_output.foil_rate_right_commanded;
								output.thrust_left_commanded = blended_output.thrust_left_commanded;
								output.thrust_right_commanded = blended_output.thrust_right_commanded;
								output.u_effective_model = blended_effective;
								output.best_feasible_active =
									output.best_feasible_active
									|| blended_output.best_feasible_active;

								if (recovery_probe) {
									// A complete bounded candidate (including a
									// documented best-feasible one) is sufficient
									// to resume FIELD operation. It does not claim
									// strict HOCBF recertification when degraded.
									if (output.candidate_valid && allocator_strict_healthy) {
										if (_allocator_recovery_streak
										    < allocatorRecoveryRequiredStreak()) {
											++_allocator_recovery_streak;
										}

										if (_aux_gate_inhibit_active
										    && _aux_gate_high_streak < allocatorRecoveryRequiredStreak()) {
											++_aux_gate_high_streak;
										}

										if (_allocator_recovery_streak
										    >= allocatorRecoveryRequiredStreak()
										    && (!_aux_gate_inhibit_active
											|| _aux_gate_high_streak
											   >= allocatorRecoveryRequiredStreak())) {
											_allocator_recovery_pending = false;
											_allocator_degraded_active = false;
											_temporary_fallback_active = false;
											_aux_gate_inhibit_active = false;
											_aux_gate_low_streak = 0;
											_allocator_invalid_streak = 0;
											_allocator_invalid_duration_us = 0;
											_allocator_invalid_start_timestamp = 0;
											_allocator_valid_streak = activationRequiredStreak();
											hold_last_feasible = false;
											use_temporary_legacy = false;
											beginBlend(input.config.shadow ? 0 : input.requested_mode,
												input.angular_velocity_timestamp_sample);
										}

									} else {
										_allocator_recovery_streak = 0;

										if (_aux_gate_inhibit_active) {
											_aux_gate_high_streak = 0;
										}
									}
								}

								const float cbf_b = fixed_mode ? input.config.b0 : output.b_hat;
								const float cbf_rho = fixed_mode
										      ? math::max(input.config.b0 - input.config.b_min,
												  input.config.b_max - input.config.b0)
										      : output.rho_b;
								const float dt2 = output.dt_used * output.dt_used;
								const float beta_upper = input.config.gamma1 * input.config.gamma2
										 * (input.config.phi_safe - input.phi)
										 - (input.config.gamma1 + input.config.gamma2)
										 * output.dt_used * input.p
										 - dt2 * (input.config.a0 * input.p + output.d_hat);
								const float beta_lower = input.config.gamma1 * input.config.gamma2
										 * (input.config.phi_safe + input.phi)
										 + (input.config.gamma1 + input.config.gamma2)
										 * output.dt_used * input.p
										 + dt2 * (input.config.a0 * input.p + output.d_hat);
								output.h_upper = input.config.phi_safe - input.phi;
								output.h_lower = input.config.phi_safe + input.phi;
								output.psi_upper = beta_upper - dt2 * cbf_b * output.u_blended
										   - dt2 * (output.rho_d + cbf_rho * std::fabs(output.u_blended));
								output.psi_lower = beta_lower + dt2 * cbf_b * output.u_blended
										   - dt2 * (output.rho_d + cbf_rho * std::fabs(output.u_blended));
								output.psi_min = math::min(output.psi_upper, output.psi_lower);

								if (!finite(output.u_blended) || !finite(output.psi_min)) {
									output.fault_flags |= ProjectionInvalidFault;

								} else {
									output.active_code = input.requested_mode;
									output.shadow_active = input.config.shadow;
									output.output_enabled = !input.config.shadow;
									output.u_selected = output.u_blended;
									output.result = input.config.shadow ? Result::Shadow : Result::Active;
									output.execution_stage_mask |= StageOutputSelected;
								}
							}
						}

						output.nonlinear_check_time_us = timingElapsed(nonlinear_start);
					}
				}
			}
		}
	}

	if (output.certificate_conflict) {
		output.fault_flags |= CertificateConflictFault;
	}

	output.total_candidate_time_us = timingElapsed(candidate_start);
#ifdef SAFE_ROLL_DISABLE_PERF
	if (input.test_stage_time_us > 0) {
		output.reference_filter_time_us =
			(output.execution_stage_mask & StageReferenceFilter) ? input.test_stage_time_us : 0;
		output.estimator_time_us =
			(output.execution_stage_mask & StageEstimator) ? input.test_stage_time_us : 0;
		output.certificate_time_us =
			(output.execution_stage_mask & StageCertificate) ? input.test_stage_time_us : 0;
		output.allocation_query_time_us =
			(output.execution_stage_mask & StageAllocationQuery) ? input.test_stage_time_us : 0;
		output.nominal_time_us =
			(output.execution_stage_mask & StageNominalControl) ? input.test_stage_time_us : 0;
		output.cbf_time_us =
			(output.execution_stage_mask & StageCbf) ? input.test_stage_time_us : 0;
		output.nonlinear_check_time_us =
			(output.execution_stage_mask & StageNonlinearCheck) ? input.test_stage_time_us : 0;
	}

	if (input.test_candidate_time_us > 0) {
		output.total_candidate_time_us = input.test_candidate_time_us;
	}
#endif

	if (!_fallback_latched) {
		const bool runtime_exceeded =
			runtimeBudgetExceeded(output.total_candidate_time_us, transition, _consecutive_runtime_overruns);
		const bool hard_runtime_exceeded = output.total_candidate_time_us > kRuntimeHardBudgetUs;

		// FIELD operation records repeated soft-budget pressure but reserves the
		// severe latch for an actual 2.5 ms hard deadline miss. Strict operation
		// retains the established five-consecutive-soft-overrun policy.
		if (runtime_exceeded && (!field_mode || hard_runtime_exceeded)) {
			output.fault_flags |= RuntimeOverrunFault;
			output.result = Result::RuntimeOverrun;
		}
	}

	output.consecutive_runtime_overrun_count = _consecutive_runtime_overruns;
	const uint32_t fatal_faults = prerequisite_faults | InvalidCertificateFault | EmptyHardIntervalFault
				      | NonlinearCandidateInvalidFault | ProjectionInvalidFault | RuntimeOverrunFault;

	if (_activation_pending && (output.fault_flags & fatal_faults) == 0 && !_fallback_latched) {
		output.fallback_active = true;
		output.output_enabled = false;
		output.active_code = 0;
		output.candidate_valid = false;
		output.u_selected = fallbackOutput(input, output.dt_used > 0.f ? output.dt_used : kNominalDt);
		output.u_blended = output.u_selected;
		output.result = Result::Disabled;
	}

	if (hold_last_feasible && !use_temporary_legacy
	    && (output.fault_flags & fatal_faults) == 0 && !_fallback_latched) {
		_temporary_fallback_active = false;
		output.active_code = input.requested_mode;
		output.shadow_active = input.config.shadow;
		output.output_enabled = !input.config.shadow;
		output.candidate_valid = false;
		output.u_selected = input.config.shadow ? input.legacy_u_roll
				    : holdLastFeasibleOutput(input, output.dt_used > 0.f ? output.dt_used : kNominalDt);
		output.u_blended = output.u_selected;
		output.result = input.config.shadow ? Result::Shadow : Result::Active;
	}

	if (use_temporary_legacy && (output.fault_flags & fatal_faults) == 0 && !_fallback_latched) {
		_temporary_fallback_active = true;
		_best_feasible_active = false;
		output.best_feasible_active = false;
		output.fallback_active = true;
		output.output_enabled = false;
		output.active_code = 0;
		output.candidate_valid = false;
		output.u_selected = fallbackOutput(input, output.dt_used > 0.f ? output.dt_used : kNominalDt);
		output.u_blended = output.u_selected;
		output.result = Result::AllocatorInvalid;
	}

	if ((output.fault_flags & fatal_faults) != 0 || _fallback_latched) {
		if ((output.fault_flags & fatal_faults) != 0 && !first_sample) {
			if (!_fallback_latched) {
				_first_fault_code = firstSetBit(output.fault_flags & fatal_faults);
				_first_fault_stage_mask = output.execution_stage_mask;
				_first_fault_total_candidate_time_us = output.total_candidate_time_us;
				_first_fault_reference_filter_time_us = output.reference_filter_time_us;
				_first_fault_estimator_time_us = output.estimator_time_us;
				_first_fault_certificate_time_us = output.certificate_time_us;
				_first_fault_allocation_query_time_us = output.allocation_query_time_us;
				_first_fault_nominal_time_us = output.nominal_time_us;
				_first_fault_cbf_time_us = output.cbf_time_us;
				_first_fault_nonlinear_check_time_us = output.nonlinear_check_time_us;
				_first_fault_snapshot.prerequisite_fault_flags =
					output.fault_flags & prerequisite_faults;
				_first_fault_snapshot.allocator_timestamp = input.allocator_timestamp;
				_first_fault_snapshot.allocator_age_us =
					input.allocator_timestamp != 0 && input.now >= input.allocator_timestamp
					? static_cast<uint32_t>(math::min(input.now - input.allocator_timestamp,
								 static_cast<uint64_t>(UINT32_MAX)))
					: UINT32_MAX;
				_first_fault_snapshot.reset_count = _reset_count;
				_first_fault_snapshot.requested_mode = input.requested_mode;
				_first_fault_snapshot.attitude_reset_counter = input.attitude_reset_counter;
				_first_fault_snapshot.history_count =
					static_cast<uint8_t>(math::constrain(_history_count, 0, HistoryLength));
				_first_fault_snapshot.activation_pending = _activation_pending;
				_first_fault_snapshot.mode_activated = _mode_activated;
				_first_fault_snapshot.in_stab = input.in_stab;
				_first_fault_snapshot.allocator_timestamp_valid = allocator_timestamp_valid;
				_first_fault_snapshot.allocator_valid = input.allocator_valid;
				_first_fault_snapshot.allocator_converged = input.allocator_converged;
				_first_fault_snapshot.allocator_finite = input.allocator_finite;
				_first_fault_snapshot.allocator_bounded = input.allocator_bounded;
				_first_fault_snapshot.foil_gate_enabled = input.foil_gate_enabled;
				_first_fault_snapshot.foil_aux_above_threshold =
					finite(input.foil_aux_value) && finite(input.foil_threshold_parameter)
					&& input.foil_aux_value > input.foil_threshold_parameter;
				_first_fault_snapshot.logical_mapping_valid = input.logical_mapping_valid;
				_first_fault_snapshot.estimator_initialized = _have_previous_p;
				_first_fault_snapshot.foil_aux_parameter = input.foil_aux_parameter;
				_first_fault_snapshot.foil_aux_value = input.foil_aux_value;
				_first_fault_snapshot.foil_threshold_parameter = input.foil_threshold_parameter;
				_first_fault_snapshot.previous_u_effective_model = input.previous_u_effective_model;
				_first_fault_snapshot.b_hat = _b_hat;
				_first_fault_snapshot.b_lower = _b_lower;
				_first_fault_snapshot.b_upper = _b_upper;
				_first_fault_snapshot.rho_b =
					math::max(std::fabs(_b_hat - _b_lower), std::fabs(_b_upper - _b_hat));
				_first_fault_snapshot.d_hat = _d_hat;
				_first_fault_snapshot.rho_d = _rho_d;
				_first_fault_snapshot.filtered_u = _filtered_u;
				_first_fault_snapshot.filtered_r = _filtered_r;
			}

			_fallback_latched = true;
		}

		if (_fallback_latched) {
			output.first_fault_code = _first_fault_code;
			output.fault_flags |= FallbackLatchedFault | _first_fault_code;
		}
		_allocator_degraded_active = false;
		_temporary_fallback_active = false;
		_best_feasible_active = false;
		_aux_gate_inhibit_active = false;
		_aux_gate_low_streak = 0;
		_aux_gate_high_streak = 0;
		output.allocator_degraded_active = false;
		output.temporary_fallback_active = false;
		output.best_feasible_active = false;
		output.aux_gate_inhibit_active = false;
		output.safety_degraded_active = false;
		output.fallback_active = true;
		output.output_enabled = false;
		output.active_code = 0;
		output.u_selected = fallbackOutput(input, output.dt_used > 0.f ? output.dt_used : kNominalDt);
		output.u_blended = output.u_selected;

		if (output.result == Result::LegacyDirect || output.result == Result::Active || output.result == Result::Shadow) {
			output.result = Result::LatchedFallback;
		}
	}

	output.first_fault_stage_mask = _first_fault_stage_mask;
	output.first_fault_total_candidate_time_us = _first_fault_total_candidate_time_us;
	output.first_fault_reference_filter_time_us = _first_fault_reference_filter_time_us;
	output.first_fault_estimator_time_us = _first_fault_estimator_time_us;
	output.first_fault_certificate_time_us = _first_fault_certificate_time_us;
	output.first_fault_allocation_query_time_us = _first_fault_allocation_query_time_us;
	output.first_fault_nominal_time_us = _first_fault_nominal_time_us;
	output.first_fault_cbf_time_us = _first_fault_cbf_time_us;
	output.first_fault_nonlinear_check_time_us = _first_fault_nonlinear_check_time_us;
	output.first_fault_snapshot = _first_fault_snapshot;
	output.activation_pending = _activation_pending;
	output.allocator_valid_streak = _allocator_valid_streak;
	output.allocator_invalid_streak = _allocator_invalid_streak;
	output.allocator_recovery_streak = _allocator_recovery_streak;
	output.activation_required_streak = activationRequiredStreak();
	output.allocator_invalid_duration_us = _allocator_invalid_duration_us;
	output.allocator_degraded_active = _allocator_degraded_active;
	output.temporary_fallback_active = _temporary_fallback_active;
	output.aux_gate_inhibit_active = _aux_gate_inhibit_active;
	output.aux_gate_low_streak = _aux_gate_low_streak;
	output.aux_gate_high_streak = _aux_gate_high_streak;
	_best_feasible_active = output.best_feasible_active;
	output.best_feasible_active = _best_feasible_active;
	output.safety_degraded_active =
		(input.requested_mode == static_cast<uint8_t>(Mode::WorstCaseFixed)
		 || input.requested_mode == static_cast<uint8_t>(Mode::Adaptive))
		&& (_allocator_degraded_active || _temporary_fallback_active || _best_feasible_active
		    || _aux_gate_inhibit_active);
	output.fallback_latched = _fallback_latched;
	output.severe_fault_code = _first_fault_code;
	output.last_feasible_u = _last_feasible_u;
	output.healthy = (output.fault_flags & fatal_faults) == 0 && !_fallback_latched;

	if (output.candidate_valid && output.active_code == input.requested_mode
	    && !output.temporary_fallback_active
	    && (output.fault_flags & fatal_faults) == 0 && finite(output.u_selected)) {
		_last_feasible_u = output.u_selected;
		_have_last_feasible = true;
		output.last_feasible_u = _last_feasible_u;
	}

	const uint64_t accounted_time_us = output.dt_used > 0.f
					   ? static_cast<uint64_t>(output.dt_used * 1e6f) : 0;

	if (output.field_mode_active && output.best_feasible_active) {
		_cumulative_best_feasible_time_us += accounted_time_us;
	}

	if (output.field_mode_active && output.temporary_fallback_active) {
		_cumulative_temporary_fallback_time_us += accounted_time_us;
	}

	if (output.field_mode_active && output.active_code == input.requested_mode && output.output_enabled) {
		_cumulative_requested_mode_active_time_us += accounted_time_us;
	}

	output.cumulative_best_feasible_time_us = _cumulative_best_feasible_time_us;
	output.cumulative_temporary_fallback_time_us = _cumulative_temporary_fallback_time_us;
	output.cumulative_requested_mode_active_time_us = _cumulative_requested_mode_active_time_us;

	_last_selected = finite(output.u_selected) ? output.u_selected : input.legacy_u_roll;
	_last_sample_timestamp = input.angular_velocity_timestamp_sample;
	_last_requested_mode = input.requested_mode;
	_last_reset_id = input.config.reset_id;
	_last_configuration_crc = output.configuration_crc32;
	_last_reset_configuration_crc = reset_configuration_crc;
	_last_enable = input.config.enable;
	_last_shadow = input.config.shadow;
	_last_attitude_reset_counter = input.attitude_reset_counter;
	_last_armed = input.armed;
	_last_in_stab = input.in_stab;
	_initialized = true;
	output.reset_count = _reset_count;
#ifndef SAFE_ROLL_DISABLE_PERF
	perf_end(_cycle_perf);
#endif
	return output;
}

SafeRollController::Output SafeRollController::observeLegacy(const Input &input)
{
	if (_initialized && _last_requested_mode != 0) {
		_returning_to_legacy = true;
		beginBlend(0, input.angular_velocity_timestamp_sample);
	}

	if (_returning_to_legacy) {
		Output output{};
		output.result = Result::LegacyDirect;
		output.requested_code = 0;
		output.active_code = 0;
		output.trial_id = input.config.trial_id;
		output.configuration_crc32 = configurationCrc32(0, input.config);
		output.reset_count = _reset_count;
		output.healthy = true;
		output.fallback_active = true;
		output.manual_roll = input.manual_roll;
		output.phi_cmd = input.phi_cmd;
		output.phi = input.phi;
		output.p = input.p;
		output.u_legacy = input.legacy_u_roll;
		output.safe_roll_hardware_ready = hardwareReady(input);
		output.runtime_soft_budget_us = kRuntimeSoftBudgetUs;
		output.runtime_hard_budget_us = kRuntimeHardBudgetUs;
		float dt = kNominalDt;

		if (_last_sample_timestamp != 0
		    && input.angular_velocity_timestamp_sample > _last_sample_timestamp) {
			dt = math::constrain(static_cast<float>(input.angular_velocity_timestamp_sample
					     - _last_sample_timestamp) * 1e-6f, kDtMin, kDtMax);
		}

		output.dt_raw = dt;
		output.dt_used = dt;
		output.u_selected = fallbackOutput(input, dt);
		output.u_blended = output.u_selected;
		const float elapsed = input.angular_velocity_timestamp_sample >= _blend_start_timestamp
				      ? static_cast<float>(input.angular_velocity_timestamp_sample
						       - _blend_start_timestamp) * 1e-6f : 0.f;

		if (elapsed >= input.config.blend_time
		    && std::fabs(output.u_selected - input.legacy_u_roll) <= kEpsilon) {
			_returning_to_legacy = false;
		}

		_last_selected = output.u_selected;
		_last_sample_timestamp = input.angular_velocity_timestamp_sample;
		_last_requested_mode = 0;
		return output;
	}

	return evaluate(input, true);
}

SafeRollController::Output SafeRollController::update(const Input &input)
{
	return evaluate(input, false);
}
