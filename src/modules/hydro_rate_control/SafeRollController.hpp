/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <lib/hydrofoil_allocation/HydrofoilAllocationModel.hpp>
#include <lib/perf/perf_counter.h>

#include <cstdint>

class SafeRollController
{
public:
	enum class Mode : uint8_t {
		Legacy = 0,
		Nominal = 1,
		WorstCaseFixed = 2,
		Adaptive = 3,
		NoAllocationAblation = 4
	};

	enum class Result : uint8_t {
		LegacyDirect = 0,
		Active,
		Shadow,
		Disabled,
		InvalidInput,
		FoilGateDisabled,
		AllocatorInvalid,
		LogicalMappingInvalid,
		InvalidConfiguration,
		InvalidCertificate,
		EmptyHardInterval,
		NonlinearCandidateInvalid,
		RuntimeOverrun,
		LatchedFallback
	};

	enum Fault : uint32_t {
		FaultNone = 0,
		TimestampNonMonotonic = 1u << 0,
		DtTooSmall = 1u << 1,
		DtTooLarge = 1u << 2,
		GyroStale = 1u << 3,
		AttitudeStale = 1u << 4,
		QuaternionNonFinite = 1u << 5,
		AngularVelocityNonFinite = 1u << 6,
		AllocatorInvalidFault = 1u << 7,
		FoilGateDisabledFault = 1u << 8,
		ControllerDisabledFault = 1u << 9,
		ModeOutOfRange = 1u << 10,
		LogicalMappingInvalidFault = 1u << 11,
		// Bits 12-14 remain reserved for Phase 1.5 message compatibility.
		InvalidConfigurationFault = 1u << 15,
		InvalidCertificateFault = 1u << 16,
		EmptyHardIntervalFault = 1u << 17,
		NonlinearCandidateInvalidFault = 1u << 18,
		ProjectionInvalidFault = 1u << 19,
		CertificateConflictFault = 1u << 20,
		RuntimeOverrunFault = 1u << 21,
		FallbackLatchedFault = 1u << 22
	};

	enum ExecutionStage : uint32_t {
		StageReferenceFilter = 1u << 0,
		StageEstimator = 1u << 1,
		StageCertificate = 1u << 2,
		StageAllocationQuery = 1u << 3,
		StageNominalControl = 1u << 4,
		StageCbf = 1u << 5,
		StageNonlinearCheck = 1u << 6,
		StageOutputSelected = 1u << 7
	};

	struct Interval {
		float lower{0.f};
		float upper{0.f};
		bool valid{false};
	};

	struct Config {
		uint8_t enable{0};
		bool field_mode{false};
		bool shadow{false};
		uint16_t trial_id{0};
		uint32_t reset_id{0};
		float blend_time{0.30f};

		float reference_wn{4.f};
		float reference_zeta{1.f};
		float reference_rate_max{2.0943951f};
		float reference_accel_max{8.7266463f};

		float a0{-0.50f};
		float b0{10.1746f};
		float b_min{6.50f};
		float b_max{13.00f};
		float phi_safe{0.34906585f};
		float u_max{1.f};
		float du_max_nominal{0.009f};

		float foil_angle_max{0.61086524f};
		float foil_rate_max{1.5707963f};
		float foil_remainder{0.00087266463f};

		float filter_lambda{0.982f};
		float b_adaptation_gain{0.055f};
		float b_adaptation_epsilon{0.010f};
		float filtered_u_min{0.015f};
		float innovation_gate{0.75f};
		float b_update_max{0.006f};

		float disturbance_gain{0.18f};
		float disturbance_radius_initial{0.06f};
		float disturbance_variation_nominal{0.0060f};
		float effectiveness_variation_nominal{4.05e-3f};
		float incremental_u_min{0.018f};
		float incremental_residual{1.5e-3f};
		float incremental_gate_factor{1.25f};
		float b_radius_initial{0.35f};

		float k1{15.f};
		float k2{7.f};
		float gamma1{0.011f};
		float gamma2{0.011f};
		float robust_gain_floor{0.50f};
	};

	struct Input {
		uint64_t now{0};
		uint64_t angular_velocity_timestamp_sample{0};
		uint64_t attitude_timestamp{0};
		uint64_t allocator_timestamp{0};
		uint8_t requested_mode{0};
		bool armed{false};
		bool in_stab{false};
		bool quaternion_finite{false};
		bool angular_velocity_finite{false};
		bool allocator_valid{false};
		bool allocator_converged{false};
		bool allocator_finite{false};
		bool allocator_bounded{false};
		uint32_t allocator_saturation_mask{0};
		bool foil_gate_enabled{false};
		bool aux_gate_enabled_raw{false};
		bool nav_gate_enabled_raw{false};
		bool aux_gate_data_valid{false};
		bool aux_gate_value_finite{false};
		uint8_t gate_close_reason{0};
		int32_t foil_aux_parameter{0};
		float foil_aux_value{0.f};
		float foil_threshold_parameter{0.f};
		int8_t foil_aux_rc_channel{0};
		bool foil_aux_mapping_valid{false};
		uint64_t manual_control_timestamp{0};
		uint64_t manual_control_timestamp_sample{0};
		uint32_t manual_control_age_us{0};
		uint8_t manual_control_data_source{0};
		bool manual_control_valid{false};
		bool rc_signal_lost{false};
		bool rc_failsafe{false};
		bool logical_mapping_valid{false};
		bool physical_mapping_verified{false}; // retained diagnostic, not an enable condition
		bool servo_ownership_verified{false}; // retained diagnostic, not an enable condition
		uint8_t attitude_reset_counter{0};
		float manual_roll{0.f};
		float phi_cmd{0.f};
		float phi{0.f};
		float p{0.f};
		float legacy_u_roll{0.f};

		hydrofoil::AllocationContext allocation_context{};
		hydrofoil::AllocationCommand allocation_command{};
		float generic_servo4{0.f}; // logical right, normalized, pre-PWM
		float generic_servo5{0.f}; // logical left, normalized, pre-PWM
		float previous_final_servo4{0.f};
		float previous_final_servo5{0.f};
		float previous_u_effective_model{0.f};
		Config config{};
#ifdef SAFE_ROLL_DISABLE_PERF
		uint32_t test_candidate_time_us{0};
		uint32_t test_stage_time_us{0};
		bool test_activation_bypass{false};
		bool test_force_finite_nonlinear_failure{false};
#endif
	};

	struct FirstFaultSnapshot {
		uint32_t prerequisite_fault_flags{0};
		uint64_t allocator_timestamp{0};
		uint32_t allocator_age_us{0};
		uint32_t reset_count{0};
		uint8_t requested_mode{0};
		uint8_t attitude_reset_counter{0};
		uint8_t history_count{0};
		bool activation_pending{false};
		bool mode_activated{false};
		bool in_stab{false};
		bool allocator_timestamp_valid{false};
		bool allocator_valid{false};
		bool allocator_converged{false};
		bool allocator_finite{false};
		bool allocator_bounded{false};
		bool foil_gate_enabled{false};
		bool foil_aux_above_threshold{false};
		bool logical_mapping_valid{false};
		bool estimator_initialized{false};
		int32_t foil_aux_parameter{0};
		float foil_aux_value{0.f};
		float foil_threshold_parameter{0.f};
		float previous_u_effective_model{0.f};
		float b_hat{0.f};
		float b_lower{0.f};
		float b_upper{0.f};
		float rho_b{0.f};
		float d_hat{0.f};
		float rho_d{0.f};
		float filtered_u{0.f};
		float filtered_r{0.f};
	};

	struct Output {
		Result result{Result::LegacyDirect};
		uint8_t requested_code{0};
		uint8_t active_code{0};
		uint16_t trial_id{0};
		uint32_t configuration_crc32{0};
		uint32_t reset_count{0};
		uint32_t fault_flags{0};
		uint32_t first_fault_code{0};
		uint32_t execution_stage_mask{0};
		uint32_t first_fault_stage_mask{0};
		uint32_t first_fault_total_candidate_time_us{0};
		uint32_t first_fault_reference_filter_time_us{0};
		uint32_t first_fault_estimator_time_us{0};
		uint32_t first_fault_certificate_time_us{0};
		uint32_t first_fault_allocation_query_time_us{0};
		uint32_t first_fault_nominal_time_us{0};
		uint32_t first_fault_cbf_time_us{0};
		uint32_t first_fault_nonlinear_check_time_us{0};
		FirstFaultSnapshot first_fault_snapshot{};
		uint32_t runtime_soft_budget_us{0};
		uint32_t runtime_hard_budget_us{0};
		uint32_t consecutive_runtime_overrun_count{0};
		uint16_t allocator_valid_streak{0};
		uint16_t allocator_invalid_streak{0};
		uint16_t allocator_recovery_streak{0};
		uint16_t activation_required_streak{0};
		uint32_t allocator_invalid_duration_us{0};
		bool healthy{false};
		bool shadow_active{false};
		bool output_enabled{false};
		bool fallback_active{false};
		bool safe_roll_hardware_ready{false};
		bool candidate_valid{false};
		bool interval_valid{false};
		bool activation_pending{false};
		bool allocator_degraded_active{false};
		bool field_mode_active{false};
		bool best_feasible_active{false};
		bool temporary_fallback_active{false};
		bool safety_degraded_active{false};
		bool aux_gate_inhibit_active{false};
		bool fallback_latched{false};
		uint32_t severe_fault_code{0};
		uint64_t cumulative_best_feasible_time_us{0};
		uint64_t cumulative_temporary_fallback_time_us{0};
		uint64_t cumulative_requested_mode_active_time_us{0};
		float last_feasible_u{0.f};
		uint16_t aux_gate_low_streak{0};
		uint16_t aux_gate_high_streak{0};
		float dt_raw{0.f};
		float dt_used{0.f};

		float manual_roll{0.f};
		float phi_cmd{0.f};
		float phi_ref{0.f};
		float phi_ref_dot{0.f};
		float phi_ref_ddot{0.f};
		float phi{0.f};
		float p{0.f};

		float u_legacy{0.f};
		float u_nominal{0.f};
		float u_candidate{0.f};
		float u_blended{0.f};
		float u_selected{0.f};
		float u_effective_model{0.f};

		float b_hat{0.f};
		float b_lower{0.f};
		float b_upper{0.f};
		float rho_b{0.f};
		float d_hat{0.f};
		float rho_d{0.f};
		float certified_width{0.f};
		float filtered_u{0.f};
		float filtered_r{0.f};
		float filtered_innovation{0.f};
		bool b_update_active{false};
		bool certificate_update_active{false};
		bool certificate_conflict{false};
		bool estimator_ready{false};
		bool certificate_ready{false};
		bool certificate_warmup{false};
		bool allocation_interval_degraded{false};
		uint8_t accepted_lag_mask{0};
		uint8_t conflict_lag_mask{0};

		Interval u_amp{};
		Interval u_rate{};
		Interval u_alloc{};
		Interval u_hard{};
		Interval u_cbf{};
		Interval u_safe{};
		float epsilon_relax{0.f};

		float h_upper{0.f};
		float h_lower{0.f};
		float psi_upper{0.f};
		float psi_lower{0.f};
		float psi_min{0.f};

		float delta_left_commanded{0.f};
		float delta_right_commanded{0.f};
		float foil_rate_left_commanded{0.f};
		float foil_rate_right_commanded{0.f};
		float thrust_left_commanded{0.f};
		float thrust_right_commanded{0.f};

		uint32_t reference_filter_time_us{0};
		uint32_t estimator_time_us{0};
		uint32_t certificate_time_us{0};
		uint32_t cbf_time_us{0};
		uint32_t allocation_query_time_us{0};
		uint32_t nominal_time_us{0};
		uint32_t nonlinear_check_time_us{0};
		uint32_t total_candidate_time_us{0};
	};

	SafeRollController();
	~SafeRollController();

	void reset(bool clear_severe_latch = true);
	Output observeLegacy(const Input &input);
	Output update(const Input &input);

	static bool modeImplemented(uint8_t mode) { return mode <= static_cast<uint8_t>(Mode::NoAllocationAblation); }
	static bool hardwareReady(const Input &input);
	static bool validateConfig(const Config &config);
	static uint32_t configurationCrc32(uint8_t requested_mode, const Config &config);
	static uint32_t resetConfigurationCrc32(const Config &config);
	static float bumplessStep(float current, float target, float max_rate, float dt);
	static Interval intersect(const Interval &a, const Interval &b);
	static Interval combineAllocationInterval(uint8_t mode, const Interval &amplitude_rate,
			const Interval &allocation, bool use_allocation, bool &degraded, bool field_mode = false);
	static float project(float value, const Interval &interval);
	static float minimumRelaxation(const Interval &cbf, const Interval &hard);
	static float scaledVariationBound(float nominal_bound, float dt);
	static uint32_t runtimeSoftBudgetUs();
	static uint32_t runtimeHardBudgetUs();
	static constexpr uint16_t activationRequiredStreak() { return 20; }
	static constexpr uint16_t allocatorHoldStreakLimit() { return 20; }
	static constexpr uint16_t allocatorTemporaryFallbackStreakLimit() { return 80; }
	static constexpr uint16_t allocatorInvalidStreakLimit() { return 81; }
	static constexpr uint16_t allocatorRecoveryRequiredStreak() { return 20; }
	static bool runtimeBudgetExceeded(uint32_t elapsed_us, bool transition,
					 uint32_t &consecutive_soft_overruns);
	static Interval refineCertificate(const Interval &prior, float delta_r, float delta_u,
					  float omega, bool &conflict);
	static float nominalControl(float a0, float p, float d_hat, float phi_ref_ddot,
				    float k1, float e1, float k2, float e2, float b_control);
	static Interval robustCbfInterval(float phi_safe, float phi, float p, float dt, float a0,
					 float d_hat, float rho_d, float b_hat, float rho_b,
					 float gamma1, float gamma2, float robust_gain_floor,
					 float &h_upper, float &h_lower);

private:
	static constexpr int HistoryLength = 40;
	static constexpr int LagCount = 5;

	struct InnovationSample {
		uint64_t timestamp{0};
		float r{0.f};
		float u_effective{0.f};
		float b_lower{0.f};
		float b_upper{0.f};
		float final_left{0.f};
		float final_right{0.f};
		bool valid{false};
	};

	Output evaluate(const Input &input, bool direct_legacy);
	void resetNewController(const Input &input, bool preserve_selected, bool clear_severe_latch);
	void pushInnovation(const InnovationSample &sample);
	const InnovationSample *historyLag(int lag) const;
	bool updateEstimators(const Input &input, Output &output, float dt);
	void updateReference(const Input &input, Output &output, float dt);
	Interval allocationInterval(const Input &input, const Config &config, float dt,
				    hydrofoil::AllocationSolution &base_solution,
				    hydrofoil::AllocationLinearization &linearization) const;
	bool checkNonlinearCandidate(const Input &input, float candidate, float dt, bool enforce_foil_constraints,
				     Output &output, float &effective_model, bool &severe_failure) const;
	float validatedCandidate(const Input &input, float requested_candidate, float dt,
				 bool enforce_foil_constraints, Output &output, bool &valid,
				 bool &severe_failure) const;
	float fallbackOutput(const Input &input, float dt);
	float holdLastFeasibleOutput(const Input &input, float dt) const;
	void beginBlend(uint8_t target_code, uint64_t timestamp);
	float blendTo(float target, const Config &config, uint64_t timestamp, const Interval &hard);
	static Interval intervalFromAffineConstraint(float base_u, float base_value, float gain,
			float value_lower, float value_upper);
	static bool finiteInterval(const Interval &interval);

	perf_counter_t _cycle_perf{nullptr};
	uint64_t _last_sample_timestamp{0};
	uint64_t _blend_start_timestamp{0};
	uint8_t _last_requested_mode{0};
	uint8_t _blend_target_code{0};
	uint8_t _last_attitude_reset_counter{0};
	uint32_t _last_reset_id{0};
	uint32_t _last_configuration_crc{0};
	uint32_t _last_reset_configuration_crc{0};
	uint8_t _last_enable{0};
	bool _last_shadow{false};
	bool _last_armed{false};
	bool _last_in_stab{false};
	bool _initialized{false};
	bool _reference_initialized{false};
	bool _have_previous_p{false};
	bool _fallback_latched{false};
	bool _returning_to_legacy{false};
	bool _activation_pending{false};
	bool _mode_activated{false};
	bool _allocator_recovery_pending{false};
	bool _allocator_degraded_active{false};
	bool _temporary_fallback_active{false};
	bool _best_feasible_active{false};
	bool _aux_gate_inhibit_active{false};
	bool _have_last_feasible{false};
	uint32_t _first_fault_code{0};
	uint32_t _first_fault_stage_mask{0};
	uint32_t _first_fault_total_candidate_time_us{0};
	uint32_t _first_fault_reference_filter_time_us{0};
	uint32_t _first_fault_estimator_time_us{0};
	uint32_t _first_fault_certificate_time_us{0};
	uint32_t _first_fault_allocation_query_time_us{0};
	uint32_t _first_fault_nominal_time_us{0};
	uint32_t _first_fault_cbf_time_us{0};
	uint32_t _first_fault_nonlinear_check_time_us{0};
	FirstFaultSnapshot _first_fault_snapshot{};
	uint32_t _consecutive_runtime_overruns{0};
	uint16_t _allocator_valid_streak{0};
	uint16_t _allocator_invalid_streak{0};
	uint16_t _allocator_recovery_streak{0};
	uint16_t _aux_gate_low_streak{0};
	uint16_t _aux_gate_high_streak{0};
	uint64_t _allocator_invalid_start_timestamp{0};
	uint32_t _allocator_invalid_duration_us{0};
	uint64_t _cumulative_best_feasible_time_us{0};
	uint64_t _cumulative_temporary_fallback_time_us{0};
	uint64_t _cumulative_requested_mode_active_time_us{0};
	float _blend_start_output{0.f};
	float _last_selected{0.f};
	float _last_feasible_u{0.f};
	float _previous_p{0.f};
	float _reference_z1{0.f};
	float _reference_z2{0.f};
	float _b_hat{10.1746f};
	float _b_lower{9.8246f};
	float _b_upper{10.5246f};
	float _d_hat{0.f};
	float _rho_d{0.06f};
	float _filtered_u{0.f};
	float _filtered_r{0.f};
	InnovationSample _history[HistoryLength]{};
	int _history_head{-1};
	int _history_count{0};
	uint32_t _reset_count{0};
};
