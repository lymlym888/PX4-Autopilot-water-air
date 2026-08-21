/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "SafeRollController.hpp"
#include "CADRC.h"

#include <mathlib/mathlib.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <type_traits>

namespace
{
SafeRollController::Input validInput(uint8_t mode = 1)
{
	SafeRollController::Input input{};
	input.now = 1'001'000;
	input.angular_velocity_timestamp_sample = 1'000'000;
	input.attitude_timestamp = 1'000'500;
	input.allocator_timestamp = 1'000'500;
	input.requested_mode = mode;
	input.quaternion_finite = true;
	input.angular_velocity_finite = true;
	input.allocator_valid = true;
	input.allocator_converged = true;
	input.allocator_finite = true;
	input.allocator_bounded = true;
	input.foil_gate_enabled = true;
	input.aux_gate_enabled_raw = true;
	input.nav_gate_enabled_raw = true;
	input.aux_gate_data_valid = true;
	input.aux_gate_value_finite = true;
	input.foil_aux_parameter = 8;
	input.foil_aux_value = 1.f;
	input.foil_threshold_parameter = 0.f;
	input.foil_aux_mapping_valid = true;
	input.manual_control_valid = true;
	input.logical_mapping_valid = true;
	input.in_stab = true;
	input.config.enable = 2;
#ifdef SAFE_ROLL_DISABLE_PERF
	input.test_activation_bypass = true;
#endif
	input.allocation_context.lift_coefficient = 2.f;
	input.allocation_context.speed_squared = 9.f;
	input.allocation_context.body_pitch = 0.08f;
	input.allocation_context.max_hydrofoil_angle = 1.134464f;
	input.allocation_context.max_thruster_thrust = 30.f;
	input.allocation_context.available_thruster_thrust = 30.f;
	const float values[4][3] {
		{0.f, 0.f, 30.f},
		{-10.f, 0.f, 0.f},
		{0.f, 0.f, -30.f},
		{10.f, 0.f, 0.f}
	};
	input.allocation_context.virtual_control_matrix = matrix::Matrix<float, 4, 3>(values);
	input.allocation_command.force_x = 24.f;
	const auto initial_solution = hydrofoil::HydrofoilAllocationModel::solve(input.allocation_context,
				      input.allocation_command);
	input.previous_final_servo5 =
		initial_solution.delta_left_commanded / input.allocation_context.max_hydrofoil_angle;
	input.previous_final_servo4 =
		initial_solution.delta_right_commanded / input.allocation_context.max_hydrofoil_angle;
	const auto initial_forward = hydrofoil::HydrofoilAllocationModel::forward(input.allocation_context,
				     initial_solution);
	input.previous_u_effective_model = initial_forward.u_phi_effective_model;
	return input;
}

void advance(SafeRollController::Input &input, float dt = 0.0025f)
{
	const uint64_t delta = static_cast<uint64_t>(dt * 1e6f);
	input.angular_velocity_timestamp_sample += delta;
	input.attitude_timestamp += delta;
	input.allocator_timestamp += delta;
	input.now += delta;
}

SafeRollController::Output runHealthy(SafeRollController &controller, SafeRollController::Input &input)
{
	controller.update(input);
	advance(input);
	return controller.update(input);
}

SafeRollController::Output activateAfterHealthyStreak(SafeRollController &controller,
		SafeRollController::Input &input, uint8_t mode)
{
#ifdef SAFE_ROLL_DISABLE_PERF
	input.test_activation_bypass = false;
#endif
	input.requested_mode = 0;
	controller.observeLegacy(input);
	advance(input);
	input.requested_mode = mode;
	SafeRollController::Output output{};

	for (uint16_t i = 0; i < SafeRollController::activationRequiredStreak(); ++i) {
		output = controller.update(input);

		if (i + 1u < SafeRollController::activationRequiredStreak()) {
			advance(input);
		}
	}

	return output;
}
}

TEST(SafeRollController, ModeZeroIsSampleExactDirectPath)
{
	SafeRollController controller;
	auto input = validInput(0);

	for (float legacy : {0.f, 0.125f, -0.75f, 1.f, -1.f, 0.33333334f}) {
		input.legacy_u_roll = legacy;
		const auto output = controller.observeLegacy(input);
		EXPECT_FLOAT_EQ(output.u_selected, legacy);
		EXPECT_EQ(output.active_code, 0);
		EXPECT_FALSE(output.fallback_active);
		advance(input);
	}
}

TEST(SafeRollController, ModeZeroPreservesLegacyCadrcStateAndOutputSamples)
{
	ADRCController before_selector;
	ADRCController after_selector;
	SafeRollController selector;
	auto input = validInput(0);
	const float samples[][4] {
		{0.f, 0.f, 0.f, 0.0025f}, {0.03f, 0.12f, 0.10f, 0.0024f},
		{0.07f, -0.08f, 0.15f, 0.0027f}, {-0.04f, -0.25f, -0.12f, 0.0025f},
		{0.18f, 0.40f, -0.08f, 0.0030f}, {-0.12f, 0.05f, 0.20f, 0.0020f}
	};

	for (const auto &sample : samples) {
		before_selector.update(sample[0], sample[2], sample[1], sample[3]);
		after_selector.update(sample[0], sample[2], sample[1], sample[3]);
		const float before = math::constrain(before_selector.getUc() * 0.001f, -1.f, 1.f);
		const float after = math::constrain(after_selector.getUc() * 0.001f, -1.f, 1.f);
		input.legacy_u_roll = after;
		const auto output = selector.observeLegacy(input);
		EXPECT_FLOAT_EQ(after, before);
		EXPECT_FLOAT_EQ(output.u_selected, before);
		EXPECT_FLOAT_EQ(after_selector._Z1(0, 0), before_selector._Z1(0, 0));
		EXPECT_FLOAT_EQ(after_selector._Z2(1, 0), before_selector._Z2(1, 0));
		advance(input, sample[3]);
	}
}

TEST(SafeRollController, ModeZeroWithEnableOneRemainsFaultFreeLegacy)
{
	SafeRollController controller;
	auto input = validInput(0);
	input.config.enable = 1;
	input.legacy_u_roll = 0.27182818f;
	const auto output = controller.observeLegacy(input);
	EXPECT_FLOAT_EQ(output.u_selected, input.legacy_u_roll);
	EXPECT_EQ(output.active_code, 0);
	EXPECT_EQ(output.fault_flags, 0u);
	EXPECT_EQ(output.first_fault_code, 0u);
	EXPECT_FALSE(output.fallback_active);
}

TEST(SafeRollController, ShoreCaseModeZeroToNominalRunsCandidatePipeline)
{
	SafeRollController controller;
	auto input = validInput(0);
	input.config.enable = 1;
	input.phi = math::radians(2.f);
	input.generic_servo4 = -0.1f;
	input.generic_servo5 = -0.1f;
	input.previous_final_servo4 += input.generic_servo4;
	input.previous_final_servo5 += input.generic_servo5;
	const auto legacy_output = controller.observeLegacy(input);

	advance(input, 0.00247f);
	input.requested_mode = 1;
	const auto output = controller.update(input);
	EXPECT_GT(output.reset_count, legacy_output.reset_count);
	EXPECT_EQ(output.active_code, 1);
	EXPECT_TRUE(output.candidate_valid);
	EXPECT_TRUE(output.output_enabled);
	EXPECT_FALSE(output.fallback_active);
	EXPECT_TRUE(std::isfinite(output.u_nominal));
	EXPECT_TRUE(std::isfinite(output.u_candidate));
	EXPECT_FALSE(output.estimator_ready);
	EXPECT_FALSE(output.certificate_ready);
	EXPECT_EQ(output.accepted_lag_mask, 0u);
	EXPECT_NE(output.execution_stage_mask & SafeRollController::StageReferenceFilter, 0u);
	EXPECT_NE(output.execution_stage_mask & SafeRollController::StageNominalControl, 0u);
	EXPECT_NE(output.execution_stage_mask & SafeRollController::StageAllocationQuery, 0u);
	EXPECT_NE(output.execution_stage_mask & SafeRollController::StageNonlinearCheck, 0u);
	EXPECT_NE(output.execution_stage_mask & SafeRollController::StageOutputSelected, 0u);
	EXPECT_EQ(output.execution_stage_mask & SafeRollController::StageCertificate, 0u);
	EXPECT_EQ(output.execution_stage_mask & SafeRollController::StageCbf, 0u);
}

TEST(SafeRollController, FaultCodeValuesMatchFlightLog)
{
	EXPECT_EQ(SafeRollController::EmptyHardIntervalFault, 0x00020000u);
	EXPECT_EQ(SafeRollController::RuntimeOverrunFault, 0x00200000u);
	EXPECT_EQ(SafeRollController::AttitudeStale, 0x00000010u);
	EXPECT_EQ(SafeRollController::AllocatorInvalidFault, 0x00000080u);
}

TEST(SafeRollController, NominalDoesNotRequireAdaptiveCertificateReadiness)
{
	SafeRollController controller;
	auto input = validInput(0);
	input.config.enable = 1;
	controller.observeLegacy(input);
	advance(input);
	input.requested_mode = 1;
	const auto output = controller.update(input);
	EXPECT_FALSE(output.estimator_ready);
	EXPECT_FALSE(output.certificate_ready);
	EXPECT_EQ(output.accepted_lag_mask, 0u);
	EXPECT_TRUE(output.interval_valid);
	EXPECT_TRUE(output.candidate_valid);
	EXPECT_EQ(output.active_code, 1);
	EXPECT_EQ(output.fault_flags & SafeRollController::InvalidCertificateFault, 0u);
}

TEST(SafeRollController, ModesOneTwoThreePreserveDepthForceAndOnlyReplaceRollCommand)
{
	for (uint8_t mode : {1u, 2u, 3u}) {
		SafeRollController controller;
		auto input = validInput(mode);
		input.allocation_context.body_pitch = 0.f;
		input.allocation_command.force_x = 20.f;
		input.allocation_command.force_z = -4.f;
		input.allocation_command.u_theta = 0.25f;
		input.allocation_command.u_psi = -0.1f;
		input.config.k1 = 0.f;
		input.config.k2 = 0.f;
		input.config.a0 = 0.f;
		input.config.blend_time = 0.05f;
		input.phi = 0.f;
		input.p = 0.f;
		input.phi_cmd = 0.f;
		input.legacy_u_roll = 0.f;
		const auto initial = hydrofoil::HydrofoilAllocationModel::solve(
					     input.allocation_context, input.allocation_command);
		ASSERT_TRUE(initial.status.valid);
		input.previous_final_servo5 =
			initial.delta_left_commanded / input.allocation_context.max_hydrofoil_angle;
		input.previous_final_servo4 =
			initial.delta_right_commanded / input.allocation_context.max_hydrofoil_angle;
		const auto initial_forward =
			hydrofoil::HydrofoilAllocationModel::forward(input.allocation_context, initial);
		ASSERT_TRUE(initial_forward.valid);
		input.previous_u_effective_model = initial_forward.u_phi_effective_model;

		auto output = runHealthy(controller, input);
		ASSERT_TRUE(output.candidate_valid) << "mode=" << unsigned(mode);
		ASSERT_EQ(output.active_code, mode);
		const auto executed = hydrofoil::HydrofoilAllocationModel::checkCandidate(
					      input.allocation_context, input.allocation_command, output.u_selected);
		ASSERT_TRUE(executed.status.valid);
		EXPECT_FLOAT_EQ(executed.source_command.force_x, input.allocation_command.force_x);
		EXPECT_FLOAT_EQ(executed.source_command.force_z, input.allocation_command.force_z);
		EXPECT_FLOAT_EQ(executed.source_command.u_theta, input.allocation_command.u_theta);
		EXPECT_FLOAT_EQ(executed.source_command.u_psi, input.allocation_command.u_psi);
		const auto components = hydrofoil::HydrofoilAllocationModel::decomposeFoilCommand(
						executed.delta_left_commanded, executed.delta_right_commanded);
		ASSERT_TRUE(components.valid);
		EXPECT_NE(std::fabs(components.symmetric), 0.f);
		EXPECT_NEAR(output.delta_left_commanded, executed.delta_left_commanded, 1e-5f);
		EXPECT_NEAR(output.delta_right_commanded, executed.delta_right_commanded, 1e-5f);
	}
}

TEST(SafeRollController, NominalSurvivesUnavailableLocalAllocationInterval)
{
	SafeRollController controller;
	auto input = validInput(0);
	input.config.enable = 1;
	// Represent a legacy commanded state outside the new-controller local
	// DMAX interval. The nonlinear allocator itself remains finite and solvable.
	input.previous_final_servo4 = 0.8f;
	input.previous_final_servo5 = 0.8f;
	controller.observeLegacy(input);
	advance(input);
	input.requested_mode = 1;
	const auto output = controller.update(input);
	EXPECT_FALSE(output.u_alloc.valid);
	EXPECT_TRUE(output.allocation_interval_degraded);
	EXPECT_TRUE(output.interval_valid);
	EXPECT_TRUE(output.candidate_valid);
	EXPECT_EQ(output.active_code, 1);
	EXPECT_EQ(output.fault_flags & SafeRollController::EmptyHardIntervalFault, 0u);
}

TEST(SafeRollController, NominalSurvivesDisjointValidAllocationAndRateIntervals)
{
	const SafeRollController::Interval amplitude_rate{-0.01f, 0.01f, true};
	const SafeRollController::Interval allocation{0.20f, 0.30f, true};
	bool degraded = false;
	const auto nominal = SafeRollController::combineAllocationInterval(
				     static_cast<uint8_t>(SafeRollController::Mode::Nominal),
				     amplitude_rate, allocation, true, degraded);
	EXPECT_TRUE(degraded);
	EXPECT_TRUE(nominal.valid);
	EXPECT_FLOAT_EQ(nominal.lower, amplitude_rate.lower);
	EXPECT_FLOAT_EQ(nominal.upper, amplitude_rate.upper);

	const auto fixed = SafeRollController::combineAllocationInterval(
				   static_cast<uint8_t>(SafeRollController::Mode::WorstCaseFixed),
				   amplitude_rate, allocation, true, degraded);
	EXPECT_FALSE(degraded);
	EXPECT_FALSE(fixed.valid);
}

TEST(SafeRollController, FixedModeDoesNotRequireAdaptiveCertificateHistory)
{
	SafeRollController controller;
	auto input = validInput(0);
	input.config.enable = 1;
	controller.observeLegacy(input);
	advance(input);
	input.requested_mode = 2;
	const auto output = controller.update(input);
	EXPECT_FALSE(output.estimator_ready);
	EXPECT_TRUE(output.certificate_ready);
	EXPECT_EQ(output.accepted_lag_mask, 0u);
	EXPECT_TRUE(output.candidate_valid);
	EXPECT_EQ(output.active_code, 2);
	EXPECT_EQ(output.fault_flags & SafeRollController::InvalidCertificateFault, 0u);
}

TEST(SafeRollController, AdaptiveUsesExplicitPriorCertificateDuringWarmup)
{
	SafeRollController controller;
	auto input = validInput(0);
	input.config.enable = 1;
	controller.observeLegacy(input);
	advance(input);
	input.requested_mode = 3;
	const auto output = controller.update(input);
	EXPECT_FALSE(output.estimator_ready);
	EXPECT_TRUE(output.certificate_ready);
	EXPECT_TRUE(output.certificate_warmup);
	EXPECT_EQ(output.accepted_lag_mask, 0u);
	EXPECT_TRUE(output.candidate_valid);
	EXPECT_EQ(output.active_code, 3);
}

TEST(SafeRollController, RuntimeSoftAndHardBudgetRules)
{
	uint32_t count = 0;
	EXPECT_EQ(SafeRollController::runtimeSoftBudgetUs(), 1000u);
	EXPECT_EQ(SafeRollController::runtimeHardBudgetUs(), 2500u);

	for (uint32_t elapsed : {150u, 300u, 500u}) {
		EXPECT_FALSE(SafeRollController::runtimeBudgetExceeded(elapsed, false, count));
		EXPECT_EQ(count, 0u);
	}

	EXPECT_FALSE(SafeRollController::runtimeBudgetExceeded(1200u, false, count));
	EXPECT_EQ(count, 1u);
	EXPECT_FALSE(SafeRollController::runtimeBudgetExceeded(500u, false, count));
	EXPECT_EQ(count, 0u);

	for (int i = 1; i <= 4; ++i) {
		EXPECT_FALSE(SafeRollController::runtimeBudgetExceeded(1200u, false, count));
		EXPECT_EQ(count, static_cast<uint32_t>(i));
	}

	EXPECT_TRUE(SafeRollController::runtimeBudgetExceeded(1200u, false, count));
	EXPECT_EQ(count, 5u);
	count = 4;
	EXPECT_TRUE(SafeRollController::runtimeBudgetExceeded(2501u, true, count));
}

TEST(SafeRollController, RuntimePolicyIsSharedByModesAndFirstTransitionIgnoresSoftBudget)
{
	for (uint8_t mode = 1; mode <= 3; ++mode) {
		SafeRollController controller;
		auto input = validInput(0);
		input.config.enable = 1;
		controller.observeLegacy(input);
		advance(input);
		input.requested_mode = mode;
		input.test_candidate_time_us = 1200;
		auto output = controller.update(input);
		EXPECT_EQ(output.active_code, mode);
		EXPECT_EQ(output.consecutive_runtime_overrun_count, 0u);
		EXPECT_EQ(output.fault_flags & SafeRollController::RuntimeOverrunFault, 0u);

		for (int i = 1; i <= 4; ++i) {
			advance(input);
			output = controller.update(input);
			EXPECT_EQ(output.active_code, mode);
			EXPECT_EQ(output.consecutive_runtime_overrun_count, static_cast<uint32_t>(i));
			EXPECT_EQ(output.fault_flags & SafeRollController::RuntimeOverrunFault, 0u);
		}

		advance(input);
		output = controller.update(input);
		EXPECT_TRUE(output.fallback_active);
		EXPECT_EQ(output.first_fault_code, SafeRollController::RuntimeOverrunFault);
		EXPECT_EQ(output.consecutive_runtime_overrun_count, 5u);
	}
}

TEST(SafeRollController, HardRuntimeFaultAndFirstFaultTimingPersistUntilReset)
{
	SafeRollController controller;
	auto input = validInput(0);
	input.config.enable = 1;
	controller.observeLegacy(input);
	advance(input);
	input.requested_mode = 1;
	input.test_candidate_time_us = 2501;
	input.test_stage_time_us = 17;
	auto output = controller.update(input);
	EXPECT_TRUE(output.fallback_active);
	EXPECT_EQ(output.first_fault_code, SafeRollController::RuntimeOverrunFault);
	EXPECT_EQ(output.first_fault_total_candidate_time_us, 2501u);
	EXPECT_EQ(output.first_fault_stage_mask, output.execution_stage_mask);
	EXPECT_EQ(output.first_fault_reference_filter_time_us, 17u);
	EXPECT_EQ(output.first_fault_estimator_time_us, 17u);
	EXPECT_EQ(output.first_fault_certificate_time_us, 0u);
	EXPECT_EQ(output.first_fault_allocation_query_time_us, 17u);
	EXPECT_EQ(output.first_fault_nominal_time_us, 17u);
	EXPECT_EQ(output.first_fault_cbf_time_us, 0u);
	EXPECT_EQ(output.first_fault_nonlinear_check_time_us, 17u);

	const uint32_t first_mask = output.first_fault_stage_mask;
	input.test_candidate_time_us = 200;
	input.test_stage_time_us = 3;
	advance(input);
	output = controller.update(input);
	EXPECT_EQ(output.first_fault_total_candidate_time_us, 2501u);
	EXPECT_EQ(output.first_fault_stage_mask, first_mask);
	EXPECT_EQ(output.first_fault_reference_filter_time_us, 17u);

	++input.config.reset_id;
	advance(input);
	output = controller.update(input);
	EXPECT_EQ(output.first_fault_code, 0u);
	EXPECT_EQ(output.first_fault_stage_mask, 0u);
	EXPECT_EQ(output.first_fault_total_candidate_time_us, 0u);
}

TEST(SafeRollController, EnablePolicy)
{
	for (uint8_t mode = 1; mode <= 4; ++mode) {
		SafeRollController controller;
		auto input = validInput(mode);
		input.config.enable = 0;
		auto output = runHealthy(controller, input);
		EXPECT_EQ(output.active_code, 0);
		EXPECT_TRUE(output.fallback_active);

		++input.config.reset_id;
		input.config.enable = 1;
		advance(input);
		output = runHealthy(controller, input);
		EXPECT_EQ(output.active_code, mode <= 3 ? mode : 0);

		++input.config.reset_id;
		input.config.enable = 2;
		advance(input);
		output = runHealthy(controller, input);
		EXPECT_EQ(output.active_code, mode);
	}
}

TEST(SafeRollController, ShadowAlwaysDrivesLegacy)
{
	SafeRollController controller;
	auto input = validInput(3);
	input.config.shadow = true;
	input.legacy_u_roll = 0.31f;
	input.allocation_command.u_phi = input.legacy_u_roll;
	const auto legacy_solution = hydrofoil::HydrofoilAllocationModel::solve(input.allocation_context,
				     input.allocation_command);
	input.previous_final_servo5 =
		legacy_solution.delta_left_commanded / input.allocation_context.max_hydrofoil_angle;
	input.previous_final_servo4 =
		legacy_solution.delta_right_commanded / input.allocation_context.max_hydrofoil_angle;
	input.requested_mode = 0;
	controller.observeLegacy(input);
	advance(input);
	input.requested_mode = 3;
	const auto output = controller.update(input);
	EXPECT_EQ(output.active_code, 3);
	EXPECT_TRUE(output.shadow_active);
	EXPECT_FALSE(output.output_enabled);
	EXPECT_FLOAT_EQ(output.u_selected, input.legacy_u_roll);
	EXPECT_TRUE(output.candidate_valid);
}

TEST(SafeRollController, OnlineModeSwitchIsBumpless)
{
	SafeRollController controller;
	auto input = validInput(1);
	auto output = runHealthy(controller, input);
	const float previous = output.u_selected;
	input.requested_mode = 3;
	advance(input);
	output = controller.update(input);
	EXPECT_FLOAT_EQ(output.u_selected, previous);
	EXPECT_EQ(output.active_code, 3);
}

TEST(SafeRollController, ModesOneTwoThreeReturnToModeZeroSmoothThenExact)
{
	for (uint8_t mode = 1; mode <= 3; ++mode) {
		SafeRollController controller;
		auto input = validInput(mode);
		auto output = runHealthy(controller, input);
		const float previous = output.u_selected;
		input.legacy_u_roll = 0.6f;
		input.requested_mode = 0;
		advance(input);
		output = controller.observeLegacy(input);
		EXPECT_FLOAT_EQ(output.u_selected, previous);
		EXPECT_TRUE(output.fallback_active);

		for (int i = 0; i < 200; ++i) {
			advance(input);
			output = controller.observeLegacy(input);
		}

		EXPECT_FLOAT_EQ(output.u_selected, input.legacy_u_roll);
		const float next_legacy = -0.2f;
		input.legacy_u_roll = next_legacy;
		advance(input);
		output = controller.observeLegacy(input);
		EXPECT_FLOAT_EQ(output.u_selected, next_legacy);
	}
}

TEST(SafeRollController, FaultFallbackIsIncrementLimitedAndLatched)
{
	SafeRollController controller;
	auto input = validInput(3);
	auto output = runHealthy(controller, input);
	const float previous = output.u_selected;
	input.legacy_u_roll = 0.8f;
	input.allocator_valid = false;
	advance(input);
	output = controller.update(input);
	EXPECT_TRUE(output.fallback_active);
	EXPECT_EQ(output.first_fault_code, SafeRollController::AllocatorInvalidFault);
	EXPECT_NE(output.fault_flags & SafeRollController::FallbackLatchedFault, 0u);
	EXPECT_LE(std::fabs(output.u_selected - previous), input.config.du_max_nominal + 1e-6f);
	float last = output.u_selected;

	for (int i = 0; i < 10; ++i) {
		advance(input);
		output = controller.update(input);
		EXPECT_LE(std::fabs(output.u_selected - last), input.config.du_max_nominal + 1e-6f);
		last = output.u_selected;
	}

	input.allocator_valid = true;
	advance(input);
	output = controller.update(input);
	EXPECT_TRUE(output.fallback_active);
	EXPECT_EQ(output.active_code, 0);
	EXPECT_EQ(output.first_fault_code, SafeRollController::AllocatorInvalidFault);
	EXPECT_NE(output.fault_flags & SafeRollController::AllocatorInvalidFault, 0u);
	EXPECT_NE(output.fault_flags & SafeRollController::FallbackLatchedFault, 0u);
	EXPECT_EQ(output.execution_stage_mask, 0u);
}

TEST(SafeRollController, ManualReferenceFilterIsFiniteAndBounded)
{
	SafeRollController controller;
	auto input = validInput(1);
	input.phi_cmd = 0.3f;
	input.p = 0.1f;
	auto output = runHealthy(controller, input);
	EXPECT_TRUE(std::isfinite(output.phi_ref));
	EXPECT_LE(std::fabs(output.phi_ref_dot), input.config.reference_rate_max);
	EXPECT_LE(std::fabs(output.phi_ref_ddot), input.config.reference_accel_max);
	EXPECT_NE(output.phi_ref, 0.f);
}

TEST(SafeRollController, PointAndDisturbanceEstimatorsRemainCertified)
{
	SafeRollController controller;
	auto input = validInput(3);
	float executed = 0.f;
	bool saw_point_update = false;
	bool saw_certificate_update = false;

	for (int i = 0; i < 80; ++i) {
		const float dt = 0.0025f;
		executed = 0.12f * std::sin(0.31f * i);
		input.previous_u_effective_model = executed;
		input.p += dt * (input.config.a0 * input.p + 9.8f * executed + 0.02f);
		input.phi += dt * input.p;
		input.phi_cmd = 0.08f * std::sin(0.1f * i);
		const auto output = controller.update(input);
		saw_point_update |= output.b_update_active;
		saw_certificate_update |= output.certificate_update_active;
		advance(input);
	}

	const auto output = controller.update(input);
	EXPECT_TRUE(std::isfinite(output.b_hat));
	EXPECT_GE(output.b_hat, output.b_lower);
	EXPECT_LE(output.b_hat, output.b_upper);
	EXPECT_GE(output.rho_d, 0.f);
	EXPECT_GT(output.certified_width, -1e-6f);
	EXPECT_TRUE(saw_point_update);
	EXPECT_TRUE(saw_certificate_update);
}

TEST(SafeRollController, NominalFormula)
{
	const float value = SafeRollController::nominalControl(-0.5f, 0.2f, 0.03f, 0.4f,
			    15.f, 0.1f, 7.f, -0.05f, 10.f);
	EXPECT_NEAR(value, (-(-0.5f) * 0.2f - 0.03f + 0.4f + 1.5f - 0.35f) / 10.f, 1e-7f);
}

TEST(SafeRollController, FixedAndAdaptiveCbfIntervalsAreFinite)
{
	float h_upper = 0.f;
	float h_lower = 0.f;
	const auto fixed = SafeRollController::robustCbfInterval(math::radians(20.f), 0.1f, 0.2f, 0.0025f,
			   -0.5f, 0.02f, 0.06f, 10.1746f, 3.5f, 0.011f, 0.011f, 0.5f,
			   h_upper, h_lower);
	EXPECT_TRUE(fixed.valid);
	EXPECT_TRUE(std::isfinite(fixed.lower));
	EXPECT_TRUE(std::isfinite(fixed.upper));
	EXPECT_GT(h_upper, 0.f);
	EXPECT_GT(h_lower, 0.f);
}

TEST(SafeRollController, MinimumRelaxationAndProjection)
{
	const SafeRollController::Interval cbf{0.4f, 0.1f, true};
	const SafeRollController::Interval hard{-0.2f, 0.2f, true};
	const float epsilon = SafeRollController::minimumRelaxation(cbf, hard);
	EXPECT_FLOAT_EQ(epsilon, 0.2f);
	const SafeRollController::Interval relaxed{cbf.lower - epsilon, cbf.upper + epsilon, true};
	const auto safe = SafeRollController::intersect(relaxed, hard);
	EXPECT_TRUE(safe.valid);
	EXPECT_FLOAT_EQ(SafeRollController::project(1.f, safe), safe.upper);
}

TEST(SafeRollController, SingleLagCertificateIntersection)
{
	bool conflict = false;
	const auto refined = SafeRollController::refineCertificate({6.5f, 13.f, true},
			     1.0f, 0.1f, 0.1f, conflict);
	EXPECT_FALSE(conflict);
	EXPECT_NEAR(refined.lower, 9.f, 1e-6f);
	EXPECT_NEAR(refined.upper, 11.f, 1e-6f);
}

TEST(SafeRollController, MultiLagIntersectionIsSequential)
{
	bool conflict = false;
	auto refined = SafeRollController::refineCertificate({6.5f, 13.f, true},
		       1.0f, 0.1f, 0.2f, conflict);
	ASSERT_FALSE(conflict);
	refined = SafeRollController::refineCertificate(refined, 2.1f, 0.2f, 0.1f, conflict);
	EXPECT_FALSE(conflict);
	EXPECT_NEAR(refined.lower, 10.f, 1e-6f);
	EXPECT_NEAR(refined.upper, 11.f, 1e-6f);
}

TEST(SafeRollController, ConflictingLagIsDiscardedAndVariationScalesWithDt)
{
	bool conflict = false;
	const SafeRollController::Interval prior{9.f, 11.f, true};
	const auto retained = SafeRollController::refineCertificate(prior, 3.f, 0.1f, 0.01f, conflict);
	EXPECT_TRUE(conflict);
	EXPECT_FLOAT_EQ(retained.lower, prior.lower);
	EXPECT_FLOAT_EQ(retained.upper, prior.upper);
	EXPECT_FLOAT_EQ(SafeRollController::scaledVariationBound(0.006f, 0.005f), 0.012f);
	EXPECT_TRUE(std::isnan(SafeRollController::scaledVariationBound(-1.f, 0.0025f)));
}

TEST(SafeRollController, AllocationAwareAndAblationIntervals)
{
	SafeRollController allocated;
	auto allocated_input = validInput(3);
	const auto allocated_output = runHealthy(allocated, allocated_input);
	EXPECT_TRUE(allocated_output.u_alloc.valid);
	EXPECT_TRUE(allocated_output.interval_valid);
	EXPECT_TRUE(allocated_output.candidate_valid);

	SafeRollController ablation;
	auto ablation_input = validInput(4);
	const auto ablation_output = runHealthy(ablation, ablation_input);
	EXPECT_TRUE(ablation_output.interval_valid);
	EXPECT_FLOAT_EQ(ablation_output.u_alloc.lower, -ablation_input.config.u_max);
	EXPECT_FLOAT_EQ(ablation_output.u_alloc.upper, ablation_input.config.u_max);
}

TEST(SafeRollController, GenericContributionIsIncludedInCommandedFoil)
{
	SafeRollController controller;
	auto input = validInput(1);
	input.generic_servo4 = -0.1f;
	input.generic_servo5 = -0.1f;
	input.previous_final_servo4 += input.generic_servo4;
	input.previous_final_servo5 += input.generic_servo5;
	const auto output = runHealthy(controller, input);
	EXPECT_TRUE(output.candidate_valid);
	EXPECT_TRUE(std::isfinite(output.delta_left_commanded));
	EXPECT_TRUE(std::isfinite(output.delta_right_commanded));
	EXPECT_NE(output.delta_left_commanded, 0.f);
	EXPECT_NE(output.delta_right_commanded, 0.f);
}

TEST(SafeRollController, DremDefaultIsDegreesConvertedToRadians)
{
	EXPECT_NEAR(math::radians(0.05f), 0.00087266463f, 1e-10f);
	EXPECT_NEAR(SafeRollController::Config{}.foil_remainder, math::radians(0.05f), 1e-9f);
}

TEST(SafeRollController, ConfigurationCrcIsDeterministicAndSensitive)
{
	SafeRollController::Config config{};
	const uint32_t first = SafeRollController::configurationCrc32(3, config);
	EXPECT_EQ(first, SafeRollController::configurationCrc32(3, config));
	config.k1 += 0.01f;
	EXPECT_NE(first, SafeRollController::configurationCrc32(3, config));
	EXPECT_NE(first, SafeRollController::configurationCrc32(2, SafeRollController::Config{}));
}

TEST(SafeRollController, ResetSensitiveAndImmediateParametersDiffer)
{
	SafeRollController controller;
	auto input = validInput(3);
	auto output = runHealthy(controller, input);
	const uint32_t initial = output.reset_count;
	input.config.k1 += 1.f;
	advance(input);
	output = controller.update(input);
	EXPECT_EQ(output.reset_count, initial);
	input.config.a0 -= 0.1f;
	advance(input);
	output = controller.update(input);
	EXPECT_GT(output.reset_count, initial);
}

TEST(SafeRollController, NonfiniteAndEmptyInputsNeverReachOutput)
{
	SafeRollController controller;
	auto input = validInput(3);
	runHealthy(controller, input);
	input.p = NAN;
	advance(input);
	auto output = controller.update(input);
	EXPECT_TRUE(output.fallback_active);
	EXPECT_TRUE(std::isfinite(output.u_selected));

	SafeRollController second;
	input = validInput(3);
	input.config.foil_remainder = input.config.foil_angle_max;
	output = runHealthy(second, input);
	EXPECT_TRUE(output.fallback_active);
	EXPECT_TRUE(std::isfinite(output.u_selected));
}

TEST(SafeRollController, MapAndOwnershipAreDiagnosticOnly)
{
	SafeRollController controller;
	auto input = validInput(1);
	input.physical_mapping_verified = false;
	input.servo_ownership_verified = false;
	const auto output = runHealthy(controller, input);
	EXPECT_EQ(output.active_code, 1);
	EXPECT_TRUE(output.safe_roll_hardware_ready);
}

TEST(SafeRollController, FixedStorageAndBoundedObject)
{
	EXPECT_TRUE(std::is_standard_layout<SafeRollController::Input>::value);
	EXPECT_LT(sizeof(SafeRollController), 8192u);
}

TEST(SafeRollController, FullSyntheticTimeSeriesRegression)
{
	struct Golden {
		int sample;
		float b_hat;
		float b_lower;
		float b_upper;
		float rho_b;
		float d_hat;
		float rho_d;
		float u_nominal;
		float cbf_lower;
		float cbf_upper;
		float hard_lower;
		float hard_upper;
		float u_candidate;
		float epsilon;
		float psi_min;
	};
	const Golden golden[] {
		{50, 10.1745996f, 9.6262989f, 10.7229004f, 0.548300743f, 0.00942601636f,
		 0.0355176441f, 0.149276659f, -0.630777359f, 0.623061955f, -0.00447792141f,
		 0.0117763132f, 0.0117763132f, 0.f, 4.14341775e-5f},
		{100, 10.1745996f, 9.42362404f, 10.9255753f, 0.750975609f, 0.0013169141f,
		 0.0400025174f, 0.144710571f, -0.68004638f, 0.6549505f, 0.00136514287f,
		 0.0113043087f, 0.0113043087f, 0.f, 4.06299914e-5f},
		{200, 10.1815729f, 9.01876068f, 11.3304386f, 1.16281223f, -0.00300076883f,
		 0.0466901138f, -0.154422894f, -0.606174529f, 0.577005446f, -0.0117751528f,
		 -0.00230428949f, -0.0117751528f, 0.f, 4.15748073e-5f},
		{399, 10.1664858f, 8.21275997f, 12.1192455f, 1.95372581f, -0.00928424485f,
		 0.0562428273f, 0.170431644f, -0.515571237f, 0.506164849f, 0.00241514575f,
		 0.0122460211f, 0.0122460211f, 0.f, 4.04680686e-5f}
	};
	SafeRollController controller;
	auto input = validInput(3);
	float executed = 0.f;
	uint32_t crc = 0;
	unsigned golden_index = 0;

	for (int i = 0; i < 400; ++i) {
		const float dt = (i % 3 == 0) ? 0.0024f : ((i % 3 == 1) ? 0.0025f : 0.0026f);
		input.previous_u_effective_model = executed;
		input.p += dt * (input.config.a0 * input.p + 10.f * executed + 0.01f * std::sin(0.03f * i));
		input.phi += dt * input.p;
		input.phi_cmd = 0.12f * std::sin(0.02f * i);
		const auto output = controller.update(input);
		ASSERT_TRUE(std::isfinite(output.u_selected));
		if (golden_index < sizeof(golden) / sizeof(golden[0]) && i == golden[golden_index].sample) {
			const Golden &expected = golden[golden_index++];
			EXPECT_NEAR(output.b_hat, expected.b_hat, 2e-6f);
			EXPECT_NEAR(output.b_lower, expected.b_lower, 2e-6f);
			EXPECT_NEAR(output.b_upper, expected.b_upper, 2e-6f);
			EXPECT_NEAR(output.rho_b, expected.rho_b, 2e-6f);
			EXPECT_NEAR(output.d_hat, expected.d_hat, 2e-6f);
			EXPECT_NEAR(output.rho_d, expected.rho_d, 2e-6f);
			EXPECT_NEAR(output.u_nominal, expected.u_nominal, 2e-6f);
			EXPECT_NEAR(output.u_cbf.lower, expected.cbf_lower, 2e-6f);
			EXPECT_NEAR(output.u_cbf.upper, expected.cbf_upper, 2e-6f);
			EXPECT_NEAR(output.u_hard.lower, expected.hard_lower, 2e-6f);
			EXPECT_NEAR(output.u_hard.upper, expected.hard_upper, 2e-6f);
			EXPECT_NEAR(output.u_candidate, expected.u_candidate, 2e-6f);
			EXPECT_NEAR(output.epsilon_relax, expected.epsilon, 2e-6f);
			EXPECT_NEAR(output.psi_min, expected.psi_min, 2e-6f);
		}
		executed = output.u_effective_model;
		crc ^= output.configuration_crc32 + static_cast<uint32_t>(i);
		advance(input, dt);
	}

	EXPECT_NE(SafeRollController::configurationCrc32(3, input.config), 0u);
	EXPECT_EQ(golden_index, sizeof(golden) / sizeof(golden[0]));
}

TEST(SafeRollController, HostPerformanceReference)
{
	SafeRollController controller;
	auto input = validInput(3);
	constexpr int iterations = 5000;
	uint64_t maximum_ns = 0;
	volatile float sink = 0.f;
	const auto total_start = std::chrono::steady_clock::now();

	for (int i = 0; i < iterations; ++i) {
		input.phi_cmd = 0.1f * std::sin(0.01f * i);
		const auto start = std::chrono::steady_clock::now();
		const auto output = controller.update(input);
		const auto stop = std::chrono::steady_clock::now();
		maximum_ns = math::max(maximum_ns, static_cast<uint64_t>(
					       std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count()));
		sink += output.u_selected;
		input.previous_u_effective_model = output.u_effective_model;
		advance(input);
	}

	const auto total_stop = std::chrono::steady_clock::now();
	const uint64_t total_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(total_stop - total_start).count();
	std::printf("HOST_PERF safe_roll_candidate average_ns=%llu maximum_ns=%llu sink=%f\n",
		    static_cast<unsigned long long>(total_ns / iterations),
		    static_cast<unsigned long long>(maximum_ns), static_cast<double>(sink));
}

TEST(SafeRollController, ActivationWaitsForTwentyConsecutiveHealthyAllocatorSamples)
{
	for (uint8_t mode = 1; mode <= 3; ++mode) {
		SafeRollController controller;
		auto input = validInput(0);
		input.test_activation_bypass = false;
		controller.observeLegacy(input);
		advance(input);
		input.requested_mode = mode;
		input.allocator_valid = false;
		auto output = controller.update(input);
		EXPECT_TRUE(output.activation_pending);
		EXPECT_EQ(output.active_code, 0);
		EXPECT_EQ(output.first_fault_code, 0u);
		EXPECT_EQ(output.allocator_valid_streak, 0u);

		input.allocator_valid = true;

		for (uint16_t i = 0; i < SafeRollController::activationRequiredStreak() - 1u; ++i) {
			advance(input);
			output = controller.update(input);
			EXPECT_TRUE(output.activation_pending);
			EXPECT_EQ(output.active_code, 0);
			EXPECT_EQ(output.first_fault_code, 0u);
		}

		advance(input);
		output = controller.update(input);
		EXPECT_FALSE(output.activation_pending);
		EXPECT_EQ(output.active_code, mode);
		EXPECT_TRUE(output.candidate_valid);
		EXPECT_TRUE(output.output_enabled);
		EXPECT_FALSE(output.fallback_active);
		EXPECT_EQ(output.allocator_valid_streak, SafeRollController::activationRequiredStreak());
	}
}

TEST(SafeRollController, ActivationPendingIntermittentInvalidResetsStreakWithoutLatching)
{
	SafeRollController controller;
	auto input = validInput(0);
	input.test_activation_bypass = false;
	controller.observeLegacy(input);
	advance(input);
	input.requested_mode = 1;
	SafeRollController::Output output{};

	for (int i = 0; i < 12; ++i) {
		input.allocator_converged = i != 7;
		output = controller.update(input);
		EXPECT_TRUE(output.activation_pending);
		EXPECT_EQ(output.first_fault_code, 0u);
		EXPECT_EQ(output.fault_flags & SafeRollController::AllocatorInvalidFault, 0u);
		advance(input);
	}

	EXPECT_EQ(output.allocator_valid_streak, 4u);
	EXPECT_EQ(output.allocator_invalid_streak, 0u);
}

TEST(SafeRollController, ActiveFirstTwentyAllocatorDropsHoldLastFeasibleWithoutLatching)
{
	for (uint8_t mode = 1; mode <= 3; ++mode) {
		SafeRollController controller;
		auto input = validInput(mode);
		auto output = activateAfterHealthyStreak(controller, input, mode);
		ASSERT_EQ(output.active_code, mode);
		const float last_feasible = output.u_selected;
		input.legacy_u_roll = 0.8f;
		input.allocator_valid = false;

		for (uint16_t i = 1; i <= SafeRollController::allocatorHoldStreakLimit(); ++i) {
			advance(input);
			output = controller.update(input);
			EXPECT_EQ(output.active_code, mode);
			EXPECT_TRUE(output.allocator_degraded_active);
			EXPECT_FALSE(output.temporary_fallback_active);
			EXPECT_FALSE(output.fallback_active);
			EXPECT_EQ(output.first_fault_code, 0u);
			EXPECT_EQ(output.allocator_invalid_streak, i);
			EXPECT_FLOAT_EQ(output.last_feasible_u, last_feasible);
			EXPECT_LE(std::fabs(output.u_selected - last_feasible), 1e-6f);
			EXPECT_EQ(output.safety_degraded_active, mode == 2 || mode == 3);
		}

		EXPECT_GE(output.allocator_invalid_duration_us, 50'000u);
	}
}

TEST(SafeRollController, AllocatorDropsTwentyOneThroughEightyUseTemporaryLegacyAndEightyOneLatches)
{
	for (uint8_t mode = 1; mode <= 3; ++mode) {
		SafeRollController controller;
		auto input = validInput(mode);
		auto output = activateAfterHealthyStreak(controller, input, mode);
		ASSERT_EQ(output.active_code, mode);
		input.allocator_converged = false;

		for (uint16_t i = 1; i <= SafeRollController::allocatorInvalidStreakLimit(); ++i) {
			advance(input);
			output = controller.update(input);
			EXPECT_EQ(output.allocator_invalid_streak, i);

			if (i <= SafeRollController::allocatorHoldStreakLimit()) {
				EXPECT_TRUE(output.allocator_degraded_active);
				EXPECT_FALSE(output.temporary_fallback_active);
				EXPECT_EQ(output.active_code, mode);
				EXPECT_EQ(output.first_fault_code, 0u);

			} else if (i <= SafeRollController::allocatorTemporaryFallbackStreakLimit()) {
				EXPECT_FALSE(output.allocator_degraded_active);
				EXPECT_TRUE(output.temporary_fallback_active);
				EXPECT_TRUE(output.fallback_active);
				EXPECT_EQ(output.active_code, 0);
				EXPECT_EQ(output.first_fault_code, 0u);

			} else {
				EXPECT_EQ(output.first_fault_code, SafeRollController::AllocatorInvalidFault);
				EXPECT_NE(output.fault_flags & SafeRollController::FallbackLatchedFault, 0u);
			}
		}
	}
}

TEST(SafeRollController, AllocatorRecoveryRequiresTwentyHealthySamplesAndAutomaticallyReblends)
{
	for (uint8_t mode = 1; mode <= 3; ++mode) {
		SafeRollController controller;
		auto input = validInput(mode);
		auto output = activateAfterHealthyStreak(controller, input, mode);
		ASSERT_EQ(output.active_code, mode);
		input.allocator_valid = false;

		for (uint16_t i = 0; i < SafeRollController::allocatorHoldStreakLimit() + 1u; ++i) {
			advance(input);
			output = controller.update(input);
		}

		ASSERT_TRUE(output.temporary_fallback_active);
		input.allocator_valid = true;

		for (uint16_t i = 1; i < SafeRollController::allocatorRecoveryRequiredStreak(); ++i) {
			advance(input);
			output = controller.update(input);
			EXPECT_EQ(output.active_code, 0);
			EXPECT_TRUE(output.temporary_fallback_active);
			EXPECT_EQ(output.allocator_recovery_streak, i);
			EXPECT_EQ(output.first_fault_code, 0u);
		}

		advance(input);
		output = controller.update(input);
		EXPECT_EQ(output.active_code, mode);
		EXPECT_FALSE(output.temporary_fallback_active);
		EXPECT_FALSE(output.allocator_degraded_active);
		EXPECT_EQ(output.allocator_recovery_streak, SafeRollController::allocatorRecoveryRequiredStreak());
		EXPECT_EQ(output.allocator_invalid_duration_us, 0u);
		EXPECT_TRUE(output.candidate_valid);
		EXPECT_EQ(output.first_fault_code, 0u);
	}
}

TEST(SafeRollController, RecoveredNominalMovesAwayFromOldLastFeasibleAfterErrorReverses)
{
	SafeRollController controller;
	auto input = validInput(1);
	input.phi = 0.15f;
	auto output = activateAfterHealthyStreak(controller, input, 1);

	for (int i = 0; i < 80; ++i) {
		advance(input);
		output = controller.update(input);
	}

	ASSERT_TRUE(output.candidate_valid);
	ASSERT_LT(output.u_selected, -0.001f);
	const float old_last_feasible = output.last_feasible_u;
	input.legacy_u_roll = output.u_selected;
	input.allocator_valid = false;

	for (uint16_t i = 0; i < SafeRollController::allocatorHoldStreakLimit() + 1u; ++i) {
		advance(input);
		output = controller.update(input);
	}

	ASSERT_TRUE(output.temporary_fallback_active);
	input.phi = -0.15f;
	input.allocator_valid = true;

	for (uint16_t i = 0; i < SafeRollController::allocatorRecoveryRequiredStreak(); ++i) {
		advance(input);
		output = controller.update(input);
	}

	ASSERT_TRUE(output.candidate_valid);
	EXPECT_GT(output.u_nominal, 0.f);
	EXPECT_FLOAT_EQ(output.u_selected, old_last_feasible);

	advance(input);
	output = controller.update(input);
	EXPECT_TRUE(output.candidate_valid);
	EXPECT_GT(output.u_selected, old_last_feasible);
	EXPECT_LE(output.u_selected - old_last_feasible,
		  input.config.du_max_nominal * output.dt_used / 0.0025f + 1e-6f);
}

TEST(SafeRollController, ActiveAllocatorNonfiniteAndFoilGateClosedRemainImmediateFatal)
{
	for (uint8_t mode = 1; mode <= 3; ++mode) {
		for (int condition = 0; condition < 2; ++condition) {
			SafeRollController controller;
			auto input = validInput(mode);
			auto output = activateAfterHealthyStreak(controller, input, mode);
			ASSERT_EQ(output.active_code, mode);

			if (condition == 0) {
				input.allocator_finite = false;

			} else {
				input.foil_gate_enabled = false;
			}

			advance(input);
			output = controller.update(input);
			const uint32_t expected = condition == 0 ? SafeRollController::AllocatorInvalidFault
						  : SafeRollController::FoilGateDisabledFault;
			EXPECT_EQ(output.first_fault_code, expected);
			EXPECT_TRUE(output.fallback_active);
			EXPECT_EQ(output.active_code, 0);
		}
	}
}

TEST(SafeRollController, ActiveAllocatorStaleAndLogicalMappingInvalidRemainImmediateFatal)
{
	for (uint8_t mode = 1; mode <= 3; ++mode) {
		for (int condition = 0; condition < 2; ++condition) {
			SafeRollController controller;
			auto input = validInput(mode);
			auto output = activateAfterHealthyStreak(controller, input, mode);
			ASSERT_EQ(output.active_code, mode);

			if (condition == 0) {
				input.allocator_timestamp = input.now - 100'001;

			} else {
				input.logical_mapping_valid = false;
			}

			advance(input);

			if (condition == 0) {
				input.allocator_timestamp -= 2'500;
			}

			output = controller.update(input);
			const uint32_t expected = condition == 0 ? SafeRollController::AllocatorInvalidFault
						  : SafeRollController::LogicalMappingInvalidFault;
			EXPECT_EQ(output.first_fault_code, expected);
			EXPECT_TRUE(output.fallback_latched);
			EXPECT_EQ(output.active_code, 0);
		}
	}
}

TEST(SafeRollController, ModeFourRetainsImmediateAllocatorInvalidBehavior)
{
	SafeRollController controller;
	auto input = validInput(4);
	input.test_activation_bypass = false;
	auto output = runHealthy(controller, input);
	ASSERT_EQ(output.active_code, 4);
	input.allocator_valid = false;
	advance(input);
	output = controller.update(input);
	EXPECT_EQ(output.first_fault_code, SafeRollController::AllocatorInvalidFault);
	EXPECT_TRUE(output.fallback_latched);
	EXPECT_FALSE(output.allocator_degraded_active);
	EXPECT_FALSE(output.temporary_fallback_active);
}

TEST(SafeRollController, FieldModeFiniteBoundedAllocatorFailureContinuesForModesOneToThree)
{
	for (uint8_t mode = 1; mode <= 3; ++mode) {
		SafeRollController controller;
		auto input = validInput(mode);
		input.config.field_mode = true;
		auto output = activateAfterHealthyStreak(controller, input, mode);
		ASSERT_EQ(output.active_code, mode);
		input.allocator_valid = false;
		input.allocator_converged = false;
		input.allocator_bounded = true;

		for (int i = 0; i < 1200; ++i) {
			advance(input);
			output = controller.update(input);
			ASSERT_EQ(output.first_fault_code, 0u);
			ASSERT_FALSE(output.fallback_latched);
			ASSERT_EQ(output.active_code, mode);
			ASSERT_TRUE(output.output_enabled);
			ASSERT_TRUE(output.best_feasible_active);
			ASSERT_TRUE(output.field_mode_active);
		}

		EXPECT_GT(output.cumulative_best_feasible_time_us, 2'900'000u);
		EXPECT_GT(output.cumulative_requested_mode_active_time_us, 2'900'000u);
		EXPECT_EQ(output.safety_degraded_active, mode == 2 || mode == 3);
	}
}

TEST(SafeRollController, FieldModeUnboundedFiniteFailureTemporarilyFallsBackWithoutLatchAndRecovers)
{
	for (uint8_t mode = 1; mode <= 3; ++mode) {
		SafeRollController controller;
		auto input = validInput(mode);
		input.config.field_mode = true;
		auto output = activateAfterHealthyStreak(controller, input, mode);
		ASSERT_EQ(output.active_code, mode);
		input.allocator_valid = false;
		input.allocator_converged = false;
		input.allocator_bounded = false;

		for (int i = 1; i <= 400; ++i) {
			advance(input);
			output = controller.update(input);
			EXPECT_EQ(output.first_fault_code, 0u);
			EXPECT_FALSE(output.fallback_latched);

			if (i <= SafeRollController::allocatorHoldStreakLimit()) {
				EXPECT_EQ(output.active_code, mode);

			} else {
				EXPECT_EQ(output.active_code, 0);
				EXPECT_TRUE(output.temporary_fallback_active);
			}
		}

		EXPECT_GT(output.cumulative_temporary_fallback_time_us, 900'000u);
		input.allocator_valid = true;
		input.allocator_converged = true;
		input.allocator_bounded = true;

		for (uint16_t i = 0; i < SafeRollController::allocatorRecoveryRequiredStreak(); ++i) {
			advance(input);
			output = controller.update(input);
		}

		EXPECT_EQ(output.active_code, mode);
		EXPECT_FALSE(output.temporary_fallback_active);
		EXPECT_TRUE(output.candidate_valid);
	}
}

TEST(SafeRollController, FieldModeStillLatchesSevereFaults)
{
	for (int condition = 0; condition < 4; ++condition) {
		SafeRollController controller;
		auto input = validInput(1);
		input.config.field_mode = true;
		auto output = activateAfterHealthyStreak(controller, input, 1);
		ASSERT_EQ(output.active_code, 1);

		switch (condition) {
		case 0: input.allocator_finite = false; break;
		case 1: input.foil_gate_enabled = false; break;
		case 2: input.logical_mapping_valid = false; break;
		default: input.p = NAN; break;
		}

		advance(input);
		output = controller.update(input);
		EXPECT_TRUE(output.fallback_latched);
		EXPECT_NE(output.severe_fault_code, 0u);
		EXPECT_EQ(output.active_code, 0);
	}
}

TEST(SafeRollController, FoilGateFaultCapturesAndPreservesFirstFaultPrerequisiteSnapshot)
{
	SafeRollController controller;
	auto input = validInput(3);
	input.config.field_mode = true;
	input.foil_aux_parameter = 3;
	input.foil_aux_value = 0.72f;
	input.foil_threshold_parameter = 0.50f;
	auto output = activateAfterHealthyStreak(controller, input, 3);
	ASSERT_EQ(output.active_code, 3);

	// Match the shore observation: the allocator may be finite and bounded but
	// not strictly valid, while a separate gate drop is the severe trigger.
	input.allocator_valid = false;
	input.allocator_converged = false;
	input.allocator_finite = true;
	input.allocator_bounded = true;
	input.foil_gate_enabled = false;
	advance(input);
	output = controller.update(input);

	ASSERT_EQ(output.first_fault_code, SafeRollController::FoilGateDisabledFault);
	ASSERT_TRUE(output.fallback_latched);
	EXPECT_EQ(output.first_fault_stage_mask, 0u);
	const auto snapshot = output.first_fault_snapshot;
	EXPECT_EQ(snapshot.prerequisite_fault_flags, SafeRollController::FoilGateDisabledFault);
	EXPECT_EQ(snapshot.requested_mode, 3);
	EXPECT_TRUE(snapshot.mode_activated);
	EXPECT_FALSE(snapshot.activation_pending);
	EXPECT_TRUE(snapshot.in_stab);
	EXPECT_TRUE(snapshot.allocator_timestamp_valid);
	EXPECT_FALSE(snapshot.allocator_valid);
	EXPECT_FALSE(snapshot.allocator_converged);
	EXPECT_TRUE(snapshot.allocator_finite);
	EXPECT_TRUE(snapshot.allocator_bounded);
	EXPECT_FALSE(snapshot.foil_gate_enabled);
	EXPECT_TRUE(snapshot.foil_aux_above_threshold);
	EXPECT_EQ(snapshot.foil_aux_parameter, 3);
	EXPECT_FLOAT_EQ(snapshot.foil_aux_value, 0.72f);
	EXPECT_FLOAT_EQ(snapshot.foil_threshold_parameter, 0.50f);
	EXPECT_TRUE(snapshot.estimator_initialized);
	// The first candidate sample initializes previous-p causally; an innovation
	// history entry is only created on the following healthy candidate sample.
	EXPECT_EQ(snapshot.history_count, 0);

	// Internal reset edges preserve both the severe latch and its original
	// forensic snapshot.
	input.foil_gate_enabled = true;
	++input.attitude_reset_counter;
	advance(input);
	output = controller.update(input);
	EXPECT_TRUE(output.fallback_latched);
	EXPECT_EQ(output.first_fault_snapshot.allocator_timestamp, snapshot.allocator_timestamp);
	EXPECT_EQ(output.first_fault_snapshot.foil_gate_enabled, snapshot.foil_gate_enabled);
	EXPECT_FLOAT_EQ(output.first_fault_snapshot.b_hat, snapshot.b_hat);
}

TEST(SafeRollController, FieldAuxGateTransientInhibitsAndAutomaticallyRecoversModesTwoAndThree)
{
	for (uint8_t mode : {2, 3}) {
		SafeRollController controller;
		auto input = validInput(mode);
		input.config.field_mode = true;
		auto output = activateAfterHealthyStreak(controller, input, mode);
		ASSERT_EQ(output.active_code, mode);
		const uint32_t reset_id = input.config.reset_id;

		input.foil_aux_parameter = 6;
		input.foil_aux_value = -1.f;
		input.foil_threshold_parameter = -0.5f;
		input.aux_gate_enabled_raw = false;
		input.nav_gate_enabled_raw = true;
		input.aux_gate_data_valid = true;
		input.aux_gate_value_finite = true;
		input.foil_gate_enabled = false;
		advance(input);
		output = controller.update(input);
		EXPECT_EQ(output.requested_code, mode);
		EXPECT_EQ(output.active_code, 0);
		EXPECT_FALSE(output.output_enabled);
		EXPECT_TRUE(output.temporary_fallback_active);
		EXPECT_TRUE(output.aux_gate_inhibit_active);
		EXPECT_EQ(output.aux_gate_low_streak, 1);
		EXPECT_EQ(output.severe_fault_code, 0u);
		EXPECT_FALSE(output.fallback_latched);

		// A finite operator gate closure can remain active indefinitely without
		// being reclassified as a software fault.
		for (int i = 0; i < 1000; ++i) {
			advance(input);
			output = controller.update(input);
			ASSERT_EQ(output.requested_code, mode);
			ASSERT_EQ(output.active_code, 0);
			ASSERT_FALSE(output.output_enabled);
			ASSERT_TRUE(output.aux_gate_inhibit_active);
			ASSERT_EQ(output.severe_fault_code, 0u);
			ASSERT_FALSE(output.fallback_latched);
		}

		input.foil_aux_value = 0.00599f;
		input.aux_gate_enabled_raw = true;
		input.foil_gate_enabled = true;

		for (uint16_t i = 0; i < SafeRollController::allocatorRecoveryRequiredStreak(); ++i) {
			advance(input);
			output = controller.update(input);

			if (i + 1u < SafeRollController::allocatorRecoveryRequiredStreak()) {
				EXPECT_EQ(output.active_code, 0);
				EXPECT_TRUE(output.temporary_fallback_active);
				EXPECT_TRUE(output.aux_gate_inhibit_active);

			} else {
				EXPECT_EQ(output.active_code, mode);
				EXPECT_FALSE(output.temporary_fallback_active);
				EXPECT_FALSE(output.aux_gate_inhibit_active);
				EXPECT_TRUE(output.output_enabled);
			}

			EXPECT_EQ(output.severe_fault_code, 0u);
			EXPECT_FALSE(output.fallback_latched);
		}

		EXPECT_EQ(input.requested_mode, mode);
		EXPECT_EQ(input.config.reset_id, reset_id);
	}
}

TEST(SafeRollController, StrictAuxClosureAndFieldAuxInvalidOrNavClosureRemainSevere)
{
	{
		SafeRollController controller;
		auto input = validInput(3);
		input.config.field_mode = false;
		auto output = activateAfterHealthyStreak(controller, input, 3);
		ASSERT_EQ(output.active_code, 3);
		input.aux_gate_enabled_raw = false;
		input.foil_gate_enabled = false;
		input.foil_aux_value = -1.f;
		input.foil_threshold_parameter = -0.5f;
		advance(input);
		output = controller.update(input);
		EXPECT_EQ(output.first_fault_code, SafeRollController::FoilGateDisabledFault);
		EXPECT_TRUE(output.fallback_latched);
	}

	for (int condition = 0; condition < 3; ++condition) {
		SafeRollController controller;
		auto input = validInput(3);
		input.config.field_mode = true;
		auto output = activateAfterHealthyStreak(controller, input, 3);
		ASSERT_EQ(output.active_code, 3);

		if (condition == 0) {
			input.foil_aux_value = NAN;
			input.aux_gate_value_finite = false;
			input.aux_gate_enabled_raw = false;
			input.foil_gate_enabled = false;

		} else if (condition == 1) {
			input.aux_gate_data_valid = false;
			input.aux_gate_enabled_raw = false;
			input.foil_gate_enabled = false;

		} else {
			input.nav_gate_enabled_raw = false;
			input.foil_gate_enabled = false;
		}

		advance(input);
		output = controller.update(input);
		EXPECT_EQ(output.first_fault_code, SafeRollController::FoilGateDisabledFault);
		EXPECT_TRUE(output.fallback_latched);
		EXPECT_FALSE(output.aux_gate_inhibit_active);
	}
}

TEST(SafeRollController, InternalResetDoesNotClearSevereLatchButModeOrResetIdDoes)
{
	SafeRollController controller;
	auto input = validInput(1);
	input.config.field_mode = true;
	auto output = activateAfterHealthyStreak(controller, input, 1);
	ASSERT_EQ(output.active_code, 1);
	input.foil_gate_enabled = false;
	advance(input);
	output = controller.update(input);
	ASSERT_TRUE(output.fallback_latched);
	const uint32_t severe = output.severe_fault_code;

	input.foil_gate_enabled = true;
	++input.attitude_reset_counter;
	advance(input);
	output = controller.update(input);
	EXPECT_TRUE(output.fallback_latched);
	EXPECT_EQ(output.severe_fault_code, severe);

	input.requested_mode = 2;
	advance(input);
	output = controller.update(input);
	EXPECT_FALSE(output.fallback_latched);
	EXPECT_EQ(output.severe_fault_code, 0u);

	++input.config.reset_id;
	advance(input);
	output = controller.update(input);
	EXPECT_FALSE(output.fallback_latched);
}

TEST(SafeRollController, FieldModeChangesConfigurationCrc)
{
	auto input = validInput(1);
	const uint32_t strict_crc = SafeRollController::configurationCrc32(input.requested_mode, input.config);
	input.config.field_mode = true;
	const uint32_t field_crc = SafeRollController::configurationCrc32(input.requested_mode, input.config);
	EXPECT_NE(strict_crc, field_crc);
}

TEST(SafeRollController, FieldModeFiniteSaturationIsDegradedButNotFatal)
{
	SafeRollController controller;
	auto input = validInput(2);
	input.config.field_mode = true;
	auto output = activateAfterHealthyStreak(controller, input, 2);
	ASSERT_EQ(output.active_code, 2);
	input.allocator_saturation_mask = hydrofoil::DeltaRight | hydrofoil::ThrustLeftHigh;
	advance(input);
	output = controller.update(input);
	EXPECT_EQ(output.active_code, 2);
	EXPECT_TRUE(output.best_feasible_active);
	EXPECT_TRUE(output.safety_degraded_active);
	EXPECT_FALSE(output.fallback_latched);
	EXPECT_EQ(output.severe_fault_code, 0u);
}

TEST(SafeRollController, FieldModeStaleAndHardRuntimeRemainFatal)
{
	{
		SafeRollController controller;
		auto input = validInput(1);
		input.config.field_mode = true;
		auto output = activateAfterHealthyStreak(controller, input, 1);
		ASSERT_EQ(output.active_code, 1);
		advance(input);
		input.attitude_timestamp = input.now - 50'001;
		output = controller.update(input);
		EXPECT_EQ(output.severe_fault_code, SafeRollController::AttitudeStale);
		EXPECT_TRUE(output.fallback_latched);
	}

	{
		SafeRollController controller;
		auto input = validInput(1);
		input.config.field_mode = true;
		auto output = activateAfterHealthyStreak(controller, input, 1);
		ASSERT_EQ(output.active_code, 1);
		advance(input);
		input.test_candidate_time_us = SafeRollController::runtimeHardBudgetUs() + 1u;
		output = controller.update(input);
		EXPECT_EQ(output.severe_fault_code, SafeRollController::RuntimeOverrunFault);
		EXPECT_TRUE(output.fallback_latched);
	}
}

TEST(SafeRollController, FieldModeSoftRuntimePressureDoesNotLatchButHardDeadlineDoes)
{
	SafeRollController controller;
	auto input = validInput(2);
	input.config.field_mode = true;
	auto output = activateAfterHealthyStreak(controller, input, 2);
	ASSERT_EQ(output.active_code, 2);

	for (int i = 0; i < 6; ++i) {
		advance(input);
		input.test_candidate_time_us = SafeRollController::runtimeSoftBudgetUs() + 200u;
		output = controller.update(input);
		EXPECT_EQ(output.severe_fault_code, 0u);
		EXPECT_FALSE(output.fallback_latched);
		EXPECT_EQ(output.active_code, 2);
	}

	advance(input);
	input.test_candidate_time_us = SafeRollController::runtimeHardBudgetUs() + 1u;
	output = controller.update(input);
	EXPECT_EQ(output.severe_fault_code, SafeRollController::RuntimeOverrunFault);
	EXPECT_TRUE(output.fallback_latched);
	EXPECT_EQ(output.active_code, 0);
}

TEST(SafeRollController, FieldModeFiniteCbfProjectionConflictIsBestFeasibleForModesTwoAndThree)
{
	for (uint8_t mode = 2; mode <= 3; ++mode) {
		SafeRollController controller;
		auto input = validInput(mode);
		input.config.field_mode = true;
		auto output = activateAfterHealthyStreak(controller, input, mode);
		ASSERT_EQ(output.active_code, mode);

		// This produces two finite intervals whose minimum-relaxation boundary
		// differs by a few float ulps. Before the FIELD classification fix the
		// empty post-relaxation intersection set ProjectionInvalidFault (0x80000).
		input.phi = math::radians(-30.f);
		input.p = 0.f;
		input.phi_cmd = math::radians(40.f);
		advance(input);
		output = controller.update(input);

		EXPECT_EQ(output.severe_fault_code, 0u);
		EXPECT_EQ(output.fault_flags & SafeRollController::ProjectionInvalidFault, 0u);
		EXPECT_FALSE(output.fallback_latched);
		EXPECT_EQ(output.requested_code, mode);
		EXPECT_EQ(output.active_code, mode);
		EXPECT_TRUE(output.candidate_valid);
		EXPECT_TRUE(output.interval_valid);
		EXPECT_TRUE(output.best_feasible_active);
		EXPECT_TRUE(output.safety_degraded_active);
		EXPECT_FALSE(output.temporary_fallback_active);
		EXPECT_TRUE(std::isfinite(output.u_selected));
	}
}

TEST(SafeRollController, StrictModeFiniteCbfProjectionConflictRetainsProjectionFault)
{
	for (uint8_t mode = 2; mode <= 3; ++mode) {
		SafeRollController controller;
		auto input = validInput(mode);
		auto output = activateAfterHealthyStreak(controller, input, mode);
		ASSERT_EQ(output.active_code, mode);
		input.phi = math::radians(-30.f);
		input.p = 0.f;
		input.phi_cmd = math::radians(40.f);
		advance(input);
		output = controller.update(input);

		EXPECT_EQ(output.first_fault_code, SafeRollController::ProjectionInvalidFault);
		EXPECT_TRUE(output.fallback_latched);
		EXPECT_EQ(output.active_code, 0);
		EXPECT_FALSE(output.best_feasible_active);
		EXPECT_FALSE(output.temporary_fallback_active);
		EXPECT_FALSE(output.safety_degraded_active);
	}
}

TEST(SafeRollController, FieldModeFiniteNonlinearFailureTemporarilyFallsBackAndAutomaticallyRecoversModesTwoAndThree)
{
	for (uint8_t mode = 2; mode <= 3; ++mode) {
		SafeRollController controller;
		auto input = validInput(mode);
		input.config.field_mode = true;
		auto output = activateAfterHealthyStreak(controller, input, mode);
		ASSERT_EQ(output.active_code, mode);
		ASSERT_TRUE(output.candidate_valid);
		const float feasible_before_failure = output.last_feasible_u;

		// Deterministically emulate a finite nonlinear validation miss while the
		// externally reported allocator sample remains healthy.
		input.test_force_finite_nonlinear_failure = true;
		input.phi = math::radians(12.f);
		input.p = math::radians(80.f);
		input.phi_cmd = math::radians(-40.f);

		for (uint16_t i = 1; i <= SafeRollController::allocatorHoldStreakLimit() + 1u; ++i) {
			advance(input);
			output = controller.update(input);
			EXPECT_EQ(output.severe_fault_code, 0u);
			EXPECT_FALSE(output.fallback_latched);
			EXPECT_EQ(output.requested_code, mode);
			EXPECT_EQ(output.allocator_invalid_streak, i);

			if (i <= SafeRollController::allocatorHoldStreakLimit()) {
				EXPECT_EQ(output.active_code, mode);
				EXPECT_FALSE(output.temporary_fallback_active);
				EXPECT_NEAR(output.u_selected, feasible_before_failure,
					    input.config.du_max_nominal * static_cast<float>(i) + 1e-5f);

			} else {
				EXPECT_EQ(output.active_code, 0);
				EXPECT_TRUE(output.temporary_fallback_active);
				EXPECT_TRUE(output.safety_degraded_active);
				EXPECT_FALSE(output.best_feasible_active);
			}
		}

		input.test_force_finite_nonlinear_failure = false;

		for (uint16_t i = 0; i < SafeRollController::allocatorRecoveryRequiredStreak(); ++i) {
			advance(input);
			output = controller.update(input);
		}

		EXPECT_EQ(output.requested_code, mode);
		EXPECT_EQ(output.active_code, mode);
		EXPECT_TRUE(output.candidate_valid);
		EXPECT_FALSE(output.temporary_fallback_active);
		EXPECT_FALSE(output.fallback_latched);
		EXPECT_EQ(output.severe_fault_code, 0u);
	}
}

TEST(SafeRollController, StrictModeFiniteNonlinearFailureRetainsSevereSemantics)
{
	for (uint8_t mode = 2; mode <= 3; ++mode) {
		SafeRollController controller;
		auto input = validInput(mode);
		auto output = activateAfterHealthyStreak(controller, input, mode);
		ASSERT_EQ(output.active_code, mode);
		input.test_force_finite_nonlinear_failure = true;
		input.phi = math::radians(2.f);
		input.p = 0.f;
		input.phi_cmd = 0.f;
		advance(input);
		output = controller.update(input);

		EXPECT_EQ(output.first_fault_code, SafeRollController::NonlinearCandidateInvalidFault);
		EXPECT_EQ(output.severe_fault_code, SafeRollController::NonlinearCandidateInvalidFault);
		EXPECT_TRUE(output.fallback_latched);
		EXPECT_EQ(output.active_code, 0);
		EXPECT_FALSE(output.best_feasible_active);
		EXPECT_FALSE(output.temporary_fallback_active);
		EXPECT_FALSE(output.safety_degraded_active);
	}
}

TEST(SafeRollController, FieldModeFiniteEmptyHardIntervalDegradesWithoutSevereLatch)
{
	SafeRollController controller;
	auto input = validInput(2);
	input.config.field_mode = true;
	input.config.u_max = 0.1f;
	input.config.du_max_nominal = 0.0001f;
	input.legacy_u_roll = 0.2f;
	const auto output = activateAfterHealthyStreak(controller, input, 2);

	EXPECT_EQ(output.severe_fault_code, 0u);
	EXPECT_EQ(output.fault_flags & SafeRollController::EmptyHardIntervalFault, 0u);
	EXPECT_FALSE(output.fallback_latched);
	EXPECT_EQ(output.requested_code, 2);
	EXPECT_TRUE(output.allocator_degraded_active || output.temporary_fallback_active);
	EXPECT_TRUE(output.safety_degraded_active);
}

TEST(SafeRollController, FieldModeNonfiniteNonlinearCandidateRemainsSevereAndClearsDegradationStates)
{
	SafeRollController controller;
	auto input = validInput(3);
	input.config.field_mode = true;
	auto output = activateAfterHealthyStreak(controller, input, 3);
	ASSERT_EQ(output.active_code, 3);
	input.allocator_saturation_mask = hydrofoil::DeltaLeft;
	input.allocation_command.force_x = NAN;
	advance(input);
	output = controller.update(input);

	EXPECT_EQ(output.first_fault_code, SafeRollController::NonlinearCandidateInvalidFault);
	EXPECT_EQ(output.severe_fault_code, SafeRollController::NonlinearCandidateInvalidFault);
	EXPECT_TRUE(output.fallback_latched);
	EXPECT_EQ(output.active_code, 0);
	EXPECT_FALSE(output.best_feasible_active);
	EXPECT_FALSE(output.temporary_fallback_active);
	EXPECT_FALSE(output.safety_degraded_active);
}
