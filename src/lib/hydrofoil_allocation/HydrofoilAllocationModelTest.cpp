/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "HydrofoilAllocationModel.hpp"
#include "SpeedContextCache.hpp"

#include <mathlib/mathlib.h>

#include <cmath>
#include <chrono>
#include <cstdio>

using hydrofoil::AllocationCommand;
using hydrofoil::AllocationContext;
using hydrofoil::AllocationSolution;
using hydrofoil::HydrofoilAllocationModel;
using hydrofoil::SpeedContextCache;
using matrix::SquareMatrix;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::Vector4f;

namespace
{
AllocationContext defaultContext()
{
	AllocationContext context{};
	context.lift_coefficient = 2.f;
	context.speed_squared = 9.f;
	context.body_pitch = 0.08f;
	context.max_hydrofoil_angle = 1.134464f;
	context.max_thruster_thrust = 30.f;
	context.available_thruster_thrust = 30.f;
	const float values[4][3] {
		{0.f, 0.f, 30.f},
		{-10.f, 0.f, 0.f},
		{0.f, 0.f, -30.f},
		{10.f, 0.f, 0.f}
	};
	context.virtual_control_matrix = matrix::Matrix<float, 4, 3>(values);
	return context;
}

struct LegacySolution {
	float delta_left;
	float delta_right;
	float thrust_left;
	float thrust_right;
};

Vector2f legacyResidual(const Vector2f &x, const AllocationContext &context, float force_x, float force_z)
{
	Vector2f out;
	out(0) = force_x - context.lift_coefficient * context.speed_squared * (x(0) + context.body_pitch)
		 * std::sin(context.body_pitch) - x(1) * std::cos(x(0));
	out(1) = force_z + context.lift_coefficient * context.speed_squared * (x(0) + context.body_pitch)
		 * std::cos(context.body_pitch) + x(1) * std::sin(x(0));
	return out;
}

SquareMatrix<float, 2> legacyJacobian(const Vector2f &x, const AllocationContext &context)
{
	SquareMatrix<float, 2> J;
	J(0, 0) = -context.lift_coefficient * context.speed_squared * std::sin(context.body_pitch)
		  + x(1) * std::sin(x(0));
	J(0, 1) = -std::cos(x(0));
	J(1, 0) = context.lift_coefficient * context.speed_squared * std::cos(context.body_pitch)
		  + x(1) * std::cos(x(0));
	J(1, 1) = std::sin(x(0));
	return J;
}

void legacySolveSide(Vector2f &x, const AllocationContext &context, float force_x, float force_z)
{
	for (int i = 0; i < 10; ++i) {
		SquareMatrix<float, 2> inverse;
		matrix::inv(legacyJacobian(x, context), inverse);
		const Vector2f step = -inverse * legacyResidual(x, context, force_x, force_z);

		if (step.norm_squared() < 1e-6f) {
			break;
		}

		x += step;
		x(0) = math::constrain(x(0), -context.max_hydrofoil_angle, context.max_hydrofoil_angle);
		x(1) = math::constrain(x(1), 0.f, context.available_thruster_thrust);
	}
}

LegacySolution legacySolve(const AllocationContext &context, const AllocationCommand &command)
{
	const Vector3f u(command.u_phi, command.u_theta, command.u_psi);
	Vector4f forces = context.virtual_control_matrix * u;
	forces += Vector4f(command.force_x * 0.5f, command.force_z * 0.5f,
			 command.force_x * 0.5f, command.force_z * 0.5f);
	float sum_force_x = forces(0) + forces(2);

	if (sum_force_x < 0.f) {
		sum_force_x *= 0.5f;
		forces(0) -= sum_force_x;
		forces(2) -= sum_force_x;
	}

	if (forces(0) < 0.f) {
		forces(2) += forces(0);
		forces(0) = 0.f;

	} else if (forces(2) < 0.f) {
		forces(0) += forces(2);
		forces(2) = 0.f;
	}

	Vector2f left(0.f, command.force_x * 0.5f);
	Vector2f right(0.f, command.force_x * 0.5f);
	legacySolveSide(left, context, forces(0), forces(1));
	legacySolveSide(right, context, forces(2), forces(3));
	return {left(0), right(0), left(1), right(1)};
}

uint32_t mirroredSaturationMask(uint32_t mask)
{
	uint32_t mirrored = mask & hydrofoil::ForceXRedistributed;

	if (mask & hydrofoil::DeltaLeft) { mirrored |= hydrofoil::DeltaRight; }

	if (mask & hydrofoil::DeltaRight) { mirrored |= hydrofoil::DeltaLeft; }

	if (mask & hydrofoil::ThrustLeftLow) { mirrored |= hydrofoil::ThrustRightLow; }

	if (mask & hydrofoil::ThrustLeftHigh) { mirrored |= hydrofoil::ThrustRightHigh; }

	if (mask & hydrofoil::ThrustRightLow) { mirrored |= hydrofoil::ThrustLeftLow; }

	if (mask & hydrofoil::ThrustRightHigh) { mirrored |= hydrofoil::ThrustLeftHigh; }

	return mirrored;
}
} // namespace

TEST(HydrofoilAllocationModel, LegacyRegressionNormalRegion)
{
	AllocationContext context = defaultContext();
	const AllocationCommand commands[] {
		{20.f, -2.f, 0.f, 0.f, 0.f},
		{30.f, 0.f, 0.3f, 0.f, 0.f},
		{30.f, 0.f, -0.3f, 0.f, 0.f},
		{24.f, 4.f, 0.2f, 0.f, 0.1f},
		{12.f, -4.f, -0.15f, 0.f, -0.1f}
	};

	for (const AllocationCommand &command : commands) {
		const LegacySolution legacy = legacySolve(context, command);
		const AllocationSolution shared = HydrofoilAllocationModel::solve(context, command);
		ASSERT_TRUE(shared.status.valid);
		EXPECT_NEAR(shared.delta_left_commanded, legacy.delta_left, 1e-6f);
		EXPECT_NEAR(shared.delta_right_commanded, legacy.delta_right, 1e-6f);
		EXPECT_NEAR(shared.thrust_left_commanded, legacy.thrust_left, 1e-5f);
		EXPECT_NEAR(shared.thrust_right_commanded, legacy.thrust_right, 1e-5f);
	}
}

TEST(HydrofoilAllocationModel, RegressionAcrossPitchThrottleAndSaturation)
{
	for (float pitch : {-0.2f, 0.f, 0.2f}) {
		for (float available_thrust : {8.f, 20.f, 30.f}) {
			AllocationContext context = defaultContext();
			context.body_pitch = pitch;
			context.available_thruster_thrust = available_thrust;

			for (float u_phi : {-0.8f, -0.2f, 0.f, 0.2f, 0.8f}) {
				const AllocationCommand command{18.f, 0.f, u_phi, 0.f, 0.f};
				const LegacySolution legacy = legacySolve(context, command);
				const AllocationSolution shared = HydrofoilAllocationModel::solve(context, command);
				EXPECT_NEAR(shared.delta_left_commanded, legacy.delta_left, 1e-5f);
				EXPECT_NEAR(shared.delta_right_commanded, legacy.delta_right, 1e-5f);
				EXPECT_NEAR(shared.thrust_left_commanded, legacy.thrust_left, 1e-4f);
				EXPECT_NEAR(shared.thrust_right_commanded, legacy.thrust_right, 1e-4f);
			}
		}
	}
}

TEST(HydrofoilAllocationModel, ForwardAndEffectiveInputAreModelBased)
{
	const AllocationContext context = defaultContext();
	const AllocationCommand command{24.f, 2.f, 0.25f, 0.f, 0.05f};
	const AllocationSolution solution = HydrofoilAllocationModel::solve(context, command);
	ASSERT_TRUE(solution.status.valid);
	const auto forward = HydrofoilAllocationModel::forward(context, solution);
	ASSERT_TRUE(forward.valid);
	EXPECT_NEAR(forward.u_phi_effective_model, command.u_phi, 2e-3f);
	EXPECT_LT(forward.reconstruction_residual, 0.05f);
}

TEST(HydrofoilAllocationModel, RollReplacementPreservesEveryNonRollCommandBitExactly)
{
	const AllocationCommand original{17.25f, -8.5f, -0.4f, 0.375f, -0.125f};
	const AllocationCommand replaced = HydrofoilAllocationModel::withRollCommand(original, 0.625f);
	EXPECT_FLOAT_EQ(replaced.force_x, original.force_x);
	EXPECT_FLOAT_EQ(replaced.force_z, original.force_z);
	EXPECT_FLOAT_EQ(replaced.u_theta, original.u_theta);
	EXPECT_FLOAT_EQ(replaced.u_psi, original.u_psi);
	EXPECT_FLOAT_EQ(replaced.u_phi, 0.625f);

	const AllocationSolution checked =
		HydrofoilAllocationModel::checkCandidate(defaultContext(), original, 0.625f);
	EXPECT_FLOAT_EQ(checked.source_command.force_x, original.force_x);
	EXPECT_FLOAT_EQ(checked.source_command.force_z, original.force_z);
	EXPECT_FLOAT_EQ(checked.source_command.u_theta, original.u_theta);
	EXPECT_FLOAT_EQ(checked.source_command.u_psi, original.u_psi);
	EXPECT_FLOAT_EQ(checked.source_command.u_phi, 0.625f);
}

TEST(HydrofoilAllocationModel, SymmetricDepthAndDifferentialRollComponentsRoundTrip)
{
	for (float symmetric : {-0.4f, 0.f, 0.35f}) {
		for (float differential : {-0.2f, 0.f, 0.25f}) {
			float left = NAN;
			float right = NAN;
			ASSERT_TRUE(HydrofoilAllocationModel::composeFoilCommand(symmetric, differential, left, right));
			const auto components = HydrofoilAllocationModel::decomposeFoilCommand(left, right);
			ASSERT_TRUE(components.valid);
			EXPECT_FLOAT_EQ(components.symmetric, symmetric);
			EXPECT_FLOAT_EQ(components.differential, differential);
		}
	}

	float left = 0.f;
	float right = 0.f;
	EXPECT_FALSE(HydrofoilAllocationModel::composeFoilCommand(NAN, 0.f, left, right));
	EXPECT_FALSE(HydrofoilAllocationModel::decomposeFoilCommand(0.f, NAN).valid);
}

TEST(HydrofoilAllocationModel, NonzeroDepthForceProducesCommonFoilComponentWithZeroRoll)
{
	AllocationContext context = defaultContext();
	context.body_pitch = 0.f;
	const AllocationCommand command{20.f, -4.f, 0.f, 0.f, 0.f};
	const AllocationSolution solution = HydrofoilAllocationModel::solve(context, command);
	ASSERT_TRUE(solution.status.valid);
	const auto components = HydrofoilAllocationModel::decomposeFoilCommand(
					solution.delta_left_commanded, solution.delta_right_commanded);
	ASSERT_TRUE(components.valid);
	EXPECT_NE(std::fabs(components.symmetric), 0.f);
	EXPECT_NEAR(components.differential, 0.f, 1e-6f);
}

TEST(HydrofoilAllocationModel, AnalyticSensitivityMatchesCentralDifference)
{
	const AllocationContext context = defaultContext();
	const AllocationCommand command{24.f, 0.f, 0.1f, 0.f, 0.f};
	const AllocationSolution solution = HydrofoilAllocationModel::solve(context, command);
	ASSERT_TRUE(solution.status.valid);
	const auto linearization = HydrofoilAllocationModel::linearize(context, solution);
	ASSERT_TRUE(linearization.valid);
	constexpr float epsilon = 1e-3f;
	const AllocationSolution plus = HydrofoilAllocationModel::checkCandidate(context, command, command.u_phi + epsilon);
	const AllocationSolution minus = HydrofoilAllocationModel::checkCandidate(context, command, command.u_phi - epsilon);
	ASSERT_TRUE(plus.status.valid);
	ASSERT_TRUE(minus.status.valid);
	EXPECT_NEAR(linearization.g_left,
		    (plus.delta_left_commanded - minus.delta_left_commanded) / (2.f * epsilon), 2e-3f);
	EXPECT_NEAR(linearization.g_right,
		    (plus.delta_right_commanded - minus.delta_right_commanded) / (2.f * epsilon), 2e-3f);
}

TEST(HydrofoilAllocationModel, InvalidSingularNonFiniteAndNonConverged)
{
	AllocationContext singular = defaultContext();
	singular.lift_coefficient = 0.f;
	const AllocationSolution singular_solution = HydrofoilAllocationModel::solve(singular, {});
	EXPECT_FALSE(singular_solution.status.valid);

	AllocationContext nonfinite = defaultContext();
	nonfinite.speed_squared = NAN;
	EXPECT_FALSE(HydrofoilAllocationModel::solve(nonfinite, {}).status.valid);

	AllocationContext saturated = defaultContext();
	saturated.max_hydrofoil_angle = 0.01f;
	saturated.available_thruster_thrust = 0.1f;
	const AllocationSolution nonconverged = HydrofoilAllocationModel::solve(saturated,
		AllocationCommand{100.f, 100.f, 1.f, 0.f, 0.f});
	EXPECT_FALSE(nonconverged.status.valid);
	EXPECT_NE(nonconverged.status.saturation_mask, 0u);
}

TEST(HydrofoilAllocationModel, BoundedNonconvergedCompleteSolutionRemainsExecutable)
{
	const AllocationContext context = defaultContext();
	AllocationSolution candidate{};
	candidate.delta_left_commanded = 0.4f;
	candidate.delta_right_commanded = 0.2f;
	candidate.thrust_left_commanded = 5.f;
	candidate.thrust_right_commanded = 6.f;
	candidate.status.finite = true;
	candidate.status.valid = false;
	candidate.status.converged = false;
	candidate.status.solve_status = hydrofoil::AllocationSolveStatus::NotConverged;

	for (uint8_t mode : {0u, 1u, 2u, 3u}) {
		const bool allow_bounded_best_feasible = mode <= 3;
		const auto selected = HydrofoilAllocationModel::selectExecutable(
					      context, candidate, allow_bounded_best_feasible,
					      0.f, 0.f, 1.f, 0.1f);
		ASSERT_TRUE(selected.usable) << "mode=" << unsigned(mode);
		EXPECT_TRUE(selected.best_feasible);
		EXPECT_TRUE(selected.rate_limited);
		EXPECT_FLOAT_EQ(selected.solution.delta_left_commanded, 0.1f);
		EXPECT_FLOAT_EQ(selected.solution.delta_right_commanded, 0.1f);
		EXPECT_FLOAT_EQ(selected.solution.thrust_left_commanded, candidate.thrust_left_commanded);
		EXPECT_FLOAT_EQ(selected.solution.thrust_right_commanded, candidate.thrust_right_commanded);
	}

	const auto strict_mode_four = HydrofoilAllocationModel::selectExecutable(
					      context, candidate, false, 0.f, 0.f, 1.f, 0.1f);
	EXPECT_FALSE(strict_mode_four.usable);
}

TEST(HydrofoilAllocationModel, AllocationModeMatrixPreservesCompleteDepthForModesZeroThroughThree)
{
	hydrofoil::ExecutableAllocation executable{};
	executable.usable = true;
	executable.best_feasible = true;
	executable.solution.delta_left_commanded = 0.45f;
	executable.solution.delta_right_commanded = 0.15f;
	executable.solution.thrust_left_commanded = 7.f;
	executable.solution.thrust_right_commanded = 9.f;
	executable.solution.status.finite = true;
	executable.solution.status.valid = false;
	executable.solution.status.converged = false;
	executable.solution.status.solve_status = hydrofoil::AllocationSolveStatus::NotConverged;

	AllocationSolution fixed{};
	fixed.delta_left_commanded = -0.1f;
	fixed.delta_right_commanded = -0.1f;
	fixed.thrust_left_commanded = 1.f;
	fixed.thrust_right_commanded = 2.f;
	AllocationSolution manual{};
	manual.delta_left_commanded = -0.3f;
	manual.delta_right_commanded = 0.3f;
	manual.thrust_left_commanded = 3.f;
	manual.thrust_right_commanded = 4.f;

	for (uint8_t safe_roll_mode : {0u, 1u, 2u, 3u}) {
		for (int32_t allocate_mode : {0, 1, 2}) {
			const auto selected = HydrofoilAllocationModel::selectAllocationModeOutput(
						      executable, allocate_mode, safe_roll_mode, fixed, manual);
			ASSERT_TRUE(selected.selected);
			EXPECT_EQ(selected.source, hydrofoil::AllocationOutputSource::CompleteExecutable);
			EXPECT_FALSE(selected.mode_branch_overrode_solution);
			EXPECT_FLOAT_EQ(selected.solution.delta_left_commanded, 0.45f);
			EXPECT_FLOAT_EQ(selected.solution.delta_right_commanded, 0.15f);
			EXPECT_FLOAT_EQ(selected.solution.thrust_left_commanded, 7.f);
			EXPECT_FLOAT_EQ(selected.solution.thrust_right_commanded, 9.f);
			const auto components = HydrofoilAllocationModel::decomposeFoilCommand(
							selected.solution.delta_left_commanded,
							selected.solution.delta_right_commanded);
			ASSERT_TRUE(components.valid);
			EXPECT_FLOAT_EQ(components.symmetric, 0.3f);
			EXPECT_FLOAT_EQ(components.differential, 0.15f);
			EXPECT_FALSE(selected.solution.status.valid);
		}
	}
}

TEST(HydrofoilAllocationModel, AllocationModeMatrixPreservesSolvedDepthAndRollComposition)
{
	const AllocationContext context = defaultContext();
	const AllocationCommand command{18.f, 12.f, 0.35f, 0.f, 0.f};
	const AllocationSolution solved = HydrofoilAllocationModel::solve(context, command);
	const auto executable = HydrofoilAllocationModel::selectExecutable(
					context, solved, true, 0.f, 0.f, 1000.f, 0.0025f);
	ASSERT_TRUE(executable.usable);
	const auto expected = HydrofoilAllocationModel::decomposeFoilCommand(
				      executable.solution.delta_left_commanded,
				      executable.solution.delta_right_commanded);
	ASSERT_TRUE(expected.valid);
	EXPECT_NE(expected.symmetric, 0.f);
	EXPECT_NE(expected.differential, 0.f);

	AllocationSolution fixed{};
	fixed.delta_left_commanded = -0.1f;
	fixed.delta_right_commanded = -0.1f;
	AllocationSolution manual{};
	manual.delta_left_commanded = -0.3f;
	manual.delta_right_commanded = 0.3f;

	for (uint8_t safe_roll_mode : {0u, 1u, 2u, 3u}) {
		for (int32_t allocate_mode : {0, 1, 2}) {
			const auto selected = HydrofoilAllocationModel::selectAllocationModeOutput(
						      executable, allocate_mode, safe_roll_mode, fixed, manual);
			ASSERT_TRUE(selected.selected);
			const auto actual = HydrofoilAllocationModel::decomposeFoilCommand(
						    selected.solution.delta_left_commanded,
						    selected.solution.delta_right_commanded);
			ASSERT_TRUE(actual.valid);
			EXPECT_FLOAT_EQ(actual.symmetric, expected.symmetric);
			EXPECT_FLOAT_EQ(actual.differential, expected.differential);
			EXPECT_FLOAT_EQ(selected.solution.thrust_left_commanded,
					executable.solution.thrust_left_commanded);
			EXPECT_FLOAT_EQ(selected.solution.thrust_right_commanded,
					executable.solution.thrust_right_commanded);
		}
	}
}

TEST(HydrofoilAllocationModel, ModeFourRetainsStrictAllocationModeBranches)
{
	hydrofoil::ExecutableAllocation executable{};
	executable.usable = true;
	executable.solution.delta_left_commanded = 0.4f;
	executable.solution.delta_right_commanded = 0.2f;
	executable.solution.thrust_left_commanded = 7.f;
	executable.solution.thrust_right_commanded = 8.f;
	AllocationSolution fixed{};
	fixed.delta_left_commanded = -0.1f;
	fixed.delta_right_commanded = -0.2f;
	AllocationSolution manual{};
	manual.delta_left_commanded = -0.3f;
	manual.delta_right_commanded = 0.3f;

	const auto normal = HydrofoilAllocationModel::selectAllocationModeOutput(executable, 0, 4, fixed, manual);
	ASSERT_TRUE(normal.selected);
	EXPECT_EQ(normal.source, hydrofoil::AllocationOutputSource::CompleteExecutable);
	EXPECT_FALSE(normal.mode_branch_overrode_solution);

	const auto fixed_selected =
		HydrofoilAllocationModel::selectAllocationModeOutput(executable, 1, 4, fixed, manual);
	ASSERT_TRUE(fixed_selected.selected);
	EXPECT_EQ(fixed_selected.source, hydrofoil::AllocationOutputSource::LegacyFixed);
	EXPECT_TRUE(fixed_selected.mode_branch_overrode_solution);
	EXPECT_FLOAT_EQ(fixed_selected.solution.delta_left_commanded, fixed.delta_left_commanded);

	const auto manual_selected =
		HydrofoilAllocationModel::selectAllocationModeOutput(executable, 2, 4, fixed, manual);
	ASSERT_TRUE(manual_selected.selected);
	EXPECT_EQ(manual_selected.source, hydrofoil::AllocationOutputSource::LegacyManual);
	EXPECT_TRUE(manual_selected.mode_branch_overrode_solution);
	EXPECT_FLOAT_EQ(manual_selected.solution.delta_right_commanded, manual.delta_right_commanded);

	executable.usable = false;

	for (int32_t allocate_mode : {0, 1, 2}) {
		const auto suppressed = HydrofoilAllocationModel::selectAllocationModeOutput(
						executable, allocate_mode, 4, fixed, manual);
		EXPECT_FALSE(suppressed.selected);
		EXPECT_EQ(suppressed.source, hydrofoil::AllocationOutputSource::Suppressed);
	}
}

TEST(HydrofoilAllocationModel, BestFeasibleRateLimitPreservesCompleteVectorAndReconstruction)
{
	const AllocationContext context = defaultContext();
	AllocationSolution candidate{};
	candidate.delta_left_commanded = 0.31533f;
	candidate.delta_right_commanded = 0.61085f;
	candidate.thrust_left_commanded = 4.f;
	candidate.thrust_right_commanded = 6.f;
	candidate.status.finite = true;
	candidate.status.valid = false;
	candidate.status.converged = false;
	candidate.status.solve_status = hydrofoil::AllocationSolveStatus::NotConverged;
	const float previous_left = 0.61086f;
	const float previous_right = 0.61086f;
	const float dt = 0.0025f;
	const float max_rate = math::radians(90.f);
	const float max_step = max_rate * dt;
	const auto executable = HydrofoilAllocationModel::selectExecutable(
					context, candidate, true, previous_left, previous_right, max_rate, dt);
	ASSERT_TRUE(executable.usable);
	ASSERT_TRUE(executable.best_feasible);
	ASSERT_TRUE(executable.rate_limited);
	EXPECT_NEAR(executable.solution.delta_left_commanded, previous_left - max_step, 1e-6f);
	EXPECT_FLOAT_EQ(executable.solution.delta_right_commanded, candidate.delta_right_commanded);
	EXPECT_FLOAT_EQ(executable.solution.thrust_left_commanded, candidate.thrust_left_commanded);
	EXPECT_FLOAT_EQ(executable.solution.thrust_right_commanded, candidate.thrust_right_commanded);

	const auto components = HydrofoilAllocationModel::decomposeFoilCommand(
					executable.solution.delta_left_commanded,
					executable.solution.delta_right_commanded);
	ASSERT_TRUE(components.valid);
	float reconstructed_left = NAN;
	float reconstructed_right = NAN;
	ASSERT_TRUE(HydrofoilAllocationModel::composeFoilCommand(
			    components.symmetric, components.differential,
			    reconstructed_left, reconstructed_right));
	EXPECT_FLOAT_EQ(reconstructed_left, executable.solution.delta_left_commanded);
	EXPECT_FLOAT_EQ(reconstructed_right, executable.solution.delta_right_commanded);
}

TEST(HydrofoilAllocationModel, NonfiniteOrHardUnboundedSolutionIsNeverExecutable)
{
	const AllocationContext context = defaultContext();
	AllocationSolution candidate{};
	candidate.status.finite = true;
	candidate.delta_left_commanded = context.max_hydrofoil_angle + 0.1f;
	candidate.delta_right_commanded = 0.f;
	candidate.thrust_left_commanded = 0.f;
	candidate.thrust_right_commanded = 0.f;
	candidate.status.solve_status = hydrofoil::AllocationSolveStatus::NotConverged;
	EXPECT_FALSE(HydrofoilAllocationModel::selectExecutable(
			     context, candidate, true, 0.f, 0.f, 1.f, 0.1f).usable);

	candidate.delta_left_commanded = NAN;
	candidate.status.finite = false;
	EXPECT_FALSE(HydrofoilAllocationModel::selectExecutable(
			     context, candidate, true, 0.f, 0.f, 1.f, 0.1f).usable);

	candidate.delta_left_commanded = 0.f;
	candidate.status.finite = true;
	candidate.status.solve_status = hydrofoil::AllocationSolveStatus::JacobianSingular;
	EXPECT_FALSE(HydrofoilAllocationModel::selectExecutable(
			     context, candidate, true, 0.f, 0.f, 1.f, 0.1f).usable);
}

TEST(HydrofoilAllocationModel, SaturatedDepthBestFeasiblePreservesSymmetricSign)
{
	AllocationContext context = defaultContext();
	context.body_pitch = 0.f;
	context.max_hydrofoil_angle = math::radians(35.f);
	context.available_thruster_thrust = 0.f;

	const AllocationSolution negative_force =
		HydrofoilAllocationModel::solve(context, AllocationCommand{0.f, -80.f, 0.f, 0.f, 0.f});
	const AllocationSolution positive_force =
		HydrofoilAllocationModel::solve(context, AllocationCommand{0.f, 80.f, 0.f, 0.f, 0.f});
	ASSERT_FALSE(negative_force.status.valid);
	ASSERT_FALSE(positive_force.status.valid);
	ASSERT_TRUE(HydrofoilAllocationModel::isFiniteBounded(context, negative_force));
	ASSERT_TRUE(HydrofoilAllocationModel::isFiniteBounded(context, positive_force));
	const auto negative_components = HydrofoilAllocationModel::decomposeFoilCommand(
						 negative_force.delta_left_commanded,
						 negative_force.delta_right_commanded);
	const auto positive_components = HydrofoilAllocationModel::decomposeFoilCommand(
						 positive_force.delta_left_commanded,
						 positive_force.delta_right_commanded);
	ASSERT_TRUE(negative_components.valid);
	ASSERT_TRUE(positive_components.valid);
	EXPECT_GT(negative_components.symmetric, 0.f);
	EXPECT_LT(positive_components.symmetric, 0.f);
	EXPECT_NEAR(std::fabs(negative_components.symmetric), math::radians(35.f), 1e-6f);
	EXPECT_NEAR(std::fabs(positive_components.symmetric), math::radians(35.f), 1e-6f);
}

TEST(HydrofoilAllocationModel, TenNewtonDepthFeedforwardChangeIsExecutedOrExplicitlyLimited)
{
	AllocationContext context = defaultContext();
	context.body_pitch = 0.f;
	context.max_hydrofoil_angle = math::radians(35.f);
	context.available_thruster_thrust = 0.f;
	const AllocationSolution lower =
		HydrofoilAllocationModel::solve(context, AllocationCommand{0.f, 15.f, 0.f, 0.f, 0.f});
	const AllocationSolution upper =
		HydrofoilAllocationModel::solve(context, AllocationCommand{0.f, 25.f, 0.f, 0.f, 0.f});
	EXPECT_FLOAT_EQ(upper.source_command.force_z - lower.source_command.force_z, 10.f);
	const auto lower_selected = HydrofoilAllocationModel::selectExecutable(
					    context, lower, true, 0.f, 0.f, 1000.f, 0.0025f);
	const auto upper_selected = HydrofoilAllocationModel::selectExecutable(
					    context, upper, true, 0.f, 0.f, 1000.f, 0.0025f);
	ASSERT_TRUE(lower_selected.usable);
	ASSERT_TRUE(upper_selected.usable);
	const auto lower_components = HydrofoilAllocationModel::decomposeFoilCommand(
					      lower_selected.solution.delta_left_commanded,
					      lower_selected.solution.delta_right_commanded);
	const auto upper_components = HydrofoilAllocationModel::decomposeFoilCommand(
					      upper_selected.solution.delta_left_commanded,
					      upper_selected.solution.delta_right_commanded);
	ASSERT_TRUE(lower_components.valid);
	ASSERT_TRUE(upper_components.valid);
	EXPECT_NE(lower_components.symmetric, 0.f);
	EXPECT_NE(upper_components.symmetric, 0.f);
	EXPECT_TRUE(std::fabs(upper_components.symmetric - lower_components.symmetric) > 1e-6f
		    || lower.status.saturation_mask != hydrofoil::SaturationNone
		    || upper.status.saturation_mask != hydrofoil::SaturationNone);
}

TEST(HydrofoilAllocationModel, CommandedAngleRateInterval)
{
	const auto interval = HydrofoilAllocationModel::commandedAngleInterval(0.2f, -0.2f, 0.6f, 1.f, 0.1f, 0.05f);
	ASSERT_TRUE(interval.valid);
	EXPECT_FLOAT_EQ(interval.left_min, 0.1f);
	EXPECT_FLOAT_EQ(interval.left_max, 0.3f);
	EXPECT_FLOAT_EQ(interval.right_min, -0.3f);
	EXPECT_FLOAT_EQ(interval.right_max, -0.1f);
	EXPECT_FALSE(HydrofoilAllocationModel::commandedAngleInterval(0.f, 0.f, 0.5f, 1.f, 0.f, 0.f).valid);
}

TEST(HydrofoilAllocationModel, TrimMigrationPureMathRegression)
{
	for (float generic_without_trim : {-0.4f, 0.f, 0.4f}) {
		for (float hydro_commanded : {-0.8f, -0.2f, 0.f, 0.2f, 0.8f}) {
			const auto comparison = HydrofoilAllocationModel::compareTrimMigration(
							generic_without_trim, -0.1f, hydro_commanded, -0.1f);
			ASSERT_TRUE(comparison.valid);
			EXPECT_FLOAT_EQ(comparison.legacy_final_commanded, comparison.migrated_final_commanded);
			EXPECT_FLOAT_EQ(comparison.difference, 0.f);
		}
	}

	EXPECT_FALSE(HydrofoilAllocationModel::composeNormalizedCommand(NAN, 0.f, -0.1f).valid);
}

TEST(HydrofoilAllocationModel, GenericHydroFinalDiagnosticComposition)
{
	const auto nominal = HydrofoilAllocationModel::composeNormalizedCommand(0.15f, 0.25f, -0.1f);
	ASSERT_TRUE(nominal.valid);
	EXPECT_FLOAT_EQ(nominal.summed_unconstrained, 0.3f);
	EXPECT_FLOAT_EQ(nominal.final_commanded, 0.3f);

	const auto saturated = HydrofoilAllocationModel::composeNormalizedCommand(0.6f, 0.7f, -0.1f);
	ASSERT_TRUE(saturated.valid);
	EXPECT_FLOAT_EQ(saturated.summed_unconstrained, 1.2f);
	EXPECT_FLOAT_EQ(saturated.final_commanded, 1.f);
}

TEST(HydrofoilAllocationModel, SignedRollMirrorScanMatchesSymmetricSoftwareContext)
{
	AllocationContext context = defaultContext();
	context.speed_squared = 1.f; // historical HY_ALT_SPEED=1 m/s
	context.body_pitch = math::radians(3.f);
	AllocationCommand command{24.f, 0.f, 0.f, 0.f, 0.f};
	unsigned valid_positive = 0;
	unsigned valid_negative = 0;
	float maximum_delta_mirror_error = 0.f;
	float maximum_thrust_mirror_error = 0.f;
	float maximum_residual_mirror_error = 0.f;
	float positive_feasible_limit = 0.f;
	float negative_feasible_limit = 0.f;

	for (int i = 0; i <= 100; ++i) {
		const float magnitude = static_cast<float>(i) * 0.01f;
		const AllocationSolution positive = HydrofoilAllocationModel::checkCandidate(context, command, magnitude);
		const AllocationSolution negative = HydrofoilAllocationModel::checkCandidate(context, command, -magnitude);
		EXPECT_EQ(positive.status.valid, negative.status.valid);
		EXPECT_EQ(positive.status.converged, negative.status.converged);
		EXPECT_EQ(positive.status.iterations, negative.status.iterations);
		EXPECT_EQ(mirroredSaturationMask(positive.status.saturation_mask),
			  negative.status.saturation_mask);
		EXPECT_NEAR(positive.status.residual, negative.status.residual, 1e-6f);
		EXPECT_NEAR(positive.delta_left_commanded, negative.delta_right_commanded, 1e-6f);
		EXPECT_NEAR(positive.delta_right_commanded, negative.delta_left_commanded, 1e-6f);
		EXPECT_NEAR(positive.thrust_left_commanded, negative.thrust_right_commanded, 1e-5f);
		EXPECT_NEAR(positive.thrust_right_commanded, negative.thrust_left_commanded, 1e-5f);

		maximum_delta_mirror_error = math::max(maximum_delta_mirror_error,
						      std::fabs(positive.delta_left_commanded
								- negative.delta_right_commanded));
		maximum_thrust_mirror_error = math::max(maximum_thrust_mirror_error,
						       std::fabs(positive.thrust_left_commanded
								 - negative.thrust_right_commanded));
		maximum_residual_mirror_error = math::max(maximum_residual_mirror_error,
							 std::fabs(positive.status.residual
								   - negative.status.residual));

		if (positive.status.valid) {
			++valid_positive;
			positive_feasible_limit = magnitude;
		}

		if (negative.status.valid) {
			++valid_negative;
			negative_feasible_limit = -magnitude;
		}

		if (positive.status.valid && negative.status.valid) {
			const auto positive_linearization = HydrofoilAllocationModel::linearize(context, positive);
			const auto negative_linearization = HydrofoilAllocationModel::linearize(context, negative);
			ASSERT_TRUE(positive_linearization.valid);
			ASSERT_TRUE(negative_linearization.valid);
			EXPECT_NEAR(positive_linearization.g_left, -negative_linearization.g_right, 1e-5f);
			EXPECT_NEAR(positive_linearization.g_right, -negative_linearization.g_left, 1e-5f);
		}
	}

	// phi and p are controller states, not HydrofoilAllocationModel inputs. Verify
	// the requested scan grid cannot change a fixed allocation context/command.
	const AllocationSolution reference = HydrofoilAllocationModel::checkCandidate(context, command, -0.32f);

	for (int phi_deg = -10; phi_deg <= 10; ++phi_deg) {
		for (float p_deg_s : {-5.f, -2.f, 0.f, 2.f, 5.f}) {
			(void)phi_deg;
			(void)p_deg_s;
			const AllocationSolution repeated = HydrofoilAllocationModel::checkCandidate(context, command, -0.32f);
			EXPECT_EQ(repeated.status.valid, reference.status.valid);
			EXPECT_FLOAT_EQ(repeated.status.residual, reference.status.residual);
			EXPECT_EQ(repeated.status.saturation_mask, reference.status.saturation_mask);
		}
	}

	EXPECT_EQ(valid_positive, valid_negative);
	EXPECT_NEAR(positive_feasible_limit, -negative_feasible_limit, 1e-6f);
	std::printf("MIRROR_SCAN valid_each=%u feasible=[%.2f,%.2f] delta_err=%g thrust_err=%g residual_err=%g"
		    " mask2=DeltaRight\n",
		    valid_positive, static_cast<double>(negative_feasible_limit),
		    static_cast<double>(positive_feasible_limit), static_cast<double>(maximum_delta_mirror_error),
		    static_cast<double>(maximum_thrust_mirror_error), static_cast<double>(maximum_residual_mirror_error));
}

TEST(HydrofoilAllocationModel, HostPerformanceReference)
{
	const AllocationContext context = defaultContext();
	AllocationCommand command{24.f, 0.f, 0.f, 0.f, 0.f};
	constexpr int iterations = 10000;
	uint64_t maximum_ns = 0;
	volatile float sink = 0.f;
	const auto total_start = std::chrono::steady_clock::now();

	for (int i = 0; i < iterations; ++i) {
		command.u_phi = static_cast<float>((i % 101) - 50) * 0.005f;
		const auto start = std::chrono::steady_clock::now();
		const AllocationSolution solution = HydrofoilAllocationModel::solve(context, command);
		const auto forward = HydrofoilAllocationModel::forward(context, solution);
		const auto linearization = HydrofoilAllocationModel::linearize(context, solution);
		const auto stop = std::chrono::steady_clock::now();
		const uint64_t elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count();
		maximum_ns = math::max(maximum_ns, elapsed_ns);
		sink += solution.delta_left_commanded + forward.u_phi_effective_model + linearization.g_left;
	}

	const auto total_stop = std::chrono::steady_clock::now();
	const uint64_t total_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(total_stop - total_start).count();
	std::printf("HOST_PERF allocation_solve_forward_linearize average_ns=%llu maximum_ns=%llu sink=%f\n",
		    static_cast<unsigned long long>(total_ns / iterations),
		    static_cast<unsigned long long>(maximum_ns), static_cast<double>(sink));
}

TEST(HydrofoilAllocationModel, MeasuredSpeedContextIsCachedBetweenLowRateMessages)
{
	SpeedContextCache cache;
	const auto initial = cache.update(true, 3.f, 1'000'000, false, 0, 0.f);
	ASSERT_TRUE(initial.valid);
	EXPECT_FLOAT_EQ(initial.speed_squared, 9.f);
	EXPECT_TRUE(initial.stale);

	const auto fresh = cache.update(true, 3.f, 1'002'500, true, 1'002'000, 16.f);
	ASSERT_TRUE(fresh.valid);
	EXPECT_FLOAT_EQ(fresh.speed_squared, 16.f);
	EXPECT_FALSE(fresh.cached);
	EXPECT_FALSE(fresh.stale);

	for (uint64_t now = 1'005'000; now < 1'500'000; now += 2'500) {
		const auto held = cache.update(true, 3.f, now, false, 0, 0.f);
		EXPECT_FLOAT_EQ(held.speed_squared, 16.f);
		EXPECT_TRUE(held.cached);
		EXPECT_FALSE(held.stale);
	}

	const auto stale = cache.update(true, 3.f, 1'502'001, false, 0, 0.f);
	EXPECT_FLOAT_EQ(stale.speed_squared, 9.f);
	EXPECT_TRUE(stale.stale);
	EXPECT_GT(stale.age_us, SpeedContextCache::StaleTimeoutUs);
}
