/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "HydrofoilAllocationModel.hpp"

#include <mathlib/mathlib.h>

#include <cmath>

using matrix::SquareMatrix;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::Vector4f;

namespace hydrofoil
{

namespace
{
constexpr int kMaxIterations = 10;
constexpr float kStepToleranceSquared = 1e-6f;
constexpr float kResidualTolerance = 1e-3f;
constexpr float kSaturationTolerance = 1e-5f;

bool finiteContext(const AllocationContext &context)
{
	return std::isfinite(context.lift_coefficient) && std::isfinite(context.speed_squared)
	       && std::isfinite(context.body_pitch) && std::isfinite(context.max_hydrofoil_angle)
	       && std::isfinite(context.max_thruster_thrust) && std::isfinite(context.available_thruster_thrust)
	       && context.virtual_control_matrix.isAllFinite() && context.lift_coefficient >= 0.f
	       && context.speed_squared >= 0.f && context.max_hydrofoil_angle > 0.f
	       && context.max_thruster_thrust > 0.f && context.available_thruster_thrust >= 0.f;
}

bool finiteCommand(const AllocationCommand &command)
{
	return std::isfinite(command.force_x) && std::isfinite(command.force_z) && std::isfinite(command.u_phi)
	       && std::isfinite(command.u_theta) && std::isfinite(command.u_psi);
}

float conditionEstimate(const SquareMatrix<float, 2> &jacobian, const SquareMatrix<float, 2> &inverse)
{
	float jacobian_norm_squared = 0.f;
	float inverse_norm_squared = 0.f;

	for (int row = 0; row < 2; ++row) {
		for (int column = 0; column < 2; ++column) {
			jacobian_norm_squared += jacobian(row, column) * jacobian(row, column);
			inverse_norm_squared += inverse(row, column) * inverse(row, column);
		}
	}

	return std::sqrt(jacobian_norm_squared * inverse_norm_squared);
}

} // namespace

Vector2f HydrofoilAllocationModel::residual(const Vector2f &x, const AllocationContext &context,
		float force_x, float force_z)
{
	const float delta = x(0);
	const float thrust = x(1);
	Vector2f out;
	out(0) = force_x - context.lift_coefficient * context.speed_squared * (delta + context.body_pitch)
		 * std::sin(context.body_pitch) - thrust * std::cos(delta);
	out(1) = force_z + context.lift_coefficient * context.speed_squared * (delta + context.body_pitch)
		 * std::cos(context.body_pitch) + thrust * std::sin(delta);
	return out;
}

SquareMatrix<float, 2> HydrofoilAllocationModel::jacobian(const Vector2f &x, const AllocationContext &context)
{
	const float delta = x(0);
	const float thrust = x(1);
	SquareMatrix<float, 2> out;
	out(0, 0) = -context.lift_coefficient * context.speed_squared * std::sin(context.body_pitch)
		    + thrust * std::sin(delta);
	out(0, 1) = -std::cos(delta);
	out(1, 0) = context.lift_coefficient * context.speed_squared * std::cos(context.body_pitch)
		    + thrust * std::cos(delta);
	out(1, 1) = std::sin(delta);
	return out;
}

AllocationStatus HydrofoilAllocationModel::solveOneSide(const AllocationContext &context, float force_x, float force_z,
		Vector2f &state, uint32_t &saturation_mask, uint32_t delta_saturation_bit,
		uint32_t thrust_low_bit, uint32_t thrust_high_bit)
{
	AllocationStatus status{};
	status.finite = state.isAllFinite();
	status.solve_status = AllocationSolveStatus::NotConverged;

	for (int i = 0; i < kMaxIterations; ++i) {
		const SquareMatrix<float, 2> J = jacobian(state, context);
		const Vector2f f = residual(state, context, force_x, force_z);
		SquareMatrix<float, 2> J_inv;

		if (!J.isAllFinite() || !f.isAllFinite() || !matrix::inv(J, J_inv) || !J_inv.isAllFinite()) {
			status.solve_status = AllocationSolveStatus::JacobianSingular;
			status.finite = J.isAllFinite() && f.isAllFinite();
			return status;
		}

		status.jacobian_condition = math::max(status.jacobian_condition, conditionEstimate(J, J_inv));
		const Vector2f delta_state = -J_inv * f;
		status.iterations = static_cast<uint16_t>(i + 1);

		if (!delta_state.isAllFinite()) {
			status.solve_status = AllocationSolveStatus::NonFinite;
			status.finite = false;
			return status;
		}

		if (delta_state.norm_squared() < kStepToleranceSquared) {
			status.converged = true;
			break;
		}

		state += delta_state;
		const float unconstrained_delta = state(0);
		const float unconstrained_thrust = state(1);
		state(0) = math::constrain(state(0), -context.max_hydrofoil_angle, context.max_hydrofoil_angle);
		state(1) = math::constrain(state(1), 0.f, context.available_thruster_thrust);

		if (fabsf(state(0) - unconstrained_delta) > kSaturationTolerance) {
			saturation_mask |= delta_saturation_bit;
		}

		if (unconstrained_thrust < 0.f) {
			saturation_mask |= thrust_low_bit;

		} else if (unconstrained_thrust > context.available_thruster_thrust) {
			saturation_mask |= thrust_high_bit;
		}
	}

	const Vector2f final_residual = residual(state, context, force_x, force_z);
	status.residual = std::sqrt(final_residual.norm_squared());
	status.finite = state.isAllFinite() && final_residual.isAllFinite() && std::isfinite(status.jacobian_condition);
	status.converged = status.converged || (status.finite && status.residual <= kResidualTolerance);
	status.valid = status.finite && status.converged;
	status.solve_status = status.valid ? AllocationSolveStatus::Success : AllocationSolveStatus::NotConverged;
	return status;
}

AllocationSolution HydrofoilAllocationModel::solve(const AllocationContext &context, const AllocationCommand &command)
{
	AllocationSolution solution{};
	solution.source_command = command;

	if (!finiteContext(context) || !finiteCommand(command)) {
		solution.status.solve_status = AllocationSolveStatus::InvalidInput;
		return solution;
	}

	const Vector3f virtual_control(command.u_phi, command.u_theta, command.u_psi);
	Vector4f forces = context.virtual_control_matrix * virtual_control;
	forces += Vector4f(command.force_x * 0.5f, command.force_z * 0.5f,
			 command.force_x * 0.5f, command.force_z * 0.5f);
	solution.virtual_forces_control = forces;

	float sum_force_x = forces(0) + forces(2);

	if (sum_force_x < 0.f) {
		sum_force_x *= 0.5f;
		forces(0) -= sum_force_x;
		forces(2) -= sum_force_x;
		solution.status.saturation_mask |= ForceXRedistributed;
	}

	if (forces(0) < 0.f) {
		forces(2) += forces(0);
		forces(0) = 0.f;
		solution.status.saturation_mask |= ForceXRedistributed;

	} else if (forces(2) < 0.f) {
		forces(0) += forces(2);
		forces(2) = 0.f;
		solution.status.saturation_mask |= ForceXRedistributed;
	}

	solution.virtual_forces_commanded = forces;
	Vector2f left(0.f, command.force_x * 0.5f);
	Vector2f right(0.f, command.force_x * 0.5f);
	uint32_t saturation_mask = solution.status.saturation_mask;
	const AllocationStatus left_status = solveOneSide(context, forces(0), forces(1), left, saturation_mask,
		DeltaLeft, ThrustLeftLow, ThrustLeftHigh);
	const AllocationStatus right_status = solveOneSide(context, forces(2), forces(3), right, saturation_mask,
		DeltaRight, ThrustRightLow, ThrustRightHigh);

	solution.delta_left_commanded = left(0);
	solution.delta_right_commanded = right(0);
	solution.thrust_left_commanded = left(1);
	solution.thrust_right_commanded = right(1);
	solution.status.iterations = left_status.iterations + right_status.iterations;
	solution.status.residual = math::max(left_status.residual, right_status.residual);
	solution.status.jacobian_condition = math::max(left_status.jacobian_condition, right_status.jacobian_condition);
	solution.status.saturation_mask = saturation_mask;
	solution.status.finite = left_status.finite && right_status.finite;
	solution.status.converged = left_status.converged && right_status.converged;
	solution.status.valid = left_status.valid && right_status.valid;
	solution.status.solve_status = !left_status.valid ? left_status.solve_status : right_status.solve_status;

	if (solution.status.valid) {
		solution.status.solve_status = AllocationSolveStatus::Success;
	}

	return solution;
}

AllocationForwardResult HydrofoilAllocationModel::forward(const AllocationContext &context,
		const AllocationSolution &solution)
{
	AllocationForwardResult result{};

	if (!finiteContext(context) || !std::isfinite(solution.delta_left_commanded)
	    || !std::isfinite(solution.delta_right_commanded) || !std::isfinite(solution.thrust_left_commanded)
	    || !std::isfinite(solution.thrust_right_commanded)) {
		return result;
	}

	const float deltas[2] {solution.delta_left_commanded, solution.delta_right_commanded};
	const float thrusts[2] {solution.thrust_left_commanded, solution.thrust_right_commanded};

	for (int i = 0; i < 2; ++i) {
		const float lift = context.lift_coefficient * context.speed_squared * (deltas[i] + context.body_pitch);
		result.virtual_forces_reconstructed(2 * i) = lift * std::sin(context.body_pitch) + thrusts[i] * std::cos(deltas[i]);
		result.virtual_forces_reconstructed(2 * i + 1) = -lift * std::cos(context.body_pitch) - thrusts[i] * std::sin(deltas[i]);
	}

	Vector4f remainder = result.virtual_forces_reconstructed;
	remainder -= Vector4f(solution.source_command.force_x * 0.5f, solution.source_command.force_z * 0.5f,
			       solution.source_command.force_x * 0.5f, solution.source_command.force_z * 0.5f);
	remainder -= context.virtual_control_matrix.col(1) * solution.source_command.u_theta;
	remainder -= context.virtual_control_matrix.col(2) * solution.source_command.u_psi;
	Vector4f phi_column;

	for (int row = 0; row < 4; ++row) {
		phi_column(row) = context.virtual_control_matrix(row, 0);
	}
	const float denominator = phi_column.norm_squared();

	if (denominator <= 1e-9f || !result.virtual_forces_reconstructed.isAllFinite()) {
		return result;
	}

	result.u_phi_effective_model = phi_column.dot(remainder) / denominator;
	const Vector4f reconstruction_error = remainder - phi_column * result.u_phi_effective_model;
	result.reconstruction_residual = std::sqrt(reconstruction_error.norm_squared());
	result.valid = std::isfinite(result.u_phi_effective_model) && std::isfinite(result.reconstruction_residual);
	return result;
}

AllocationLinearization HydrofoilAllocationModel::linearize(const AllocationContext &context,
		const AllocationSolution &solution)
{
	AllocationLinearization out{};

	if (!finiteContext(context) || !solution.status.finite) {
		return out;
	}

	const Vector2f states[2] {
		Vector2f(solution.delta_left_commanded, solution.thrust_left_commanded),
		Vector2f(solution.delta_right_commanded, solution.thrust_right_commanded)
	};

	for (int i = 0; i < 2; ++i) {
		const SquareMatrix<float, 2> J = jacobian(states[i], context);
		SquareMatrix<float, 2> J_inv;

		if (!J.isAllFinite() || !matrix::inv(J, J_inv) || !J_inv.isAllFinite()) {
			out.solve_status = AllocationSolveStatus::JacobianSingular;
			return out;
		}

		const Vector2f force_gain(context.virtual_control_matrix(2 * i, 0),
				  context.virtual_control_matrix(2 * i + 1, 0));
		const Vector2f state_gain = -J_inv * force_gain;

		if (!state_gain.isAllFinite()) {
			out.solve_status = AllocationSolveStatus::NonFinite;
			return out;
		}

		if (i == 0) {
			out.g_left = state_gain(0);
			out.thrust_gain_left = state_gain(1);
			out.jacobian_condition_left = conditionEstimate(J, J_inv);

		} else {
			out.g_right = state_gain(0);
			out.thrust_gain_right = state_gain(1);
			out.jacobian_condition_right = conditionEstimate(J, J_inv);
		}
	}

	out.valid = true;
	out.solve_status = AllocationSolveStatus::Success;
	return out;
}

AllocationSolution HydrofoilAllocationModel::checkCandidate(const AllocationContext &context,
		const AllocationCommand &command, float candidate_u_phi)
{
	const AllocationCommand candidate = withRollCommand(command, candidate_u_phi);
	return solve(context, candidate);
}

AllocationCommand HydrofoilAllocationModel::withRollCommand(const AllocationCommand &command, float candidate_u_phi)
{
	AllocationCommand candidate = command;
	candidate.u_phi = candidate_u_phi;
	return candidate;
}

FoilCommandComponents HydrofoilAllocationModel::decomposeFoilCommand(float delta_left, float delta_right)
{
	FoilCommandComponents out{};

	if (!std::isfinite(delta_left) || !std::isfinite(delta_right)) {
		return out;
	}

	out.symmetric = 0.5f * (delta_left + delta_right);
	out.differential = 0.5f * (delta_left - delta_right);
	out.valid = std::isfinite(out.symmetric) && std::isfinite(out.differential);
	return out;
}

bool HydrofoilAllocationModel::composeFoilCommand(float symmetric, float differential, float &delta_left,
		float &delta_right)
{
	if (!std::isfinite(symmetric) || !std::isfinite(differential)) {
		return false;
	}

	delta_left = symmetric + differential;
	delta_right = symmetric - differential;
	return std::isfinite(delta_left) && std::isfinite(delta_right);
}

bool HydrofoilAllocationModel::isFiniteBounded(const AllocationContext &context,
		const AllocationSolution &solution)
{
	if (!finiteContext(context) || !solution.status.finite
	    || !std::isfinite(solution.delta_left_commanded)
	    || !std::isfinite(solution.delta_right_commanded)
	    || !std::isfinite(solution.thrust_left_commanded)
	    || !std::isfinite(solution.thrust_right_commanded)) {
		return false;
	}

	const float thrust_limit = math::min(context.max_thruster_thrust, context.available_thruster_thrust);
	return std::fabs(solution.delta_left_commanded) <= context.max_hydrofoil_angle + kSaturationTolerance
	       && std::fabs(solution.delta_right_commanded) <= context.max_hydrofoil_angle + kSaturationTolerance
	       && solution.thrust_left_commanded >= -kSaturationTolerance
	       && solution.thrust_right_commanded >= -kSaturationTolerance
	       && solution.thrust_left_commanded <= thrust_limit + kSaturationTolerance
	       && solution.thrust_right_commanded <= thrust_limit + kSaturationTolerance;
}

ExecutableAllocation HydrofoilAllocationModel::selectExecutable(const AllocationContext &context,
		const AllocationSolution &candidate, bool allow_bounded_best_feasible,
		float previous_delta_left, float previous_delta_right,
		float max_commanded_rate, float dt)
{
	ExecutableAllocation out{};
	const bool bounded = isFiniteBounded(context, candidate);
	const bool bounded_nonconvergence = candidate.status.finite
					    && candidate.status.solve_status == AllocationSolveStatus::NotConverged;

	if (!bounded || (!candidate.status.valid
			 && !(allow_bounded_best_feasible && bounded_nonconvergence))) {
		return out;
	}

	out.solution = candidate;
	out.usable = true;
	out.best_feasible = !candidate.status.valid;

	// The legacy allocator always emitted its finite constrained Newton iterate.
	// Keep that behavior for a bounded best-feasible result, while adding an
	// explicit commanded-angle slew boundary to the degradation path.
	if (out.best_feasible && std::isfinite(previous_delta_left) && std::isfinite(previous_delta_right)
	    && std::isfinite(max_commanded_rate) && max_commanded_rate > 0.f
	    && std::isfinite(dt) && dt > 0.f) {
		const float max_step = max_commanded_rate * dt;
		const float left = math::constrain(candidate.delta_left_commanded,
						  previous_delta_left - max_step, previous_delta_left + max_step);
		const float right = math::constrain(candidate.delta_right_commanded,
						   previous_delta_right - max_step, previous_delta_right + max_step);
		out.rate_limited = fabsf(left - candidate.delta_left_commanded) > kSaturationTolerance
				   || fabsf(right - candidate.delta_right_commanded) > kSaturationTolerance;
		out.solution.delta_left_commanded = math::constrain(left, -context.max_hydrofoil_angle,
						      context.max_hydrofoil_angle);
		out.solution.delta_right_commanded = math::constrain(right, -context.max_hydrofoil_angle,
						       context.max_hydrofoil_angle);
	}

	return out;
}

AllocationModeOutput HydrofoilAllocationModel::selectAllocationModeOutput(
	const ExecutableAllocation &executable, int32_t allocate_mode, uint8_t safe_roll_mode,
	const AllocationSolution &legacy_fixed, const AllocationSolution &legacy_manual)
{
	AllocationModeOutput out{};

	if (!executable.usable || allocate_mode < 0 || allocate_mode > 2 || safe_roll_mode > 4) {
		return out;
	}

	// Modes 0-3 are closed-loop production controllers. Their source command
	// already contains the selected u_phi, so the complete executable nonlinear
	// solution is the only branch that preserves force_z/u_theta and thrust.
	if (safe_roll_mode <= 3) {
		out.solution = executable.solution;
		out.source = AllocationOutputSource::CompleteExecutable;
		out.selected = true;
		return out;
	}

	// Mode 4 retains the historical allocation-mode branches for strict
	// ablation/special-purpose testing.
	if (allocate_mode == 0) {
		out.solution = executable.solution;
		out.source = AllocationOutputSource::CompleteExecutable;

	} else if (allocate_mode == 1) {
		out.solution = legacy_fixed;
		out.source = AllocationOutputSource::LegacyFixed;
		out.mode_branch_overrode_solution = true;

	} else {
		out.solution = legacy_manual;
		out.source = AllocationOutputSource::LegacyManual;
		out.mode_branch_overrode_solution = true;
	}

	out.selected = true;
	return out;
}

CommandedAngleInterval HydrofoilAllocationModel::commandedAngleInterval(float delta_left_commanded,
		float delta_right_commanded, float max_commanded_angle, float max_commanded_rate, float dt,
		float remainder_margin)
{
	CommandedAngleInterval out{};

	if (!std::isfinite(delta_left_commanded) || !std::isfinite(delta_right_commanded)
	    || !std::isfinite(max_commanded_angle) || !std::isfinite(max_commanded_rate) || !std::isfinite(dt)
	    || !std::isfinite(remainder_margin) || max_commanded_angle <= 0.f || max_commanded_rate < 0.f
	    || dt <= 0.f || remainder_margin < 0.f || remainder_margin >= max_commanded_angle) {
		return out;
	}

	const float absolute_limit = max_commanded_angle - remainder_margin;
	const float step = max_commanded_rate * dt;
	out.left_min = math::max(-absolute_limit, delta_left_commanded - step);
	out.left_max = math::min(absolute_limit, delta_left_commanded + step);
	out.right_min = math::max(-absolute_limit, delta_right_commanded - step);
	out.right_max = math::min(absolute_limit, delta_right_commanded + step);
	out.valid = out.left_min <= out.left_max && out.right_min <= out.right_max;
	return out;
}

NormalizedCommandComposition HydrofoilAllocationModel::composeNormalizedCommand(float generic_without_neutral,
		float hydro_commanded, float neutral_calibration)
{
	NormalizedCommandComposition out{};

	if (!std::isfinite(generic_without_neutral) || !std::isfinite(hydro_commanded)
	    || !std::isfinite(neutral_calibration)) {
		return out;
	}

	out.generic_without_neutral = generic_without_neutral;
	out.hydro_commanded = hydro_commanded;
	out.neutral_calibration = neutral_calibration;
	out.summed_unconstrained = generic_without_neutral + hydro_commanded + neutral_calibration;
	out.final_commanded = math::constrain(out.summed_unconstrained, -1.f, 1.f);
	out.valid = true;
	return out;
}

TrimMigrationComparison HydrofoilAllocationModel::compareTrimMigration(float generic_without_trim,
		float legacy_generic_trim, float hydro_commanded, float migrated_neutral_calibration)
{
	TrimMigrationComparison out{};
	const NormalizedCommandComposition legacy =
		composeNormalizedCommand(generic_without_trim, hydro_commanded, legacy_generic_trim);
	const NormalizedCommandComposition migrated =
		composeNormalizedCommand(generic_without_trim, hydro_commanded, migrated_neutral_calibration);

	if (!legacy.valid || !migrated.valid) {
		return out;
	}

	out.legacy_final_commanded = legacy.final_commanded;
	out.migrated_final_commanded = migrated.final_commanded;
	out.difference = out.migrated_final_commanded - out.legacy_final_commanded;
	out.valid = true;
	return out;
}

} // namespace hydrofoil
