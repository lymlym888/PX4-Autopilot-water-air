/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>

#include <cstdint>

namespace hydrofoil
{

enum class AllocationSolveStatus : uint8_t {
	Success = 0,
	InvalidInput,
	JacobianSingular,
	NonFinite,
	NotConverged
};

enum AllocationSaturation : uint32_t {
	SaturationNone = 0,
	DeltaLeft = 1u << 0,
	DeltaRight = 1u << 1,
	ThrustLeftLow = 1u << 2,
	ThrustLeftHigh = 1u << 3,
	ThrustRightLow = 1u << 4,
	ThrustRightHigh = 1u << 5,
	ForceXRedistributed = 1u << 6
};

struct AllocationContext {
	float lift_coefficient{0.f};
	float speed_squared{0.f};
	float body_pitch{0.f};
	float max_hydrofoil_angle{0.f};
	float max_thruster_thrust{0.f};
	float available_thruster_thrust{0.f};
	matrix::Matrix<float, 4, 3> virtual_control_matrix{};
};

struct AllocationCommand {
	float force_x{0.f};
	float force_z{0.f};
	float u_phi{0.f};
	float u_theta{0.f};
	float u_psi{0.f};
};

struct FoilCommandComponents {
	bool valid{false};
	float symmetric{0.f};
	float differential{0.f};
};

struct AllocationStatus {
	bool valid{false};
	bool converged{false};
	bool finite{false};
	uint16_t iterations{0};
	float residual{0.f};
	uint32_t saturation_mask{SaturationNone};
	float jacobian_condition{0.f};
	AllocationSolveStatus solve_status{AllocationSolveStatus::InvalidInput};
};

struct AllocationSolution {
	float delta_left_commanded{0.f};
	float delta_right_commanded{0.f};
	float thrust_left_commanded{0.f};
	float thrust_right_commanded{0.f};
	matrix::Vector4f virtual_forces_control{}; // before allocator force redistribution: Lx, Lz, Rx, Rz
	matrix::Vector4f virtual_forces_commanded{}; // after allocator force redistribution: Lx, Lz, Rx, Rz
	AllocationCommand source_command{};
	AllocationStatus status{};
};

struct ExecutableAllocation {
	AllocationSolution solution{};
	bool usable{false};
	bool best_feasible{false};
	bool rate_limited{false};
};

enum class AllocationOutputSource : uint8_t {
	Suppressed = 0,
	CompleteExecutable = 1,
	LegacyFixed = 2,
	LegacyManual = 3
};

struct AllocationModeOutput {
	AllocationSolution solution{};
	AllocationOutputSource source{AllocationOutputSource::Suppressed};
	bool selected{false};
	bool mode_branch_overrode_solution{false};
};

struct AllocationForwardResult {
	bool valid{false};
	matrix::Vector4f virtual_forces_reconstructed{};
	float u_phi_effective_model{0.f};
	float reconstruction_residual{0.f};
};

struct AllocationLinearization {
	bool valid{false};
	float g_left{0.f};
	float g_right{0.f};
	float thrust_gain_left{0.f};
	float thrust_gain_right{0.f};
	float jacobian_condition_left{0.f};
	float jacobian_condition_right{0.f};
	AllocationSolveStatus solve_status{AllocationSolveStatus::InvalidInput};
};

struct CommandedAngleInterval {
	bool valid{false};
	float left_min{0.f};
	float left_max{0.f};
	float right_min{0.f};
	float right_max{0.f};
};

struct NormalizedCommandComposition {
	bool valid{false};
	float generic_without_neutral{0.f};
	float hydro_commanded{0.f};
	float neutral_calibration{0.f};
	float summed_unconstrained{0.f};
	float final_commanded{0.f};
};

struct TrimMigrationComparison {
	bool valid{false};
	float legacy_final_commanded{0.f};
	float migrated_final_commanded{0.f};
	float difference{0.f};
};

class HydrofoilAllocationModel
{
public:
	static AllocationSolution solve(const AllocationContext &context, const AllocationCommand &command);
	static AllocationForwardResult forward(const AllocationContext &context, const AllocationSolution &solution);
	static AllocationLinearization linearize(const AllocationContext &context, const AllocationSolution &solution);
	static bool isFiniteBounded(const AllocationContext &context, const AllocationSolution &solution);
	static ExecutableAllocation selectExecutable(const AllocationContext &context,
			const AllocationSolution &candidate, bool allow_bounded_best_feasible,
			float previous_delta_left, float previous_delta_right,
			float max_commanded_rate, float dt);
	static AllocationModeOutput selectAllocationModeOutput(const ExecutableAllocation &executable,
			int32_t allocate_mode, uint8_t safe_roll_mode,
			const AllocationSolution &legacy_fixed,
			const AllocationSolution &legacy_manual);
	// SafeRoll owns only u_phi. All non-roll entries are copied bit-for-bit.
	static AllocationCommand withRollCommand(const AllocationCommand &command, float candidate_u_phi);
	static AllocationSolution checkCandidate(const AllocationContext &context, const AllocationCommand &command,
			float candidate_u_phi);
	static FoilCommandComponents decomposeFoilCommand(float delta_left, float delta_right);
	static bool composeFoilCommand(float symmetric, float differential, float &delta_left, float &delta_right);
	static CommandedAngleInterval commandedAngleInterval(float delta_left_commanded, float delta_right_commanded,
			float max_commanded_angle, float max_commanded_rate, float dt, float remainder_margin);
	static NormalizedCommandComposition composeNormalizedCommand(float generic_without_neutral,
			float hydro_commanded, float neutral_calibration);
	static TrimMigrationComparison compareTrimMigration(float generic_without_trim, float legacy_generic_trim,
			float hydro_commanded, float migrated_neutral_calibration);

private:
	static matrix::Vector2f residual(const matrix::Vector2f &x, const AllocationContext &context,
			float force_x, float force_z);
	static matrix::SquareMatrix<float, 2> jacobian(const matrix::Vector2f &x, const AllocationContext &context);
	static AllocationStatus solveOneSide(const AllocationContext &context, float force_x, float force_z,
			matrix::Vector2f &state, uint32_t &saturation_mask, uint32_t delta_saturation_bit,
			uint32_t thrust_low_bit, uint32_t thrust_high_bit);
};

} // namespace hydrofoil
