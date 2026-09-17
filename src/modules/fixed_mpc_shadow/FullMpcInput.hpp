#pragma once

#include <math.h>
#include <stdint.h>

namespace fixed_mpc_shadow
{

inline float input_clamp(float value, float low, float high)
{
	return value < low ? low : (value > high ? high : value);
}

// Depth/common mode has priority. Exhausted differential margin is saturation,
// not invalid data; do not reduce the common command to recover roll authority.
inline float full_differential_headroom(float mean_servo, float servo_limit)
{
	return 2.f * fmaxf(servo_limit - fabsf(mean_servo), 0.f);
}

// The identified input is Servo4 - Servo5, before PWM reversal/scaling.
inline int model_input_sign(int left_index, int right_index)
{
	return (left_index == 3 && right_index == 4) ? 1 :
	       ((left_index == 4 && right_index == 3) ? -1 : 0);
}

struct FullMpcInput {
	float trim{0.f};
	float correction_limit{0.f};
	float candidate_limit{0.f};
	float mean_servo{0.f};
	float servo_limit{0.f};
	int model_sign{0};

	bool valid() const
	{
		return __builtin_isfinite(trim) && __builtin_isfinite(correction_limit) && correction_limit >= 0.f &&
		       __builtin_isfinite(candidate_limit) && candidate_limit >= 0.f &&
		       __builtin_isfinite(mean_servo) && __builtin_isfinite(servo_limit) && servo_limit >= 0.f &&
		       fabsf(mean_servo) <= 1.f && (model_sign == 1 || model_sign == -1);
	}
};

struct FullMpcCommand {
	float correction{0.f};
	float requested_target{0.f};
	float target{0.f};
	float model_input{0.f};
	bool candidate_clamped{false};
	bool headroom_clamped{false};
	bool slew_limited{false};
	bool constraint_conflict{false};
};

struct FullTrimProjection {
	float candidate{0.f};
	float trim{0.f};
	float bound{0.f};
	float reserve{0.f};
	bool projected{false};
	bool valid{false};
};

// Called only when latching: reserve the configured two-sided correction range.
// This is a feasible software operating point, not a mechanical trim calibration.
inline FullTrimProjection project_full_trim(const FullMpcInput &input)
{
	FullTrimProjection result{};
	if (!input.valid()) { return result; }
	result.candidate = input.trim;
	const float headroom = full_differential_headroom(input.mean_servo, input.servo_limit);
	result.bound = fminf(input.candidate_limit, headroom);
	result.reserve = fminf(input.correction_limit, result.bound);
	const float center_limit = fmaxf(result.bound - result.reserve, 0.f);
	result.trim = input_clamp(input.trim, -center_limit, center_limit);
	result.projected = fabsf(result.trim - input.trim) > 1.e-6f;
	result.valid = true;
	return result;
}

inline FullMpcCommand implement_full_candidate(float model_proposal, const FullMpcInput &input)
{
	FullMpcCommand command{};
	command.correction = input_clamp(input.model_sign * model_proposal - input.trim,
					-input.correction_limit, input.correction_limit);
	command.requested_target = input.trim + command.correction;
	const float candidate = input_clamp(command.requested_target, -input.candidate_limit, input.candidate_limit);
	const float headroom = full_differential_headroom(input.mean_servo, input.servo_limit);
	command.target = input_clamp(candidate, -headroom, headroom);
	command.model_input = input.model_sign * command.target;
	command.candidate_clamped = fabsf(candidate - command.requested_target) > 1.e-6f;
	command.headroom_clamped = fabsf(command.target - candidate) > 1.e-6f ||
		fabsf(input.mean_servo) > input.servo_limit;
	return command;
}

inline FullMpcCommand implement_full_constrained_candidate(float model_proposal, float previous_model,
		float maximum_step, const FullMpcInput &input)
{
	FullMpcCommand command = implement_full_candidate(model_proposal, input);
	const float preferred = command.target;
	const float previous = input.model_sign * previous_model;
	const float bound = fminf(input.candidate_limit, full_differential_headroom(input.mean_servo, input.servo_limit));
	const float low = fmaxf(-bound, input.trim - input.correction_limit);
	const float high = fminf(bound, input.trim + input.correction_limit);
	float target = preferred;
	const bool conflict = low > high;

	if (!conflict) {
		const float rate_low = fmaxf(low, previous - maximum_step);
		const float rate_high = fminf(high, previous + maximum_step);
		// If feedback lies outside the new feasible set, recover to its closest
		// point. Otherwise optimize inside the joint interval, not serial clamps.
		target = rate_low <= rate_high ? input_clamp(preferred, rate_low, rate_high) :
			 input_clamp(previous, low, high);
	}

	if (fabsf(target - preferred) > 1.e-6f) {
		command = implement_full_candidate(input.model_sign * target, input);
	}

	command.slew_limited = fabsf(target - preferred) > 1.e-6f;
	command.constraint_conflict = conflict;
	return command;
}

// Call with a clock sampled AFTER copying the messages being checked.
inline float input_age_ms(uint64_t checked_at, uint64_t timestamp)
{
	if (timestamp == 0) { return 1.e9f; }
	return timestamp <= checked_at ? (checked_at - timestamp) / 1000.f :
	       -static_cast<float>(timestamp - checked_at) / 1000.f;
}

inline bool input_fresh(uint64_t checked_at, uint64_t timestamp, uint64_t maximum_age_us)
{
	return timestamp != 0 && timestamp <= checked_at && checked_at - timestamp <= maximum_age_us;
}

struct FullMpcFeedback {
	uint64_t timestamp{0};
	uint64_t entry_timestamp{0};
	uint32_t session{0};
	uint8_t mode{0};
	int left_index{-1};
	int right_index{-1};
	float left{0.f};
	float right{0.f};
};

enum class FullHistorySource { Invalid, EntryTrim, FinalOutput };

inline FullHistorySource full_previous_input(uint64_t now, uint64_t entry, uint32_t session,
		int left, int right, float trim, const FullMpcFeedback &feedback, float &previous)
{
	const int sign = model_input_sign(left, right);
	if (sign == 0 || !__builtin_isfinite(trim) || entry == 0 || entry > now || session == 0) {
		return FullHistorySource::Invalid;
	}
	if (feedback.mode == 6 && feedback.entry_timestamp == entry && feedback.session == session) {
		if (!input_fresh(now, feedback.timestamp, 200000) || feedback.timestamp < entry ||
		    feedback.left_index != left || feedback.right_index != right ||
		    !__builtin_isfinite(feedback.left) || !__builtin_isfinite(feedback.right) ||
		    fabsf(feedback.left) > 1.f || fabsf(feedback.right) > 1.f) {
			return FullHistorySource::Invalid;
		}
		previous = sign * (feedback.left - feedback.right);
		return FullHistorySource::FinalOutput;
	}
	if (input_fresh(now, entry, 200000)) {
		previous = sign * trim;
		return FullHistorySource::EntryTrim;
	}
	return FullHistorySource::Invalid;
}

enum InputRejection : uint32_t {
	AttitudeMissing = 1u << 0,
	RateMissing = 1u << 1,
	AttitudeZeroTime = 1u << 2,
	RateZeroTime = 1u << 3,
	AttitudeFuture = 1u << 4,
	RateFuture = 1u << 5,
	AttitudeStale = 1u << 6,
	RateStale = 1u << 7,
	AttitudeNonfinite = 1u << 8,
	RateNonfinite = 1u << 9,
	EulerNonfinite = 1u << 10,
	BaseInvalid = 1u << 11,
	TorqueInvalid = 1u << 12,
	MappingInvalid = 1u << 13,
	CommandSpaceInvalid = 1u << 14,
	FeedbackInvalid = 1u << 15,
	SolverNonfinite = 1u << 16,
	ManualReferenceInvalid = 1u << 17,
};

} // namespace fixed_mpc_shadow
