#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>

#define fixed_mpc_shadow baseline_shadow
#include STAGE5A_BASELINE_HEADER
#undef fixed_mpc_shadow
#include "FixedLinearMPC.hpp"
#include "FixedMPCParams.hpp"
#include "FixedMpcInterface.hpp"
#include "FullRollNominalModel.hpp"

using namespace fixed_mpc_shadow;

static bool close(float a, float b) { return std::fabs(a - b) < 2.e-6f; }

static float cost(MpcState state, float u, const FullMpcInput &input)
{
	float total = 0.f;
	const float trim = input.model_sign * input.trim;
	const float correction = u - trim;
	float last = state.previous_delta - trim;
	for (int step = 0; step < kHorizon; ++step) {
		float phi, p;
		full_roll_nominal_step(state.phi_rad, state.p_rad_s, correction, phi, p);
		total += kQPhi * phi * phi + kQP * p * p + kRDelta * correction * correction +
			 kRDDelta * (correction-last)*(correction-last);
		state.phi_rad = phi;
		state.p_rad_s = p;
		last = correction;
	}
	return total + full_roll_terminal_cost(state.phi_rad, state.p_rad_s, correction);
}

int main()
{
	FixedLinearMPC mpc;
	baseline_shadow::FixedLinearMPC old;
	int regression_count = 0;
	for (int angle = -25; angle <= 25; ++angle) {
		for (int rate = -6; rate <= 6; ++rate) {
			for (int prev = -5; prev <= 5; ++prev) {
				MpcState state{angle*.01745329252f, rate*.1f, prev*.07f};
				const auto a = mpc.solve_lqr_clipping(state);
				const auto b = old.solve_lqr_clipping({state.phi_rad,state.p_rad_s,state.previous_delta});
				assert(a.delta == b.delta && a.delta_raw == b.delta_raw && a.delta_clipped == b.delta_clipped &&
				       a.delta_amplitude_limited == b.delta_amplitude_limited &&
				       a.delta_rate_limited == b.delta_rate_limited && a.solver_success == b.solver_success);
				++regression_count;
			}
		}
	}
	std::printf("legacy_solver_bitwise,PASS,%d cases against backed-up pre-edit implementation\n",regression_count);
	FullMpcInput input{};
	input.trim = -.260418206f;
	input.correction_limit = .05f;
	input.candidate_limit = .5f;
	input.mean_servo = .02f;
	input.servo_limit = .98f;
	input.model_sign = -1;
	assert(model_input_sign(4,3)==-1 && model_input_sign(3,4)==1);
	assert(model_input_sign(3,3)==0 && model_input_sign(0,1)==0);
	for (int angle = -15; angle <= 15; ++angle) {
		for (int rate = -8; rate <= 8; ++rate) {
			MpcState state{angle*.01745329252f,rate*.02f,.26f};
			const auto r=mpc.solve_full_roll(state,input);
			assert(r.proposal.solver_success);
			const float left=input.mean_servo+r.command.target*.5f;
			const float right=input.mean_servo-r.command.target*.5f;
			assert(close(r.command.model_input,right-left));
			assert(close(r.command.target,input.trim+r.command.correction));
			assert(std::fabs(r.command.correction)<=input.correction_limit+1.e-6f);
			assert(std::fabs(left)<=input.servo_limit && std::fabs(right)<=input.servo_limit);
			const float chosen_cost=cost(state,right-left,input);
			for (int i=-12; i<=12; ++i) {
				const float raw=kDeltaMax*static_cast<float>(i)/4.f;
				const float amp=input_clamp(raw,-kDeltaMax,kDeltaMax);
				const float trim=input.model_sign*input.trim;
				const float previous=state.previous_delta-trim;
				const float rate_u=input_clamp(amp,previous-kDDeltaMax,previous+kDDeltaMax);
				const auto alternative=implement_full_candidate(trim+rate_u,input);
				assert(chosen_cost<=cost(state,alternative.model_input,input)+2.e-5f);
			}
			FullMpcInput reversed=input;
			reversed.model_sign=1;
			reversed.trim=-input.trim;
			const auto rr=mpc.solve_full_roll(state,reversed);
			assert(close(r.command.model_input,rr.command.model_input));
			assert(close(r.command.target,-rr.command.target));
		}
	}
	std::puts("model_execution_coordinate,PASS,527 states nominal continuous cost reconstruction and reversed index order");
	for (float trim : {-.7f, -.26f, 0.f, .26f, .7f}) {
		for (float mean : {-.98f,-.9f,0.f,.9f,.98f}) {
			for (float limit : {0.f,.05f,.35f}) {
				input.trim=trim; input.mean_servo=mean; input.correction_limit=limit;
				const auto r=mpc.solve_full_roll({.1f,0.f,-trim},input);
				assert(r.proposal.solver_success);
				assert(std::fabs(mean+.5f*r.command.target)<=.980001f);
				assert(std::fabs(mean-.5f*r.command.target)<=.980001f);
				assert(std::fabs(r.command.target)<=.500001f);
				assert(close(r.command.model_input,-r.command.target));
			}
		}
	}
	input.trim=.49f;input.mean_servo=0.f;input.correction_limit=.05f;
	auto clamped=implement_full_candidate(-.8f,input);
	assert(clamped.candidate_clamped && close(clamped.target,.5f));
	input.mean_servo=.97f;
	clamped=implement_full_candidate(-.8f,input);
	assert(clamped.headroom_clamped && close(clamped.target,.02f));
	input.mean_servo=1.f;
	assert(!mpc.solve_full_roll({0.f,0.f,0.f},input).proposal.solver_success);
	input.mean_servo=0.f;input.trim=std::numeric_limits<float>::quiet_NaN();
	assert(!mpc.solve_full_roll({0.f,0.f,0.f},input).proposal.solver_success);
	std::puts("limits_and_invalid_input,PASS,75 boundary cases plus clamp and nonfinite rejection");

	const uint64_t before_copy=1000000, message=1000001, after_copy=1000010;
	assert((before_copy-message)/1000.f>1.e12f);
	assert(close(input_age_ms(after_copy,message),.009f));
	assert(input_fresh(after_copy,message,200000));
	assert(!input_fresh(before_copy,message,200000));
	assert(input_age_ms(before_copy,message)<0.f);
	assert(!input_fresh(after_copy,0,200000));
	assert(input_fresh(after_copy,after_copy-200000,200000));
	assert(!input_fresh(after_copy,after_copy-200001,200000));
	std::puts("timestamp_race,PASS,old unsigned underflow reproduced fresh-after-copy accepted true stale future zero rejected");

	FullMpcFeedback feedback{};
	float previous=0.f;
	assert(full_previous_input(1000010,1000000,1,4,3,-.26f,feedback,previous)==FullHistorySource::EntryTrim);
	assert(close(previous,.26f));
	feedback.timestamp=1000010;feedback.entry_timestamp=1000000;feedback.session=1;feedback.mode=6;
	feedback.left_index=4;feedback.right_index=3;feedback.left=-.1f;feedback.right=.15f;
	assert(full_previous_input(1000020,1000000,1,4,3,-.26f,feedback,previous)==FullHistorySource::FinalOutput);
	assert(close(previous,.25f));
	assert(full_previous_input(1300020,1000000,1,4,3,-.26f,feedback,previous)==FullHistorySource::Invalid);
	assert(full_previous_input(1000005,1000000,1,4,3,-.26f,feedback,previous)==FullHistorySource::Invalid);
	feedback.left=std::numeric_limits<float>::quiet_NaN();
	assert(full_previous_input(1000020,1000000,1,4,3,-.26f,feedback,previous)==FullHistorySource::Invalid);
	feedback.left=-.1f;
	assert(full_previous_input(1400010,1400000,2,4,3,-.3f,feedback,previous)==FullHistorySource::EntryTrim);
	assert(close(previous,.3f));
	assert(full_previous_input(1700010,1400000,2,4,3,-.3f,feedback,previous)==FullHistorySource::Invalid);
	std::puts("applied_history,PASS,actual feedback entry bootstrap expired future NaN old session and reentry");
	for (int i=0;i<2000;++i) { assert(!full_duration_timed_out(0.f,i*.02f)); }
	assert(!full_duration_timed_out(3.f,3.f) && full_duration_timed_out(3.f,3.01f));
	std::puts("duration_semantics,PASS,40 seconds unlimited and positive finite timeout helper");
	std::puts("NOTE,NOT_TESTED,real hardware hydrodynamics mechanical angle and full PX4 scheduling");
}
