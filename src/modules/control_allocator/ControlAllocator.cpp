/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ControlAllocator.cpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocator.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace time_literals;

ControlAllocator::ControlAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_control_allocator_status_pub[0].advertise();
	_control_allocator_status_pub[1].advertise();

	_actuator_motors_pub.advertise();
	_actuator_servos_pub.advertise();
	_actuator_servos_trim_pub.advertise();

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_R%u_SLEW", i);
		_param_handles.slew_rate_motors[i] = param_find(buffer);
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV%u_SLEW", i);
		_param_handles.slew_rate_servos[i] = param_find(buffer);
	}

	_param_handles.fmpc_mode = param_find("FMPC_MODE");

	for (int i = 0; i < 2; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "HY_SV%u_IDX", i);
		_param_handles.hy_sv_idx[i] = param_find(buffer);
	}

	parameters_updated();
}

ControlAllocator::~ControlAllocator()
{
	for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
		delete _control_allocation[i];
	}

	delete _actuator_effectiveness;

	perf_free(_loop_perf);
}

bool
ControlAllocator::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!_vehicle_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	ScheduleDelayed(50_ms);
#endif

	return true;
}

void
ControlAllocator::parameters_updated()
{
	_has_slew_rate = false;

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		param_get(_param_handles.slew_rate_motors[i], &_params.slew_rate_motors[i]);
		_has_slew_rate |= _params.slew_rate_motors[i] > FLT_EPSILON;
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		param_get(_param_handles.slew_rate_servos[i], &_params.slew_rate_servos[i]);
		_has_slew_rate |= _params.slew_rate_servos[i] > FLT_EPSILON;
	}

	// Allocation method & effectiveness source
	// Do this first: in case a new method is loaded, it will be configured below
	bool updated = update_effectiveness_source();
	update_allocation_method(updated); // must be called after update_effectiveness_source()

	if (_num_control_allocation == 0) {
		return;
	}

	for (int i = 0; i < _num_control_allocation; ++i) {
		_control_allocation[i]->updateParameters();
	}

	update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::CONFIGURATION_UPDATE);
}

void
ControlAllocator::update_allocation_method(bool force)
{
	AllocationMethod configured_method = (AllocationMethod)_param_ca_method.get();

	if (!_actuator_effectiveness) {
		PX4_ERR("_actuator_effectiveness null");
		return;
	}

	if (_allocation_method_id != configured_method || force) {

		matrix::Vector<float, NUM_ACTUATORS> actuator_sp[ActuatorEffectiveness::MAX_NUM_MATRICES];

		// Cleanup first
		for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
			// Save current state
			if (_control_allocation[i] != nullptr) {
				actuator_sp[i] = _control_allocation[i]->getActuatorSetpoint();
			}

			delete _control_allocation[i];
			_control_allocation[i] = nullptr;
		}

		_num_control_allocation = _actuator_effectiveness->numMatrices();

		AllocationMethod desired_methods[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getDesiredAllocationMethod(desired_methods);

		bool normalize_rpy[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getNormalizeRPY(normalize_rpy);

		for (int i = 0; i < _num_control_allocation; ++i) {
			AllocationMethod method = configured_method;

			if (configured_method == AllocationMethod::AUTO) {
				method = desired_methods[i];
			}

			switch (method) {
			case AllocationMethod::PSEUDO_INVERSE:
				_control_allocation[i] = new ControlAllocationPseudoInverse();
				break;

			case AllocationMethod::SEQUENTIAL_DESATURATION:
				_control_allocation[i] = new ControlAllocationSequentialDesaturation();
				break;

			default:
				PX4_ERR("Unknown allocation method");
				break;
			}

			if (_control_allocation[i] == nullptr) {
				PX4_ERR("alloc failed");
				_num_control_allocation = 0;

			} else {
				_control_allocation[i]->setNormalizeRPY(normalize_rpy[i]);
				_control_allocation[i]->setActuatorSetpoint(actuator_sp[i]);
			}
		}

		_allocation_method_id = configured_method;
	}
}

bool
ControlAllocator::update_effectiveness_source()
{
	const EffectivenessSource source = (EffectivenessSource)_param_ca_airframe.get();

	if (_effectiveness_source_id != source) {

		// try to instanciate new effectiveness source
		ActuatorEffectiveness *tmp = nullptr;

		switch (source) {
		case EffectivenessSource::NONE:
		case EffectivenessSource::MULTIROTOR:
			tmp = new ActuatorEffectivenessMultirotor(this);
			break;

		case EffectivenessSource::STANDARD_VTOL:
			tmp = new ActuatorEffectivenessStandardVTOL(this);
			break;

		case EffectivenessSource::TILTROTOR_VTOL:
			tmp = new ActuatorEffectivenessTiltrotorVTOL(this);
			break;

		case EffectivenessSource::TAILSITTER_VTOL:
			tmp = new ActuatorEffectivenessTailsitterVTOL(this);
			break;

		case EffectivenessSource::ROVER_ACKERMANN:
			tmp = new ActuatorEffectivenessRoverAckermann();
			break;

		case EffectivenessSource::ROVER_DIFFERENTIAL:
			// differential_drive_control does allocation and publishes directly to actuator_motors topic
			break;

		case EffectivenessSource::FIXED_WING:
			tmp = new ActuatorEffectivenessFixedWing(this);
			break;

		case EffectivenessSource::MOTORS_6DOF: // just a different UI from MULTIROTOR
			tmp = new ActuatorEffectivenessUUV(this);
			break;

		case EffectivenessSource::MULTIROTOR_WITH_TILT:
			tmp = new ActuatorEffectivenessMCTilt(this);
			break;

		case EffectivenessSource::CUSTOM:
			tmp = new ActuatorEffectivenessCustom(this);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_ESC:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::MOTORS);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_SERVO:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::SERVOS);
			break;

		case EffectivenessSource::HELICOPTER_COAXIAL:
			tmp = new ActuatorEffectivenessHelicopterCoaxial(this);
			break;

		default:
			PX4_ERR("Unknown airframe");
			break;
		}

		// Replace previous source with new one
		if (tmp == nullptr) {
			// It did not work, forget about it
			PX4_ERR("Actuator effectiveness init failed");
			_param_ca_airframe.set((int)_effectiveness_source_id);

		} else {
			// Swap effectiveness sources
			delete _actuator_effectiveness;
			_actuator_effectiveness = tmp;

			// Save source id
			_effectiveness_source_id = source;
		}

		return true;
	}

	return false;
}

void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		_vehicle_thrust_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	// Push backup schedule
	ScheduleDelayed(50_ms);
#endif

	// Check if parameters have changed
	if (_parameter_update_sub.updated() && !_armed) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		if (_handled_motor_failure_bitmask == 0) {
			// We don't update the geometry after an actuator failure, as it could lead to unexpected results
			// (e.g. a user could add/remove motors, such that the bitmask isn't correct anymore)
			updateParams();
			parameters_updated();
		}
	}

	if (_num_control_allocation == 0 || _actuator_effectiveness == nullptr) {
		return;
	}

	{
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {

			_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

			ActuatorEffectiveness::FlightPhase flight_phase{ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT};

			// Check if the current flight phase is HOVER or FIXED_WING
			if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				flight_phase = ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT;

			} else {
				flight_phase = ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;
			}

			// Special cases for VTOL in transition
			if (vehicle_status.is_vtol && vehicle_status.in_transition_mode) {
				if (vehicle_status.in_transition_to_fw) {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_HF_TO_FF;

				} else {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_FF_TO_HF;
				}
			}

			// Forward to effectiveness source
			_actuator_effectiveness->setFlightPhase(flight_phase);
		}
	}

	{
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
			_publish_controls = vehicle_control_mode.flag_control_allocation_enabled;
		}
	}

	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);
		// PX4_INFO("_torque_sp: %f %f %f", (double)_torque_sp(0), (double)_torque_sp(1), (double)_torque_sp(2));

		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	// Also run allocator on thrust setpoint changes if the torque setpoint
	// has not been updated for more than 5ms
	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);

		if (dt > 0.005f) {
			do_update = true;
			_timestamp_sample = vehicle_thrust_setpoint.timestamp_sample;
		}
	}
	// PX4_INFO("_thrust_torque_sp: %f %f %f %f %f %f", (double)_thrust_sp(0), (double)_thrust_sp(1), (double)_thrust_sp(2), (double)_torque_sp(0), (double)_torque_sp(1), (double)_torque_sp(2));

	if (do_update) {
		_last_run = now;

		check_for_motor_failures();

		update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::NO_EXTERNAL_UPDATE);

		// Set control setpoint vector(s)
		matrix::Vector<float, NUM_AXES> c[ActuatorEffectiveness::MAX_NUM_MATRICES];
		c[0](0) = _torque_sp(0);
		c[0](1) = _torque_sp(1);
		c[0](2) = _torque_sp(2);
		c[0](3) = _thrust_sp(0);
		c[0](4) = _thrust_sp(1);
		c[0](5) = _thrust_sp(2);

		if (_num_control_allocation > 1) {
			if (_vehicle_torque_setpoint1_sub.copy(&vehicle_torque_setpoint)) {
				c[1](0) = vehicle_torque_setpoint.xyz[0];
				c[1](1) = vehicle_torque_setpoint.xyz[1];
				c[1](2) = vehicle_torque_setpoint.xyz[2];
			}

			if (_vehicle_thrust_setpoint1_sub.copy(&vehicle_thrust_setpoint)) {
				c[1](3) = vehicle_thrust_setpoint.xyz[0];
				c[1](4) = vehicle_thrust_setpoint.xyz[1];
				c[1](5) = vehicle_thrust_setpoint.xyz[2];
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {

			_control_allocation[i]->setControlSetpoint(c[i]);

			// Do allocation
			_control_allocation[i]->allocate();
			_actuator_effectiveness->allocateAuxilaryControls(dt, i, _control_allocation[i]->_actuator_sp); //flaps and spoilers
			_actuator_effectiveness->updateSetpoint(c[i], i, _control_allocation[i]->_actuator_sp,
								_control_allocation[i]->getActuatorMin(), _control_allocation[i]->getActuatorMax());

			if (_has_slew_rate) {
				_control_allocation[i]->applySlewRateLimit(dt);
			}

			_control_allocation[i]->clipActuatorSetpoint();
		}
	}

	// PX4_INFO("_actuator_sp: %f %f %f %f %f %f %f %f %f ", (double)_control_allocation[0]->_actuator_sp(0), (double)_control_allocation[0]->_actuator_sp(1), (double)_control_allocation[0]->_actuator_sp(2),
	// 					(double)_control_allocation[0]->_actuator_sp(3), (double)_control_allocation[0]->_actuator_sp(4), (double)_control_allocation[0]->_actuator_sp(5),
	// 					(double)_control_allocation[0]->_actuator_sp(6), (double)_control_allocation[0]->_actuator_sp(7), (double)_control_allocation[0]->_actuator_sp(8));
	// PX4_INFO("%f %f %f %f %f %f %f", (double)_control_allocation[0]->_actuator_sp(9), (double)_control_allocation[0]->_actuator_sp(10), (double)_control_allocation[0]->_actuator_sp(11),
	// 					(double)_control_allocation[0]->_actuator_sp(12), (double)_control_allocation[0]->_actuator_sp(13), (double)_control_allocation[0]->_actuator_sp(14),
	// 					(double)_control_allocation[0]->_actuator_sp(15));


	// Publish actuator setpoint and allocator status
	publish_actuator_controls();

	// Publish status at limited rate, as it's somewhat expensive and we use it for slower dynamics
	// (i.e. anti-integrator windup)
	if (now - _last_status_pub >= 5_ms) {
		publish_control_allocator_status(0);

		if (_num_control_allocation > 1) {
			publish_control_allocator_status(1);
		}

		_last_status_pub = now;
	}

	perf_end(_loop_perf);
}

void
ControlAllocator::update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason)
{
	ActuatorEffectiveness::Configuration config{};

	if (reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE
	    && hrt_elapsed_time(&_last_effectiveness_update) < 100_ms) { // rate-limit updates
		return;
	}

	if (_actuator_effectiveness->getEffectivenessMatrix(config, reason)) {
		_last_effectiveness_update = hrt_absolute_time();

		memcpy(_control_allocation_selection_indexes, config.matrix_selection_indexes,
		       sizeof(_control_allocation_selection_indexes));

		// Get the minimum and maximum depending on type and configuration
		ActuatorEffectiveness::ActuatorVector minimum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector maximum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector slew_rate[ActuatorEffectiveness::MAX_NUM_MATRICES];
		int actuator_idx = 0;
		int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

		actuator_servos_trim_s trims{};
		static_assert(actuator_servos_trim_s::NUM_CONTROLS == actuator_servos_s::NUM_CONTROLS, "size mismatch");

		for (int actuator_type = 0; actuator_type < (int)ActuatorType::COUNT; ++actuator_type) {
			_num_actuators[actuator_type] = config.num_actuators[actuator_type];

			for (int actuator_type_idx = 0; actuator_type_idx < config.num_actuators[actuator_type]; ++actuator_type_idx) {
				if (actuator_idx >= NUM_ACTUATORS) {
					_num_actuators[actuator_type] = 0;
					PX4_ERR("Too many actuators");
					break;
				}

				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if ((ActuatorType)actuator_type == ActuatorType::MOTORS) {
					if (actuator_type_idx >= MAX_NUM_MOTORS) {
						PX4_ERR("Too many motors");
						_num_actuators[actuator_type] = 0;
						break;
					}

					if (_param_r_rev.get() & (1u << actuator_type_idx)) {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;

					} else {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 0.f;
					}

					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_motors[actuator_type_idx];

				} else if ((ActuatorType)actuator_type == ActuatorType::SERVOS) {
					if (actuator_type_idx >= MAX_NUM_SERVOS) {
						PX4_ERR("Too many servos");
						_num_actuators[actuator_type] = 0;
						break;
					}

					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_servos[actuator_type_idx];
					trims.trim[actuator_type_idx] = config.trim[selected_matrix](actuator_idx_matrix[selected_matrix]);

				} else {
					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
				}

				maximum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 1.f;

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		// Handle failed actuators
		if (_handled_motor_failure_bitmask) {
			actuator_idx = 0;
			memset(&actuator_idx_matrix, 0, sizeof(actuator_idx_matrix));

			for (int motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if (_handled_motor_failure_bitmask & (1 << motors_idx)) {
					ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[selected_matrix];

					for (int i = 0; i < NUM_AXES; i++) {
						matrix(i, actuator_idx_matrix[selected_matrix]) = 0.0f;
					}
				}

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {
			_control_allocation[i]->setActuatorMin(minimum[i]);
			_control_allocation[i]->setActuatorMax(maximum[i]);
			_control_allocation[i]->setSlewRateLimit(slew_rate[i]);

			// Set all the elements of a row to 0 if that row has weak authority.
			// That ensures that the algorithm doesn't try to control axes with only marginal control authority,
			// which in turn would degrade the control of the main axes that actually should and can be controlled.

			ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[i];

			for (int n = 0; n < NUM_AXES; n++) {
				bool all_entries_small = true;

				for (int m = 0; m < config.num_actuators_matrix[i]; m++) {
					if (fabsf(matrix(n, m)) > 0.05f) {
						all_entries_small = false;
					}
				}

				if (all_entries_small) {
					matrix.row(n) = 0.f;
				}
			}

			// Assign control effectiveness matrix
			int total_num_actuators = config.num_actuators_matrix[i];
			_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i],
					config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
		}

		trims.timestamp = hrt_absolute_time();
		_actuator_servos_trim_pub.publish(trims);
	}
}

void
ControlAllocator::publish_control_allocator_status(int matrix_index)
{
	control_allocator_status_s control_allocator_status{};
	control_allocator_status.timestamp = hrt_absolute_time();

	// TODO: disabled motors (?)

	// Allocated control
	const matrix::Vector<float, NUM_AXES> &allocated_control = _control_allocation[matrix_index]->getAllocatedControl();

	// Unallocated control
	const matrix::Vector<float, NUM_AXES> unallocated_control = _control_allocation[matrix_index]->getControlSetpoint() -
			allocated_control;
	control_allocator_status.unallocated_torque[0] = unallocated_control(0);
	control_allocator_status.unallocated_torque[1] = unallocated_control(1);
	control_allocator_status.unallocated_torque[2] = unallocated_control(2);
	control_allocator_status.unallocated_thrust[0] = unallocated_control(3);
	control_allocator_status.unallocated_thrust[1] = unallocated_control(4);
	control_allocator_status.unallocated_thrust[2] = unallocated_control(5);

	// override control_allocator_status in customized saturation logic for certain effectiveness types
	_actuator_effectiveness->getUnallocatedControl(matrix_index, control_allocator_status);

	// Allocation success flags
	control_allocator_status.torque_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_torque[0],
			control_allocator_status.unallocated_torque[1],
			control_allocator_status.unallocated_torque[2]).norm_squared() < 1e-6f);
	control_allocator_status.thrust_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_thrust[0],
			control_allocator_status.unallocated_thrust[1],
			control_allocator_status.unallocated_thrust[2]).norm_squared() < 1e-6f);

	// Actuator saturation
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_sp = _control_allocation[matrix_index]->getActuatorSetpoint();
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_min = _control_allocation[matrix_index]->getActuatorMin();
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_max = _control_allocation[matrix_index]->getActuatorMax();

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		if (actuator_sp(i) > (actuator_max(i) - FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;

		} else if (actuator_sp(i) < (actuator_min(i) + FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_LOWER;
		}
	}

	// Handled motor failures
	control_allocator_status.handled_motor_failure_mask = _handled_motor_failure_bitmask;

	_control_allocator_status_pub[matrix_index].publish(control_allocator_status);
}

void
ControlAllocator::publish_actuator_controls()
{
	if (!_publish_controls) {
		return;
	}

	actuator_motors_s actuator_motors;
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _timestamp_sample;

	actuator_servos_s actuator_servos;
	actuator_servos.timestamp = actuator_motors.timestamp;
	actuator_servos.timestamp_sample = _timestamp_sample;

	actuator_motors.reversible_flags = _param_r_rev.get();

	int actuator_idx = 0;
	int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

	// uint32_t stopped_motors = _actuator_effectiveness->getStoppedMotors() | _handled_motor_failure_bitmask;

	_hydro_motors_sub.update(&_hydro_motors);
	_hydro_servos_sub.update(&_hydro_servos);
	_fixed_mpc_allocator_status_sub.update(&_fixed_mpc_allocator_status);

	_manual_control_setpoint_sub.copy(&_manual_control_setpoint);

	// 螺旋桨启用/禁用功能
	bool enable_propeller = false;
	float prop_aux;
	switch (_param_hy_prop_aux.get()) {
		case 0:
		prop_aux = 0;
		break;

		case 1:
		prop_aux = _manual_control_setpoint.aux1;
		break;

		case 2:
		prop_aux = _manual_control_setpoint.aux2;
		break;

		case 3:
		prop_aux = _manual_control_setpoint.aux3;
		break;

		case 4:
		prop_aux = _manual_control_setpoint.aux4;
		break;

		case 5:
		prop_aux = _manual_control_setpoint.aux5;
		break;

		case 6:
		prop_aux = _manual_control_setpoint.aux6;
		break;

		case 7:
		prop_aux = -1.0f;
		break;

		case 8:
		prop_aux = 1.0f;
		break;

		default:
		prop_aux = 0;
	}
	prop_aux *= _param_hy_prop_auxgain.get();

	if(prop_aux > _param_hy_prop_thr.get())
	{
		enable_propeller = true;
	}

	int prop_idx = _param_hy_prop_idx.get();

	// 副翼启用/禁用功能
	bool enable_aileron = false;
	float aile_aux;
	switch (_param_hy_aile_aux.get()) {
		case 0:
		aile_aux = 0;
		break;

		case 1:
		aile_aux = _manual_control_setpoint.aux1;
		break;

		case 2:
		aile_aux = _manual_control_setpoint.aux2;
		break;

		case 3:
		aile_aux = _manual_control_setpoint.aux3;
		break;

		case 4:
		aile_aux = _manual_control_setpoint.aux4;
		break;

		case 5:
		aile_aux = _manual_control_setpoint.aux5;
		break;

		case 6:
		aile_aux = _manual_control_setpoint.aux6;
		break;

		case 7:
		aile_aux = -1.0f;
		break;

		case 8:
		aile_aux = 1.0f;
		break;

		default:
		aile_aux = 0;
	}
	aile_aux *= _param_hy_aile_auxgain.get();

	if(aile_aux > _param_hy_aile_thr.get())
	{
		enable_aileron = true;
	}

	int aile_0_idx = _param_hy_aile0_idx.get();
	int aile_1_idx = _param_hy_aile1_idx.get();

	// motors
	int motors_idx;

	for (motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
		int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
		float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
		actuator_sp += _hydro_motors.control[motors_idx];// 加上hydro_allocator的输出

		//螺旋桨禁用
		if(!enable_propeller && motors_idx == prop_idx-1){
			actuator_sp = 0;
		}

		if(PX4_ISFINITE(actuator_sp))
		{
			actuator_sp = math::constrain(actuator_sp, -1.0f, 1.0f);
			if(actuator_sp < 0.02f && actuator_sp > -0.02f)
			{
				actuator_motors.control[motors_idx] = NAN;
			}
			else
			{
				actuator_motors.control[motors_idx] = actuator_sp;
			}
		}
		else
		{
			actuator_motors.control[motors_idx] = NAN;
		}
		// stopped motors用于在杆量小于2%时关闭电机，避免遥控器误差造成的影响，但在这里会错误地关闭水翼电机，所以把stopMaskedMotorsWithZeroThrust函数中的内容移到这里，取代原来的功能
		// if (stopped_motors & (1u << motors_idx)) {
		// 	actuator_motors.control[motors_idx] = NAN;
		// }

		++actuator_idx_matrix[selected_matrix];
		++actuator_idx;
	}

	for (int i = motors_idx; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	_actuator_motors_pub.publish(actuator_motors);

	// servos
	if (_num_actuators[1] > 0) {
		const int servo_count = math::min(_num_actuators[1], static_cast<int>(actuator_servos_s::NUM_CONTROLS));
		float ordinary_servo[actuator_servos_s::NUM_CONTROLS] {};
		float servo_min[actuator_servos_s::NUM_CONTROLS] {};
		float servo_max[actuator_servos_s::NUM_CONTROLS] {};

		for (int servos_idx = 0; servos_idx < servo_count; servos_idx++) {
			const int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
			const int selected_index = actuator_idx_matrix[selected_matrix];
			ordinary_servo[servos_idx] = _control_allocation[selected_matrix]->getActuatorSetpoint()(selected_index);
			// Keep the original final actuator_servos bounds for legacy modes.
			servo_min[servos_idx] = -1.f;
			servo_max[servos_idx] = 1.f;
			++actuator_idx_matrix[selected_matrix];
			++actuator_idx;
		}

		for (int servos_idx = 0; servos_idx < servo_count; servos_idx++) {
			float actuator_sp = ordinary_servo[servos_idx] + _hydro_servos.control[servos_idx];

			// Preserve the existing aileron-enable safety behavior for every non-selected channel.
			if (!enable_aileron && (servos_idx == aile_0_idx - 1 || servos_idx == aile_1_idx - 1)) {
				actuator_sp = 0.f;
			}

			actuator_servos.control[servos_idx] = PX4_ISFINITE(actuator_sp) ?
				math::constrain(actuator_sp, servo_min[servos_idx], servo_max[servos_idx]) : NAN;
		}

		int32_t requested_mode_param = 1;
		int32_t left_index_param = 0;
		int32_t right_index_param = 0;

		if (_param_handles.fmpc_mode != PARAM_INVALID) {
			param_get(_param_handles.fmpc_mode, &requested_mode_param);
		}

		if (_param_handles.hy_sv_idx[0] != PARAM_INVALID) {
			param_get(_param_handles.hy_sv_idx[0], &left_index_param);
		}

		if (_param_handles.hy_sv_idx[1] != PARAM_INVALID) {
			param_get(_param_handles.hy_sv_idx[1], &right_index_param);
		}

		const uint8_t requested_mode = static_cast<uint8_t>(math::constrain(requested_mode_param, int32_t{0}, int32_t{6}));
		const int left_index = left_index_param - 1;
		const int right_index = right_index_param - 1;
		const bool indices_valid = left_index >= 0 && right_index >= 0 && left_index < servo_count &&
					   right_index < servo_count && left_index != right_index;
		const hrt_abstime now = actuator_servos.timestamp;
		const bool status_timestamp_valid = _fixed_mpc_allocator_status.timestamp > 0 &&
						  _fixed_mpc_allocator_status.timestamp <= now;
		const bool allocator_status_fresh = status_timestamp_valid &&
						(now - _fixed_mpc_allocator_status.timestamp <= roll_fin_output::kSourceMaxAgeUs);
		const bool status_indices_match = _fixed_mpc_allocator_status.left_index == left_index &&
						 _fixed_mpc_allocator_status.right_index == right_index;
		const bool legacy_source_current = allocator_status_fresh && status_indices_match &&
						 _fixed_mpc_allocator_status.requested_mode == requested_mode;

		roll_fin_output::Input selector_input{};
		selector_input.now = now;
		selector_input.requested_mode = requested_mode;
		selector_input.left_index = left_index;
		selector_input.right_index = right_index;
		selector_input.servo_count = servo_count;
		selector_input.legacy_source_current = legacy_source_current;

		if (indices_valid) {
			const bool left_channel_enabled = enable_aileron ||
				(left_index != aile_0_idx - 1 && left_index != aile_1_idx - 1);
			const bool right_channel_enabled = enable_aileron ||
				(right_index != aile_0_idx - 1 && right_index != aile_1_idx - 1);
			selector_input.left_channel_enabled = left_channel_enabled;
			selector_input.right_channel_enabled = right_channel_enabled;
			selector_input.ordinary_left = ordinary_servo[left_index];
			selector_input.ordinary_right = ordinary_servo[right_index];
			selector_input.left_min = servo_min[left_index];
			selector_input.left_max = servo_max[left_index];
			selector_input.right_min = servo_min[right_index];
			selector_input.right_max = servo_max[right_index];

			if (legacy_source_current) {
				selector_input.legacy_hydro_left = _fixed_mpc_allocator_status.srv_l_o;
				selector_input.legacy_hydro_right = _fixed_mpc_allocator_status.srv_r_o;

			} else {
				selector_input.legacy_hydro_left = _hydro_servos.control[left_index];
				selector_input.legacy_hydro_right = _hydro_servos.control[right_index];
			}
		}

		selector_input.source.status_timestamp = _fixed_mpc_allocator_status.timestamp;
		selector_input.source.source_timestamp = _fixed_mpc_allocator_status.source_timestamp;
		selector_input.source.mode_entry_timestamp = _fixed_mpc_allocator_status.mode_entry_timestamp;
		selector_input.source.selector_epoch = _fixed_mpc_allocator_status.selector_epoch;
		selector_input.source.sequence = _fixed_mpc_allocator_status.source_sequence;
		selector_input.source.session_id = _fixed_mpc_allocator_status.mode_session_id;
		selector_input.source.requested_mode = _fixed_mpc_allocator_status.requested_mode;
		selector_input.source.source_mode = _fixed_mpc_allocator_status.source_mode;
		selector_input.source.left_index = _fixed_mpc_allocator_status.left_index;
		selector_input.source.right_index = _fixed_mpc_allocator_status.right_index;
		selector_input.source.command_valid = _fixed_mpc_allocator_status.command_valid;
		selector_input.source.fallback_valid = _fixed_mpc_allocator_status.fallback_valid;
		selector_input.source.command_left = _fixed_mpc_allocator_status.srv_l_o;
		selector_input.source.command_right = _fixed_mpc_allocator_status.srv_r_o;
		selector_input.source.fallback_left = _fixed_mpc_allocator_status.srv_l_b;
		selector_input.source.fallback_right = _fixed_mpc_allocator_status.srv_r_b;
		selector_input.source.requested_diff = _fixed_mpc_allocator_status.requested_target_a;
		selector_input.source.implemented_diff = _fixed_mpc_allocator_status.source_implemented_target_a;
		selector_input.source.fallback_reason = _fixed_mpc_allocator_status.fb_r;

		const roll_fin_output::Result selection = roll_fin_output::select(selector_input, _roll_fin_selector_state);

		roll_fin_output::apply_selected_pair(selector_input, selection, actuator_servos.control,
						     actuator_servos_s::NUM_CONTROLS);

		fixed_mpc_final_output_status_s final_status{};
		final_status.timestamp = now;
		final_status.source_timestamp = _fixed_mpc_allocator_status.source_timestamp;
		final_status.source_mode_entry_timestamp = _fixed_mpc_allocator_status.mode_entry_timestamp;
		final_status.selector_epoch = _roll_fin_selector_state.mode_entry_timestamp;
		final_status.source_selector_epoch = _fixed_mpc_allocator_status.selector_epoch;
		final_status.last_event_timestamp = _roll_fin_selector_state.last_event_timestamp;
		final_status.source_sequence = _fixed_mpc_allocator_status.source_sequence;
		final_status.source_session_id = _fixed_mpc_allocator_status.mode_session_id;
		final_status.authority_transition_count = _roll_fin_selector_state.authority_transition_count;
		final_status.fallback_event_count = _roll_fin_selector_state.fallback_event_count;
		final_status.requested_mode = requested_mode;
		final_status.selected_owner = static_cast<uint8_t>(selection.owner);
		final_status.selector_reason = static_cast<uint8_t>(selection.reason);
		final_status.upstream_fallback_reason = _fixed_mpc_allocator_status.fb_r;
		final_status.last_event_reason = _roll_fin_selector_state.last_event_reason;
		final_status.last_upstream_fallback_reason = _roll_fin_selector_state.last_upstream_fallback_reason;
		final_status.left_index = static_cast<int8_t>(left_index);
		final_status.right_index = static_cast<int8_t>(right_index);
		final_status.source_fresh = selection.source_fresh;
		final_status.source_session_valid = selection.source_session_valid;
		final_status.source_command_valid = selection.source_command_valid;
		final_status.source_fallback_valid = selection.source_fallback_valid;
		final_status.target_clamped = selection.target_clamped || _fixed_mpc_allocator_status.source_target_clamped;
		final_status.channels_enabled = indices_valid && selector_input.left_channel_enabled &&
						selector_input.right_channel_enabled;
		final_status.ordinary_left = indices_valid ? ordinary_servo[left_index] : NAN;
		final_status.ordinary_right = indices_valid ? ordinary_servo[right_index] : NAN;
		final_status.hydro_left = indices_valid ?
			(requested_mode == roll_fin_output::kModeFull ? selector_input.source.command_left :
			 selector_input.legacy_hydro_left) : NAN;
		final_status.hydro_right = indices_valid ?
			(requested_mode == roll_fin_output::kModeFull ? selector_input.source.command_right :
			 selector_input.legacy_hydro_right) : NAN;
		final_status.final_left = indices_valid ? actuator_servos.control[left_index] : NAN;
		final_status.final_right = indices_valid ? actuator_servos.control[right_index] : NAN;
		final_status.requested_target_a = _fixed_mpc_allocator_status.requested_target_a;
		final_status.source_implemented_target_a = _fixed_mpc_allocator_status.source_implemented_target_a;
		final_status.final_implemented_diff_a = indices_valid ?
			actuator_servos.control[left_index] - actuator_servos.control[right_index] : NAN;
		final_status.takeover_jump_a = selection.takeover_jump;
		_fixed_mpc_final_output_status_pub.publish(final_status);

		// PX4_INFO("_num_actuators[1]: %i , NUM_CONTROLS: %i", _num_actuators[1], actuator_servos_s::NUM_CONTROLS); // 8 8
		for (int i = servo_count; i < actuator_servos_s::NUM_CONTROLS; i++) {
			actuator_servos.control[i] = NAN;
		}

		_actuator_servos_pub.publish(actuator_servos);
	}
	// PX4_INFO("actuator_servos: %f %f %f %f %f %f %f %f", (double)actuator_servos.control[0], (double)actuator_servos.control[1], (double)actuator_servos.control[2], (double)actuator_servos.control[3],
	// 			(double)actuator_servos.control[4], (double)actuator_servos.control[5], (double)actuator_servos.control[6], (double)actuator_servos.control[7]);
}

void
ControlAllocator::check_for_motor_failures()
{
	failure_detector_status_s failure_detector_status;

	if ((FailureMode)_param_ca_failure_mode.get() > FailureMode::IGNORE
	    && _failure_detector_status_sub.update(&failure_detector_status)) {
		if (failure_detector_status.fd_motor) {

			if (_handled_motor_failure_bitmask != failure_detector_status.motor_failure_mask) {
				// motor failure bitmask changed
				switch ((FailureMode)_param_ca_failure_mode.get()) {
				case FailureMode::REMOVE_FIRST_FAILING_MOTOR: {
						// Count number of failed motors
						const int num_motors_failed = math::countSetBits(failure_detector_status.motor_failure_mask);

						// Only handle if it is the first failure
						if (_handled_motor_failure_bitmask == 0 && num_motors_failed == 1) {
							_handled_motor_failure_bitmask = failure_detector_status.motor_failure_mask;
							PX4_WARN("Removing motor from allocation (0x%x)", _handled_motor_failure_bitmask);

							for (int i = 0; i < _num_control_allocation; ++i) {
								_control_allocation[i]->setHadActuatorFailure(true);
							}

							update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
						}
					}
					break;

				default:
					break;
				}

			}

		} else if (_handled_motor_failure_bitmask != 0) {
			// Clear bitmask completely
			PX4_INFO("Restoring all motors");
			_handled_motor_failure_bitmask = 0;

			for (int i = 0; i < _num_control_allocation; ++i) {
				_control_allocation[i]->setHadActuatorFailure(false);
			}

			update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
		}
	}
}

int ControlAllocator::task_spawn(int argc, char *argv[])
{
	ControlAllocator *instance = new ControlAllocator();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ControlAllocator::print_status()
{
	PX4_INFO("Running");

	// Print current allocation method
	switch (_allocation_method_id) {
	case AllocationMethod::NONE:
		PX4_INFO("Method: None");
		break;

	case AllocationMethod::PSEUDO_INVERSE:
		PX4_INFO("Method: Pseudo-inverse");
		break;

	case AllocationMethod::SEQUENTIAL_DESATURATION:
		PX4_INFO("Method: Sequential desaturation");
		break;

	case AllocationMethod::AUTO:
		PX4_INFO("Method: Auto");
		break;
	}

	// Print current airframe
	if (_actuator_effectiveness != nullptr) {
		PX4_INFO("Effectiveness Source: %s", _actuator_effectiveness->name());
	}

	// Print current effectiveness matrix
	for (int i = 0; i < _num_control_allocation; ++i) {
		const ActuatorEffectiveness::EffectivenessMatrix &effectiveness = _control_allocation[i]->getEffectivenessMatrix();

		if (_num_control_allocation > 1) {
			PX4_INFO("Instance: %i", i);
		}

		PX4_INFO("  Effectiveness.T =");
		effectiveness.T().print();
		PX4_INFO("  minimum =");
		_control_allocation[i]->getActuatorMin().T().print();
		PX4_INFO("  maximum =");
		_control_allocation[i]->getActuatorMax().T().print();
		PX4_INFO("  Configured actuators: %i", _control_allocation[i]->numConfiguredActuators());
	}

	if (_handled_motor_failure_bitmask) {
		PX4_INFO("Failed motors: %i (0x%x)", math::countSetBits(_handled_motor_failure_bitmask),
			 _handled_motor_failure_bitmask);
	}

	// Print perf
	perf_print_counter(_loop_perf);

	return 0;
}

int ControlAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ControlAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("control_allocator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Control Allocator app start / stop handling function
 */
extern "C" __EXPORT int control_allocator_main(int argc, char *argv[]);

int control_allocator_main(int argc, char *argv[])
{
	return ControlAllocator::main(argc, argv);
}
