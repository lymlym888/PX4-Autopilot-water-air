uint64 timestamp                         # publication time [us]
uint64 timestamp_sample                  # gyro/control sample time [us]

uint8 CONTROLLER_ID_LEGACY = 0
uint8 CONTROLLER_ID_CADRC = 1
uint8 CONTROLLER_ID_LADRC = 2
uint8 CONTROLLER_ID_MCEO_ADRC = 3
uint8 CONTROLLER_ID_AMCEO_ADRC = 4
uint8 CONTROLLER_ID_NADRC = 5

uint8 controller_id                      # active controller ID
uint8 requested_controller_id            # current selector request
uint8 previous_controller_id             # active controller before last accepted edge
uint32 reset_counter
uint32 fault_counter

bool output_valid
bool reference_valid
bool manual_valid
bool magnitude_limited
bool rate_limited
bool selector_edge
bool switch_blocked
bool fallback_active
bool dt_invalid                         # current new-controller interval failed timing validity
bool timing_fault                       # timing-fault recovery/reset is required

float32 phi                              # measured roll angle [rad]
float32 p                                # measured body roll rate [rad/s]

float32 manual_roll_raw                  # raw manual roll stick [-1, 1]
float32 command_roll_latched             # latched roll-angle command [rad]
float32 manual_command_age_s              # age of latched manual sample [s]
uint8 manual_data_source

float32 phi_ref                          # generated roll reference [rad]
float32 phi_ref_dot                      # generated roll-reference rate [rad/s]
float32 phi_ref_ddot                     # generated roll-reference acceleration [rad/s^2]

float32 u_requested                      # normalized request before common limiter
float32 u_applied                        # actual normalized roll command after common limiter

float32 dt_measured                      # measured gyro/control interval [s]
float32 dt_used                          # controller integration interval [s]

# Controller-specific diagnostics. Unused fields are NaN.
float32 state_1
float32 state_2
float32 state_3
float32 state_4
float32 disturbance_estimate
float32 observer_bandwidth
float32 rho
float32 sigma_e
float32 sigma_u
