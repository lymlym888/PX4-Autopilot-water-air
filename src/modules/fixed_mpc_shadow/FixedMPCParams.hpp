#pragma once

// Generated from Stage 4a fixed_mpc_parameter_export.json.
// Shadow-mode/HIL readiness only. Physical actuator sign remains unconfirmed.

#ifndef FIXED_MPC_SHADOW_ALLOW_ACTIVE_OUTPUT
#define FIXED_MPC_SHADOW_ALLOW_ACTIVE_OUTPUT 0
#endif

namespace fixed_mpc_shadow {

static constexpr float kTs = 0.02f;
static constexpr int kHorizon = 10;
static constexpr float kA00 = 0.9955910379423533f;
static constexpr float kA01 = 0.020069261247937932f;
static constexpr float kA10 = -0.2204481028823385f;
static constexpr float kA11 = 1.0034630623968968f;
static constexpr float kB0 = -0.0004309727952737038f;
static constexpr float kB1 = -0.021548639763685186f;
static constexpr float kC0 = 9.660239187144425e-05f;
static constexpr float kC1 = 0.004830119593572212f;
static constexpr float kQPhi = 1.0f;
static constexpr float kQP = 0.5f;
static constexpr float kRDelta = 0.005f;
static constexpr float kRDDelta = 0.05f;
static constexpr float kDeltaMax = 0.35f;
static constexpr float kDDeltaMax = 0.06f;
static constexpr float kShadowCorrectionLimit = 0.05f;
static constexpr float kRollAbsMaxRad = 15.0f * 0.01745329252f;
static constexpr float kRollRateAbsMaxRadS = 80.0f * 0.01745329252f;
static constexpr float kBatteryMinV = 14.0f;
static constexpr uint32_t kSolveTimeMaxUs = 10000u;

}  // namespace fixed_mpc_shadow
