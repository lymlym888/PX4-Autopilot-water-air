#include <parameters/param.h>

/**
 * Fixed MPC framework mode.
 *
 * 0 = OFF / CADRC_ONLY
 * 1 = SHADOW_ONLY
 * 2 = GATED_SHADOW
 * 3 = ACTIVE_TINY
 * 4 = FALLBACK / CADRC_ONLY
 * 5 = ACTIVE_BOUNDED_MPC
 * 6 = FULL_ROLL_MPC_TEST
 *
 * @group Fixed MPC
 */
PARAM_DEFINE_INT32(FMPC_MODE, 1);

/**
 * Allow active tiny correction if other gates pass.
 *
 * 0 = forbidden
 * 1 = may be allowed
 *
 * @group Fixed MPC
 */
PARAM_DEFINE_INT32(FMPC_ACT_EN, 0);

/**
 * User-confirmed servo-domain sign for tiny correction.
 *
 * 0 = not confirmed
 * 1 = confirmed by staged bench check
 *
 * @group Fixed MPC
 */
PARAM_DEFINE_INT32(FMPC_SIGN_OK, 0);

/**
 * Tiny active correction limit.
 *
 * Runtime-clamped to <= 0.02 in code.
 *
 * @unit norm
 * @min 0.0
 * @max 0.02
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_ACT_LIM, 0.0f);

/**
 * Direction-gate base threshold.
 *
 * @unit norm
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_G_BASEMIN, 0.10f);

/**
 * Gated-shadow correction limit.
 *
 * @unit norm
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_G_LIM, 0.05f);

/**
 * Low-base gated correction limit.
 *
 * @unit norm
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_G_LOWLIM, 0.02f);

/**
 * Active roll bound.
 *
 * @unit deg
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_ROLL_MAX, 8.0f);

/**
 * Active roll-rate bound.
 *
 * @unit deg/s
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_RATE_MAX, 35.0f);

/**
 * Active candidate differential bound.
 *
 * @unit norm
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_CAND_MAX, 0.30f);

/**
 * Full roll-MPC final differential candidate bound for trim-aware Mode 6.
 *
 * Used when FMPC_MODE=6 and FMPC_FTRIM_EN=1 to validate
 * d_target_a = d_base_a + full_mpc_corr_a.
 *
 * @unit norm
 * @min 0.10
 * @max 0.80
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_FCAND_MAX, 0.50f);

/**
 * Allow bounded active MPC if other gates pass.
 *
 * 0 = forbidden
 * 1 = may be allowed
 *
 * @group Fixed MPC
 */
PARAM_DEFINE_INT32(FMPC_BND_EN, 0);

/**
 * Bounded active MPC correction limit.
 *
 * Runtime-clamped to <= 0.05 in code.
 *
 * @unit norm
 * @min 0.0
 * @max 0.05
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_BND_LIM, 0.02f);

/**
 * Servo saturation headroom margin for bounded/full modes.
 *
 * @unit norm
 * @min 0.0
 * @max 0.1
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_SAT_MARGIN, 0.02f);

/**
 * Allow full roll-MPC differential mode if other gates pass.
 *
 * 0 = forbidden
 * 1 = may be allowed
 *
 * @group Fixed MPC
 */
PARAM_DEFINE_INT32(FMPC_FULL_EN, 0);

/**
 * Full roll-MPC differential limit.
 *
 * Runtime-clamped to <= 0.35 in code.
 *
 * @unit norm
 * @min 0.0
 * @max 0.35
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_FULL_LIM, 0.05f);

/**
 * Full roll-MPC active duration timeout.
 *
 * Runtime-clamped to <= 5.0 s in code.
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_FULL_DUR, 3.0f);

/**
 * Full roll-MPC source selection.
 *
 * 0 = d_gate_corr_a
 * 1 = d_corr_a / d_corr
 * 2 = d_pre
 *
 * @group Fixed MPC
 */
PARAM_DEFINE_INT32(FMPC_FULL_SRC, 2);

/**
 * Preserve live CADRC/manual trim in full roll-MPC mode.
 *
 * 0 = old absolute full-target behavior
 * 1 = trim-aligned behavior using current d_base_a as trim
 *
 * @group Fixed MPC
 */
PARAM_DEFINE_INT32(FMPC_FTRIM_EN, 1);
