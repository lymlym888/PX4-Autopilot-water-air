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
 * 6 = full roll-MPC only in Stabilized flight mode (NAVIGATION_STATE_STAB).
 * Outside Stabilized, 6 uses the Mode 1 legacy flight-mode path with no MPC output.
 * No separate full-enable, trim-confirmation or test-duration parameter is required.
 * Sensor/reference validity, source freshness and actuator bounds still apply.
 *
 * @group Fixed MPC
 */
PARAM_DEFINE_INT32(FMPC_MODE, 1);

/**
 * Allow active tiny correction if other gates pass.
 *
 * 0 = forbidden
 * 1 = may be allowed
 * Used by Mode 3/5 only; does not gate Mode 6.
 *
 * @group Fixed MPC
 */
PARAM_DEFINE_INT32(FMPC_ACT_EN, 0);

/**
 * User-confirmed servo-domain sign for tiny correction.
 *
 * 0 = not confirmed
 * 1 = confirmed by staged bench check
 * Used by Mode 3/5 only; does not gate Mode 6.
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
 * Used when FMPC_MODE=6 and FMPC_FTRIM_EN=1 to clamp the requested
 * d_target_a = d_trim_latched_a + full_mpc_corr_a.
 * This is left-minus-right normalized command, not per-servo travel.
 * Physical servo bounds, common-mode headroom and SAT_MARGIN still apply.
 * Set the envelope in Mode 1 before entering Mode 6; trim stays latched.
 *
 * @unit norm
 * @min 0.10
 * @max 2.0
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
 * Full roll-MPC trim-relative differential correction limit.
 *
 * Left-minus-right normalized correction, not a single-servo travel fraction.
 * Runtime-clamped to <= 2.0. FULL_SRC=0/1 retain the additional 0.05 cap;
 * FULL_SRC=2 uses this limit. FCAND_MAX and physical headroom still apply.
 * Set the envelope in Mode 1 before entering Mode 6; trim stays latched.
 *
 * @unit norm
 * @min 0.0
 * @max 2.0
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_FULL_LIM, 0.05f);

/**
 * Mode 6 differential command slew rate.
 *
 * Normalized left-minus-right differential change per second, not servo deg/s.
 * At the fixed 50-Hz update rate, 3.0 retains the original 0.06 step.
 * Applies only to Mode 6. Physical-bound recovery may exceed this normal step.
 * Read at runtime without reboot. Adjust in Mode 1, then re-test Mode 6.
 * Higher values may increase jitter and actuator loads. Zero does not disable it.
 *
 * @unit 1/s
 * @min 1.5
 * @max 6.0
 * @decimal 2
 * @increment 0.3
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_FULL_SLEW, 3.0f);

/**
 * Mode 6 roll-angle-error feedback multiplier.
 *
 * Multiplies actual roll minus reference before full-roll prediction.
 * 1.0 retains the original response. Larger values request stronger correction
 * for the same angle error, including small errors. Not a deadband or Q weight.
 * Rate feedback, trim, model matrices and output limits are not multiplied.
 * Applies at all error magnitudes in Mode 6 only; limits can mask its effect.
 * Read at runtime without reboot. Adjust in Mode 1, one setting at a time.
 * Increased gain can reduce damping and amplify noise; bench/water validation required.
 *
 * @min 0.5
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_ERR_GAIN, 1.0f);

/**
 * Full roll-MPC source selection.
 *
 * Mode 6 evaluates the implemented trim-relative command in model coordinates.
 * 0/1 = retain the shadow correction cap of 0.05 in addition to FULL_LIM.
 * 2 = use FULL_LIM without the additional shadow correction cap.
 *
 * @group Fixed MPC
 */
PARAM_DEFINE_INT32(FMPC_FULL_SRC, 2);

/**
 * Use a configured static differential trim in full roll-MPC mode.
 *
 * 0 = old absolute full-target behavior
 * 1 = use FMPC_TRIM_A, validated and frozen on each STAB/Mode 6 entry.
 * Live CADRC output is not a trim source. No separate confirmation is required.
 *
 * @group Fixed MPC
 */
PARAM_DEFINE_INT32(FMPC_FTRIM_EN, 1);

/**
 * Static full roll-MPC differential operating point.
 *
 * Normalized HY_SV0 minus HY_SV1 command, before PWM reversal/scaling.
 * Zero denotes zero command differential, not a calibrated mechanical angle.
 * Review the value and physical mapping in Mode 1 before entering STAB/Mode 6.
 * The value is captured on entry and does not follow CADRC. No confirmation flag.
 * An out-of-bounds value is rejected, never silently projected or zeroed.
 *
 * @unit norm
 * @min -2.0
 * @max 2.0
 * @group Fixed MPC
 */
PARAM_DEFINE_FLOAT(FMPC_TRIM_A, 0.0f);
