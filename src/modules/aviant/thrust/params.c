/**
 * Top motor bitmask
 *
 * Bit N = motor N (0-based). E.g. 0b1111 = motors 0-3.
 *
 * @group Aviant
 * @min 0
 * @max 4095
 * @bit 0 Motor 1
 * @bit 1 Motor 2
 * @bit 2 Motor 3
 * @bit 3 Motor 4
 * @bit 4 Motor 5
 * @bit 5 Motor 6
 * @bit 6 Motor 7
 * @bit 7 Motor 8
 * @bit 8 Motor 9
 * @bit 9 Motor 10
 * @bit 10 Motor 11
 * @bit 11 Motor 12
 */
PARAM_DEFINE_INT32(AV_THR_MOT_TOP, 0);

/**
 * Pusher motor bitmask
 *
 * Bit N = motor N (0-based). E.g. 0b10000 = motor 4.
 *
 * @group Aviant
 * @min 0
 * @max 4095
 * @bit 0 Motor 1
 * @bit 1 Motor 2
 * @bit 2 Motor 3
 * @bit 3 Motor 4
 * @bit 4 Motor 5
 * @bit 5 Motor 6
 * @bit 6 Motor 7
 * @bit 7 Motor 8
 * @bit 8 Motor 9
 * @bit 9 Motor 10
 * @bit 10 Motor 11
 * @bit 11 Motor 12
 */
PARAM_DEFINE_INT32(AV_THR_MOT_PSH, 0);

/* ── Anomalous current model ── */

/**
 * Top motor phase current reference
 *
 * @group Aviant
 * @unit A
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(AV_THR_TOP_IREF, 0.0f);

/**
 * Top motor current exponent
 *
 * @group Aviant
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(AV_THR_TOP_K, 0.0f);

/**
 * Top motor phase voltage reference for current model
 *
 * @group Aviant
 * @unit V
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(AV_THR_TOP_VREF, 0.0f);

/**
 * Pusher motor phase current reference
 *
 * @group Aviant
 * @unit A
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(AV_THR_PSH_IREF, 0.0f);

/**
 * Pusher motor current exponent
 *
 * @group Aviant
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(AV_THR_PSH_K, 0.0f);

/**
 * Pusher motor phase voltage reference for current model
 *
 * @group Aviant
 * @unit V
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(AV_THR_PSH_VREF, 0.0f);

/**
 * Anomalous current critical low threshold
 *
 * Critical when filtered error <= -AV_THR_CRI_LO_A. 0 disables.
 *
 * @group Aviant
 * @unit A
 * @decimal 1
 * @min 0
 */
PARAM_DEFINE_FLOAT(AV_THR_CRI_LO_A, 0.0f);

/**
 * Anomalous current warning low threshold
 *
 * Warning when filtered error <= -AV_THR_WRN_LO_A. 0 disables.
 *
 * @group Aviant
 * @unit A
 * @decimal 1
 * @min 0
 */
PARAM_DEFINE_FLOAT(AV_THR_WRN_LO_A, 0.0f);

/**
 * Anomalous current warning high threshold
 *
 * Warning when filtered error >= AV_THR_WRN_HI_A. 0 disables.
 *
 * @group Aviant
 * @unit A
 * @decimal 1
 * @min 0
 */
PARAM_DEFINE_FLOAT(AV_THR_WRN_HI_A, 0.0f);

/**
 * Anomalous current critical high threshold
 *
 * Critical when filtered error >= AV_THR_CRI_HI_A. 0 disables.
 *
 * @group Aviant
 * @unit A
 * @decimal 1
 * @min 0
 */
PARAM_DEFINE_FLOAT(AV_THR_CRI_HI_A, 0.0f);

/**
 * Anomalous current LPF time constant
 *
 * @group Aviant
 * @unit s
 * @decimal 1
 * @min 0
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(AV_THR_CUR_TAU, 0.0f);

/**
 * Phase voltage LPF time constant
 *
 * @group Aviant
 * @unit s
 * @decimal 1
 * @min 0
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(AV_THR_VPHA_TAU, 3.0f);

/* ── Top motor phase voltage limits ── */

/**
 * Top motor phase voltage warning limit
 *
 * Steady-state temperature exceeds spec in worst-case weather.
 *
 * @group Aviant
 * @unit V
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(AV_THR_TOP_WRN_V, 0.0f);

/**
 * Top motor phase voltage critical limit
 *
 * Magnetic saturation expected.
 *
 * @group Aviant
 * @unit V
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(AV_THR_TOP_CRI_V, 0.0f);

/* ── Pusher motor phase voltage limits ── */

/**
 * Pusher motor phase voltage warning limit
 *
 * @group Aviant
 * @unit V
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(AV_THR_PSH_WRN_V, 0.0f);

/**
 * Pusher motor phase voltage critical limit
 *
 * @group Aviant
 * @unit V
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(AV_THR_PSH_CRI_V, 0.0f);
