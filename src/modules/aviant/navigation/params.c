/**
 * Navigation indicator calculation interval
 *
 * The navigation indicator will be recalculated at this interval
 *
 * @group Aviant
 * @unit ms
 * @decimal 1
 * @min 10
 * @max 1000
 * @reboot_required true
 */
PARAM_DEFINE_INT32(AV_NAV_INT, 100);

/**
 * RTK/mag difference warning threshold
 *
 * The threshold for heading difference between GNSS and magnetometer to trigger redundancy warning
 * If set to 0, no redundancy warning will be triggered based on heading difference
 *
 * @group Aviant
 * @unit deg
 * @decimal 1
 * @min 0
 * @max 180
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_NAV_HDIFF_W, 40.0f);
