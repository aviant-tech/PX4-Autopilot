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
 * Magnetometer innovation test ratio fail threshold
 *
 * Don't consider the magnetometer healthy if any of the innovation test ratios exceed this threshold
 *
 * Ignored if set to 0.0
 *
 * @group Aviant
 * @decimal 1
 * @min 0
 * @max 1
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(AV_NAV_MAG_ITR, 0.40f);
