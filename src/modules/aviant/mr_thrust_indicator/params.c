/**
 * Thrust indicator calculation interval
 *
 * The thrust indicator will be recalculated at this interval
 *
 * @group Aviant
 * @unit ms
 * @decimal 1
 * @min 10
 * @max 1000
 * @reboot_required true
 */
PARAM_DEFINE_INT32(AV_THR_IND_INT, 100);

/**
* Thrust indicator time constant
*
* Time constant for the 1st order IIR filter which smooths the thrust indicator over time.
* The higher the constant, the more filtering
*
* @group Aviant
* @unit s
* @decimal 1
* @min 0
* @max 60
* @reboot_required true
*/
PARAM_DEFINE_FLOAT(AV_THR_IND_TAU, 2.f);
