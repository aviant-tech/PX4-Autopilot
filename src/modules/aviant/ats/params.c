/**
 * ATS Timeout threshold
 *
 * Time threshold for considering the flight controller as not sending data.
 *
 * @group Aviant
 * @unit ms
 * @decimal 1
 * @min 0
 * @max 1000
 * @reboot_required true
 */
PARAM_DEFINE_INT32(AV_ATS_TIMEOUT, 150);

/**
 * ATS Acceleration norm threshold
 *
 * Acceleration norm threshold for determining when to deploy the parachute
 *
 * @group Aviant
 * @unit m/s^2
 * @decimal 1
 * @min 0
 * @max 1000
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(AV_ATS_ACC_NORM, 5);

/**
 * ATS Roll angle threshold
 *
 * If the roll angle exceeds this threshold, this criterion is considered met.
 *
 * @group Aviant
 * @unit deg
 * @decimal 1
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(AV_ATS_ROLL_ANG, 80);

/**
 * ATS Pitch angle threshold
 *
 * If the pitch angle exceeds this threshold, this criterion is considered met.
 *
 * @group Aviant
 * @unit deg
 * @decimal 1
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(AV_ATS_PITCH_ANG, 60);

/**
 * ATS active
 *
 * When the ATS is active, it will check the trigger conditions and
 * command flight termination / parachute deployment
 *
 *
 * @group Aviant
 * @boolean
 * @reboot_required true
 */
PARAM_DEFINE_INT32(AV_ATS_ACTIVE, 0);