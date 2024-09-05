/**
 * @file fake_gps_params.c
 *
 * Parameters for fake gps
 *
 * @author None <none@none.none>
 */


/**
 * Enable fake GPS
 *
 * if set to 1, the fake GPS will be enabled
 *
 * @boolean
 * @group Fake GPS
 */
PARAM_DEFINE_INT32(FAKE_GPS_EN, 0);


/**
 * Fake GPS Latitude
 *
 * This value is used as the fake GPS latitude.
 *
 * @unit deg
 * @min -90
 * @max 90
 * @decimal 6
 * @increment 0.000001
 * @group Fake GPS
 */
PARAM_DEFINE_FLOAT(FAKE_GPS_LAT, 47.398039);

/**
 * Fake GPS Longitude
 *
 * This value is used as the fake GPS longitude.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 6
 * @increment 0.000001
 * @group Fake GPS
 */
PARAM_DEFINE_FLOAT(FAKE_GPS_LON, 8.545572);

/**
 * Fake GPS altitude
 *
 * This value is used as the fake GPS altitude
 *
 * @unit deg
 * @min -100
 * @max 1000
 * @decimal 2
 * @increment 0.01
 * @group Fake GPS
 */
PARAM_DEFINE_FLOAT(FAKE_GPS_ALT, 10.0);
