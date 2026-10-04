/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * Enable simulated GPS sinstance
 *
 * @reboot_required true
 * @min 0
 * @max 1
 * @group Sensors
 * @value 0 Disabled
 * @value 1 Enabled
  */
PARAM_DEFINE_INT32(SENS_EN_GPSSIM, 0);

/**
 * simulated GPS number of satellites used
 *
 * @min 0
 * @max  50
 * @group Simulator
 */
PARAM_DEFINE_INT32(SIM_GPS_USED, 10);

/**
 * Simulated dual GPS with moving-baseline heading
 *
 * 1: two RTK-fixed receivers in a moving-baseline pair. sensor_gps instance 0 is the
 * rover at SIM_GPS_POS + SIM_GPS_REL, it reports the heading of the base->rover vector
 * (sensor_gps.heading, sensor_gnss_relative); instance 1 is the moving base at SIM_GPS_POS.
 * 0: one GPS at the CG (default).
 *
 * @boolean
 * @reboot_required true
 * @group Simulator
 */
PARAM_DEFINE_INT32(SIM_GPS_DUAL, 0);

/**
 * Simulated GPS antenna X position (moving base with SIM_GPS_DUAL)
 *
 * Body frame (forward) position relative to the CG. Only used with SIM_GPS_DUAL 1.
 *
 * @unit m
 * @decimal 2
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_POS_X, 0.f);

/**
 * Simulated GPS antenna Y position (moving base with SIM_GPS_DUAL)
 *
 * Body frame (right) position relative to the CG. Only used with SIM_GPS_DUAL 1.
 *
 * @unit m
 * @decimal 2
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_POS_Y, 0.f);

/**
 * Simulated GPS antenna Z position (moving base with SIM_GPS_DUAL)
 *
 * Body frame (down) position relative to the CG. Only used with SIM_GPS_DUAL 1.
 *
 * @unit m
 * @decimal 2
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_POS_Z, 0.f);

/**
 * Simulated GPS rover antenna X offset from the moving base
 *
 * Body frame (forward) offset of the rover antenna from the moving base antenna (SIM_GPS_DUAL 1),
 * the simulated counterpart of SENS_GNSSREL_PX.
 *
 * @unit m
 * @decimal 2
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_REL_X, -0.5f);

/**
 * Simulated GPS rover antenna Y offset from the moving base
 *
 * Body frame (right) offset of the rover antenna from the moving base antenna (SIM_GPS_DUAL 1),
 * the simulated counterpart of SENS_GNSSREL_PY.
 *
 * @unit m
 * @decimal 2
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_REL_Y, 0.f);

/**
 * Simulated GPS rover antenna Z offset from the moving base
 *
 * Body frame (down) offset of the rover antenna from the moving base antenna (SIM_GPS_DUAL 1),
 * the simulated counterpart of SENS_GNSSREL_PZ.
 *
 * @unit m
 * @decimal 2
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GPS_REL_Z, 0.f);
