/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * Selector error reduce threshold
 *
 * EKF2 instances have to be better than the selected by at least this amount before their relative score can be reduced.
 *
 * @group EKF2
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_ERR_RED, 0.2f);

/**
 * Selector angular rate threshold
 *
 * EKF2 selector angular rate error threshold for comparing gyros. Angular rate vector differences larger than this will result in accumulated angular error.
 *
 * @group EKF2
 * @unit deg/s
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_IMU_RAT, 7.0f);

/**
 * Selector angular threshold.
 *
 * EKF2 selector maximum accumulated angular error threshold for comparing gyros. Accumulated angular error larger than this will result in the sensor being declared faulty.
 *
 * @group EKF2
 * @unit deg
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_IMU_ANG, 15.0f);

/**
 * Selector acceleration threshold
 *
 * EKF2 selector acceleration error threshold for comparing accelerometers. Acceleration vector differences larger than this will result in accumulated velocity error.
 *
 * @group EKF2
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_IMU_ACC, 1.0f);

/**
 * Selector angular threshold.
 *
 * EKF2 selector maximum accumulated velocity threshold for comparing accelerometers. Accumulated velocity error larger than this will result in the sensor being declared faulty.
 *
 * @group EKF2
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_IMU_VEL, 2.0f);

/**
 * When to select GNSS-denied EKF2 instances.
 *
 * Only has an effect if EKF2_GNSS_DENIED is set to 1.
 *
 * @group EKF2
 * @value 0 Never
 * @value 1 OFFBOARD mode
 * @value 2 OFFBOARD or DISARMED
 * @value 3 Always
 *
 */
PARAM_DEFINE_INT32(EKF2_SEL_GNSSDEN, 0);

/**
 * Ignore EKF2 instances with specific device ID.
 *
 * Set to -1 to disable this feature.
 *
 * @group EKF2
 *
 */
PARAM_DEFINE_INT32(EKF2_SEL_IGN_ID, -1);
