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
 * Multi GPS primary instance
 *
 * This defines the preferred GPS receiver instance.
 * The GPS selection logic waits until the primary receiver is available to
 * send data to the EKF even if a secondary instance is already available.
 * The secondary instance is then only used if the primary one times out.
 *
 * @group Sensors
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(SENS_GPS_PRIM, 0);

/**
 * GPS timeout
 *
 * The maximum time the system will wait for a GPS message
 * before considering the sensor unhealthy.
 *
 * @group Sensors
 * @min 0.1
 * @max 10.0
 * @unit s
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SENS_GPS_TOUT, 0.33f);

/**
 * GPS selection hysteresis
 *
 * Require the primary instance to be healthy for at least this amount of
 * time before switching back from the secondary instance.
 *
 * If the secondary instance is unhealthy, while the primary instance is healthy,
 * the system will switch instantly, ignoring this parameter.
 *
 * @group Sensors
 * @min 0.0
 * @max 120.0
 */
PARAM_DEFINE_FLOAT(SENS_GPS_HYST, 5.0f);
