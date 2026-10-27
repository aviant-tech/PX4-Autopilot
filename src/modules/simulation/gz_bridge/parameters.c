/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * Simulator Gazebo bridge enable
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(SIM_GZ_EN, 0);

/**
 * Enable laser/lidar sensors in Gazebo bridge
 *
 * @boolean
 * @reboot_required true
 * @group Simulator
 */
PARAM_DEFINE_INT32(SIM_GZ_EN_LIDAR, 1);

/**
 * Simulated motor thrust model factor
 *
 * Shapes the rotor speed sent to Gazebo so that the simulated thrust follows
 * thrust = (1 - f) * u + f * u^2 (normalised) of the motor command u.
 * Set it equal to THR_MDL_FAC of the simulated vehicle. With 1 the rotor speed
 * is proportional to the command, i.e. thrust is quadratic.
 * u is normalised over [SIM_GZ_EC_MINn, SIM_GZ_EC_MAXn] and the shaped rotor
 * speed is mapped back to that range; outputs at or below the minimum are not shaped.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_GZ_THR_MDL, 1.0f);
