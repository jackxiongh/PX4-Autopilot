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
 * @file ActuatorEffectivenessOmniHex.cpp
 *
 * Actuator effectiveness for omnidirectional hexacopter
 *
 * @author Yueqian Liu <yueqianliu@outlook.com>
 */

#include "ActuatorEffectivenessOmniHex.hpp"

ActuatorEffectivenessOmniHex::ActuatorEffectivenessOmniHex() : ModuleParams(nullptr) {}

void ActuatorEffectivenessOmniHex::computeEffectivenessMatrix(float arm_length, float thrust_coef, float torque_coef, matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness)
{
    float l = arm_length;
    float ct = thrust_coef;
    float kt = torque_coef;

    effectiveness.setZero();

    effectiveness(0, 0) = -ct * l;
    effectiveness(0, 1) = -kt;
    effectiveness(0, 2) = ct * l;
    effectiveness(0, 3) = -kt;
    effectiveness(0, 4) = 0.5f * ct * l;
    effectiveness(0, 5) = 0.5f * kt;
    effectiveness(0, 6) = -0.5f * ct * l;
    effectiveness(0, 7) = 0.5f * kt;
    effectiveness(0, 8) = -0.5f * ct * l;
    effectiveness(0, 9) = 0.5f * kt;
    effectiveness(0, 10) = 0.5f * ct * l;
    effectiveness(0, 11) = 0.5f * kt;

    effectiveness(1, 4) = 0.5f * sqrtf(3) * ct * l;
    effectiveness(1, 5) = 0.5f * sqrtf(3) * kt;
    effectiveness(1, 6) = -0.5f * sqrtf(3) * ct * l;
    effectiveness(1, 7) = 0.5f * sqrtf(3) * kt;
    effectiveness(1, 8) = 0.5f * sqrtf(3) * ct * l;
    effectiveness(1, 9) = -0.5f * sqrtf(3) * kt;
    effectiveness(1, 10) = -0.5f * sqrtf(3) * ct * l;
    effectiveness(1, 11) = -0.5f * sqrtf(3) * kt;

    effectiveness(2, 0) = -kt;
    effectiveness(2, 1) = ct * l;
    effectiveness(2, 2) = kt;
    effectiveness(2, 3) = ct * l;
    effectiveness(2, 4) = -kt;
    effectiveness(2, 5) = ct * l;
    effectiveness(2, 6) = kt;
    effectiveness(2, 7) = ct * l;
    effectiveness(2, 8) = kt;
    effectiveness(2, 9) = ct * l;
    effectiveness(2, 10) = -kt;
    effectiveness(2, 11) = ct * l;
    
    effectiveness(3, 1) = -ct;
    effectiveness(3, 3) = ct;
    effectiveness(3, 5) = 0.5f * ct;
    effectiveness(3, 7) = -0.5f * ct;
    effectiveness(3, 9) = -0.5f * ct;
    effectiveness(3, 11) = 0.5f * ct;

    effectiveness(4, 5) = 0.5f * sqrtf(3) * ct;
    effectiveness(4, 7) = -0.5f * sqrtf(3) * ct;
    effectiveness(4, 9) = 0.5f * sqrtf(3) * ct;
    effectiveness(4, 11) = -0.5f * sqrtf(3) * ct;

    effectiveness(5, 0) = -ct;
    effectiveness(5, 2) = -ct;
    effectiveness(5, 4) = -ct;
    effectiveness(5, 6) = -ct;
    effectiveness(5, 8) = -ct;
    effectiveness(5, 10) = -ct;

}

bool ActuatorEffectivenessOmniHex::getEffectivenessMatrix(matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &matrix)
{
    if(_parameter_update_sub.updated())
    {
        
        parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
        
        float al = _param_ca_oh_al.get();
        float ct = _param_ca_oh_ct.get();
        float kt = _param_ca_oh_kt.get();

        computeEffectivenessMatrix(al, ct, kt, matrix);
        return true;
    }
    return false;
}