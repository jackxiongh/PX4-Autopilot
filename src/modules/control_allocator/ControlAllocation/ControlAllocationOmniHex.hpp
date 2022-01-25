/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationOmniHex.hpp
 *
 * Control allocation algorithm exclusively built for omni hex
 *
 * @author Yueqian Liu <yueqianliu@outlook.com>
 */

#pragma once

#include "ControlAllocationPseudoInverse.hpp"

#include <px4_platform_common/module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/arm_rotation.h>
#include <uORB/topics/vehicle_status.h>

class ControlAllocationOmniHex: public ControlAllocationPseudoInverse, public ModuleParams
{

public:
    
    ControlAllocationOmniHex() : ModuleParams(nullptr)
    {
        _allocation_raw.setZero();
        _rotor_sp.setZero();
        _servo_sp.setZero();
        _last_servo_sp.setZero();
        _servo_delta.setZero();
        _allocation_sp.setZero();
        _force_sp.setZero();
    }

    virtual ~ControlAllocationOmniHex() = default;

    void allocate() override;

    void publish_servo_setpoint();

    void print_debug_info();

    void updateParameters() override;

    void limit_servo_delta();

    void handle_singularity();

    float angle_between(matrix::Vector<float, 3> a, matrix::Vector<float, 3> b);

private:

    uORB::Publication<arm_rotation_s> _arm_rotation_pub{ORB_ID(arm_rotation)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

    matrix::Vector<float, NUM_ACTUATORS> _allocation_raw{};
    matrix::Vector<float, NUM_ACTUATORS> _rotor_sp{};
	matrix::Vector<float, 6> _servo_sp{};
    matrix::Vector<float, 6> _last_servo_sp{};
    matrix::Vector<float, 6> _servo_delta{};
    matrix::Vector<float, NUM_ACTUATORS> _allocation_sp{};
    matrix::Vector<float, 3> _force_sp{};

    float _servo_delta_lim = 0.05f;
    bool _limit_reached = false;
    bool _debug_switch_flag = false;
    float _force_norm_thresh = 5.0f;
    float _damp_thresh = 0.175f;
    float _align_thresh = 0.09f;
    int _rotate_direction = 0;

    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::CA_OH_ARM_D_MAX>) _param_ca_oh_arm_d_max,
        (ParamFloat<px4::params::CA_OH_FORCE_THR>) _param_ca_oh_force_thr,
        (ParamBool<px4::params::CA_OH_DEBUG_EN>) _param_ca_oh_debug_en,
        (ParamFloat<px4::params::CA_OH_DAMP_THR>) _param_ca_oh_damp_thr,
        (ParamFloat<px4::params::CA_OH_ALGN_THR>) _param_ca_oh_algn_thr,
        (ParamInt<px4::params::CA_OH_ROTATE_DIR>) _param_ca_oh_rotate_dir
    )
};

