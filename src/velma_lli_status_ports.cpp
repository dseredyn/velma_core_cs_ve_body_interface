/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "velma_low_level_interface/velma_lli_status_ports.h"

using velma_low_level_interface_msgs::VelmaLowLevelStatus;

namespace velma_lli_types {

// example of specialized code
// arm
/*
template < >
void PortData<Eigen::VectorXd, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_t_type, &VelmaLowLevelCommandArm::t>::convertToROS() {
    for (int i = 0; i < 7; ++i) {
        getDataRef()(i) = container_.t[i];
    }
}
template < >
void PortData<Eigen::VectorXd, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_t_type, &VelmaLowLevelCommandArm::t>::convertFromROS() {
    for (int i = 0; i < 7; ++i) {
        container_.t[i] = getDataRef()(i);
    }
}
*/

//
// ArmStatus_Ports interface
//
template <template <typename Type> class T >
ArmStatus_Ports<T >::ArmStatus_Ports(RTT::TaskContext &tc, const std::string &prefix) :
    q_(tc, prefix + "_q"),
    dq_(tc, prefix + "_dq"),
    t_(tc, prefix + "_t"),
    gt_(tc, prefix + "_gt"),
    w_(tc, prefix + "_w"),
    mmx_(tc, prefix + "_mmx"),
    friIntfState_(tc, prefix + "_friIntfState"),
    friRobotState_(tc, prefix + "_friRobotState")
{}

// read ports
template <>
bool ArmStatus_Ports<RTT::InputPort >::readPorts() {
    bool result = q_.operation();
    result &= dq_.operation();
    result &= t_.operation();
    result &= gt_.operation();
    result &= w_.operation();
    result &= mmx_.operation();
    result &= friIntfState_.operation();
    result &= friRobotState_.operation();
    return result;
}

// write ports
template <>
void ArmStatus_Ports<RTT::OutputPort >::writePorts() {
    q_.operation();
    dq_.operation();
    t_.operation();
    gt_.operation();
    w_.operation();
    mmx_.operation();
    friIntfState_.operation();
    friRobotState_.operation();
}


template <>
void ArmStatus_Ports<RTT::OutputPort >::convertFromROS(const VelmaLowLevelStatusArm &ros) {
    q_.convertFromROS(ros);
    dq_.convertFromROS(ros);
    t_.convertFromROS(ros);
    gt_.convertFromROS(ros);
    w_.convertFromROS(ros);
    mmx_.convertFromROS(ros);
    friIntfState_.convertFromROS(ros);
    friRobotState_.convertFromROS(ros);
}

template <>
void ArmStatus_Ports<RTT::InputPort >::convertToROS(VelmaLowLevelStatusArm &ros) {
    q_.convertToROS(ros);
    dq_.convertToROS(ros);
    t_.convertToROS(ros);
    gt_.convertToROS(ros);
    w_.convertToROS(ros);
    mmx_.convertToROS(ros);
    friIntfState_.convertToROS(ros);
    friRobotState_.convertToROS(ros);
}





//
// HandStatus_Ports interface
//
template <template <typename Type> class T >
HandStatus_Ports<T>::HandStatus_Ports(RTT::TaskContext &tc, const std::string &prefix) :
    q_(tc, prefix + "_q"),
    s_(tc, prefix + "_s")
{
}

// read ports
template <>
bool HandStatus_Ports<RTT::InputPort >::readPorts() {
    bool result = q_.operation();
    result &= s_.operation();
    return result;
}

// write ports
template <>
void HandStatus_Ports<RTT::OutputPort >::writePorts() {
    q_.operation();
    s_.operation();
}

template <>
void HandStatus_Ports<RTT::OutputPort >::convertFromROS(const VelmaLowLevelStatusHand &ros) {
    q_.convertFromROS(ros);
    s_.convertFromROS(ros);
}

template <>
void HandStatus_Ports<RTT::InputPort >::convertToROS(VelmaLowLevelStatusHand &ros) {
    q_.convertToROS(ros);
    s_.convertToROS(ros);
}


//
// MotorStatus_Ports interface
//
template <template <typename Type> class T >
MotorStatus_Ports<T>::MotorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix) :
    q_(tc, prefix + "_q"),
    dq_(tc, prefix + "_dq"),
    valid_(false)
{
}

// read ports
template <>
bool MotorStatus_Ports<RTT::InputPort >::readPorts() {
    valid_ = q_.operation();
    valid_ &= dq_.operation();
    return valid_;
}

// write ports
template <>
void MotorStatus_Ports<RTT::OutputPort >::writePorts() {
    if (valid_) {
        q_.operation();
        dq_.operation();
    }
}

template <>
void MotorStatus_Ports<RTT::OutputPort >::convertFromROS(const VelmaLowLevelStatusMotor &ros) {
    q_.convertFromROS(ros);
    dq_.convertFromROS(ros);
    valid_ = ros.valid;
}

template <>
void MotorStatus_Ports<RTT::InputPort >::convertToROS(VelmaLowLevelStatusMotor &ros) {
    q_.convertToROS(ros);
    dq_.convertToROS(ros);
    ros.valid = valid_;
}



//
// FTSensorStatus_Ports interface
//
template <template <typename Type> class T>
FTSensorStatus_Ports<T >::FTSensorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix) :
    rw_(tc, prefix + "_rw"),
    ffw_(tc, prefix + "_ffw"),
    sfw_(tc, prefix + "_sfw")
{
}

// read ports
template <>
bool FTSensorStatus_Ports<RTT::InputPort >::readPorts() {
    bool result = rw_.operation();
    result &= ffw_.operation();
    result &= sfw_.operation();
    return result;
}

// write ports
template <>
void FTSensorStatus_Ports<RTT::OutputPort >::writePorts() {
    rw_.operation();
    ffw_.operation();
    sfw_.operation();
}

template <>
void FTSensorStatus_Ports<RTT::OutputPort >::convertFromROS(const VelmaLowLevelStatusFT &ros) {
    rw_.convertFromROS(ros);
    ffw_.convertFromROS(ros);
    sfw_.convertFromROS(ros);
}

template <>
void FTSensorStatus_Ports<RTT::InputPort >::convertToROS(VelmaLowLevelStatusFT &ros) {
    rw_.convertToROS(ros);
    ffw_.convertToROS(ros);
    sfw_.convertToROS(ros);
}

//
// VelmaStatus_Ports interface
//
template <template <typename Type> class T>
VelmaStatus_Ports<T >::VelmaStatus_Ports(RTT::TaskContext &tc) :
    rArm_(tc, "status_rArm"),
    lArm_(tc, "status_lArm"),
    rHand_(tc, "status_rHand"),
    lHand_(tc, "status_lHand"),
    rFt_(tc, "status_rFt"),
    lFt_(tc, "status_lFt"),
// TODO
    tMotor_q_(tc, "status_tMotor_q"),
    tMotor_dq_(tc, "status_tMotor_dq"),
    hpMotor_q_(tc, "status_hpMotor_q"),
    hpMotor_dq_(tc, "status_hpMotor_dq"),
    htMotor_q_(tc, "status_htMotor_q"),
    htMotor_dq_(tc, "status_htMotor_dq"),
    rHand_p_(tc, "status_rHand_p"),
    lHand_f_(tc, "status_lHand_f"),
    test_(tc, "status_test"),
    rArm_valid_(false),
    lArm_valid_(false),
    rHand_valid_(false),
    lHand_valid_(false),
    rFt_valid_(false),
    lFt_valid_(false),
    tMotor_valid_(false),
    hpMotor_valid_(false),
    htMotor_valid_(false),
    tact_valid_(false),
    optoforce_valid_(false)
{
}

// read ports
template <>
void VelmaStatus_Ports<RTT::InputPort >::readPorts() {
    rArm_valid_ = rArm_.readPorts();
    lArm_valid_ = lArm_.readPorts();
    rHand_valid_ = rHand_.readPorts();
    lHand_valid_ = lHand_.readPorts();
    rFt_valid_ = rFt_.readPorts();
    lFt_valid_ = lFt_.readPorts();
    tMotor_valid_ = tMotor_q_.operation();
    tMotor_valid_ &= tMotor_dq_.operation();
    hpMotor_valid_ = hpMotor_q_.operation();
    hpMotor_valid_ &= hpMotor_dq_.operation();
    htMotor_valid_ = htMotor_q_.operation();
    htMotor_valid_ &= htMotor_dq_.operation();
    tact_valid_ = rHand_p_.operation();
    optoforce_valid_ = lHand_f_.operation();
    test_.operation();
}

// write ports
template <>
void VelmaStatus_Ports<RTT::OutputPort >::writePorts() {
    if (rArm_valid_) {
        rArm_.writePorts();
    }

    if (lArm_valid_) {
        lArm_.writePorts();
    }

    if (rHand_valid_) {
        rHand_.writePorts();
    }

    if (lHand_valid_) {
        lHand_.writePorts();
    }

    if (rFt_valid_) {
        rFt_.writePorts();
    }

    if (lFt_valid_) {
        lFt_.writePorts();
    }

    if (tMotor_valid_) {
        tMotor_q_.operation();
        tMotor_dq_.operation();
    }

    if (hpMotor_valid_) {
        hpMotor_q_.operation();
        hpMotor_dq_.operation();
    }

    if (htMotor_valid_) {
        htMotor_q_.operation();
        htMotor_dq_.operation();
    }

    if (tact_valid_) {
        rHand_p_.operation();
    }

    if (optoforce_valid_) {
        lHand_f_.operation();
    }

    test_.operation();
}

template <>
void VelmaStatus_Ports<RTT::OutputPort >::convertFromROS(const VelmaLowLevelStatus &ros) {
    rArm_.convertFromROS(ros.rArm);
    rArm_valid_ = ros.rArm_valid;

    lArm_.convertFromROS(ros.lArm);
    lArm_valid_ = ros.lArm_valid;

    rHand_.convertFromROS(ros.rHand);
    rHand_valid_ = ros.rHand_valid;

    lHand_.convertFromROS(ros.lHand);
    lHand_valid_ = ros.lHand_valid;

    rFt_.convertFromROS(ros.rFt);
    rFt_valid_ = ros.rFt_valid;

    lFt_.convertFromROS(ros.lFt);
    lFt_valid_ = ros.lFt_valid;

    tMotor_q_.convertFromROS(ros);
    tMotor_dq_.convertFromROS(ros);
    tMotor_valid_ = ros.tMotor_valid;

    hpMotor_q_.convertFromROS(ros);
    hpMotor_dq_.convertFromROS(ros);
    hpMotor_valid_ = ros.hpMotor_valid;

    htMotor_q_.convertFromROS(ros);
    htMotor_dq_.convertFromROS(ros);
    htMotor_valid_ = ros.htMotor_valid;

    rHand_p_.convertFromROS(ros);
    tact_valid_ = ros.tact_valid;

    lHand_f_.convertFromROS(ros);
    optoforce_valid_ = ros.optoforce_valid;

    test_.convertFromROS(ros);
}

template <>
void VelmaStatus_Ports<RTT::InputPort >::convertToROS(VelmaLowLevelStatus &ros) {
    rArm_.convertToROS(ros.rArm);
    ros.rArm_valid = rArm_valid_;

    lArm_.convertToROS(ros.lArm);
    ros.lArm_valid = lArm_valid_;

    rHand_.convertToROS(ros.rHand);
    ros.rHand_valid = rHand_valid_;

    lHand_.convertToROS(ros.lHand);
    ros.lHand_valid = lHand_valid_;

    rFt_.convertToROS(ros.rFt);
    ros.rFt_valid = rFt_valid_;

    lFt_.convertToROS(ros.lFt);
    ros.lFt_valid = lFt_valid_;

    tMotor_q_.convertToROS(ros);
    tMotor_dq_.convertToROS(ros);
    ros.tMotor_valid = tMotor_valid_;

    hpMotor_q_.convertToROS(ros);
    hpMotor_dq_.convertToROS(ros);
    ros.hpMotor_valid = hpMotor_valid_;

    htMotor_q_.convertToROS(ros);
    htMotor_dq_.convertToROS(ros);
    ros.htMotor_valid = htMotor_valid_;

    rHand_p_.convertToROS(ros);
    ros.tact_valid = tact_valid_;

    lHand_f_.convertToROS(ros);
    ros.optoforce_valid = optoforce_valid_;

    test_.convertToROS(ros);
}

};  // namespace velma_lli_types

//
// VelmaLLIStatusInput interface
//
VelmaLLIStatusInput::VelmaLLIStatusInput(RTT::TaskContext &tc) :
    ports_in_(tc)
{
}

void VelmaLLIStatusInput::readPorts(velma_low_level_interface_msgs::VelmaLowLevelStatus &status) {
    ports_in_.readPorts();
    ports_in_.convertToROS(status);
}

bool VelmaLLIStatusInput::isAllDataValid() const {
    return ports_in_.rArm_valid_ &&
        ports_in_.lArm_valid_ &&
        ports_in_.rHand_valid_ &&
        ports_in_.lHand_valid_ &&
        ports_in_.rFt_valid_ &&
        ports_in_.lFt_valid_ &&
        ports_in_.tMotor_valid_ &&
        ports_in_.hpMotor_valid_ &&
        ports_in_.htMotor_valid_ &&
        ports_in_.tact_valid_ &&
        ports_in_.optoforce_valid_;
}

//
// VelmaLLIStatusOutput interface
//
VelmaLLIStatusOutput::VelmaLLIStatusOutput(RTT::TaskContext &tc) :
    ports_out_(tc)
{
}

void VelmaLLIStatusOutput::writePorts(const velma_low_level_interface_msgs::VelmaLowLevelStatus &status) {
    ports_out_.convertFromROS(status);
    ports_out_.writePorts();
}

