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

#include "velma_lli_status_ports.h"

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
ArmStatus_Ports<T >::ArmStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusArm &ros) :
    q_(tc, prefix + "_q", ros),
    dq_(tc, prefix + "_dq", ros),
    t_(tc, prefix + "_t", ros),
    gt_(tc, prefix + "_gt", ros),
    w_(tc, prefix + "_w", ros),
    mmx_(tc, prefix + "_mmx", ros),
    friIntfState_(tc, prefix + "_friIntfState", ros),
    friRobotState_(tc, prefix + "_friRobotState", ros)
{}

// read ports
template <>
void ArmStatus_Ports<RTT::InputPort >::readPorts() {
    q_.operation();
    dq_.operation();
    t_.operation();
    gt_.operation();
    w_.operation();
    mmx_.operation();
    friIntfState_.operation();
    friRobotState_.operation();
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
void ArmStatus_Ports<RTT::OutputPort >::convertFromROS() {
    q_.convertFromROS();
    dq_.convertFromROS();
    t_.convertFromROS();
    gt_.convertFromROS();
    w_.convertFromROS();
    mmx_.convertFromROS();
    friIntfState_.convertFromROS();
    friRobotState_.convertFromROS();
}

template <>
void ArmStatus_Ports<RTT::InputPort >::convertToROS() {
    q_.convertToROS();
    dq_.convertToROS();
    t_.convertToROS();
    gt_.convertToROS();
    w_.convertToROS();
    mmx_.convertToROS();
    friIntfState_.convertToROS();
    friRobotState_.convertToROS();
}





//
// HandStatus_Ports interface
//
template <template <typename Type> class T >
HandStatus_Ports<T>::HandStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusHand &ros) :
    q_(tc, prefix + "_q", ros),
    s_(tc, prefix + "_s", ros)
{
}

// read ports
template <>
void HandStatus_Ports<RTT::InputPort >::readPorts() {
    q_.operation();
    s_.operation();
}

// write ports
template <>
void HandStatus_Ports<RTT::OutputPort >::writePorts() {
    q_.operation();
    s_.operation();
}

template <>
void HandStatus_Ports<RTT::OutputPort >::convertFromROS() {
    q_.convertFromROS();
    s_.convertFromROS();
}

template <>
void HandStatus_Ports<RTT::InputPort >::convertToROS() {
    q_.convertToROS();
    s_.convertToROS();
}

//
// FTSensorStatus_Ports interface
//
template <template <typename Type> class T>
FTSensorStatus_Ports<T >::FTSensorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusFT &ros) :
    rw_(tc, prefix + "_rw", ros),
    ffw_(tc, prefix + "_ffw", ros),
    sfw_(tc, prefix + "_sfw", ros)
{
}

// read ports
template <>
void FTSensorStatus_Ports<RTT::InputPort >::readPorts() {
    rw_.operation();
    ffw_.operation();
    sfw_.operation();
}

// write ports
template <>
void FTSensorStatus_Ports<RTT::OutputPort >::writePorts() {
    rw_.operation();
    ffw_.operation();
    sfw_.operation();
}

template <>
void FTSensorStatus_Ports<RTT::OutputPort >::convertFromROS() {
    rw_.convertFromROS();
    ffw_.convertFromROS();
    sfw_.convertFromROS();
}

template <>
void FTSensorStatus_Ports<RTT::InputPort >::convertToROS() {
    rw_.convertToROS();
    ffw_.convertToROS();
    sfw_.convertToROS();
}

//
// VelmaStatus_Ports interface
//
template <template <typename Type> class T>
VelmaStatus_Ports<T >::VelmaStatus_Ports(RTT::TaskContext &tc, VelmaLowLevelStatus &ros) :
    rArm_(tc, "status_rArm", ros.rArm),
    lArm_(tc, "status_lArm", ros.lArm),
    rHand_(tc, "status_rHand", ros.rHand),
    lHand_(tc, "status_lHand", ros.lHand),
    rFt_(tc, "status_rFt", ros.rFt),
    lFt_(tc, "status_lFt", ros.lFt),
    tMotor_q_(tc, "status_tMotor_q", ros),
    tMotor_dq_(tc, "status_tMotor_dq", ros),
    hpMotor_q_(tc, "status_hpMotor_q", ros),
    hpMotor_dq_(tc, "status_hpMotor_dq", ros),
    htMotor_q_(tc, "status_htMotor_q", ros),
    htMotor_dq_(tc, "status_htMotor_dq", ros),
    rHand_p_(tc, "status_rHand_p", ros),
    lHand_f_(tc, "status_lHand_f", ros)
{
}

// read ports
template <>
void VelmaStatus_Ports<RTT::InputPort >::readPorts() {
    rArm_.readPorts();
    lArm_.readPorts();
    rHand_.readPorts();
    lHand_.readPorts();
    rFt_.readPorts();
    lFt_.readPorts();
    tMotor_q_.operation();
    tMotor_dq_.operation();
    hpMotor_q_.operation();
    hpMotor_dq_.operation();
    htMotor_q_.operation();
    htMotor_dq_.operation();
    rHand_p_.operation();
    lHand_f_.operation();
}

// write ports
template <>
void VelmaStatus_Ports<RTT::OutputPort >::writePorts() {
    rArm_.writePorts();
    lArm_.writePorts();
    rHand_.writePorts();
    lHand_.writePorts();
    rFt_.writePorts();
    lFt_.writePorts();
    tMotor_q_.operation();
    tMotor_dq_.operation();
    hpMotor_q_.operation();
    hpMotor_dq_.operation();
    htMotor_q_.operation();
    htMotor_dq_.operation();
    rHand_p_.operation();
    lHand_f_.operation();
}

template <>
void VelmaStatus_Ports<RTT::OutputPort >::convertFromROS() {
    rArm_.convertFromROS();
    lArm_.convertFromROS();
    rHand_.convertFromROS();
    lHand_.convertFromROS();
    rFt_.convertFromROS();
    lFt_.convertFromROS();
    tMotor_q_.convertFromROS();
    tMotor_dq_.convertFromROS();
    hpMotor_q_.convertFromROS();
    hpMotor_dq_.convertFromROS();
    htMotor_q_.convertFromROS();
    htMotor_dq_.convertFromROS();
    rHand_p_.convertFromROS();
    lHand_f_.convertFromROS();
}

template <>
void VelmaStatus_Ports<RTT::InputPort >::convertToROS() {
    rArm_.convertToROS();
    lArm_.convertToROS();
    rHand_.convertToROS();
    lHand_.convertToROS();
    rFt_.convertToROS();
    lFt_.convertToROS();
    tMotor_q_.convertToROS();
    tMotor_dq_.convertToROS();
    hpMotor_q_.convertToROS();
    hpMotor_dq_.convertToROS();
    htMotor_q_.convertToROS();
    htMotor_dq_.convertToROS();
    rHand_p_.convertToROS();
    lHand_f_.convertToROS();
}

};

//
// VelmaLLIStatusInput interface
//
VelmaLLIStatusInput::VelmaLLIStatusInput(RTT::TaskContext &tc, VelmaLowLevelStatus &ros) :
    ports_in_(tc, ros)
{
}

void VelmaLLIStatusInput::readPorts(velma_low_level_interface_msgs::VelmaLowLevelStatus &status) {
    ports_in_.readPorts();
    ports_in_.convertToROS();
}



//
// VelmaLLIStatusOutput interface
//
VelmaLLIStatusOutput::VelmaLLIStatusOutput(RTT::TaskContext &tc, VelmaLowLevelStatus &ros) :
    ports_out_(tc, ros)
{
}

void VelmaLLIStatusOutput::writePorts(const velma_low_level_interface_msgs::VelmaLowLevelStatus &status) {
    ports_out_.convertFromROS();
    ports_out_.writePorts();
}

