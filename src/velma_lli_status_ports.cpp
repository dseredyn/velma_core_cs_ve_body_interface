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
    tMotor_q_(tc, "status_tMotor_q"),
    tMotor_dq_(tc, "status_tMotor_dq"),
    hpMotor_q_(tc, "status_hpMotor_q"),
    hpMotor_dq_(tc, "status_hpMotor_dq"),
    htMotor_q_(tc, "status_htMotor_q"),
    htMotor_dq_(tc, "status_htMotor_dq"),
    rHand_p_(tc, "status_rHand_p"),
    lHand_f_(tc, "status_lHand_f"),
    test_(tc, "status_test")
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
    test_.operation();
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
    test_.operation();
}

template <>
void VelmaStatus_Ports<RTT::OutputPort >::convertFromROS(const VelmaLowLevelStatus &ros) {
    rArm_.convertFromROS(ros.rArm);
    lArm_.convertFromROS(ros.lArm);
    rHand_.convertFromROS(ros.rHand);
    lHand_.convertFromROS(ros.lHand);
    rFt_.convertFromROS(ros.rFt);
    lFt_.convertFromROS(ros.lFt);
    tMotor_q_.convertFromROS(ros);
    tMotor_dq_.convertFromROS(ros);
    hpMotor_q_.convertFromROS(ros);
    hpMotor_dq_.convertFromROS(ros);
    htMotor_q_.convertFromROS(ros);
    htMotor_dq_.convertFromROS(ros);
    rHand_p_.convertFromROS(ros);
    lHand_f_.convertFromROS(ros);
    test_.convertFromROS(ros);
}

template <>
void VelmaStatus_Ports<RTT::InputPort >::convertToROS(VelmaLowLevelStatus &ros) {
    rArm_.convertToROS(ros.rArm);
    lArm_.convertToROS(ros.lArm);
    rHand_.convertToROS(ros.rHand);
    lHand_.convertToROS(ros.lHand);
    rFt_.convertToROS(ros.rFt);
    lFt_.convertToROS(ros.lFt);
    tMotor_q_.convertToROS(ros);
    tMotor_dq_.convertToROS(ros);
    hpMotor_q_.convertToROS(ros);
    hpMotor_dq_.convertToROS(ros);
    htMotor_q_.convertToROS(ros);
    htMotor_dq_.convertToROS(ros);
    rHand_p_.convertToROS(ros);
    lHand_f_.convertToROS(ros);
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

