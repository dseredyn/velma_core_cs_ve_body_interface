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
/*
//
// FTSensorCommand_Ports interface
//
template <template <typename Type> class T>
FTSensorCommand_Ports<T >::FTSensorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix) {
}
*/
//
// VelmaStatus_Ports interface
//
template <template <typename Type> class T>
VelmaStatus_Ports<T >::VelmaStatus_Ports(RTT::TaskContext &tc, VelmaLowLevelStatus &ros) :
    tMotor_q_(tc, "status_tMotor_q", ros),
    rArm_(tc, "status_rArm", ros.rArm),
    lArm_(tc, "status_lArm", ros.lArm),
    rHand_(tc, "status_rHand", ros.rHand),
    lHand_(tc, "status_lHand", ros.lHand)
/*    rHandTactile_cmd_(tc, "rHandTactile_cmd", ros),
    tMotor_i_(tc, "torsoMotorCurrentCommand", ros),
    hpMotor_i_(tc, "headPanMotorCurrentCommand", ros),
    htMotor_i_(tc, "headTiltMotorCurrentCommand", ros),
    hpMotor_q_(tc, "headPanMotorPositionCommand", ros),
    htMotor_q_(tc, "headTiltMotorPositionCommand", ros),
    hpMotor_dq_(tc, "headPanMotorVelocityCommand", ros),
    htMotor_dq_(tc, "headTiltMotorVelocityCommand", ros)
*/
{
}

// read ports
template <>
void VelmaStatus_Ports<RTT::InputPort >::readPorts() {
    tMotor_q_.operation();
    rArm_.readPorts();
    lArm_.readPorts();
    rHand_.readPorts();
    lHand_.readPorts();
/*    rHandTactile_cmd_.operation();
    tMotor_i_.operation();
    hpMotor_i_.operation();
    htMotor_i_.operation();
    hpMotor_q_.operation();
    htMotor_q_.operation();
    hpMotor_dq_.operation();
    htMotor_dq_.operation();
*/
}

// write ports
template <>
void VelmaStatus_Ports<RTT::OutputPort >::writePorts() {
    tMotor_q_.operation();
    rArm_.writePorts();
    lArm_.writePorts();
    rHand_.writePorts();
    lHand_.writePorts();
/*    rHandTactile_cmd_.operation();
    tMotor_i_.operation();
    hpMotor_i_.operation();
    htMotor_i_.operation();
    hpMotor_q_.operation();
    htMotor_q_.operation();
    hpMotor_dq_.operation();
    htMotor_dq_.operation();
*/
}

template <>
void VelmaStatus_Ports<RTT::OutputPort >::convertFromROS() {
    tMotor_q_.convertFromROS();
    rArm_.convertFromROS();
    lArm_.convertFromROS();
    rHand_.convertFromROS();
    lHand_.convertFromROS();
/*    rHandTactile_cmd_.convertFromROS();
    tMotor_i_.convertFromROS();
    hpMotor_i_.convertFromROS();
    htMotor_i_.convertFromROS();
    hpMotor_q_.convertFromROS();
    htMotor_q_.convertFromROS();
    hpMotor_dq_.convertFromROS();
    htMotor_dq_.convertFromROS();
*/
}

template <>
void VelmaStatus_Ports<RTT::InputPort >::convertToROS() {
    tMotor_q_.convertToROS();
    rArm_.convertToROS();
    lArm_.convertToROS();
    rHand_.convertToROS();
    lHand_.convertToROS();
/*    rHandTactile_cmd_.convertToROS();
    tMotor_i_.convertToROS();
    hpMotor_i_.convertToROS();
    htMotor_i_.convertToROS();
    hpMotor_q_.convertToROS();
    htMotor_q_.convertToROS();
    hpMotor_dq_.convertToROS();
    htMotor_dq_.convertToROS();
*/
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

