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

#include "velma_low_level_interface/velma_lli_command_ports.h"

using velma_low_level_interface_msgs::VelmaLowLevelCommand;
using velma_low_level_interface_msgs::VelmaLowLevelCommandArm;

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
// ArmCommand_Ports interface
//
template <template <typename Type> class T >
ArmCommand_Ports<T >::ArmCommand_Ports(RTT::TaskContext &tc, const std::string &prefix) :
    t_(tc, prefix + "_t"),
    cmd_(tc, prefix + "_cmd")
{}

// read ports
template <>
void ArmCommand_Ports<RTT::InputPort >::readPorts() {
    t_.operation();
    cmd_.operation();
}

// write ports
template <>
void ArmCommand_Ports<RTT::OutputPort >::writePorts() {
    t_.operation();
    cmd_.operation();
}


template <>
void ArmCommand_Ports<RTT::OutputPort >::convertFromROS(const VelmaLowLevelCommandArm &ros) {
    t_.convertFromROS(ros);
    cmd_.convertFromROS(ros);
}

template <>
void ArmCommand_Ports<RTT::InputPort >::convertToROS(VelmaLowLevelCommandArm &ros) {
    t_.convertToROS(ros);
    cmd_.convertToROS(ros);
}





//
// HandCommand_Ports interface
//
template <template <typename Type> class T >
HandCommand_Ports<T>::HandCommand_Ports(RTT::TaskContext &tc, const std::string &prefix) :
    q_(tc, prefix + "_q"),
    dq_(tc, prefix + "_dq"),
    max_i_(tc, prefix + "_max_i"),
    max_p_(tc, prefix + "_max_p"),
    hold_(tc, prefix + "_hold")
{
}

// read ports
template <>
void HandCommand_Ports<RTT::InputPort >::readPorts() {
    q_.operation();
    dq_.operation();
    max_i_.operation();
    max_p_.operation();
    hold_.operation();
}

// write ports
template <>
void HandCommand_Ports<RTT::OutputPort >::writePorts() {
    q_.operation();
    dq_.operation();
    max_i_.operation();
    max_p_.operation();
    hold_.operation();
}

template <>
void HandCommand_Ports<RTT::OutputPort >::convertFromROS(const VelmaLowLevelCommandHand &ros) {
    q_.convertFromROS(ros);
    dq_.convertFromROS(ros);
    max_i_.convertFromROS(ros);
    max_p_.convertFromROS(ros);
    hold_.convertFromROS(ros);
}

template <>
void HandCommand_Ports<RTT::InputPort >::convertToROS(VelmaLowLevelCommandHand &ros) {
    q_.convertToROS(ros);
    dq_.convertToROS(ros);
    max_i_.convertToROS(ros);
    max_p_.convertToROS(ros);
    hold_.convertToROS(ros);
}

//
// FTSensorCommand_Ports interface
//
template <template <typename Type> class T>
FTSensorCommand_Ports<T >::FTSensorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix) {
}

//
// VelmaCommand_Ports interface
//
template <template <typename Type> class T>
VelmaCommand_Ports<T >::VelmaCommand_Ports(RTT::TaskContext &tc) :
    rArm_(tc, "cmd_rArm"),
    lArm_(tc, "cmd_lArm"),
    rHand_(tc, "cmd_rHand"),
    lHand_(tc, "cmd_lHand"),
    rHand_tactileCmd_(tc, "cmd_rHand_tactileCmd"),
    tMotor_i_(tc, "cmd_tMotor_i"),
    hpMotor_i_(tc, "cmd_hpMotor_i"),
    htMotor_i_(tc, "cmd_htMotor_i"),
    hpMotor_q_(tc, "cmd_hpMotor_q"),
    htMotor_q_(tc, "cmd_htMotor_q"),
    hpMotor_dq_(tc, "cmd_hpMotor_dq"),
    htMotor_dq_(tc, "cmd_htMotor_dq"),
    test_(tc, "cmd_test")
{
}

// read ports
template <>
void VelmaCommand_Ports<RTT::InputPort >::readPorts() {
    rArm_.readPorts();
    lArm_.readPorts();
    rHand_.readPorts();
    lHand_.readPorts();
    rHand_tactileCmd_.operation();
    tMotor_i_.operation();
    hpMotor_i_.operation();
    htMotor_i_.operation();
    hpMotor_q_.operation();
    htMotor_q_.operation();
    hpMotor_dq_.operation();
    htMotor_dq_.operation();
    test_.operation();
}

// write ports
template <>
void VelmaCommand_Ports<RTT::OutputPort >::writePorts() {
    rArm_.writePorts();
    lArm_.writePorts();
    rHand_.writePorts();
    lHand_.writePorts();
    rHand_tactileCmd_.operation();
    tMotor_i_.operation();
    hpMotor_i_.operation();
    htMotor_i_.operation();
    hpMotor_q_.operation();
    htMotor_q_.operation();
    hpMotor_dq_.operation();
    htMotor_dq_.operation();
    test_.operation();
}

template <>
void VelmaCommand_Ports<RTT::OutputPort >::convertFromROS(const VelmaLowLevelCommand &ros) {
    rArm_.convertFromROS(ros.rArm);
    lArm_.convertFromROS(ros.lArm);
    rHand_.convertFromROS(ros.rHand);
    lHand_.convertFromROS(ros.lHand);
    rHand_tactileCmd_.convertFromROS(ros);
    tMotor_i_.convertFromROS(ros);
    hpMotor_i_.convertFromROS(ros);
    htMotor_i_.convertFromROS(ros);
    hpMotor_q_.convertFromROS(ros);
    htMotor_q_.convertFromROS(ros);
    hpMotor_dq_.convertFromROS(ros);
    htMotor_dq_.convertFromROS(ros);
    test_.convertFromROS(ros);

}

template <>
void VelmaCommand_Ports<RTT::InputPort >::convertToROS(VelmaLowLevelCommand &ros) {
    rArm_.convertToROS(ros.rArm);
    lArm_.convertToROS(ros.lArm);
    rHand_.convertToROS(ros.rHand);
    lHand_.convertToROS(ros.lHand);
    rHand_tactileCmd_.convertToROS(ros);
    tMotor_i_.convertToROS(ros);
    hpMotor_i_.convertToROS(ros);
    htMotor_i_.convertToROS(ros);
    hpMotor_q_.convertToROS(ros);
    htMotor_q_.convertToROS(ros);
    hpMotor_dq_.convertToROS(ros);
    htMotor_dq_.convertToROS(ros);
    test_.convertToROS(ros);
}

};

//
// VelmaLLICommandInput interface
//
VelmaLLICommandInput::VelmaLLICommandInput(RTT::TaskContext &tc) :
    ports_in_(tc)
{
}

void VelmaLLICommandInput::readPorts(velma_low_level_interface_msgs::VelmaLowLevelCommand &command) {
    ports_in_.readPorts();
    ports_in_.convertToROS(command);
}



//
// VelmaLLICommandOutput interface
//
VelmaLLICommandOutput::VelmaLLICommandOutput(RTT::TaskContext &tc) :
    ports_out_(tc)
{
}

void VelmaLLICommandOutput::writePorts(const velma_low_level_interface_msgs::VelmaLowLevelCommand &command) {
    ports_out_.convertFromROS(command);
    ports_out_.writePorts();
}

