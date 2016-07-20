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

#include "velma_lli_command_ports.h"

using velma_low_level_interface_msgs::VelmaLowLevelCommand;
using velma_low_level_interface_msgs::VelmaLowLevelCommandArm;

namespace velma_lli_command_types {

template <template <typename Type> class T, typename innerT, typename rosC, typename rosT, rosT rosC::*ptr>
Port<T, innerT, rosC, rosT, ptr>::Port(RTT::TaskContext &tc, const std::string &port_name, rosC &container) :
    container_(container)
{
    tc.ports()->addPort(port_name + "_INPORT", port_);
}

template <template <typename Type> class T, typename innerT, typename rosC, typename rosT, rosT rosC::*ptr>
void Port<T, innerT, rosC, rosT, ptr>::convertFromROS() {
}

template <template <typename Type> class T, typename innerT, typename rosC, typename rosT, rosT rosC::*ptr>
void Port<T, innerT, rosC, rosT, ptr>::writePorts() {
    port_.write(data_);
}

template <template <typename Type> class T, typename innerT, typename rosC, typename rosT, rosT rosC::*ptr>
void Port<T, innerT, rosC, rosT, ptr>::readPorts() {
    port_.read(data_);
}

// specialized code
// arm
template < >
void Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_t_type, &VelmaLowLevelCommandArm::t>::convertToROS() {
    for (int i = 0; i < 7; ++i) {
        data_(i) = container_.t[i];
    }
}
template < >
void Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_t_type, &VelmaLowLevelCommandArm::t>::convertFromROS() {
    for (int i = 0; i < 7; ++i) {
        container_.t[i] = data_(i);
    }
}

template < >
void Port<RTT::InputPort, int32_t, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_cmd_type, &VelmaLowLevelCommandArm::cmd>::convertToROS() {
    data_ = container_.cmd;
}
template < >
void Port<RTT::OutputPort, int32_t, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_cmd_type, &VelmaLowLevelCommandArm::cmd>::convertFromROS() {
    container_.cmd = data_;
}

// hand
template < >
void Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_q_type, &VelmaLowLevelCommandHand::q>::convertToROS() {
    for (int i = 0; i < 4; ++i) {
        data_(i) = container_.q[i];
    }
}
template < >
void Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_q_type, &VelmaLowLevelCommandHand::q>::convertFromROS() {
    for (int i = 0; i < 4; ++i) {
        container_.q[i] = data_(i);
    }
}

template < >
void Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_dq_type, &VelmaLowLevelCommandHand::dq>::convertToROS() {
    for (int i = 0; i < 4; ++i) {
        data_(i) = container_.dq[i];
    }
}
template < >
void Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_dq_type, &VelmaLowLevelCommandHand::dq>::convertFromROS() {
    for (int i = 0; i < 4; ++i) {
        container_.dq[i] = data_(i);
    }
}

template < >
void Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_max_i_type, &VelmaLowLevelCommandHand::max_i>::convertToROS() {
    for (int i = 0; i < 4; ++i) {
        data_(i) = container_.max_i[i];
    }
}
template < >
void Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_max_i_type, &VelmaLowLevelCommandHand::max_i>::convertFromROS() {
    for (int i = 0; i < 4; ++i) {
        container_.max_i[i] = data_(i);
    }
}

template < >
void Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_max_p_type, &VelmaLowLevelCommandHand::max_p>::convertToROS() {
    for (int i = 0; i < 4; ++i) {
        data_(i) = container_.max_p[i];
    }
}
template < >
void Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_max_p_type, &VelmaLowLevelCommandHand::max_p>::convertFromROS() {
    for (int i = 0; i < 4; ++i) {
        container_.max_p[i] = data_(i);
    }
}

template < >
void Port<RTT::InputPort, bool, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_hold_type, &VelmaLowLevelCommandHand::hold>::convertToROS() {
    data_ = container_.hold;
}
template < >
void Port<RTT::OutputPort, bool, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_hold_type, &VelmaLowLevelCommandHand::hold>::convertFromROS() {
    container_.hold = data_;
}

// tactile
template < >
void Port<RTT::InputPort, int32_t, VelmaLowLevelCommand, VelmaLowLevelCommand::_rHandTactile_cmd_type, &VelmaLowLevelCommand::rHandTactile_cmd>::convertToROS() {
    data_ = container_.rHandTactile_cmd;
}
template < >
void Port<RTT::OutputPort, int32_t, VelmaLowLevelCommand, VelmaLowLevelCommand::_rHandTactile_cmd_type, &VelmaLowLevelCommand::rHandTactile_cmd>::convertFromROS() {
    container_.rHandTactile_cmd = data_;
}

// head and torso
template < >
void Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_tMotor_i_type, &VelmaLowLevelCommand::tMotor_i>::convertToROS() {
    data_ = container_.tMotor_i;
}
template < >
void Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_tMotor_i_type, &VelmaLowLevelCommand::tMotor_i>::convertFromROS() {
    container_.tMotor_i = data_;
}

template < >
void Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_i_type, &VelmaLowLevelCommand::hpMotor_i>::convertToROS() {
    data_ = container_.hpMotor_i;
}
template < >
void Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_i_type, &VelmaLowLevelCommand::hpMotor_i>::convertFromROS() {
    container_.hpMotor_i = data_;
}

template < >
void Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_i_type, &VelmaLowLevelCommand::htMotor_i>::convertToROS() {
    data_ = container_.htMotor_i;
}
template < >
void Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_i_type, &VelmaLowLevelCommand::htMotor_i>::convertFromROS() {
    container_.htMotor_i = data_;
}

template < >
void Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_q_type, &VelmaLowLevelCommand::hpMotor_q>::convertToROS() {
    data_ = container_.hpMotor_q;
}
template < >
void Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_q_type, &VelmaLowLevelCommand::hpMotor_q>::convertFromROS() {
    container_.hpMotor_q = data_;
}

template < >
void Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_q_type, &VelmaLowLevelCommand::htMotor_q>::convertToROS() {
    data_ = container_.htMotor_q;
}
template < >
void Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_q_type, &VelmaLowLevelCommand::htMotor_q>::convertFromROS() {
    container_.htMotor_q = data_;
}

template < >
void Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_dq_type, &VelmaLowLevelCommand::hpMotor_dq>::convertToROS() {
    data_ = container_.hpMotor_dq;
}
template < >
void Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_dq_type, &VelmaLowLevelCommand::hpMotor_dq>::convertFromROS() {
    container_.hpMotor_dq = data_;
}

template < >
void Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_dq_type, &VelmaLowLevelCommand::htMotor_dq>::convertToROS() {
    data_ = container_.htMotor_dq;
}
template < >
void Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_dq_type, &VelmaLowLevelCommand::htMotor_dq>::convertFromROS() {
    container_.htMotor_dq = data_;
}

template <template <typename Type> class T > ArmCommand_Ports<T >::ArmCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelCommandArm &ros) :
    JointTorque_(tc, prefix + "_JointTorqueCommand", ros),
    KRLcmd_(tc, prefix + "_KRLcmd", ros)
{}

// read ports
template <> void ArmCommand_Ports<RTT::InputPort >::readPorts() {
    JointTorque_.readPorts();
    KRLcmd_.readPorts();
}

// write ports
template <> void ArmCommand_Ports<RTT::OutputPort >::writePorts() {
    JointTorque_.writePorts();
    KRLcmd_.writePorts();
}


template <> void ArmCommand_Ports<RTT::OutputPort >::convertFromROS() {
    JointTorque_.convertFromROS();
    KRLcmd_.convertFromROS();
}

template <> void ArmCommand_Ports<RTT::InputPort >::convertToROS() {
    JointTorque_.convertToROS();
    KRLcmd_.convertToROS();
}





template <template <typename Type> class T > HandCommand_Ports<T>::HandCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelCommandHand &ros) :
    q_(tc, prefix + "_q", ros),
    dq_(tc, prefix + "_dq", ros),
    hand_max_i_(tc, prefix + "_max_i", ros),
    hand_max_p_(tc, prefix + "_max_p", ros),
    hand_hold_(tc, prefix + "_hold", ros)
{
}

// read ports
template <> void HandCommand_Ports<RTT::InputPort >::readPorts() {
    q_.readPorts();
    dq_.readPorts();
    hand_max_i_.readPorts();
    hand_max_p_.readPorts();
    hand_hold_.readPorts();
}

// write ports
template <> void HandCommand_Ports<RTT::OutputPort >::writePorts() {
    q_.writePorts();
    dq_.writePorts();
    hand_max_i_.writePorts();
    hand_max_p_.writePorts();
    hand_hold_.writePorts();
}

template <> void HandCommand_Ports<RTT::OutputPort >::convertFromROS() {
    q_.convertFromROS();
    dq_.convertFromROS();
    hand_max_i_.convertFromROS();
    hand_max_p_.convertFromROS();
    hand_hold_.convertFromROS();
}

template <> void HandCommand_Ports<RTT::InputPort >::convertToROS() {
    q_.convertToROS();
    dq_.convertToROS();
    hand_max_i_.convertToROS();
    hand_max_p_.convertToROS();
    hand_hold_.convertToROS();
}

// input port specialized constructor
template <> FTSensorCommand_Ports<RTT::InputPort>::FTSensorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix) {
}

// output port specialized constructor
template <> FTSensorCommand_Ports<RTT::OutputPort>::FTSensorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix) {
}

// input port specialized constructor
template <template <typename Type> class T> VelmaCommand_Ports<T>::VelmaCommand_Ports(RTT::TaskContext &tc, VelmaLowLevelCommand &ros) :
    rArm_(tc, "rArm", ros.rArm),
    lArm_(tc, "lArm", ros.lArm),
    rHand_(tc, "rHand", ros.rHand),
    lHand_(tc, "lHand", ros.lHand),
    rHandTactile_cmd_(tc, "rHandTactile_cmd", ros),
    tMotor_i_(tc, "torsoMotorCurrentCommand", ros),
    hpMotor_i_(tc, "headPanMotorCurrentCommand", ros),
    htMotor_i_(tc, "headTiltMotorCurrentCommand", ros),
    hpMotor_q_(tc, "headPanMotorPositionCommand", ros),
    htMotor_q_(tc, "headTiltMotorPositionCommand", ros),
    hpMotor_dq_(tc, "headPanMotorVelocityCommand", ros),
    htMotor_dq_(tc, "headTiltMotorVelocityCommand", ros)
{
}

// read ports
template <> void VelmaCommand_Ports<RTT::InputPort >::readPorts() {
    rArm_.readPorts();
    lArm_.readPorts();
    rHand_.readPorts();
    lHand_.readPorts();
    rHandTactile_cmd_.readPorts();
    tMotor_i_.readPorts();
    hpMotor_i_.readPorts();
    htMotor_i_.readPorts();
    hpMotor_q_.readPorts();
    htMotor_q_.readPorts();
    hpMotor_dq_.readPorts();
    htMotor_dq_.readPorts();
}

// write ports
template <> void VelmaCommand_Ports<RTT::OutputPort >::writePorts() {
    rArm_.writePorts();
    lArm_.writePorts();
    rHand_.writePorts();
    lHand_.writePorts();
    rHandTactile_cmd_.writePorts();
    tMotor_i_.writePorts();
    hpMotor_i_.writePorts();
    htMotor_i_.writePorts();
    hpMotor_q_.writePorts();
    htMotor_q_.writePorts();
    hpMotor_dq_.writePorts();
    htMotor_dq_.writePorts();
}

template <> void VelmaCommand_Ports<RTT::OutputPort >::convertFromROS() {
    rArm_.convertFromROS();
    lArm_.convertFromROS();
    rHand_.convertFromROS();
    lHand_.convertFromROS();
    rHandTactile_cmd_.convertFromROS();
    tMotor_i_.convertFromROS();
    hpMotor_i_.convertFromROS();
    htMotor_i_.convertFromROS();
    hpMotor_q_.convertFromROS();
    htMotor_q_.convertFromROS();
    hpMotor_dq_.convertFromROS();
    htMotor_dq_.convertFromROS();

}

template <> void VelmaCommand_Ports<RTT::InputPort >::convertToROS() {
    rArm_.convertToROS();
    lArm_.convertToROS();
    rHand_.convertToROS();
    lHand_.convertToROS();
    rHandTactile_cmd_.convertToROS();
    tMotor_i_.convertToROS();
    hpMotor_i_.convertToROS();
    htMotor_i_.convertToROS();
    hpMotor_q_.convertToROS();
    htMotor_q_.convertToROS();
    hpMotor_dq_.convertToROS();
    htMotor_dq_.convertToROS();
}

};

VelmaLLICommandInput::VelmaLLICommandInput(RTT::TaskContext &tc, VelmaLowLevelCommand &ros) :
    ports_in_(tc, ros)
{
}

void VelmaLLICommandInput::readPorts(velma_low_level_interface_msgs::VelmaLowLevelCommand &command) {
    ports_in_.readPorts();
    ports_in_.convertToROS();
}




VelmaLLICommandOutput::VelmaLLICommandOutput(RTT::TaskContext &tc, VelmaLowLevelCommand &ros) :
    ports_out_(tc, ros)
{
}

void VelmaLLICommandOutput::writePorts(const velma_low_level_interface_msgs::VelmaLowLevelCommand &command) {
    ports_out_.convertFromROS();
    ports_out_.writePorts();
}

