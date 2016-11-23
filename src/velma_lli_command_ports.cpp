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

#include "velma_core_cs_ve_body_interface/velma_lli_command_ports.h"

using namespace velma_core_cs_ve_body_msgs;

namespace velma_lli_types {

//
// ArmCommand_Ports interface
//
template <template <typename Type> class T >
ArmCommand_Ports<T >::ArmCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelCommandArm VelmaLowLevelCommand::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommandArm > >(new Port<T, Eigen::Matrix<double,7,1>, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_t_type >(tc, prefix + "_t", &VelmaLowLevelCommandArm::t)));

}

//
// HandCommand_Ports interface
//
template <template <typename Type> class T >
HandCommand_Ports<T>::HandCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelCommandHand VelmaLowLevelCommand::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommandHand > >(new Port<T, Eigen::Matrix<double,4,1>, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_q_type >(tc, prefix + "_q", &VelmaLowLevelCommandHand::q)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommandHand > >(new Port<T, Eigen::Matrix<double,4,1>, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_dq_type >(tc, prefix + "_dq", &VelmaLowLevelCommandHand::dq)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommandHand > >(new Port<T, Eigen::Matrix<double,4,1>, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_max_i_type >(tc, prefix + "_max_i", &VelmaLowLevelCommandHand::max_i)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommandHand > >(new Port<T, Eigen::Matrix<double,4,1>, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_max_p_type >(tc, prefix + "_max_p", &VelmaLowLevelCommandHand::max_p)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommandHand > >(new Port<T, bool, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_hold_type >(tc, prefix + "_hold", &VelmaLowLevelCommandHand::hold)));

}

//
// SimpleCommand_Ports interface
//
template <template <typename Type> class T >
SimpleCommand_Ports<T>::SimpleCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelCommandSimple VelmaLowLevelCommand::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommandSimple > >(new Port<T, int32_t, VelmaLowLevelCommandSimple, VelmaLowLevelCommandSimple::_cmd_type>(tc, prefix + "_cmd", &VelmaLowLevelCommandSimple::cmd)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommandSimple > >(new Port<T, bool, VelmaLowLevelCommandSimple, VelmaLowLevelCommandSimple::_valid_type>(tc, prefix + "_valid", &VelmaLowLevelCommandSimple::valid)));
}

//
// MotorCommand_Ports interface
//
template <template <typename Type> class T >
MotorCommand_Ports<T >::MotorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelCommandMotor VelmaLowLevelCommand::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommandMotor > >(new Port<T, double, VelmaLowLevelCommandMotor, VelmaLowLevelCommandMotor::_i_type >(tc, prefix + "_i", &VelmaLowLevelCommandMotor::i)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommandMotor > >(new Port<T, double, VelmaLowLevelCommandMotor, VelmaLowLevelCommandMotor::_q_type >(tc, prefix + "_q", &VelmaLowLevelCommandMotor::q)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommandMotor > >(new Port<T, double, VelmaLowLevelCommandMotor, VelmaLowLevelCommandMotor::_dq_type >(tc, prefix + "_dq", &VelmaLowLevelCommandMotor::dq)));
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
VelmaCommand_Ports<T >::VelmaCommand_Ports(RTT::TaskContext &tc)
{
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommand > >(new Port<T, uint32_t, VelmaLowLevelCommand, VelmaLowLevelCommand::_test_type >(tc, "cmd_test", &VelmaLowLevelCommand::test)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommand > >(new ArmCommand_Ports<T >(tc, "cmd_rArm", &VelmaLowLevelCommand::rArm)), &VelmaLowLevelCommand::rArm_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommand > >(new ArmCommand_Ports<T >(tc, "cmd_lArm", &VelmaLowLevelCommand::lArm)), &VelmaLowLevelCommand::lArm_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommand > >(new HandCommand_Ports<T >(tc, "cmd_rHand", &VelmaLowLevelCommand::rHand)), &VelmaLowLevelCommand::rHand_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommand > >(new HandCommand_Ports<T >(tc, "cmd_lHand", &VelmaLowLevelCommand::lHand)), &VelmaLowLevelCommand::lHand_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommand > >(new SimpleCommand_Ports<T >(tc, "cmd_rTact", &VelmaLowLevelCommand::rTact)), &VelmaLowLevelCommand::rTact_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommand > >(new MotorCommand_Ports<T >(tc, "cmd_tMotor", &VelmaLowLevelCommand::tMotor)), &VelmaLowLevelCommand::tMotor_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommand > >(new MotorCommand_Ports<T >(tc, "cmd_hpMotor", &VelmaLowLevelCommand::hpMotor)), &VelmaLowLevelCommand::hpMotor_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommand > >(new MotorCommand_Ports<T >(tc, "cmd_htMotor", &VelmaLowLevelCommand::htMotor)), &VelmaLowLevelCommand::htMotor_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelCommand > >(new SimpleCommand_Ports<T >(tc, "cmd_safety_controller", &VelmaLowLevelCommand::sc)), &VelmaLowLevelCommand::sc_valid);

}

};

