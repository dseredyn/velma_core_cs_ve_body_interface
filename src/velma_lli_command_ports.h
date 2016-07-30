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

#ifndef VELMA_LLI_COMMAND_PORTS_H_
#define VELMA_LLI_COMMAND_PORTS_H_

#include <cstring>

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/os/TimeService.hpp"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <std_msgs/Int32.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "velma_low_level_interface_msgs/VelmaLowLevelCommand.h"
#include <barrett_hand_controller_msgs/BHPressureState.h>

#include <kuka_lwr_fri/friComm.h>

#include "eigen_conversions/eigen_msg.h"

#include "velma_lli_ports.h"

using velma_low_level_interface_msgs::VelmaLowLevelCommand;
using velma_low_level_interface_msgs::VelmaLowLevelCommandArm;
using velma_low_level_interface_msgs::VelmaLowLevelCommandHand;

namespace velma_lli_types {

template <template <typename Type> class T>
class ArmCommand_Ports {
public:
    ArmCommand_Ports();
    ArmCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelCommandArm &ros);

    void readPorts();
    void writePorts();

    void convertFromROS();
    void convertToROS();

    Port<T, Eigen::VectorXd, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_t_type, &VelmaLowLevelCommandArm::t> t_;
    Port<T, std_msgs::Int32, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_cmd_type, &VelmaLowLevelCommandArm::cmd> cmd_;
};

template <template <typename Type> class T>
class HandCommand_Ports {
public:
    HandCommand_Ports();
    HandCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelCommandHand &ros);

    void readPorts();
    void writePorts();

    void convertFromROS();
    void convertToROS();

    Port<T, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_q_type, &VelmaLowLevelCommandHand::q> q_;
    Port<T, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_dq_type, &VelmaLowLevelCommandHand::dq> dq_;
    Port<T, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_max_i_type, &VelmaLowLevelCommandHand::max_i> max_i_;
    Port<T, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_max_p_type, &VelmaLowLevelCommandHand::max_p> max_p_;
    Port<T, bool, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_hold_type, &VelmaLowLevelCommandHand::hold> hold_;
};

template <template <typename Type> class T>
class FTSensorCommand_Ports {
public:
    FTSensorCommand_Ports();
    FTSensorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix);

    void readPorts();
    void writePorts();
};

template <template <typename Type> class T>
class VelmaCommand_Ports{
public:
    VelmaCommand_Ports();
    VelmaCommand_Ports(RTT::TaskContext &tc, VelmaLowLevelCommand &ros);

    void readPorts();
    void writePorts();

    void convertFromROS();
    void convertToROS();

    Port<T, uint32_t, VelmaLowLevelCommand, VelmaLowLevelCommand::_test_type, &VelmaLowLevelCommand::test> test_;

    // right LWR
    ArmCommand_Ports<T > rArm_;

    // left LWR
    ArmCommand_Ports<T > lArm_;

    // right BarrettHand
    HandCommand_Ports<T > rHand_;

    // left BarrettHand
    HandCommand_Ports<T > lHand_;

    // BarrettHand tactile sensors
    Port<T, int32_t, VelmaLowLevelCommand, VelmaLowLevelCommand::_rHand_tactileCmd_type, &VelmaLowLevelCommand::rHand_tactileCmd> rHand_tactileCmd_;

    // torsoMotorCurrentCommand
    Port<T, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_tMotor_i_type, &VelmaLowLevelCommand::tMotor_i> tMotor_i_;

    // headPanMotorCurrentCommand
    Port<T, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_i_type, &VelmaLowLevelCommand::hpMotor_i> hpMotor_i_;

    // headTiltMotorCurrentCommand
    Port<T, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_i_type, &VelmaLowLevelCommand::htMotor_i> htMotor_i_;

    // headPanMotorPositionCommand
    Port<T, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_q_type, &VelmaLowLevelCommand::hpMotor_q> hpMotor_q_;

    // headTiltMotorPositionCommand
    Port<T, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_q_type, &VelmaLowLevelCommand::htMotor_q> htMotor_q_;

    // headPanMotorVelocityCommand
    Port<T, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_dq_type, &VelmaLowLevelCommand::hpMotor_dq> hpMotor_dq_;

    // headTiltMotorVelocityCommand
    Port<T, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_dq_type, &VelmaLowLevelCommand::htMotor_dq> htMotor_dq_;
};

};  // namespace velma_lli_types

class VelmaLLICommandInput {
public:
    VelmaLLICommandInput(RTT::TaskContext &tc, VelmaLowLevelCommand &ros);

    void readPorts(velma_low_level_interface_msgs::VelmaLowLevelCommand &command);

protected:
    velma_lli_types::VelmaCommand_Ports<RTT::InputPort > ports_in_;
};

class VelmaLLICommandOutput {
public:
    VelmaLLICommandOutput(RTT::TaskContext &tc, VelmaLowLevelCommand &ros);

    void writePorts(const velma_low_level_interface_msgs::VelmaLowLevelCommand &command);

protected:
    velma_lli_types::VelmaCommand_Ports<RTT::OutputPort > ports_out_;
};

#endif  // VELMA_LLI_COMMAND_PORTS_H_

