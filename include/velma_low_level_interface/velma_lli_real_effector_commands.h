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

#ifndef __VELMA_LLI_REAL_EFFECTOR_COMMANDS_H__
#define __VELMA_LLI_REAL_EFFECTOR_COMMANDS_H__

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
#include "velma_low_level_interface_msgs/VelmaRealEffectorCommand.h"
#include <barrett_hand_controller_msgs/BHPressureState.h>

#include "eigen_conversions/eigen_msg.h"

#include "common_interfaces/interface_ports.h"

#include "velma_low_level_interface/velma_lli_port_data.h"
#include "velma_low_level_interface/velma_lli_real_effector_port_data.h"

using velma_low_level_interface_msgs::VelmaRealEffectorCommand;
using velma_low_level_interface_msgs::VelmaLowLevelCommandArm;
using velma_low_level_interface_msgs::VelmaLowLevelCommandHand;
using velma_low_level_interface_msgs::VelmaLowLevelCommandSimple;
using velma_low_level_interface_msgs::VelmaLowLevelCommandMotor;

using namespace interface_ports;

namespace velma_lli_types {

template <template <typename Type> class T>
class RE_ArmCommand_Ports : public PortsContainer<VelmaRealEffectorCommand, VelmaLowLevelCommandArm > {
public:
    RE_ArmCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelCommandArm VelmaRealEffectorCommand::*ptr);
};

template <template <typename Type> class T>
class RE_HandCommand_Ports : public PortsContainer<VelmaRealEffectorCommand, VelmaLowLevelCommandHand > {
public:
    RE_HandCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelCommandHand VelmaRealEffectorCommand::*ptr);
};

template <template <typename Type> class T>
class RE_SimpleCommand_Ports : public PortsContainer<VelmaRealEffectorCommand, VelmaLowLevelCommandSimple > {
public:
    RE_SimpleCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelCommandSimple VelmaRealEffectorCommand::*ptr);
};

template <template <typename Type> class T>
class RE_MotorCommand_Ports : public PortsContainer<VelmaRealEffectorCommand, VelmaLowLevelCommandMotor > {
public:
    RE_MotorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelCommandMotor VelmaRealEffectorCommand::*ptr);
};


template <template <typename Type> class T>
class RE_FTSensorCommand_Ports {
public:
    RE_FTSensorCommand_Ports(RTT::TaskContext &tc, const std::string &prefix);
};

template <template <typename Type> class T>
class RE_VelmaCommand_Ports : public PortsContainerOuter<VelmaRealEffectorCommand > {
public:
    typedef VelmaRealEffectorCommand Container;
    RE_VelmaCommand_Ports(RTT::TaskContext &tc);
};

template class RE_VelmaCommand_Ports<RTT::InputPort >;
template class RE_VelmaCommand_Ports<RTT::OutputPort >;

};  // namespace velma_lli_types

#endif  // __VELMA_LLI_REAL_EFFECTOR_COMMANDS_H__

