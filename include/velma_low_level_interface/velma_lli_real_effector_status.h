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

#ifndef __VELMA_LLI_REAL_EFFECTOR_STATUS_H__
#define __VELMA_LLI_REAL_EFFECTOR_STATUS_H__

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
#include "velma_low_level_interface_msgs/VelmaRealEffectorStatus.h"
#include "velma_low_level_interface_msgs/VelmaLowLevelStatusArm.h"
#include "velma_low_level_interface_msgs/VelmaLowLevelStatusHand.h"
#include "velma_low_level_interface_msgs/VelmaLowLevelStatusFT.h"
#include "barrett_hand_controller_msgs/BHPressureState.h"

#include "eigen_conversions/eigen_msg.h"

#include "common_interfaces/interface_ports.h"

#include "velma_low_level_interface/velma_lli_port_data.h"
#include "velma_low_level_interface/velma_lli_real_effector_port_data.h"

using velma_low_level_interface_msgs::VelmaRealEffectorStatus;
using velma_low_level_interface_msgs::VelmaLowLevelStatusArm;
using velma_low_level_interface_msgs::VelmaLowLevelStatusHand;
using velma_low_level_interface_msgs::VelmaLowLevelStatusMotor;
using velma_low_level_interface_msgs::VelmaLowLevelStatusFT;
using velma_low_level_interface_msgs::VelmaLowLevelStatusSC;

using namespace interface_ports;

namespace velma_lli_types {

template <template <typename Type> class T>
class RE_ArmStatus_Ports : public PortsContainer<VelmaRealEffectorStatus, VelmaLowLevelStatusArm > {
public:
    RE_ArmStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusArm VelmaRealEffectorStatus::*ptr);
};

template <template <typename Type> class T>
class RE_HandStatus_Ports : public PortsContainer<VelmaRealEffectorStatus, VelmaLowLevelStatusHand > {
public:
    RE_HandStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusHand VelmaRealEffectorStatus::*ptr);
};

template <template <typename Type> class T>
class RE_MotorStatus_Ports : public PortsContainer<VelmaRealEffectorStatus, VelmaLowLevelStatusMotor > {
public:
    RE_MotorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusMotor VelmaRealEffectorStatus::*ptr);
};

template <template <typename Type> class T>
class RE_FTSensorStatus_Ports : public PortsContainer<VelmaRealEffectorStatus, VelmaLowLevelStatusFT > {
public:
    RE_FTSensorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusFT VelmaRealEffectorStatus::*ptr);
};

template <template <typename Type> class T>
class RE_VelmaStatus_Ports : public PortsContainerOuter<VelmaRealEffectorStatus > {
public:
    typedef VelmaRealEffectorStatus Container;
    RE_VelmaStatus_Ports(RTT::TaskContext &tc);
};

template class RE_VelmaStatus_Ports<RTT::InputPort >;
template class RE_VelmaStatus_Ports<RTT::OutputPort >;

};  // namespace velma_lli_types

#endif  // __VELMA_LLI_REAL_EFFECTOR_STATUS_H__

