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

#ifndef __VELMA_LLI_REAL_EFFECTOR_PORT_DATA_H__
#define __VELMA_LLI_REAL_EFFECTOR_PORT_DATA_H__

#include <cstring>

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/os/TimeService.hpp"
#include "Eigen/Dense"

#include <std_msgs/Int32.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "common_interfaces/interface_port_data.h"

#include "velma_core_cs_ve_body_msgs/VelmaRealEffectorArmFriIntf.h"
#include "velma_core_cs_ve_body_msgs/VelmaRealEffectorArmFriRobot.h"

#include <kuka_lwr_fri/friComm.h>

namespace interface_ports {

using namespace velma_core_cs_ve_body_msgs;

template < >
class PortRawData<tFriIntfState, VelmaRealEffectorArmFriIntf > {
public:
    PortRawData() {
    }

    void convertFromROS(const VelmaRealEffectorArmFriIntf &ros) {
        data_.timestamp = ros.timestamp;
        data_.state = ros.state;
        data_.quality = ros.quality;
        data_.desiredMsrSampleTime = ros.desiredMsrSampleTime;
        data_.desiredCmdSampleTime = ros.desiredCmdSampleTime;
        data_.safetyLimits = ros.safetyLimits;
        data_.stat.answerRate = ros.answerRate;
        data_.stat.latency = ros.latency;
        data_.stat.jitter = ros.jitter;
        data_.stat.missRate = ros.missRate;
        data_.stat.missCounter = ros.missCounter;
    }

    void convertToROS(VelmaRealEffectorArmFriIntf &ros) {
        ros.timestamp = data_.timestamp;
        ros.state = data_.state;
        ros.quality = data_.quality;
        ros.desiredMsrSampleTime = data_.desiredMsrSampleTime;
        ros.desiredCmdSampleTime = data_.desiredCmdSampleTime;
        ros.safetyLimits = data_.safetyLimits;
        ros.answerRate = data_.stat.answerRate;
        ros.latency = data_.stat.latency;
        ros.jitter = data_.stat.jitter;
        ros.missRate = data_.stat.missRate;
        ros.missCounter = data_.stat.missCounter;
    }

    tFriIntfState data_;
};

template < >
class PortRawData<tFriRobotState, VelmaRealEffectorArmFriRobot > {
public:
    PortRawData() {
    }

    void convertFromROS(const VelmaRealEffectorArmFriRobot &ros) {
        data_.power = ros.power;
        data_.control = ros.control;
        data_.error = ros.error;
        data_.warning = ros.warning;
        for (int i = 0; i < ros.temperature.size(); ++i) {
            data_.temperature[i] = ros.temperature[i];
        }
    }

    void convertToROS(VelmaRealEffectorArmFriRobot &ros) {
        ros.power = data_.power;
        ros.control = data_.control;
        ros.error = data_.error;
        ros.warning = data_.warning;
        for (int i = 0; i < ros.temperature.size(); ++i) {
            data_.temperature[i] = ros.temperature[i];
        }
    }

    tFriRobotState data_;
};

};  // namespace interface_ports

#endif  // __VELMA_LLI_REAL_EFFECTOR_PORT_DATA_H__

