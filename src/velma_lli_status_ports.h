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

#ifndef VELMA_LLI_STATUS_PORTS_H_
#define VELMA_LLI_STATUS_PORTS_H_

#include <cstring>

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/os/TimeService.hpp"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "velma_low_level_interface_msgs/VelmaLowLevelStatus.h"
#include <barrett_hand_controller_msgs/BHPressureState.h>

#include <kuka_lwr_fri/friComm.h>

#include "eigen_conversions/eigen_msg.h"

namespace velma_lli_status_types {

typedef Eigen::Matrix<double, 7, 7> Matrix77d;

template<typename T>
using Data = T;

template <template <typename Type> class T>
class ArmStatus_Ports {
public:
    ArmStatus_Ports();
    ArmStatus_Ports(RTT::TaskContext &tc, const std::string &prefix);

    void readPorts(ArmStatus_Ports<Data > &data);
    void writePorts(const ArmStatus_Ports<Data > &data);

    T<tFriRobotState >              RobotState_;
    T<tFriIntfState >               FRIState_;
    T<Eigen::VectorXd >             JointPosition_;
    T<Eigen::VectorXd >             JointVelocity_;
    T<geometry_msgs::Wrench >       CartesianWrench_;
    T<Matrix77d >                   MassMatrix_;
    T<Eigen::VectorXd >             JointTorque_;
    T<Eigen::VectorXd >             GravityTorque_;
};

template <template <typename Type> class T>
class HandStatus_Ports {
public:
    HandStatus_Ports();
    HandStatus_Ports(RTT::TaskContext &tc, const std::string &prefix);

    void readPorts(HandStatus_Ports<Data > &data);
    void writePorts(const HandStatus_Ports<Data > &data);

    T<uint32_t >                    status_;
    T<Eigen::VectorXd >             q_;
    T<Eigen::VectorXd >             t_;
};

template <template <typename Type> class T>
class FTSensorStatus_Ports {
public:
    FTSensorStatus_Ports();
    FTSensorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix);

    void readPorts(FTSensorStatus_Ports<Data > &data);
    void writePorts(const FTSensorStatus_Ports<Data > &data);

    T<geometry_msgs::Wrench >       raw_wrench_;
    T<geometry_msgs::Wrench >       fast_filtered_wrench_;
    T<geometry_msgs::Wrench >       slow_filtered_wrench_;
};

template <template <typename Type> class T>
class VelmaStatus_Ports{
public:
    VelmaStatus_Ports();
    VelmaStatus_Ports(RTT::TaskContext &tc);

    void readPorts(VelmaStatus_Ports<Data > &data);
    void writePorts(const VelmaStatus_Ports<Data > &data);

    // right LWR
    ArmStatus_Ports<T > rArm_;

    // left LWR
    ArmStatus_Ports<T > lArm_;

    // right BarrettHand
    HandStatus_Ports<T > rHand_;

    // left BarrettHand
    HandStatus_Ports<T > lHand_;

    // BarrettHand tactile sensors
    T<barrett_hand_controller_msgs::BHPressureState >   tactile_;
    T<Eigen::Vector4d >                                 max_pressure_;

    // right F/T sensor
    FTSensorStatus_Ports<T > rFT_;

    // left F/T sensor
    FTSensorStatus_Ports<T > lFT_;

    // optoforce sensors on left BarrettHand
    T<geometry_msgs::WrenchStamped > force_[3];

    // torso
    T<double > t_MotorPosition_;
    T<double > t_MotorVelocity_;

    // head
    T<double > hp_q_;
    T<double > hp_v_;
    T<double > ht_q_;
    T<double > ht_v_;
};

};  // namespace velma_lli_status_types

class VelmaLLIStatusInput {
public:
    VelmaLLIStatusInput(RTT::TaskContext &tc);

    void readPorts(velma_low_level_interface_msgs::VelmaLowLevelStatus &status);

protected:
    velma_lli_status_types::VelmaStatus_Ports<velma_lli_status_types::Data > in_;
    velma_lli_status_types::VelmaStatus_Ports<RTT::InputPort > ports_in_;
};

class VelmaLLIStatusOutput {
public:
    VelmaLLIStatusOutput(RTT::TaskContext &tc);

    void writePorts(const velma_low_level_interface_msgs::VelmaLowLevelStatus &status);

protected:
    velma_lli_status_types::VelmaStatus_Ports<velma_lli_status_types::Data > out_;
    velma_lli_status_types::VelmaStatus_Ports<RTT::OutputPort > ports_out_;
};

#endif  // VELMA_LLI_STATUS_PORTS_H_

