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

#include <std_msgs/Int32.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "velma_low_level_interface_msgs/VelmaLowLevelStatus.h"
#include "velma_low_level_interface_msgs/VelmaLowLevelStatusArm.h"
#include "velma_low_level_interface_msgs/VelmaLowLevelStatusHand.h"
#include "velma_low_level_interface_msgs/VelmaLowLevelStatusFT.h"
#include "barrett_hand_controller_msgs/BHPressureState.h"

#include <kuka_lwr_fri/friComm.h>

#include "eigen_conversions/eigen_msg.h"

#include "velma_lli_ports.h"

using velma_low_level_interface_msgs::VelmaLowLevelStatus;
using velma_low_level_interface_msgs::VelmaLowLevelStatusArm;
using velma_low_level_interface_msgs::VelmaLowLevelStatusHand;
using velma_low_level_interface_msgs::VelmaLowLevelStatusFT;

namespace velma_lli_types {

template <template <typename Type> class T>
class ArmStatus_Ports {
public:
    ArmStatus_Ports();
    ArmStatus_Ports(RTT::TaskContext &tc, const std::string &prefix);

    void readPorts();
    void writePorts();

    void convertFromROS(const VelmaLowLevelStatusArm &ros);
    void convertToROS(VelmaLowLevelStatusArm &ros);

    Port<T, Eigen::VectorXd, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_q_type, &VelmaLowLevelStatusArm::q> q_;
    Port<T, Eigen::VectorXd, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_dq_type, &VelmaLowLevelStatusArm::dq> dq_;
    Port<T, Eigen::VectorXd, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_t_type, &VelmaLowLevelStatusArm::t> t_;
    Port<T, Eigen::VectorXd, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_gt_type, &VelmaLowLevelStatusArm::gt> gt_;
    Port<T, geometry_msgs::Wrench, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_w_type, &VelmaLowLevelStatusArm::w> w_;
    Port<T, Eigen::Matrix<double, 7, 7>, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_mmx_type, &VelmaLowLevelStatusArm::mmx> mmx_;
    Port<T, tFriIntfState, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_friIntfState_type, &VelmaLowLevelStatusArm::friIntfState> friIntfState_;
    Port<T, tFriRobotState, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_friRobotState_type, &VelmaLowLevelStatusArm::friRobotState> friRobotState_;
};

template <template <typename Type> class T>
class HandStatus_Ports {
public:
    HandStatus_Ports();
    HandStatus_Ports(RTT::TaskContext &tc, const std::string &prefix);

    void readPorts();
    void writePorts();

    void convertFromROS(const VelmaLowLevelStatusHand &ros);
    void convertToROS(VelmaLowLevelStatusHand &ros);

    Port<T, Eigen::VectorXd, VelmaLowLevelStatusHand, VelmaLowLevelStatusHand::_q_type, &VelmaLowLevelStatusHand::q> q_;
    Port<T, uint32_t, VelmaLowLevelStatusHand, VelmaLowLevelStatusHand::_s_type, &VelmaLowLevelStatusHand::s> s_;
};

template <template <typename Type> class T>
class FTSensorStatus_Ports {
public:
    FTSensorStatus_Ports();
    FTSensorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix);

    void readPorts();
    void writePorts();

    void convertFromROS(const VelmaLowLevelStatusFT &ros);
    void convertToROS(VelmaLowLevelStatusFT &ros);

    Port<T, geometry_msgs::Wrench, VelmaLowLevelStatusFT, VelmaLowLevelStatusFT::_rw_type, &VelmaLowLevelStatusFT::rw> rw_;
    Port<T, geometry_msgs::Wrench, VelmaLowLevelStatusFT, VelmaLowLevelStatusFT::_ffw_type, &VelmaLowLevelStatusFT::ffw> ffw_;
    Port<T, geometry_msgs::Wrench, VelmaLowLevelStatusFT, VelmaLowLevelStatusFT::_sfw_type, &VelmaLowLevelStatusFT::sfw> sfw_;
};

template <template <typename Type> class T>
class VelmaStatus_Ports{
public:
    VelmaStatus_Ports();
    VelmaStatus_Ports(RTT::TaskContext &tc);

    void readPorts();
    void writePorts();

    void convertFromROS(const VelmaLowLevelStatus &ros);
    void convertToROS(VelmaLowLevelStatus &ros);

    Port<T, uint32_t, VelmaLowLevelStatus, VelmaLowLevelStatus::_test_type, &VelmaLowLevelStatus::test> test_;

    // right LWR
    ArmStatus_Ports<T > rArm_;

    // left LWR
    ArmStatus_Ports<T > lArm_;

    // right BarrettHand
    HandStatus_Ports<T > rHand_;

    // left BarrettHand
    HandStatus_Ports<T > lHand_;

    // right FT sensor
    FTSensorStatus_Ports<T > rFt_;

    // left FT sensor
    FTSensorStatus_Ports<T > lFt_;

    // torsoMotorPosition
    Port<T, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_tMotor_q_type, &VelmaLowLevelStatus::tMotor_q> tMotor_q_;

    // torsoMotorVelocity
    Port<T, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_tMotor_dq_type, &VelmaLowLevelStatus::tMotor_dq> tMotor_dq_;

    // headPanMotorPosition
    Port<T, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_hpMotor_q_type, &VelmaLowLevelStatus::hpMotor_q> hpMotor_q_;

    // headPanMotorVelocity
    Port<T, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_hpMotor_dq_type, &VelmaLowLevelStatus::hpMotor_dq> hpMotor_dq_;

    // headTiltMotorPosition
    Port<T, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_htMotor_q_type, &VelmaLowLevelStatus::htMotor_q> htMotor_q_;

    // headTiltMotorVelocity
    Port<T, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_htMotor_dq_type, &VelmaLowLevelStatus::htMotor_dq> htMotor_dq_;

    Port<T, barrett_hand_controller_msgs::BHPressureState, VelmaLowLevelStatus, VelmaLowLevelStatus::_rHand_p_type, &VelmaLowLevelStatus::rHand_p> rHand_p_;

    Port<T, VelmaLowLevelStatus::_lHand_f_type, VelmaLowLevelStatus, VelmaLowLevelStatus::_lHand_f_type, &VelmaLowLevelStatus::lHand_f> lHand_f_;
};

};  // namespace velma_lli_types

class VelmaLLIStatusInput {
public:
    VelmaLLIStatusInput(RTT::TaskContext &tc);

    void readPorts(velma_low_level_interface_msgs::VelmaLowLevelStatus &status);

protected:
    velma_lli_types::VelmaStatus_Ports<RTT::InputPort > ports_in_;
};

class VelmaLLIStatusOutput {
public:
    VelmaLLIStatusOutput(RTT::TaskContext &tc);

    void writePorts(const velma_low_level_interface_msgs::VelmaLowLevelStatus &status);

protected:
    velma_lli_types::VelmaStatus_Ports<RTT::OutputPort > ports_out_;
};

#endif  // VELMA_LLI_STATUS_PORTS_H_

