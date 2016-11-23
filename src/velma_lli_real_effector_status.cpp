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

#include "velma_core_cs_ve_body_interface/velma_lli_real_effector_status.h"

using namespace interface_ports;
using namespace velma_core_cs_ve_body_msgs;

namespace velma_lli_types {

//
// ArmStatus_Ports interface
//
template <template <typename Type> class T >
RE_ArmStatus_Ports<T >::RE_ArmStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusArm VelmaRealEffectorStatus::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusArm > >(new Port<T, Eigen::Matrix<double,7,1>, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_q_type >(tc, prefix + "_q", &VelmaLowLevelStatusArm::q)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusArm > >(new Port<T, Eigen::Matrix<double,7,1>, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_dq_type >(tc, prefix + "_dq", &VelmaLowLevelStatusArm::dq)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusArm > >(new Port<T, Eigen::Matrix<double,7,1>, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_t_type >(tc, prefix + "_t", &VelmaLowLevelStatusArm::t)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusArm > >(new Port<T, Eigen::Matrix<double,7,1>, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_gt_type >(tc, prefix + "_gt", &VelmaLowLevelStatusArm::gt)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusArm > >(new Port<T, geometry_msgs::Wrench, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_w_type >(tc, prefix + "_w", &VelmaLowLevelStatusArm::w)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusArm > >(new Port<T, Eigen::Matrix<double,7,7>, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_mmx_type >(tc, prefix + "_mmx", &VelmaLowLevelStatusArm::mmx)));
}

//
// HandStatus_Ports interface
//
template <template <typename Type> class T >
RE_HandStatus_Ports<T >::RE_HandStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusHand VelmaRealEffectorStatus::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusHand > >(new Port<T, Eigen::Matrix<double,8,1>, VelmaLowLevelStatusHand, VelmaLowLevelStatusHand::_q_type >(tc, prefix + "_q", &VelmaLowLevelStatusHand::q)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusHand > >(new Port<T, uint32_t, VelmaLowLevelStatusHand, VelmaLowLevelStatusHand::_s_type >(tc, prefix + "_s", &VelmaLowLevelStatusHand::s)));

}

//
// MotorStatus_Ports interface
//
template <template <typename Type> class T >
RE_MotorStatus_Ports<T >::RE_MotorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusMotor VelmaRealEffectorStatus::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusMotor > >(new Port<T, double, VelmaLowLevelStatusMotor, VelmaLowLevelStatusMotor::_q_type >(tc, prefix + "_q", &VelmaLowLevelStatusMotor::q)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusMotor > >(new Port<T, double, VelmaLowLevelStatusMotor, VelmaLowLevelStatusMotor::_dq_type >(tc, prefix + "_dq", &VelmaLowLevelStatusMotor::dq)));
}

//
// FTSensorStatus_Ports interface
//
template <template <typename Type> class T >
RE_FTSensorStatus_Ports<T >::RE_FTSensorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusFT VelmaRealEffectorStatus::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusFT > >(new Port<T, geometry_msgs::Wrench, VelmaLowLevelStatusFT, VelmaLowLevelStatusFT::_rw_type >(tc, prefix + "_rw", &VelmaLowLevelStatusFT::rw)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusFT > >(new Port<T, geometry_msgs::Wrench, VelmaLowLevelStatusFT, VelmaLowLevelStatusFT::_ffw_type >(tc, prefix + "_ffw", &VelmaLowLevelStatusFT::ffw)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusFT > >(new Port<T, geometry_msgs::Wrench, VelmaLowLevelStatusFT, VelmaLowLevelStatusFT::_sfw_type >(tc, prefix + "_sfw", &VelmaLowLevelStatusFT::sfw)));

}

//
// VelmaStatus_Ports interface
//
template <template <typename Type> class T>
RE_VelmaStatus_Ports<T >::RE_VelmaStatus_Ports(RTT::TaskContext &tc)
{
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >(new Port<T, uint32_t, VelmaRealEffectorStatus, VelmaRealEffectorStatus::_test_type >(tc, "status_test", &VelmaRealEffectorStatus::test)));
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >( new RE_ArmStatus_Ports<T >(tc, "status_rArm", &VelmaRealEffectorStatus::rArm) ), &VelmaRealEffectorStatus::rArm_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >( new RE_ArmStatus_Ports<T >(tc, "status_lArm", &VelmaRealEffectorStatus::lArm) ), &VelmaRealEffectorStatus::lArm_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >(new Port<T, tFriRobotState, VelmaRealEffectorStatus, VelmaRealEffectorStatus::_rArmFriRobot_type >(tc, "status_rArmFriRobot", &VelmaRealEffectorStatus::rArmFriRobot)));
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >(new Port<T, tFriIntfState, VelmaRealEffectorStatus, VelmaRealEffectorStatus::_rArmFriIntf_type >(tc, "status_rArmFriIntf", &VelmaRealEffectorStatus::rArmFriIntf)));
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >(new Port<T, tFriRobotState, VelmaRealEffectorStatus, VelmaRealEffectorStatus::_rArmFriRobot_type >(tc, "status_lArmFriRobot", &VelmaRealEffectorStatus::lArmFriRobot)));
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >(new Port<T, tFriIntfState, VelmaRealEffectorStatus, VelmaRealEffectorStatus::_rArmFriIntf_type >(tc, "status_lArmFriIntf", &VelmaRealEffectorStatus::lArmFriIntf)));
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >( new RE_HandStatus_Ports<T > (tc, "status_rHand", &VelmaRealEffectorStatus::rHand) ), &VelmaRealEffectorStatus::rHand_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >( new RE_HandStatus_Ports<T > (tc, "status_lHand", &VelmaRealEffectorStatus::lHand) ), &VelmaRealEffectorStatus::lHand_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >( new RE_FTSensorStatus_Ports<T > (tc, "status_rFt", &VelmaRealEffectorStatus::rFt) ), &VelmaRealEffectorStatus::rFt_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >( new RE_FTSensorStatus_Ports<T > (tc, "status_lFt", &VelmaRealEffectorStatus::lFt) ), &VelmaRealEffectorStatus::lFt_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >( new RE_MotorStatus_Ports<T > (tc, "status_tMotor", &VelmaRealEffectorStatus::tMotor) ), &VelmaRealEffectorStatus::tMotor_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >( new RE_MotorStatus_Ports<T > (tc, "status_hpMotor", &VelmaRealEffectorStatus::hpMotor) ), &VelmaRealEffectorStatus::hpMotor_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >( new RE_MotorStatus_Ports<T > (tc, "status_htMotor", &VelmaRealEffectorStatus::htMotor) ), &VelmaRealEffectorStatus::htMotor_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >(new Port<T, barrett_hand_controller_msgs::BHPressureState, VelmaRealEffectorStatus, VelmaRealEffectorStatus::_rHand_p_type> (tc, "status_rHand_p", &VelmaRealEffectorStatus::rHand_p)), &VelmaRealEffectorStatus::rHand_p_valid);
    addPort(boost::shared_ptr<PortInterface<VelmaRealEffectorStatus > >(new Port<T, VelmaRealEffectorStatus::_lHand_f_type, VelmaRealEffectorStatus, VelmaRealEffectorStatus::_lHand_f_type> (tc, "status_lHand_f", &VelmaRealEffectorStatus::lHand_f)), &VelmaRealEffectorStatus::lHand_f_valid);
}

};  // namespace velma_lli_types

