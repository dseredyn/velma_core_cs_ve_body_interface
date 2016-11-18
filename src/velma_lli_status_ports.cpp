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

#include "velma_low_level_interface/velma_lli_status_ports.h"

using namespace interface_ports;
using velma_low_level_interface_msgs::VelmaLowLevelStatus;
using velma_low_level_interface_msgs::VelmaLowLevelStatusArm;

namespace velma_lli_types {

//
// ArmStatus_Ports interface
//
template <template <typename Type> class T >
ArmStatus_Ports<T >::ArmStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusArm VelmaLowLevelStatus::*ptr) :
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
HandStatus_Ports<T >::HandStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusHand VelmaLowLevelStatus::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusHand > >(new Port<T, Eigen::Matrix<double,8,1>, VelmaLowLevelStatusHand, VelmaLowLevelStatusHand::_q_type >(tc, prefix + "_q", &VelmaLowLevelStatusHand::q)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusHand > >(new Port<T, uint32_t, VelmaLowLevelStatusHand, VelmaLowLevelStatusHand::_s_type >(tc, prefix + "_s", &VelmaLowLevelStatusHand::s)));

}

//
// MotorStatus_Ports interface
//
template <template <typename Type> class T >
MotorStatus_Ports<T >::MotorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusMotor VelmaLowLevelStatus::*ptr) :
    PortsContainer(ptr)
{
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusMotor > >(new Port<T, double, VelmaLowLevelStatusMotor, VelmaLowLevelStatusMotor::_q_type >(tc, prefix + "_q", &VelmaLowLevelStatusMotor::q)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatusMotor > >(new Port<T, double, VelmaLowLevelStatusMotor, VelmaLowLevelStatusMotor::_dq_type >(tc, prefix + "_dq", &VelmaLowLevelStatusMotor::dq)));
}

//
// FTSensorStatus_Ports interface
//
template <template <typename Type> class T >
FTSensorStatus_Ports<T >::FTSensorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix, VelmaLowLevelStatusFT VelmaLowLevelStatus::*ptr) :
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
VelmaStatus_Ports<T >::VelmaStatus_Ports(RTT::TaskContext &tc)
{
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >(new Port<T, uint32_t, VelmaLowLevelStatus, VelmaLowLevelStatus::_test_type >(tc, "status_test", &VelmaLowLevelStatus::test)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >( new ArmStatus_Ports<T >(tc, "status_rArm", &VelmaLowLevelStatus::rArm) )).setName("rArm");
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >( new ArmStatus_Ports<T >(tc, "status_lArm", &VelmaLowLevelStatus::lArm) )).setName("lArm");
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >( new HandStatus_Ports<T > (tc, "status_rHand", &VelmaLowLevelStatus::rHand) )).setName("rHand");
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >( new HandStatus_Ports<T > (tc, "status_lHand", &VelmaLowLevelStatus::lHand) )).setName("lHand");
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >( new FTSensorStatus_Ports<T > (tc, "status_rFt", &VelmaLowLevelStatus::rFt) )).setName("rFt");
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >( new FTSensorStatus_Ports<T > (tc, "status_lFt", &VelmaLowLevelStatus::lFt) )).setName("lFt");
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >( new MotorStatus_Ports<T > (tc, "status_tMotor", &VelmaLowLevelStatus::tMotor) )).setName("tMotor");
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >( new MotorStatus_Ports<T > (tc, "status_hpMotor", &VelmaLowLevelStatus::hpMotor) )).setName("hpMotor");
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >( new MotorStatus_Ports<T > (tc, "status_htMotor", &VelmaLowLevelStatus::htMotor) )).setName("htMotor");
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >(new Port<T, barrett_hand_controller_msgs::BHPressureState, VelmaLowLevelStatus, VelmaLowLevelStatus::_rHand_p_type> (tc, "status_rHand_p", &VelmaLowLevelStatus::rHand_p)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >(new Port<T, VelmaLowLevelStatus::_lHand_f_type, VelmaLowLevelStatus, VelmaLowLevelStatus::_lHand_f_type> (tc, "status_lHand_f", &VelmaLowLevelStatus::lHand_f)));
    addPort(boost::shared_ptr<PortInterface<VelmaLowLevelStatus > >(new Port<T, VelmaLowLevelStatus::_sc_type, VelmaLowLevelStatus, VelmaLowLevelStatus::_sc_type> (tc, "status_sc", &VelmaLowLevelStatus::sc)));
}

};  // namespace velma_lli_types

