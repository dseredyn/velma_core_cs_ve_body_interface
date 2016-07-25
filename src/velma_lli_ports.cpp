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

#include "velma_lli_ports.h"
#include "velma_low_level_interface_msgs/VelmaLowLevelCommand.h"
#include "velma_low_level_interface_msgs/VelmaLowLevelStatus.h"

namespace velma_lli_types {

//
// data initialization
//
// general constructor
template <typename innerT, typename rosT >
PortRawData<innerT, rosT >::PortRawData() {}

// specialized constructors
template < >
PortRawData<Eigen::VectorXd, boost::array<double, 7ul> >::PortRawData() : data_(7) {}

template < >
PortRawData<Eigen::VectorXd, boost::array<double, 4ul> >::PortRawData() : data_(4) {}

template < >
PortRawData<Eigen::VectorXi, boost::array<int32_t, 4ul> >::PortRawData() : data_(4) {}

template < >
PortRawData<Eigen::VectorXi, boost::array<uint8_t, 40ul> >::PortRawData() : data_(40) {}

template < >
PortRawData<Eigen::VectorXi, boost::array<uint8_t, 36ul> >::PortRawData() : data_(36) {}

template < >
PortRawData<Eigen::VectorXd, boost::array<double, 28ul> >::PortRawData() : data_(28) {}

//
// data conversion
//
// general conversion
template <typename innerT, typename rosT >
void PortRawData<innerT, rosT >::convertFromROS(const rosT &ros) {
    data_ = ros;
}

template <typename innerT, typename rosT >
void PortRawData<innerT, rosT >::convertToROS(rosT &ros) {
    ros = data_;
}

// specialized conversions
template <>
void PortRawData<Eigen::VectorXd, boost::array<double, 7ul> >::convertFromROS(const boost::array<double, 7ul> &ros) {
    for (int i = 0; i < 7; ++i) {
        data_(i) = ros[i];
    }
}

template <>
void PortRawData<Eigen::VectorXd, boost::array<double, 7ul> >::convertToROS(boost::array<double, 7ul> &ros) {
    for (int i = 0; i < 7; ++i) {
        ros[i] = data_(i);
    }
}

template <>
void PortRawData<Eigen::VectorXd, boost::array<double, 4ul> >::convertFromROS(const boost::array<double, 4ul> &ros) {
    for (int i = 0; i < 4; ++i) {
        data_(i) = ros[i];
    }
}

template <>
void PortRawData<Eigen::VectorXd, boost::array<double, 4ul> >::convertToROS(boost::array<double, 4ul> &ros) {
    for (int i = 0; i < 4; ++i) {
        ros[i] = data_(i);
    }
}

template <>
void PortRawData<Eigen::VectorXi, boost::array<int32_t, 4ul> >::convertFromROS(const boost::array<int32_t, 4ul> &ros) {
    for (int i = 0; i < 4; ++i) {
        data_(i) = ros[i];
    }
}

template <>
void PortRawData<Eigen::VectorXi, boost::array<int32_t, 4ul> >::convertToROS(boost::array<int32_t, 4ul> &ros) {
    for (int i = 0; i < 4; ++i) {
        ros[i] = data_(i);
    }
}

template <>
void PortRawData<Eigen::VectorXi, boost::array<uint8_t, 40ul> >::convertFromROS(const boost::array<uint8_t, 40ul> &ros) {
    for (int i = 0; i < 40; ++i) {
        data_(i) = ros[i];
    }
}

template <>
void PortRawData<Eigen::VectorXi, boost::array<uint8_t, 40ul> >::convertToROS(boost::array<uint8_t, 40ul> &ros) {
    for (int i = 0; i < 40; ++i) {
        ros[i] = data_(i);
    }
}

template <>
void PortRawData<Eigen::VectorXi, boost::array<uint8_t, 36ul> >::convertFromROS(const boost::array<uint8_t, 36ul> &ros) {
    for (int i = 0; i < 36; ++i) {
        data_(i) = ros[i];
    }
}

template <>
void PortRawData<Eigen::VectorXi, boost::array<uint8_t, 36ul> >::convertToROS(boost::array<uint8_t, 36ul> &ros) {
    for (int i = 0; i < 36; ++i) {
        ros[i] = data_(i);
    }
}

// 7x7 mass matrix
template <>
void PortRawData<Eigen::Matrix<double, 7, 7>, boost::array<double, 28ul> >::convertFromROS(const boost::array<double, 28ul> &ros) {
    for (int i = 0, idx = 0; i < 7; ++i) {
        for (int j = i; j < 7; ++j) {
            data_(i,j) = data_(j,i) = ros[idx++];
        }
    }
}

template <>
void PortRawData<Eigen::Matrix<double, 7, 7>, boost::array<double, 28ul> >::convertToROS(boost::array<double, 28ul> &ros) {
    for (int i = 0, idx = 0; i < 7; ++i) {
        for (int j = i; j < 7; ++j) {
            ros[idx++] = data_(i,j);
        }
    }
}

//
// PortData operations
//
template < > PortSuffix<RTT::InputPort >::PortSuffix() : str_("INPORT") {}
template < > PortSuffix<RTT::OutputPort >::PortSuffix() : str_("OUTPORT") {}

template <typename innerT, typename rosC, typename rosT, rosT rosC::*ptr >
PortData<innerT, rosC, rosT, ptr>::PortData(rosC &container) :
    container_(container)
{}

template <typename innerT, typename rosC, typename rosT, rosT rosC::*ptr >
void PortData<innerT, rosC, rosT, ptr >::convertFromROS() {
    data_.convertFromROS(container_.*ptr);
}

template <typename innerT, typename rosC, typename rosT, rosT rosC::*ptr >
void PortData<innerT, rosC, rosT, ptr >::convertToROS() {
    data_.convertToROS(container_.*ptr);
}

template <typename innerT, typename rosC, typename rosT, rosT rosC::*ptr >
innerT& PortData<innerT, rosC, rosT, ptr >::getDataRef() {
    return data_.data_;
}

// example of specialized code
// arm
/*
template < >
void PortData<Eigen::VectorXd, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_t_type, &VelmaLowLevelCommandArm::t>::convertToROS() {
    for (int i = 0; i < 7; ++i) {
        getDataRef()(i) = container_.t[i];
    }
}
template < >
void PortData<Eigen::VectorXd, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_t_type, &VelmaLowLevelCommandArm::t>::convertFromROS() {
    for (int i = 0; i < 7; ++i) {
        container_.t[i] = getDataRef()(i);
    }
}
*/

//
// Port operations
//
template <template <typename Type> class T, typename innerT, typename rosC, typename rosT, rosT rosC::*ptr >
void Port<T, innerT, rosC, rosT, ptr >::convertFromROS() {
    data_.convertFromROS();
}

template <template <typename Type> class T, typename innerT, typename rosC, typename rosT, rosT rosC::*ptr >
void Port<T, innerT, rosC, rosT, ptr >::convertToROS() {
    data_.convertToROS();
}

template <template <typename Type> class T, typename innerT, typename rosC, typename rosT, rosT rosC::*ptr>
Port<T, innerT, rosC, rosT, ptr>::Port(RTT::TaskContext &tc, const std::string &port_name, rosC &container) :
    container_(container),
    data_(container),
    po_(tc, port_name)
{
    po_.setDataSample(data_.getDataRef());
}


template <template <typename Type> class T, typename innerT, typename rosC, typename rosT, rosT rosC::*ptr>
void Port<T, innerT, rosC, rosT, ptr>::operation() {
    po_.operation(data_.getDataRef());
}

template <typename innerT >
PortOperation<RTT::InputPort, innerT>::PortOperation(RTT::TaskContext &tc, const std::string &port_name) {
    tc.ports()->addPort(port_name + "_INPORT", port_);
}

template <typename innerT >
void PortOperation<RTT::InputPort, innerT>::operation(innerT &data) {
    port_.read(data);
}

template <typename innerT >
PortOperation<RTT::OutputPort, innerT>::PortOperation(RTT::TaskContext &tc, const std::string &port_name) {
    tc.ports()->addPort(port_name + "_OUTPORT", port_);
}

template <typename innerT >
void PortOperation<RTT::OutputPort, innerT>::operation(innerT &data) {
    port_.write(data);
}

template <typename innerT >
void PortOperation<RTT::InputPort, innerT>::setDataSample(innerT &data) {
    // no operation for input port
}

template <typename innerT >
void PortOperation<RTT::OutputPort, innerT>::setDataSample(innerT &data) {
    port_.setDataSample(data);
}

using velma_low_level_interface_msgs::VelmaLowLevelCommand;
using velma_low_level_interface_msgs::VelmaLowLevelCommandArm;
using velma_low_level_interface_msgs::VelmaLowLevelCommandHand;

template class Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_t_type, &VelmaLowLevelCommandArm::t>;
template class Port<RTT::InputPort, int32_t, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_cmd_type, &VelmaLowLevelCommandArm::cmd>;
template class Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_q_type, &VelmaLowLevelCommandHand::q>;
template class Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_dq_type, &VelmaLowLevelCommandHand::dq>;
template class Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_max_i_type, &VelmaLowLevelCommandHand::max_i>;
template class Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_max_p_type, &VelmaLowLevelCommandHand::max_p>;
template class Port<RTT::InputPort, bool, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_hold_type, &VelmaLowLevelCommandHand::hold>;
template class Port<RTT::InputPort, int32_t, VelmaLowLevelCommand, VelmaLowLevelCommand::_rHand_tactileCmd_type, &VelmaLowLevelCommand::rHand_tactileCmd>;
template class Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_tMotor_i_type, &VelmaLowLevelCommand::tMotor_i>;
template class Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_i_type, &VelmaLowLevelCommand::hpMotor_i>;
template class Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_i_type, &VelmaLowLevelCommand::htMotor_i>;
template class Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_q_type, &VelmaLowLevelCommand::hpMotor_q>;
template class Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_q_type, &VelmaLowLevelCommand::htMotor_q>;
template class Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_dq_type, &VelmaLowLevelCommand::hpMotor_dq>;
template class Port<RTT::InputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_dq_type, &VelmaLowLevelCommand::htMotor_dq>;

template class Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_t_type, &VelmaLowLevelCommandArm::t>;
template class Port<RTT::OutputPort, int32_t, VelmaLowLevelCommandArm, VelmaLowLevelCommandArm::_cmd_type, &VelmaLowLevelCommandArm::cmd>;
template class Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_q_type, &VelmaLowLevelCommandHand::q>;
template class Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_dq_type, &VelmaLowLevelCommandHand::dq>;
template class Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_max_i_type, &VelmaLowLevelCommandHand::max_i>;
template class Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_max_p_type, &VelmaLowLevelCommandHand::max_p>;
template class Port<RTT::OutputPort, bool, VelmaLowLevelCommandHand, VelmaLowLevelCommandHand::_hold_type, &VelmaLowLevelCommandHand::hold>;
template class Port<RTT::OutputPort, int32_t, VelmaLowLevelCommand, VelmaLowLevelCommand::_rHand_tactileCmd_type, &VelmaLowLevelCommand::rHand_tactileCmd>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_tMotor_i_type, &VelmaLowLevelCommand::tMotor_i>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_i_type, &VelmaLowLevelCommand::hpMotor_i>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_i_type, &VelmaLowLevelCommand::htMotor_i>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_q_type, &VelmaLowLevelCommand::hpMotor_q>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_q_type, &VelmaLowLevelCommand::htMotor_q>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_dq_type, &VelmaLowLevelCommand::hpMotor_dq>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_dq_type, &VelmaLowLevelCommand::htMotor_dq>;

using velma_low_level_interface_msgs::VelmaLowLevelStatus;
using velma_low_level_interface_msgs::VelmaLowLevelStatusArm;
using velma_low_level_interface_msgs::VelmaLowLevelStatusHand;
using velma_low_level_interface_msgs::VelmaLowLevelStatusFT;

template class Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_q_type, &VelmaLowLevelStatusArm::q>;
template class Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_dq_type, &VelmaLowLevelStatusArm::dq>;
template class Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_t_type, &VelmaLowLevelStatusArm::t>;
template class Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_gt_type, &VelmaLowLevelStatusArm::gt>;
template class Port<RTT::InputPort, geometry_msgs::Wrench, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_w_type, &VelmaLowLevelStatusArm::w>;
template class Port<RTT::InputPort, Eigen::Matrix<double, 7, 7>, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_mmx_type, &VelmaLowLevelStatusArm::mmx>;
template class Port<RTT::InputPort, Eigen::VectorXi, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_friIntfState_type, &VelmaLowLevelStatusArm::friIntfState>;
template class Port<RTT::InputPort, Eigen::VectorXi, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_friRobotState_type, &VelmaLowLevelStatusArm::friRobotState>;
template class Port<RTT::InputPort, Eigen::VectorXd, VelmaLowLevelStatusHand, VelmaLowLevelStatusHand::_q_type, &VelmaLowLevelStatusHand::q>;
template class Port<RTT::InputPort, Eigen::VectorXi, VelmaLowLevelStatusHand, VelmaLowLevelStatusHand::_s_type, &VelmaLowLevelStatusHand::s>;
template class Port<RTT::InputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_tMotor_q_type, &VelmaLowLevelStatus::tMotor_q>;
template class Port<RTT::InputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_tMotor_dq_type, &VelmaLowLevelStatus::tMotor_dq>;
template class Port<RTT::InputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_hpMotor_q_type, &VelmaLowLevelStatus::hpMotor_q>;
template class Port<RTT::InputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_hpMotor_dq_type, &VelmaLowLevelStatus::hpMotor_dq>;
template class Port<RTT::InputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_htMotor_q_type, &VelmaLowLevelStatus::htMotor_q>;
template class Port<RTT::InputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_htMotor_dq_type, &VelmaLowLevelStatus::htMotor_dq>;
template class Port<RTT::InputPort, barrett_hand_controller_msgs::BHPressureState, VelmaLowLevelStatus, VelmaLowLevelStatus::_rHand_p_type, &VelmaLowLevelStatus::rHand_p>;
template class Port<RTT::InputPort, VelmaLowLevelStatus::_lHand_f_type, VelmaLowLevelStatus, VelmaLowLevelStatus::_lHand_f_type, &VelmaLowLevelStatus::lHand_f>;
template class Port<RTT::InputPort, geometry_msgs::Wrench, VelmaLowLevelStatusFT, VelmaLowLevelStatusFT::_rw_type, &VelmaLowLevelStatusFT::rw>;
template class Port<RTT::InputPort, geometry_msgs::Wrench, VelmaLowLevelStatusFT, VelmaLowLevelStatusFT::_ffw_type, &VelmaLowLevelStatusFT::ffw>;
template class Port<RTT::InputPort, geometry_msgs::Wrench, VelmaLowLevelStatusFT, VelmaLowLevelStatusFT::_sfw_type, &VelmaLowLevelStatusFT::sfw>;

template class Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_q_type, &VelmaLowLevelStatusArm::q>;
template class Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_dq_type, &VelmaLowLevelStatusArm::dq>;
template class Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_t_type, &VelmaLowLevelStatusArm::t>;
template class Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_gt_type, &VelmaLowLevelStatusArm::gt>;
template class Port<RTT::OutputPort, geometry_msgs::Wrench, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_w_type, &VelmaLowLevelStatusArm::w>;
template class Port<RTT::OutputPort, Eigen::Matrix<double, 7, 7>, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_mmx_type, &VelmaLowLevelStatusArm::mmx>;
template class Port<RTT::OutputPort, Eigen::VectorXi, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_friIntfState_type, &VelmaLowLevelStatusArm::friIntfState>;
template class Port<RTT::OutputPort, Eigen::VectorXi, VelmaLowLevelStatusArm, VelmaLowLevelStatusArm::_friRobotState_type, &VelmaLowLevelStatusArm::friRobotState>;
template class Port<RTT::OutputPort, Eigen::VectorXd, VelmaLowLevelStatusHand, VelmaLowLevelStatusHand::_q_type, &VelmaLowLevelStatusHand::q>;
template class Port<RTT::OutputPort, Eigen::VectorXi, VelmaLowLevelStatusHand, VelmaLowLevelStatusHand::_s_type, &VelmaLowLevelStatusHand::s>;
template class Port<RTT::OutputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_tMotor_q_type, &VelmaLowLevelStatus::tMotor_q>;
template class Port<RTT::OutputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_tMotor_dq_type, &VelmaLowLevelStatus::tMotor_dq>;
template class Port<RTT::OutputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_hpMotor_q_type, &VelmaLowLevelStatus::hpMotor_q>;
template class Port<RTT::OutputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_hpMotor_dq_type, &VelmaLowLevelStatus::hpMotor_dq>;
template class Port<RTT::OutputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_htMotor_q_type, &VelmaLowLevelStatus::htMotor_q>;
template class Port<RTT::OutputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_htMotor_dq_type, &VelmaLowLevelStatus::htMotor_dq>;
template class Port<RTT::OutputPort, barrett_hand_controller_msgs::BHPressureState, VelmaLowLevelStatus, VelmaLowLevelStatus::_rHand_p_type, &VelmaLowLevelStatus::rHand_p>;
template class Port<RTT::OutputPort, VelmaLowLevelStatus::_lHand_f_type, VelmaLowLevelStatus, VelmaLowLevelStatus::_lHand_f_type, &VelmaLowLevelStatus::lHand_f>;
template class Port<RTT::OutputPort, geometry_msgs::Wrench, VelmaLowLevelStatusFT, VelmaLowLevelStatusFT::_rw_type, &VelmaLowLevelStatusFT::rw>;
template class Port<RTT::OutputPort, geometry_msgs::Wrench, VelmaLowLevelStatusFT, VelmaLowLevelStatusFT::_ffw_type, &VelmaLowLevelStatusFT::ffw>;
template class Port<RTT::OutputPort, geometry_msgs::Wrench, VelmaLowLevelStatusFT, VelmaLowLevelStatusFT::_sfw_type, &VelmaLowLevelStatusFT::sfw>;

};  // namespace velma_lli_types

