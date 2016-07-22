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
{ }


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
template class Port<RTT::InputPort, int32_t, VelmaLowLevelCommand, VelmaLowLevelCommand::_rHandTactile_cmd_type, &VelmaLowLevelCommand::rHandTactile_cmd>;
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
template class Port<RTT::OutputPort, int32_t, VelmaLowLevelCommand, VelmaLowLevelCommand::_rHandTactile_cmd_type, &VelmaLowLevelCommand::rHandTactile_cmd>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_tMotor_i_type, &VelmaLowLevelCommand::tMotor_i>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_i_type, &VelmaLowLevelCommand::hpMotor_i>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_i_type, &VelmaLowLevelCommand::htMotor_i>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_q_type, &VelmaLowLevelCommand::hpMotor_q>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_q_type, &VelmaLowLevelCommand::htMotor_q>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_hpMotor_dq_type, &VelmaLowLevelCommand::hpMotor_dq>;
template class Port<RTT::OutputPort, double, VelmaLowLevelCommand, VelmaLowLevelCommand::_htMotor_dq_type, &VelmaLowLevelCommand::htMotor_dq>;

using velma_low_level_interface_msgs::VelmaLowLevelStatus;

template class Port<RTT::InputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_tMotor_q_type, &VelmaLowLevelStatus::tMotor_q>;

template class Port<RTT::OutputPort, double, VelmaLowLevelStatus, VelmaLowLevelStatus::_tMotor_q_type, &VelmaLowLevelStatus::tMotor_q>;

};  // namespace velma_lli_types
