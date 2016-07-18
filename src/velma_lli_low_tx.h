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

#ifndef VELMA_LLI_LOW_TX_H_
#define VELMA_LLI_LOW_TX_H_

#include <cstring>

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/os/TimeService.hpp"
#include "Eigen/Dense"
#include "Eigen/LU"

#include "velma_low_level_interface_msgs/VelmaLowLevelStatus.h"

#include "eigen_conversions/eigen_msg.h"

class VelmaLLILowTx: public RTT::TaskContext {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit VelmaLLILowTx(const std::string &name);

  bool configureHook();

  bool startHook();

  void stopHook();

  void updateHook();

/*  typedef Eigen::MatrixXd Jacobian;
  typedef Eigen::MatrixXd Inertia;
  typedef Eigen::VectorXd Joints;
  typedef Eigen::VectorXd Stiffness;
  typedef Eigen::VectorXd Spring;
  typedef Eigen::VectorXd Force;
  typedef Eigen::Matrix<double, 4, 1> ToolMass;
  typedef Eigen::Matrix<double, 7, 1> Tool;
*/
 private:
//  RTT::InputPort<Eigen::VectorXd> port_joint_position_;
//  RTT::InputPort<Eigen::VectorXd> port_joint_velocity_;
//  RTT::InputPort<Eigen::MatrixXd> port_mass_matrix_inv_;

//  std::vector<RTT::InputPort<geometry_msgs::Pose>* > port_cartesian_position_command_;
  RTT::OutputPort<velma_low_level_interface_msgs::VelmaLowLevelStatus> port_status_out_;
//  std::vector<RTT::InputPort<geometry_msgs::Pose>* > port_tool_position_command_;
//  std::vector<RTT::InputPort<cartesian_trajectory_msgs::CartesianImpedance>* > port_cartesian_impedance_command_;

//  RTT::InputPort<Eigen::VectorXd> port_nullspace_torque_command_;

//  RTT::OutputPort<Eigen::VectorXd> port_joint_torque_command_;
    velma_low_level_interface_msgs::VelmaLowLevelStatus status_out_;
};

#endif  // VELMA_LLI_LOW_TX_H_

