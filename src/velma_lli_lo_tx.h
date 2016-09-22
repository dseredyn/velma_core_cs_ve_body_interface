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

#ifndef VELMA_LLI_LO_TX_H_
#define VELMA_LLI_LO_TX_H_

#include <cstring>

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/os/TimeService.hpp"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include "velma_low_level_interface_msgs/VelmaLowLevelStatus.h"
#include <barrett_hand_controller_msgs/BHPressureState.h>

#include "eigen_conversions/eigen_msg.h"

#include "velma_low_level_interface/velma_lli_status_ports.h"

class VelmaLLILoTx: public RTT::TaskContext {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit VelmaLLILoTx(const std::string &name);

  bool configureHook();

  bool startHook();

  void stopHook();

  void updateHook();

 private:

    // low-level interface port
    RTT::OutputPort<velma_low_level_interface_msgs::VelmaLowLevelStatus> port_status_out_;

    // low-level interface port variable
    velma_low_level_interface_msgs::VelmaLowLevelStatus status_;

//    VelmaLLIStatusInput in_;
    RTT::InputPort<VelmaLowLevelStatus> port_status_in_;
};

#endif  // VELMA_LLI_LO_TX_H_

