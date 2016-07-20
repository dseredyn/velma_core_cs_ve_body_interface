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

#include <rtt/Component.hpp>

#include "velma_lli_hi_tx.h"

  VelmaLLIHiTx::VelmaLLIHiTx(const std::string &name) :
    RTT::TaskContext(name, PreOperational),
    in_(*this, cmd_out_)
  {
    this->ports()->addPort("command_OUTPORT", port_cmd_out_);
  }

  bool VelmaLLIHiTx::configureHook() {
    cmd_out_.rHandTactile_cmd = 10000;
    return true;
  }

  bool VelmaLLIHiTx::startHook() {
//    RESTRICT_ALLOC;

//    UNRESTRICT_ALLOC;
    return true;
  }

  void VelmaLLIHiTx::stopHook() {
  }

  void VelmaLLIHiTx::updateHook() {
//    RESTRICT_ALLOC;
    // write outputs
//    UNRESTRICT_ALLOC;
    in_.readPorts(cmd_out_);
//    cmd_out_.rHandTactile_cmd--;
    std::cout << "VelmaLLIHiTx " << cmd_out_.rHandTactile_cmd << std::endl;
    port_cmd_out_.write(cmd_out_);
  }

