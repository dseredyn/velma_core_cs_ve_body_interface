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

#include "velma_lli_lo_rx.h"

  VelmaLLILoRx::VelmaLLILoRx(const std::string &name) :
    RTT::TaskContext(name, PreOperational),
    out_(*this)
  {
//    this->ports()->addEventPort("command_INPORT", port_cmd_in_);
    this->ports()->addPort("command_INPORT", port_cmd_in_);
    this->ports()->addPort("comm_status_OUTPORT", port_comm_status_out_);
  }

  bool VelmaLLILoRx::configureHook() {
    return true;
  }

  bool VelmaLLILoRx::startHook() {
//    RESTRICT_ALLOC;

//    UNRESTRICT_ALLOC;
    return true;
  }

  void VelmaLLILoRx::stopHook() {
  }

void VelmaLLILoRx::updateHook() {
//    RESTRICT_ALLOC;
    // write outputs
//    UNRESTRICT_ALLOC;
    uint32_t comm_status_out = 0;

    if (port_cmd_in_.read(cmd_in_) == RTT::NewData) {
        comm_status_out = 1;

        out_.writePorts(cmd_in_);

        RTT::TaskContext::PeerList l = this->getPeerList();
        for (RTT::TaskContext::PeerList::const_iterator it = l.begin(); it != l.end(); ++it) {
            this->getPeer( *it )->getActivity()->trigger();
//            std::cout << "VelmaLLIHiRx peer list: " << (*it) << " ";
        }
//        std::cout << std::endl;
//        this->getPeer("scheme")->getActivity()->trigger();
    }
    else {
        comm_status_out = 0;
    }

    port_comm_status_out_.write(comm_status_out);
}

