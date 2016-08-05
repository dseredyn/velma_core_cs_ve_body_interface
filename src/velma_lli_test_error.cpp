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

#include <rtt/base/PortInterface.hpp>

#include "velma_lli_test_error.h"
#include "velma_lli_test_generator.h"

VelmaTestError::VelmaTestError(const std::string &name) :
    RTT::TaskContext(name, PreOperational),
    out_(*this)
{
//    this->ports()->addPort("command_INPORT", port_cmd_in_);
    this->ports()->addPort("comm_status_INPORT", port_comm_status_in_);
}

bool VelmaTestError::configureHook() {
    return true;
}

bool VelmaTestError::startHook() {
//    RESTRICT_ALLOC;

    no_new_data_ = 0;
    connecting_ = false;

//    UNRESTRICT_ALLOC;
    return true;
}

void VelmaTestError::stopHook() {
}

void VelmaTestError::updateHook() {
//    RESTRICT_ALLOC;

    uint32_t comm_status_in = 0;
    if (port_comm_status_in_.read(comm_status_in) == RTT::NewData) {
        if (comm_status_in == 0) {
            velma_low_level_interface_msgs::VelmaLowLevelStatus status_gen;
            VelmaLLITestGenerator gen_;
            gen_.generate(0, cmd_out_, status_gen);
            out_.writePorts(cmd_out_);
        }
    }
    else {
        std::cout << "VelmaTestError error: no comm status data" << std::endl;
    }


}

