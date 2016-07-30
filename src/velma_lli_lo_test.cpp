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

#include "velma_lli_lo_test.h"

VelmaLLILoTest::VelmaLLILoTest(const std::string &name) :
    RTT::TaskContext(name, PreOperational),
    in_(*this, cmd_in_),
    out_(*this, status_out_)
{
}

bool VelmaLLILoTest::configureHook() {
    return true;
}

bool VelmaLLILoTest::startHook() {
//    RESTRICT_ALLOC;
    no_rec_counter_ = 0;

    rand_seed_ = 1;

    velma_low_level_interface_msgs::VelmaLowLevelCommand cmd_gen;
    velma_low_level_interface_msgs::VelmaLowLevelStatus status_gen;
    gen_.generate(rand_seed_, cmd_gen, status_gen);
    status_out_ = status_gen;
    prev_cmd_in_ = cmd_gen;
    out_.writePorts(status_out_);
    std::cout << "VelmaLLILoTest::startHook send " << status_out_.test << std::endl;

//    std::cout << "VelmaLLILoTest::startHook status_out_.tMotor_q " << status_out_.tMotor_q << std::endl;

//    UNRESTRICT_ALLOC;
    return true;
}

void VelmaLLILoTest::stopHook() {
}

void VelmaLLILoTest::updateHook() {
//    RESTRICT_ALLOC;

    in_.readPorts(cmd_in_);

//    std::cout << "VelmaLLILoTest received: " << cmd_in_.test << "  should be " << prev_cmd_in_.test << std::endl;

    // check if the data was generated and compare the received command data
    // with generated command data
    if (gen_.toStr(cmd_in_) == gen_.toStr(prev_cmd_in_)) {
        // generate new data
        ++rand_seed_;
        velma_low_level_interface_msgs::VelmaLowLevelCommand cmd_gen;
        velma_low_level_interface_msgs::VelmaLowLevelStatus status_gen;
//        std::cout << "VelmaLLILoTest: generate " << rand_seed_ << std::endl;
        gen_.generate(rand_seed_, cmd_gen, status_gen);
        // send new status data
        status_out_ = status_gen;

        // save the generated command data
        prev_cmd_in_ = cmd_gen;
//        prev_status_out_ = status_gen;

        no_rec_counter_ = 0;
        out_.writePorts(status_out_);
//        std::cout << "VelmaLLILoTest: send " << status_out_.test << std::endl;
    }
    else {
        ++no_rec_counter_;
    }

    if (no_rec_counter_ > 0) {
        std::cout << "VelmaLLILoTest no new data during " << no_rec_counter_ << " loops" << std::endl;
    }

    if (no_rec_counter_ > 20) {
        std::cout << "VelmaLLILoTest ERROR" << std::endl;
        stop();
    }

    // write outputs
//    UNRESTRICT_ALLOC;
}

