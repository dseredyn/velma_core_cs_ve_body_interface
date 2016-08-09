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

#include "velma_lli_hi_test.h"

VelmaLLIHiTest::VelmaLLIHiTest(const std::string &name) :
    RTT::TaskContext(name, PreOperational),
    in_(*this),
    out_(*this)
{
}

bool VelmaLLIHiTest::configureHook() {
    return true;
}

bool VelmaLLIHiTest::startHook() {
    no_rec_counter_ = 0;
    return true;
}

void VelmaLLIHiTest::stopHook() {
}

void VelmaLLIHiTest::updateHook() {

    in_.readPorts(status_in_);

    int rand_seed = status_in_.test;
    velma_low_level_interface_msgs::VelmaLowLevelCommand cmd_gen;
    velma_low_level_interface_msgs::VelmaLowLevelStatus status_gen;
    gen_.generate(rand_seed, cmd_gen, status_gen);

    // compare the received status data
    // with generated status data
    if (gen_.toStr(status_in_) == gen_.toStr(status_gen)) {
        // send new command data
        cmd_out_ = cmd_gen;

        no_rec_counter_ = 0;
        out_.writePorts(cmd_out_);
    }
    else {
        ++no_rec_counter_;
    }

    if (no_rec_counter_ > 0) {
        std::cout << "VelmaLLIHiTest no new data during " << no_rec_counter_ << " loops, status_in.tMotor_q: " << status_in_.tMotor_q <<
            "   status_gen.tMotor_q: " << status_gen.tMotor_q << std::endl;
    }

    if (no_rec_counter_ > 20) {
        std::cout << "VelmaLLIHiTest ERROR" << std::endl;
        stop();
    }
//    UNRESTRICT_ALLOC;
}

