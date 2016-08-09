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
#include <rtt/Logger.hpp>

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

#include "velma_lli_hi_tx.h"

using namespace velma_low_level_interface_msgs;
using namespace RTT;

VelmaLLIHiTx::VelmaLLIHiTx(const std::string &name) :
    TaskContext(name, PreOperational),
    in_(*this)
{
}

bool VelmaLLIHiTx::configureHook() {
    Logger::In in("VelmaLLIHiTx::configureHook");

    if (connect_channel("velma_lli_cmd", &chan_) != 0) {
        Logger::log() << Logger::Error << "connect_channel failed" << Logger::endl;
    }

    int ret = create_writer(&chan_, &wr_);

    Logger::log() << Logger::Info << "created writer, max_readers: " << chan_.hdr->max_readers << Logger::endl;
    Logger::log() << Logger::Info << "wr_.inuse: " << (size_t)(wr_.inuse) << Logger::endl;

    if (ret != 0) {

        if (ret == -1) {
            Logger::log() << Logger::Error << "invalid writer_t pointer" << Logger::endl;
        }

        if (ret == -2) {
            Logger::log() << Logger::Error << "no writers slots avalible" << Logger::endl;
        }
        return false;
    }

    return true;
}

void VelmaLLIHiTx::cleanupHook() {
    Logger::In in("VelmaLLIHiTx::cleanupHook");

    Logger::log() << Logger::Info << "releasing writer" << Logger::endl;
    Logger::log() << Logger::Info << "wr_.inuse: " << (size_t)(wr_.inuse) << Logger::endl;
    release_writer(&wr_);     // this segfaults

    Logger::log() << Logger::Info << "disconnecting channel" << Logger::endl;
    disconnect_channel(&chan_);
}

bool VelmaLLIHiTx::startHook() {
    void *pbuf = NULL;
    writer_buffer_get(&wr_, &pbuf);
    buf_ = reinterpret_cast<VelmaLowLevelCommand*>(pbuf);
    return true;
}

void VelmaLLIHiTx::stopHook() {
}

void VelmaLLIHiTx::updateHook() {
    Logger::In in("VelmaLLIHiTx::updateHook");
//    RESTRICT_ALLOC;
    // write outputs
//    UNRESTRICT_ALLOC;
    in_.readPorts(cmd_out_);

    if (buf_ == NULL) {
        Logger::log() << Logger::Error << "writer get NULL buffer" << Logger::endl;
    }
    else {
        *buf_ = cmd_out_;
        writer_buffer_write(&wr_);
    }
    void *pbuf = NULL;
    writer_buffer_get(&wr_, &pbuf);
    buf_ = reinterpret_cast<VelmaLowLevelCommand*>(pbuf);
}

