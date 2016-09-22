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

#include "velma_lli_lo_rx.h"

using namespace velma_low_level_interface_msgs;

using namespace RTT;

VelmaLLILoRx::VelmaLLILoRx(const std::string &name) :
    TaskContext(name, PreOperational),
    shm_name_("velma_lli_cmd"),
    buf_prev_(NULL)
{
    this->ports()->addPort("command_OUTPORT", port_command_out_);
}

bool VelmaLLILoRx::configureHook() {
    Logger::In in("VelmaLLILoRx::configureHook");

    if (create_shm_object(shm_name_, sizeof(VelmaLowLevelCommand), 1) != 0) {
        Logger::log() << Logger::Error << "create_shm_object failed" << Logger::endl;
        return false;
    }

    if (connect_channel(shm_name_, &chan_) != 0) {
        Logger::log() << Logger::Error << "connect_channel failed" << Logger::endl;
        return false;
    }

    int ret = create_reader(&chan_, &re_);

    if (ret != 0) {

        if (ret == -1) {
            Logger::log() << Logger::Error << "invalid reader_t pointer" << Logger::endl;
        }

        if (ret == -2) {
            Logger::log() << Logger::Error << "no reader slots avalible" << Logger::endl;
        }
        return false;
    }

    return true;
}

void VelmaLLILoRx::cleanupHook() {
    const size_t size = CHANNEL_DATA_SIZE(chan_.hdr->size, chan_.hdr->max_readers);

    release_reader(&re_);

    disconnect_channel(&chan_);

    delete_shm_object(shm_name_);
}

bool VelmaLLILoRx::startHook() {
//    RESTRICT_ALLOC;
    buf_prev_ = reinterpret_cast<VelmaLowLevelCommand*>( reader_buffer_get(&re_) );

    receiving_data_ = false;

//    UNRESTRICT_ALLOC;
    return true;
}

void VelmaLLILoRx::stopHook() {
}

void VelmaLLILoRx::updateHook() {
    Logger::In in("VelmaLLILoRx::updateHook");
//    RESTRICT_ALLOC;
    // write outputs
//    UNRESTRICT_ALLOC;
    Logger::log() << Logger::Debug << Logger::endl;

    VelmaLowLevelCommand *buf = NULL;

    if (receiving_data_) {
        buf = reinterpret_cast<VelmaLowLevelCommand*>( reader_buffer_timedwait(&re_, 1, 0) );
    }
    else {
        buf = reinterpret_cast<VelmaLowLevelCommand*>( reader_buffer_get(&re_) );
    }

//    VelmaLowLevelCommand *buf = reinterpret_cast<VelmaLowLevelCommand*>( reader_buffer_get(&re_) );

    if (buf == NULL) {
        Logger::log() << Logger::Debug << "could not receive data (NULL buffer)" << Logger::endl;
        receiving_data_ = false;
    }
    else {
        if (buf != buf_prev_) {
            buf_prev_ = buf;
            port_command_out_.write(*buf);
            Logger::log() << Logger::Debug << "received new data" << Logger::endl;
            receiving_data_ = true;
        }
        else {
            Logger::log() << Logger::Debug << "could not receive data" << Logger::endl;
            receiving_data_ = false;
        }
    }
}

