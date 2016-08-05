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

VelmaLLILoRx::VelmaLLILoRx(const std::string &name) :
    RTT::TaskContext(name, PreOperational),
    out_(*this),
    shm_name_("velma_lli_cmd")
{
    this->ports()->addPort("comm_status_OUTPORT", port_comm_status_out_);
}

bool VelmaLLILoRx::configureHook() {
    const size_t size = sizeof(VelmaLowLevelCommand);
    const uint32_t readers = 1;

    channel_hdr_t *shm_hdr_;

//    shm_fd_ = shm_open(shm_name_, O_RDWR | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
    shm_fd_ = shm_open(shm_name_, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
    if (shm_fd_ < 0) {
        std::string err_str(strerror(errno));
        std::cout << "VelmaLLILoRx shm_open failed: " << err_str << std::endl;
        return false;
    }

    if (ftruncate(shm_fd_, CHANNEL_DATA_SIZE(size, readers)) != 0) {
        std::cout << "VelmaLLILoRx ftruncate failed" << std::endl;
        shm_unlink(shm_name_);
        return false;
    }

    shm_hdr_ = reinterpret_cast<channel_hdr_t*>( mmap(NULL, CHANNEL_DATA_SIZE(size, readers), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0) );

    if (shm_hdr_ == MAP_FAILED) {
        std::string err_str(strerror(errno));
        std::cout << "VelmaLLILoRx mmap failed: " << err_str << std::endl;
        shm_unlink(shm_name_);
        return false;
    }

    init_channel_hdr(size, readers, shm_hdr_);

    init_channel(shm_hdr_, &chan_);

    int ret = create_reader(&chan_, &re_);

    if (ret != 0) {

        if (ret == -1)
            printf("invalid reader_t pointer\n");

        if (ret == -2)
            printf("no reader slots avalible\n");
        return 0;
    }

    return true;
}

void VelmaLLILoRx::cleanupHook() {
    const size_t size = CHANNEL_DATA_SIZE(chan_.hdr->size, chan_.hdr->max_readers);

    release_reader(&re_);

    munmap(chan_.hdr, size);
    chan_.reader_ids = NULL;
    chan_.reading = NULL;
    free(chan_.buffer);
    chan_.buffer = NULL;
    chan_.hdr = NULL;

    close(shm_fd_);
    shm_unlink(shm_name_);
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

    VelmaLowLevelCommand *buf = reinterpret_cast<VelmaLowLevelCommand*>( reader_buffer_get(&re_) );

    if (buf == NULL) {
        std::cout << "VelmaLLILoRx reader got NULL buffer" << std::endl;
    }

    if (buf != buf_prev_) {
        buf_prev_ = buf;

        comm_status_out = 1;

        cmd_in_ = *buf;
        out_.writePorts(cmd_in_);
    }
    else {
        comm_status_out = 0;
    }

    port_comm_status_out_.write(comm_status_out);
}

