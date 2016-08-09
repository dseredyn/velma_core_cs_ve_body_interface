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

/*    int shm_fd;
    channel_hdr_t *shm_hdr;
    const char *shm_name = "velma_lli_cmd";

    shm_fd = shm_open(shm_name, O_RDWR, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
    if (shm_fd < 0) {
        std::string err_str(strerror(errno));
        std::cout << "VelmaLLIHiTx shm_open failed: " << err_str << std::endl;
        return false;
    }

    struct stat sb;
    fstat(shm_fd, &sb);

    shm_hdr = reinterpret_cast<channel_hdr_t*>( mmap(NULL, sb.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0) );

    if (shm_hdr == MAP_FAILED) {
        std::string err_str(strerror(errno));
        std::cout << "VelmaLLIHiTx mmap failed failed: " << err_str << std::endl;
        return false;
    }

    if (CHANNEL_DATA_SIZE(shm_hdr->size, shm_hdr->max_readers) != sb.st_size) {
        std::cout << "VelmaLLIHiTx error data" << std::endl;
        return false;
    }

    init_channel(shm_hdr, &chan_);
*/

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
//    const size_t size = CHANNEL_DATA_SIZE(chan_.hdr->size, chan_.hdr->max_readers);
    Logger::In in("VelmaLLIHiTx::cleanupHook");

    Logger::log() << Logger::Info << "releasing writer" << Logger::endl;
    Logger::log() << Logger::Info << "wr_.inuse: " << (size_t)(wr_.inuse) << Logger::endl;
    release_writer(&wr_);     // this segfaults

    Logger::log() << Logger::Info << "disconnecting channel" << Logger::endl;
    disconnect_channel(&chan_);

/*    munmap(chan_.hdr, size);
    chan_.reader_ids = NULL;
    chan_.reading = NULL;
    free(chan_.buffer);
    chan_.buffer = NULL;
    chan_.hdr = NULL;
*/
}

bool VelmaLLIHiTx::startHook() {
//    RESTRICT_ALLOC;
    void *pbuf = NULL;
    writer_buffer_get(&wr_, &pbuf);
    buf_ = reinterpret_cast<VelmaLowLevelCommand*>(pbuf);

//    UNRESTRICT_ALLOC;
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
//    std::cout << "VelmaLLIHiTx" << std::endl;

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

