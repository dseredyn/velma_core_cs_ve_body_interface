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
#include <rtt/types/TypeInfo.hpp>
#include <rtt/Logger.hpp>
//#include <mqueue.h>

#include "velma_lli_lo_tx.h"

#include <rtt_rosclock/rtt_rosclock.h>

using namespace RTT;

VelmaLLILoTx::VelmaLLILoTx(const std::string &name) :
    RTT::TaskContext(name, PreOperational),
    in_(*this)
{
//    mq_unlink("/lli_status");
    this->ports()->addPort("status_OUTPORT", port_status_out_);
//    this->ports()->addPort("status_INPORT", port_status_in_);
}

bool VelmaLLILoTx::configureHook() {
    Logger::In in("VelmaLLILoTx::configureHook");
/*
    if (create_shm_object("velma_lli_sta", sizeof(VelmaLowLevelStatus), 1) != 0) {
        Logger::log() << Logger::Error << "create_shm_object failed" << Logger::endl;
        return false;
    }

    if (connect_channel("velma_lli_sta", &chan_) != 0) {
        Logger::log() << Logger::Error << "connect_channel failed" << Logger::endl;
        return false;
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
*/
    return true;
}

void VelmaLLILoTx::cleanupHook() {
    Logger::In in("VelmaLLIHiTx::cleanupHook");
/*
    Logger::log() << Logger::Info << "releasing writer" << Logger::endl;
    Logger::log() << Logger::Info << "wr_.inuse: " << (size_t)(wr_.inuse) << Logger::endl;
    release_writer(&wr_);     // this segfaults

    Logger::log() << Logger::Info << "disconnecting channel" << Logger::endl;
    disconnect_channel(&chan_);
*/
}

bool VelmaLLILoTx::startHook() {
//    RESTRICT_ALLOC;
//    UNRESTRICT_ALLOC;
/*
    void *pbuf = NULL;
    writer_buffer_get(&wr_, &pbuf);
    buf_ = reinterpret_cast<VelmaLowLevelStatus*>(pbuf);
*/
    return true;
}

void VelmaLLILoTx::stopHook() {
}

void VelmaLLILoTx::updateHook() {
    Logger::In in("VelmaLLILoTx::updateHook");

    ros::Time wall_time = rtt_rosclock::host_wall_now();
    double sec = wall_time.toSec();
    long nsec = sec;
    Logger::log() << Logger::Debug << (nsec%2000) << " " << (sec - nsec) << Logger::endl;

//    RESTRICT_ALLOC;
    in_.readPorts(status_);
// TODO: check for new data
//    if (port_status_in_.read(status_) == RTT::NewData) {

    if (in_.isAllDataValid()) {
        // write outputs
        Logger::log() << Logger::Debug << "test: " << status_.test << Logger::endl;
        port_status_out_.write(status_);
    }
/*
    if (buf_ == NULL) {
        Logger::log() << Logger::Error << "writer get NULL buffer" << Logger::endl;
        error();
    }
    else {
        *buf_ = status_;
        writer_buffer_write(&wr_);
        Logger::log() << Logger::Debug << "sending command" << Logger::endl;
    }
    void *pbuf = NULL;
    writer_buffer_get(&wr_, &pbuf);
    buf_ = reinterpret_cast<VelmaLowLevelStatus*>(pbuf);
*/
//    UNRESTRICT_ALLOC;
}

