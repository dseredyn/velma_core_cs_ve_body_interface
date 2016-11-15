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

#include "velma_lli_hi_rx.h"

#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt/extras/SlaveActivity.hpp>

using namespace RTT;

VelmaLLIHiRx::VelmaLLIHiRx(const std::string &name) :
    TaskContext(name, PreOperational),
    out_(*this),
    shm_name_("velma_lli_sta"),
    buf_prev_(NULL)
{
    this->ports()->addEventPort("status_INPORT", port_status_in_);

    this->addOperation("pushBackPeerExecution", &VelmaLLIHiRx::pushBackPeerExecution, this, RTT::ClientThread)
        .doc("enable HW operation");
}

bool VelmaLLIHiRx::pushBackPeerExecution(const std::string &peer_name) {
    Logger::In in("VelmaLLIHiRx::pushBackPeerExecution");
    if (isConfigured() || isRunning()) {
        Logger::log() << Logger::Warning << "this operation should be invoked before configure"
                      << Logger::endl;
        return false;
    }
    peer_list_.push_back(peer_name);
    return true;
}

bool VelmaLLIHiRx::configureHook() {
    Logger::In in("VelmaLLIHiRx::configureHook");

    TaskContext::PeerList l = this->getPeerList();
    if (peer_list_.size() != l.size()) {
        Logger::log() << Logger::Error << "peer_list_.size() != l.size()   "
                      << peer_list_.size() << " != " << l.size() << Logger::endl;
        return false;
    }

    for (std::list<std::string >::const_iterator it2 = peer_list_.begin(); it2 != peer_list_.end(); ++it2) {
        TaskContext *tc = NULL;
        bool found = false;
        for (TaskContext::PeerList::const_iterator it = l.begin(); it != l.end(); ++it) {
            if ( (*it) == (*it2)) {
                tc = this->getPeer( (*it) );
                break;
            }
        }
        if (tc == NULL) {
            Logger::log() << Logger::Error << "could not find peer "
                          << (*it2) << " in peer execution list" << Logger::endl;
            return false;
        }
        peers_.push_back(tc);
    }

    for (std::list<TaskContext* >::iterator it = peers_.begin(); it != peers_.end(); ++it) {
        (*it)->setActivity( new RTT::extras::SlaveActivity(this->getActivity(), (*it)->engine()));

        Attribute<bool> triggerOnStart = (*it)->attributes()->getAttribute("TriggerOnStart");
        if (!triggerOnStart.ready()) {
            Logger::log() << Logger::Error << "attribute TriggerOnStart of peer "
                          << (*it)->getName() << " is not ready" << Logger::endl;
            return false;
        }
        triggerOnStart.set(false);
    }

    mTriggerOnStart = false;

/*
    if (connect_channel(shm_name_, &chan_) != 0) {
        Logger::log() << Logger::Error << "connect_channel failed" << Logger::endl;
        return false;
    }

    int ret = create_reader(&chan_, &re_);

    if (ret != 0) {

        if (ret == -1) {
            Logger::log() << Logger::Error << "invalid reader_t pointer" << Logger::endl;
        }
        else if (ret == -2) {
            Logger::log() << Logger::Error << "no reader slots avalible" << Logger::endl;
        }
        else {
            Logger::log() << Logger::Error << "create_reader error: " << ret << Logger::endl;
        }
        return false;
    }
//*/
    return true;
}

void VelmaLLIHiRx::cleanupHook() {
/*
    const size_t size = CHANNEL_DATA_SIZE(chan_.hdr->size, chan_.hdr->max_readers);

    release_reader(&re_);

    disconnect_channel(&chan_);

    delete_shm_object(shm_name_);
//*/
}

bool VelmaLLIHiRx::startHook() {
//    RESTRICT_ALLOC;
//    buf_prev_ = reinterpret_cast<VelmaLowLevelStatus*>( reader_buffer_get(&re_) );
//    UNRESTRICT_ALLOC;
    return true;
}

void VelmaLLIHiRx::stopHook() {
}

void VelmaLLIHiRx::updateHook() {
//    RESTRICT_ALLOC;
    // write outputs
//    UNRESTRICT_ALLOC;
    uint32_t test_prev = status_in_.test;

/*

    VelmaLowLevelStatus *buf = reinterpret_cast<VelmaLowLevelStatus*>( reader_buffer_timedwait(&re_, 1, 0) );

    bool received = false;
    if (buf == NULL) {
        Logger::log() << Logger::Error << "could not receive data (NULL buffer)" << Logger::endl;
        error();
    }
    else {
        if (buf != buf_prev_) {
            buf_prev_ = buf;

//            Logger::log() << Logger::Debug << "test: " << buf->test << Logger::endl;
            received = true;

            status_in_ = *buf;
//            port_command_out_.write(*buf);
        }
        else {
        }
    }

//*/

//    if (received) {
    if (port_status_in_.read(status_in_) == NewData) {

        if (test_prev == status_in_.test) {
            Logger::In in("VelmaLLIHiRx::updateHook");
            Logger::log() << Logger::Warning << "executed updateHook twice for the same packet " << status_in_.test << Logger::endl;
//            error();
        }

        out_.writePorts(status_in_);

        for (std::list<TaskContext* >::iterator it = peers_.begin(); it != peers_.end(); ++it) {
            if (!(*it)->update()) {
                Logger::In in("VelmaLLIHiRx::updateHook");
                Logger::log() << Logger::Error << (*it)->getName() << "->update() has failed" << Logger::endl;
                error();
            }
        }
    }
    else {
/*        Logger::log() << Logger::Warning << "received old data: "
                      << "Cycle: " << (mCycleCounter - mCycleCounter_prev_)
                      << " IO: " << (mIOCounter - mIOCounter_prev_)
                      << " TimeOut: " << (mTimeOutCounter - mTimeOutCounter_prev_)
                      << " Trigger: " << (mTriggerCounter - mTriggerCounter_prev_)
                      << Logger::endl;
*/
    }
//    trigger();
}

