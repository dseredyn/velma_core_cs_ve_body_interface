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

#include "velma_lli_monitor.h"

#include <rtt/transports/mqueue/MQLib.hpp>

VelmaLLIMonitor::VelmaLLIMonitor(const std::string &name) :
    RTT::TaskContext(name, PreOperational)
{
//    this->ports()->addPort("command_INPORT", port_cmd_in_);
    this->ports()->addPort("comm_status_INPORT", port_comm_status_in_);
}

bool VelmaLLIMonitor::configureHook() {
    return true;
}

bool VelmaLLIMonitor::startHook() {
//    RESTRICT_ALLOC;

    no_new_data_ = 0;
    connecting_ = false;

//    UNRESTRICT_ALLOC;
    return true;
}

void VelmaLLIMonitor::stopHook() {
}

void VelmaLLIMonitor::updateHook() {
//    RESTRICT_ALLOC;
    // write outputs
//    UNRESTRICT_ALLOC;

//    bool initialized = false;
    RTT::TaskContext *peer = this->getPeer("lli_lo_rx");
    RTT::base::PortInterface *pi = NULL;
    if (peer != NULL) {
        RTT::Service::shared_ptr srv = peer->provides();
        if (srv != NULL) {
//            initialized = true;
            pi = srv->getPort("command_INPORT");
        }
    }

    if (pi != NULL) {
        if (pi->connected()) {
            uint32_t comm_status_in = 0;
            if (port_comm_status_in_.read(comm_status_in) == RTT::NewData) {
                no_new_data_ = 0;
            }
            else {
                ++no_new_data_;
                if ((!connecting_ && no_new_data_ > 15) || no_new_data_ > 150) {
                    std::cout << "VelmaLLIMonitor: no new data - disconnecting" << std::endl;
                    pi->disconnect();
//                    port_cmd_in_.disconnect();
                    no_new_data_ = 0;
                }
            }

        }
        else {  // if (pi->connected())
            // try to create a stream
            RTT::ConnPolicy conn;
            conn.transport = ORO_MQUEUE_PROTOCOL_ID;                 // the MQueue protocol id
            conn.name_id   = "/lli_command";    // the connection id
            if (pi->createStream(conn)) {
/*                if (!port_cmd_in_.createStream(conn)) {
                    pi->disconnect();
                    std::cout << "VelmaLLIMonitor: could not stream port_cmd_in_" << std::endl;
                }
                else {
                    std::cout << "VelmaLLIMonitor: successfuly streamed" << std::endl;
                }
*/
                connecting_ = true;
                connecting_counter_ = 0;
                std::cout << "VelmaLLIMonitor: successfuly streamed" << std::endl;
            }
            else {
                std::cout << "VelmaLLIMonitor: could not stream pi" << std::endl;
            }
        }
    }
    else {
        std::cout << "VelmaLLIMonitor: port is not created" << std::endl;
    }

    if (connecting_ && connecting_counter_ > 1000) {
        connecting_ = false;
    }
    else {
        ++connecting_counter_;
    }

/*
        RTT::TaskContext::PeerList l = this->getPeerList();
        for (RTT::TaskContext::PeerList::const_iterator it = l.begin(); it != l.end(); ++it) {
            RTT::Service::shared_ptr srv = this->getPeer( *it )->provides();
            srv->getPort("");
//            std::cout << "VelmaLLIHiRx peer list: " << (*it) << " ";
        }
*/
}

