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
#include <rtt/base/PortInterface.hpp>

#include "velma_hi_state.h"

using namespace RTT;

VelmaHiState::VelmaHiState(const std::string &name) :
    TaskContext(name, PreOperational),
    cmd_sc_(-1)
{
    addProperty("behaviors", behaviors_str_);

    this->ports()->addPort("cmd_sc_OUTPORT", port_cmd_sc_out_);

    this->addOperation("switchBehavior", &BehaviorManager::switchBehavior, this, RTT::OwnThread)
        .doc("switch behavior");

    this->addOperation("enable_hw", &BehaviorManager::enableHw, this, RTT::OwnThread)
        .doc("enable HW operation");

    this->addOperation("enable_controller", &BehaviorManager::enableController, this, RTT::OwnThread)
        .doc("enable high-level controller");

}

bool VelmaHiState::enableHw() {
    cmd_sc_ = 1;
    return true;
}

bool VelmaHiState::enableController() {
    cmd_sc_ = 2;
    return true;
}

bool VelmaHiState::switchBehavior(const std::string &behavior_name) {
    Logger::In in("BehaviorManager::switchBehavior");
    for (int i = 0; i < bl_.behaviors.size(); ++i) {
        if (bl_.behaviors[i].name == behavior_name) {
            bool result = switchBlocks_(bl_.behaviors[i].stopped, bl_.behaviors[i].running, true, false);
            if (!result) {
                Logger::log() << Logger::Warning << "error in switchBlocks" << Logger::endl;
            }
            else {
                Logger::log() << Logger::Info << "switched to behavior: " << behavior_name << "  running: " << bl_.behaviors[i].running.size() << "  stopped: " << bl_.behaviors[i].stopped.size() << Logger::endl;
            }
            return result;
        }
    }
    Logger::log() << Logger::Warning << "wrong behavior name: " << behavior_name << Logger::endl;
    return false;
}

bool VelmaHiState::configureHook() {
    Logger::In in("BehaviorManager::configureHook");

    if (getPeriod() == 0) {
        Logger::log() << Logger::Error << "it should be a periodic activity" << Logger::endl;
        return false;
    }

    BehaviorsDesc bd;

    bd.parse(behaviors_str_);
    bl_ = bd.bl_;

    if (bl_.behaviors.size() <= 0) {
        Logger::log() << Logger::Error << "wrong number of behaviors: " << bl_.behaviors.size() << Logger::endl;
        return false;
    }

    Logger::log() << Logger::Info << "number of behaviors: " << bl_.behaviors.size() << Logger::endl;
    
    for (int i = 0; i < bl_.behaviors.size(); ++i) {
        Logger::log() << Logger::Info << "behavior: " << bl_.behaviors[i].name << Logger::endl;
    }


    TaskContext::PeerList l = this->getPeerList();
    if (l.size() != 1) {
        Logger::log() << Logger::Error << "wrong number of peers: " << l.size() << ", should be 1" << Logger::endl;
        return false;
    }

    TaskContext::PeerList::const_iterator it = l.begin();
    scheme_ = this->getPeer( (*it) );

    RTT::OperationInterfacePart *switchBlocksOp = scheme_->getOperation("switchBlocks");
    if (switchBlocksOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << (*it) << " has no matching operation switchBlocks" << Logger::endl;
        return false;
    }

    switchBlocks_ = RTT::OperationCaller<bool(std::vector<std::string>&, std::vector<std::string>&, bool, bool)>(
        switchBlocksOp, scheme_->engine());

    RTT::OperationInterfacePart *hasBlockOp = scheme_->getOperation("hasBlock");
    if (hasBlockOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << (*it) << " has no matching operation hasBlock" << Logger::endl;
        return false;
    }

    hasBlock_ =  RTT::OperationCaller<bool(const std::string &)>(
        hasBlockOp, scheme_->engine());

    for (int i = 0; i < bl_.switchable_components.size(); ++i) {
        if (!hasBlock_(bl_.switchable_components[i])) {
            Logger::log() << Logger::Error << "could not find a component: " << bl_.switchable_components[i] << " in the blocks list" << Logger::endl;
            return false;
        }
    }

    return true;
}

bool VelmaHiState::startHook() {
    Logger::In in("BehaviorManager::startHook");

    if (!switchBehavior(bl_.initial_behavior)) {
        Logger::log() << Logger::Error << "could not execute switchBehavior for initial behavior: " << bl_.initial_behavior << Logger::endl;
        return false;
    }

    if (!scheme_->isRunning()) {
        Logger::log() << Logger::Error << "conman should be in running state" << Logger::endl;
        return false;
    }

    return true;
}

void VelmaHiState::stopHook() {
}

void VelmaHiState::updateHook() {
    Logger::In in("BehaviorManager::updateHook");

    if (cmd_sc_ == 1 || cmd_sc_ == 2) {
        port_cmd_sc_out_.write(cmd_sc_);
        cmd_sc_ = -1;
    }
    scheme_->getActivity()->trigger();
}

ORO_LIST_COMPONENT_TYPE(VelmaHiState)
ORO_CREATE_COMPONENT_LIBRARY();

