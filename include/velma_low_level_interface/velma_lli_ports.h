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

#ifndef __VELMA_LLI_PORTS_H__
#define __VELMA_LLI_PORTS_H__

#include <cstring>

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include "rtt/os/TimeService.hpp"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <std_msgs/Int32.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include "eigen_conversions/eigen_msg.h"

#include "velma_low_level_interface/velma_lli_port_data.h"

namespace velma_lli_types {

template <typename innerT, typename rosC, typename rosT, rosT rosC::*ptr >
class PortData {
public:
    PortData() {}

    void convertFromROS(const rosC &container) {
        data_.convertFromROS(container.*ptr);
    }

    void convertToROS(rosC &container) {
        data_.convertToROS(container.*ptr);
    }

    innerT& getDataRef() {
        return data_.data_;
    }

protected:
    PortRawData<innerT, rosT > data_;
};

template <template <typename Type> class T, typename innerT >
class PortOperation { };

template <typename innerT >
class PortOperation<RTT::InputPort, innerT> {
public:
    PortOperation(RTT::TaskContext &tc, const std::string &port_name) {
        tc.ports()->addPort(port_name + "_INPORT", port_);
    }

    bool operation(innerT &data) {
        return port_.read(data) == RTT::NewData;
    }

    void setDataSample(innerT &data) {
        // no operation for input port
    }

protected:
    RTT::InputPort<innerT > port_;
};

template <typename innerT >
class PortOperation<RTT::OutputPort, innerT> {
public:
    PortOperation(RTT::TaskContext &tc, const std::string &port_name) {
        tc.ports()->addPort(port_name + "_OUTPORT", port_);
    }

    bool operation(innerT &data) {
        port_.write(data);
        return true;
    }

    void setDataSample(innerT &data) {
        port_.setDataSample(data);
    }

protected:
    RTT::OutputPort<innerT > port_;
};


template <template <typename Type> class T, typename innerT, typename rosC, typename rosT, rosT rosC::*ptr >
class Port {
public:
    Port(RTT::TaskContext &tc, const std::string &port_name) :
        po_(tc, port_name)
    {
        po_.setDataSample(data_.getDataRef());
    }

    void convertFromROS(const rosC &container) {
        data_.convertFromROS(container);
    }

    void convertToROS(rosC &container) {
        data_.convertToROS(container);
    }

    bool operation() {
        return po_.operation(data_.getDataRef());
    }

protected:

    PortOperation<T, innerT> po_;

    PortData<innerT, rosC, rosT, ptr > data_;
};

};  // namespace velma_lli_types

#endif  // __VELMA_LLI_PORTS_H__

