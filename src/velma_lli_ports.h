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

#ifndef VELMA_LLI_PORTS_H_
#define VELMA_LLI_PORTS_H_

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

#include <kuka_lwr_fri/friComm.h>

#include "eigen_conversions/eigen_msg.h"

namespace velma_lli_types {

typedef Eigen::Matrix<double, 7, 7> Matrix77d;

template <typename innerT, typename rosT >
class PortRawData {
public:
    PortRawData();
    void convertFromROS(const rosT &ros);
    void convertToROS(rosT &ros);
    innerT data_;
};

template <typename innerT, typename rosC, typename rosT, rosT rosC::*ptr >
class PortData {
public:
    PortData(rosC &container);
    void convertFromROS();
    void convertToROS();
    innerT& getDataRef();
protected:
    rosC &container_;
    PortRawData<innerT, rosT > data_;
};

template <template <typename Type> class T >
class PortSuffix {
public:
    PortSuffix();
    std::string str_;
};

template <template <typename Type> class T, typename innerT >
class PortOperation { };

template <typename innerT >
class PortOperation<RTT::InputPort, innerT> {
public:
    PortOperation(RTT::TaskContext &tc, const std::string &port_name);
    void operation(innerT &data);
    void setDataSample(innerT &data);

protected:
    RTT::InputPort<innerT > port_;
};

template <typename innerT >
class PortOperation<RTT::OutputPort, innerT> {
public:
    PortOperation(RTT::TaskContext &tc, const std::string &port_name);
    void operation(innerT &data);
    void setDataSample(innerT &data);

protected:
    RTT::OutputPort<innerT > port_;
};


template <template <typename Type> class T, typename innerT, typename rosC, typename rosT, rosT rosC::*ptr >
class Port {
public:
    Port(RTT::TaskContext &tc, const std::string &port_name, rosC &container);
    void convertFromROS();
    void convertToROS();
    void operation();

protected:

    rosC &container_;

    PortOperation<T, innerT> po_;

    PortSuffix<T > port_suffix_;
    PortData<innerT, rosC, rosT, ptr > data_;
};

};  // namespace velma_lli_types

#endif  // VELMA_LLI_PORTS_H_

