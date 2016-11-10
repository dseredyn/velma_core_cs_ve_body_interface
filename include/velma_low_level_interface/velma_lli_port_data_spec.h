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

#ifndef __VELMA_LLI_PORT_DATA_SPEC_H__
#define __VELMA_LLI_PORT_DATA_SPEC_H__

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

// specialized data type: array of double
template <int SIZE>
class PortRawData<Eigen::Matrix<double,SIZE,1>, boost::array<double, SIZE> > {
public:
    PortRawData() { }

    void convertFromROS(const boost::array<double, SIZE> &ros) {
        for (int i = 0; i < SIZE; ++i) {
            data_(i) = ros[i];
        }
    }

    void convertToROS(boost::array<double, SIZE> &ros) {
        for (int i = 0; i < SIZE; ++i) {
            ros[i] = data_(i);
        }
    }

    Eigen::Matrix<double,SIZE,1> data_;
};

template class PortRawData<Eigen::Matrix<double,4,1>, boost::array<double, 4> >;
template class PortRawData<Eigen::Matrix<double,7,1>, boost::array<double, 7> >;
template class PortRawData<Eigen::Matrix<double,8,1>, boost::array<double, 8> >;

/*
// specialized data type: array of double
template <typename rosT>
class PortRawData<Eigen::Matrix<double,7,1>, rosT > {
public:
    PortRawData() { }

    void convertFromROS(const rosT &ros) {
        for (int i = 0; i < rosT::size(); ++i) {
            data_(i) = ros[i];
        }
    }

    void convertToROS(rosT &ros) {
        for (int i = 0; i < rosT::size(); ++i) {
            ros[i] = data_(i);
        }
    }

    Eigen::Matrix<double,7,1> data_;
};

template <typename rosT>
class PortRawData<Eigen::Matrix<double,4,1>, rosT > {
public:
    PortRawData() { }

    void convertFromROS(const rosT &ros) {
        for (int i = 0; i < rosT::size(); ++i) {
            data_(i) = ros[i];
        }
    }

    void convertToROS(rosT &ros) {
        for (int i = 0; i < rosT::size(); ++i) {
            ros[i] = data_(i);
        }
    }

    Eigen::Matrix<double,4,1> data_;
};

template <typename rosT>
class PortRawData<Eigen::Matrix<double,8,1>, rosT > {
public:
    PortRawData() { }

    void convertFromROS(const rosT &ros) {
        for (int i = 0; i < rosT::size(); ++i) {
            data_(i) = ros[i];
        }
    }

    void convertToROS(rosT &ros) {
        for (int i = 0; i < rosT::size(); ++i) {
            ros[i] = data_(i);
        }
    }

    Eigen::Matrix<double,8,1> data_;
};
*/

// specialized data type: 7x7 mass matrix
template < >
class PortRawData<Eigen::Matrix<double, 7, 7>, boost::array<double, 28ul> > {
public:
    PortRawData() {
    }

    void convertFromROS(const boost::array<double, 28ul> &ros) {
        for (int i = 0, idx = 0; i < 7; ++i) {
            for (int j = i; j < 7; ++j) {
                data_(i,j) = data_(j,i) = ros[idx++];
            }
        }
    }

    void convertToROS(boost::array<double, 28ul> &ros) {
        for (int i = 0, idx = 0; i < 7; ++i) {
            for (int j = i; j < 7; ++j) {
                ros[idx++] = data_(i,j);
            }
        }
    }

    Eigen::Matrix<double, 7, 7> data_;
};

};  // namespace velma_lli_types

#endif  // __VELMA_LLI_PORT_DATA_SPEC_H__

