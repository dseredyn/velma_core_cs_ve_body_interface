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

#ifndef __VELMA_LLI_PORT_DATA_H__
#define __VELMA_LLI_PORT_DATA_H__

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

namespace Eigen {
    typedef Matrix<double, 7, 7> Matrix77d;
};

namespace velma_lli_types {

// general data type
template <typename innerT, typename rosT >
class PortRawData {
public:
    PortRawData() {
    }

    void convertFromROS(const rosT &ros) {
        data_ = ros;
    }

    void convertToROS(rosT &ros) {
        ros = data_;
    }

    innerT data_;
};

// specialized data type
template < >
class PortRawData<Eigen::VectorXd, boost::array<double, 7ul> > {
public:
    PortRawData() : data_(7) {
    }

    void convertFromROS(const boost::array<double, 7ul> &ros) {
        for (int i = 0; i < 7; ++i) {
            data_(i) = ros[i];
        }
    }

    void convertToROS(boost::array<double, 7ul> &ros) {
        for (int i = 0; i < 7; ++i) {
            ros[i] = data_(i);
        }
    }

    Eigen::VectorXd data_;
};

template < >
class PortRawData<Eigen::VectorXd, boost::array<double, 4ul> > {
public:
    PortRawData() : data_(4) {
    }

    void convertFromROS(const boost::array<double, 4ul> &ros) {
        for (int i = 0; i < 4; ++i) {
            data_(i) = ros[i];
        }
    }

    void convertToROS(boost::array<double, 4ul> &ros) {
        for (int i = 0; i < 4; ++i) {
            ros[i] = data_(i);
        }
    }

    Eigen::VectorXd data_;
};

template < >
class PortRawData<Eigen::VectorXd, boost::array<double, 8ul> > {
public:
    PortRawData() : data_(8) {
    }

    void convertFromROS(const boost::array<double, 8ul> &ros) {
        for (int i = 0; i < 8; ++i) {
            data_(i) = ros[i];
        }
    }

    void convertToROS(boost::array<double, 8ul> &ros) {
        for (int i = 0; i < 8; ++i) {
            ros[i] = data_(i);
        }
    }

    Eigen::VectorXd data_;
};

// 7x7 mass matrix
template < >
class PortRawData<Eigen::Matrix77d, boost::array<double, 28ul> > {
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

    Eigen::Matrix77d data_;
};

};  // namespace velma_lli_types

#endif  // __VELMA_LLI_PORT_DATA_H__

