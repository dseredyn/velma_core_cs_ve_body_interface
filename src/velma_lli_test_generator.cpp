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

#include "velma_lli_test_generator.h"

VelmaLLITestGenerator::VelmaLLITestGenerator() {
    srand(0);
}

void VelmaLLITestGenerator::generate(uint32_t seed, velma_low_level_interface_msgs::VelmaLowLevelCommand &cmd, velma_low_level_interface_msgs::VelmaLowLevelStatus &status) {

    srand(seed);

    cmd.test = seed;

    // random command
    cmd.rHand_tactileCmd = static_cast<int32_t >(rand());
    cmd.tMotor_i = static_cast<double >(rand());
    cmd.hpMotor_i = static_cast<double >(rand());
    cmd.htMotor_i = static_cast<double >(rand());
    cmd.hpMotor_q = static_cast<double >(rand());
    cmd.htMotor_q = static_cast<double >(rand());
    cmd.hpMotor_dq = static_cast<double >(rand());
    cmd.htMotor_dq = static_cast<double >(rand());

    for (int i = 0; i < 7; ++i) {
        cmd.lArm.t[i] = static_cast<double >(rand());
        cmd.rArm.t[i] = static_cast<double >(rand());
    }
    cmd.lArm.cmd.data = static_cast<int32_t >(rand());
    cmd.lArm.cmd_valid = true;
    cmd.rArm.cmd.data = static_cast<int32_t >(rand());
    cmd.rArm.cmd_valid = true;

    for (int i = 0; i < 4; ++i) {
        cmd.lHand.q[i] = static_cast<double >(rand());
        cmd.lHand.dq[i] = static_cast<double >(rand());
        cmd.lHand.max_p[i] = static_cast<double >(rand());
        cmd.lHand.max_i[i] = static_cast<double >(rand());
        cmd.rHand.q[i] = static_cast<double >(rand());
        cmd.rHand.dq[i] = static_cast<double >(rand());
        cmd.rHand.max_p[i] = static_cast<double >(rand());
        cmd.rHand.max_i[i] = static_cast<double >(rand());
    }

    cmd.lHand.hold = static_cast<bool >(rand()%2);
    cmd.rHand.hold = static_cast<bool >(rand()%2);
    cmd.lHand_valid = true;
    cmd.rHand_valid = true;

    // random status
    status.test = seed;

    for (int i = 0; i < 4; ++i) {
        status.lHand.q[i] = static_cast<double >(rand());
        status.rHand.q[i] = static_cast<double >(rand());
    }
    status.lHand.s = static_cast<uint32_t >(rand());
    status.rHand.s = static_cast<uint32_t >(rand());

    for (int i = 0; i < 7; ++i) {
        status.lArm.q[i] = static_cast<double >(rand());
        status.lArm.dq[i] = static_cast<double >(rand());
        status.lArm.t[i] = static_cast<double >(rand());
        status.lArm.gt[i] = static_cast<double >(rand());
        status.rArm.q[i] = static_cast<double >(rand());
        status.rArm.dq[i] = static_cast<double >(rand());
        status.rArm.t[i] = static_cast<double >(rand());
        status.rArm.gt[i] = static_cast<double >(rand());
    }

    for (int i = 0; i < 28; ++i) {
        status.lArm.mmx[i] = static_cast<double >(rand());
        status.rArm.mmx[i] = static_cast<double >(rand());
    }

    for (int i = 0; i < 40; ++i) {
        status.lArm.friIntfState[i] = static_cast<uint8_t >(rand());
        status.rArm.friIntfState[i] = static_cast<uint8_t >(rand());
    }

    for (int i = 0; i < 36; ++i) {
        status.lArm.friRobotState[i] = static_cast<uint8_t >(rand());
        status.rArm.friRobotState[i] = static_cast<uint8_t >(rand());
    }

    status.lArm.w.force.x = static_cast<double >(rand());
    status.lArm.w.force.y = static_cast<double >(rand());
    status.lArm.w.force.z = static_cast<double >(rand());
    status.lArm.w.torque.x = static_cast<double >(rand());
    status.lArm.w.torque.y = static_cast<double >(rand());
    status.lArm.w.torque.z = static_cast<double >(rand());

    status.tMotor_q = static_cast<double >(rand());
    status.tMotor_dq = static_cast<double >(rand());
    status.hpMotor_q = static_cast<double >(rand());
    status.hpMotor_dq = static_cast<double >(rand());
    status.htMotor_q = static_cast<double >(rand());
    status.htMotor_dq = static_cast<double >(rand());

    for (int i = 0; i < 24; ++i) {
        status.rHand_p.finger1_tip[i] = static_cast<int16_t >(rand());
        status.rHand_p.finger2_tip[i] = static_cast<int16_t >(rand());
        status.rHand_p.finger3_tip[i] = static_cast<int16_t >(rand());
        status.rHand_p.palm_tip[i] = static_cast<int16_t >(rand());
    }

    for (int i = 0; i < 3; ++i) {
        status.lHand_f[i].wrench.force.x = static_cast<double >(rand());
        status.lHand_f[i].wrench.force.y = static_cast<double >(rand());
        status.lHand_f[i].wrench.force.z = static_cast<double >(rand());
        status.lHand_f[i].wrench.torque.x = static_cast<double >(rand());
        status.lHand_f[i].wrench.torque.y = static_cast<double >(rand());
        status.lHand_f[i].wrench.torque.z = static_cast<double >(rand());
    }

    status.rFt.rw.force.x = static_cast<double >(rand());
    status.rFt.rw.force.y = static_cast<double >(rand());
    status.rFt.rw.force.z = static_cast<double >(rand());
    status.rFt.rw.torque.x = static_cast<double >(rand());
    status.rFt.rw.torque.y = static_cast<double >(rand());
    status.rFt.rw.torque.z = static_cast<double >(rand());

    status.lFt.rw.force.x = static_cast<double >(rand());
    status.lFt.rw.force.y = static_cast<double >(rand());
    status.lFt.rw.force.z = static_cast<double >(rand());
    status.lFt.rw.torque.x = static_cast<double >(rand());
    status.lFt.rw.torque.y = static_cast<double >(rand());
    status.lFt.rw.torque.z = static_cast<double >(rand());
}

std::string VelmaLLITestGenerator::toStr(const velma_low_level_interface_msgs::VelmaLowLevelCommand &cmd) {
    std::ostringstream os;
    os << cmd;
    return os.str();
}

std::string VelmaLLITestGenerator::toStr(const velma_low_level_interface_msgs::VelmaLowLevelStatus &status) {
    std::ostringstream os;
    os << status;
    return os.str();
}

