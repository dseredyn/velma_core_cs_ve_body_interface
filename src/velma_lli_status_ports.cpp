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

#include "velma_lli_status_ports.h"

namespace velma_lli_types {

// port data specialized constructor
template <> ArmStatus_Ports<Data>::ArmStatus_Ports() {
    JointPosition_.resize(7);
    JointPosition_.resize(7);
    JointVelocity_.resize(7);
    JointTorque_.resize(7);
    GravityTorque_.resize(7);
}

// input port specialized constructor
template <> ArmStatus_Ports<RTT::InputPort>::ArmStatus_Ports(RTT::TaskContext &tc, const std::string &prefix) {
    tc.ports()->addPort(prefix + "_RobotState_INPORT",       RobotState_);
    tc.ports()->addPort(prefix + "_FRIState_INPORT",         FRIState_);
    tc.ports()->addPort(prefix + "_JointPosition_INPORT",    JointPosition_);
    tc.ports()->addPort(prefix + "_JointVelocity_INPORT",    JointVelocity_);
    tc.ports()->addPort(prefix + "_CartesianWrench_INPORT",  CartesianWrench_);
    tc.ports()->addPort(prefix + "_MassMatrix_INPORT",       MassMatrix_);
    tc.ports()->addPort(prefix + "_JointTorque_INPORT",      JointTorque_);
    tc.ports()->addPort(prefix + "_GravityTorque_INPORT",    GravityTorque_);
}

// output port specialized constructor
template <> ArmStatus_Ports<RTT::OutputPort>::ArmStatus_Ports(RTT::TaskContext &tc, const std::string &prefix) {
}

// read ports
template <> void ArmStatus_Ports<RTT::InputPort >::readPorts(ArmStatus_Ports<Data > &data) {
    RobotState_.read(data.RobotState_);
    FRIState_.read(data.FRIState_);
    JointPosition_.read(data.JointPosition_);
    JointVelocity_.read(data.JointVelocity_);
    CartesianWrench_.read(data.CartesianWrench_);
    MassMatrix_.read(data.MassMatrix_);
    JointTorque_.read(data.JointTorque_);
    GravityTorque_.read(data.GravityTorque_);
}

// write ports
template <> void ArmStatus_Ports<RTT::OutputPort >::writePorts(const ArmStatus_Ports<Data > &data) {
    RobotState_.write(data.RobotState_);
    FRIState_.write(data.FRIState_);
    JointPosition_.write(data.JointPosition_);
    JointVelocity_.write(data.JointVelocity_);
    CartesianWrench_.write(data.CartesianWrench_);
    MassMatrix_.write(data.MassMatrix_);
    JointTorque_.write(data.JointTorque_);
    GravityTorque_.write(data.GravityTorque_);
}



// port data specialized constructor
template <> HandStatus_Ports<Data>::HandStatus_Ports() {
    q_.resize(4);
    t_.resize(4);
}

// input port specialized constructor
template <> HandStatus_Ports<RTT::InputPort>::HandStatus_Ports(RTT::TaskContext &tc, const std::string &prefix) {
    tc.ports()->addPort(prefix + "_status_INPORT",  status_);
    tc.ports()->addPort(prefix + "_q_INPORT",       q_);
    tc.ports()->addPort(prefix + "_t_INPORT",       t_);
}

// output port specialized constructor
template <> HandStatus_Ports<RTT::OutputPort>::HandStatus_Ports(RTT::TaskContext &tc, const std::string &prefix) {
}

// read ports
template <> void HandStatus_Ports<RTT::InputPort >::readPorts(HandStatus_Ports<Data > &data) {
    status_.read(data.status_);
    q_.read(data.q_);
    t_.read(data.t_);
}

// write ports
template <> void HandStatus_Ports<RTT::OutputPort >::writePorts(const HandStatus_Ports<Data > &data) {
    status_.write(data.status_);
    q_.write(data.q_);
    t_.write(data.t_);
}




// port data specialized constructor
template <> FTSensorStatus_Ports<Data>::FTSensorStatus_Ports() {
}

// input port specialized constructor
template <> FTSensorStatus_Ports<RTT::InputPort>::FTSensorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix) {
    tc.ports()->addPort(prefix + "_RawWrench",              raw_wrench_);
    tc.ports()->addPort(prefix + "_FastFilteredWrench",     fast_filtered_wrench_);
    tc.ports()->addPort(prefix + "_SlowFilteredWrench",     slow_filtered_wrench_);
}

// output port specialized constructor
template <> FTSensorStatus_Ports<RTT::OutputPort>::FTSensorStatus_Ports(RTT::TaskContext &tc, const std::string &prefix) {
}

// read ports
template <> void FTSensorStatus_Ports<RTT::InputPort >::readPorts(FTSensorStatus_Ports<Data > &data) {
    raw_wrench_.read(data.raw_wrench_);
    fast_filtered_wrench_.read(data.fast_filtered_wrench_);
    slow_filtered_wrench_.read(data.slow_filtered_wrench_);
}

// write ports
template <> void FTSensorStatus_Ports<RTT::OutputPort >::writePorts(const FTSensorStatus_Ports<Data > &data) {
    raw_wrench_.write(data.raw_wrench_);
    fast_filtered_wrench_.write(data.fast_filtered_wrench_);
    slow_filtered_wrench_.write(data.slow_filtered_wrench_);
}





// port data specialized constructor
template <> VelmaStatus_Ports<Data>::VelmaStatus_Ports() :
    rArm_(),
    lArm_(),
    rHand_(),
    lHand_(),
    rFT_(),
    lFT_()
{
    tactile_.finger1_tip.resize(24);
    tactile_.finger2_tip.resize(24);
    tactile_.finger3_tip.resize(24);
    tactile_.palm_tip.resize(24);
}

// input port specialized constructor
template <> VelmaStatus_Ports<RTT::InputPort>::VelmaStatus_Ports(RTT::TaskContext &tc):
    rArm_(tc, "rArm"),
    lArm_(tc, "lArm"),
    rHand_(tc, "rHand"),
    lHand_(tc, "lHand"),
    rFT_(tc, "rFt)"),
    lFT_(tc, "lFt)")
{
    tc.ports()->addPort("BHPressureState_INPORT",       tactile_);
    tc.ports()->addPort("BHMaxPressure_INPORT",         max_pressure_);

    tc.ports()->addPort("Optoforce0_INPORT",            force_[0]);
    tc.ports()->addPort("Optoforce1_INPORT",            force_[1]);
    tc.ports()->addPort("Optoforce2_INPORT",            force_[2]);

    tc.ports()->addPort("torsoMotorPosition_INPORT",    t_MotorPosition_);
    tc.ports()->addPort("torsoMotorVelocity_INPORT",    t_MotorVelocity_);

    tc.ports()->addPort("headPanMotorPosition_INPORT",  hp_q_);
    tc.ports()->addPort("headPanMotorVelocity_INPORT",  hp_v_);

    tc.ports()->addPort("headTiltMotorPosition_INPORT", ht_q_);
    tc.ports()->addPort("headTiltMotorVelocity_INPORT", ht_v_);
}

// output port specialized constructor
template <> VelmaStatus_Ports<RTT::OutputPort>::VelmaStatus_Ports(RTT::TaskContext &tc) :
    rArm_(tc, "rArm"),
    lArm_(tc, "lArm"),
    rHand_(tc, "rHand"),
    lHand_(tc, "lHand"),
    rFT_(tc, "rFt)"),
    lFT_(tc, "lFt)")
{
}

// read ports
template <> void VelmaStatus_Ports<RTT::InputPort >::readPorts(VelmaStatus_Ports<Data > &data) {
    rArm_.readPorts(data.rArm_);
    lArm_.readPorts(data.lArm_);

    rHand_.readPorts(data.rHand_);
    lHand_.readPorts(data.lHand_);

    tactile_.read(data.tactile_);
    max_pressure_.read(data.max_pressure_);

    rFT_.readPorts(data.rFT_);
    lFT_.readPorts(data.lFT_);

    force_[0].read(data.force_[0]);
    force_[1].read(data.force_[1]);
    force_[2].read(data.force_[2]);

    t_MotorPosition_.read(data.t_MotorPosition_);
    t_MotorVelocity_.read(data.t_MotorVelocity_);

    hp_q_.read(data.hp_q_);
    hp_v_.read(data.hp_v_);
    ht_q_.read(data.ht_q_);
    ht_v_.read(data.ht_v_);
}

// write ports
template <> void VelmaStatus_Ports<RTT::OutputPort >::writePorts(const VelmaStatus_Ports<Data > &data) {
    rArm_.writePorts(data.rArm_);
    lArm_.writePorts(data.lArm_);

    rHand_.writePorts(data.rHand_);
    lHand_.writePorts(data.lHand_);

    tactile_.write(data.tactile_);
    max_pressure_.write(data.max_pressure_);

    rFT_.writePorts(data.rFT_);
    lFT_.writePorts(data.lFT_);

    force_[0].write(data.force_[0]);
    force_[1].write(data.force_[1]);
    force_[2].write(data.force_[2]);

    t_MotorPosition_.write(data.t_MotorPosition_);
    t_MotorVelocity_.write(data.t_MotorVelocity_);

    hp_q_.write(data.hp_q_);
    hp_v_.write(data.hp_v_);
    ht_q_.write(data.ht_q_);
    ht_v_.write(data.ht_v_);
}

};

VelmaLLIStatusInput::VelmaLLIStatusInput(RTT::TaskContext &tc) :
    in_(),
    ports_in_(tc)
{
}

void VelmaLLIStatusInput::readPorts(velma_low_level_interface_msgs::VelmaLowLevelStatus &status) {
    ports_in_.readPorts(in_);
/*
    in_.rArm_.
    RobotState_.read(data.RobotState_);
    FRIState_.read(data.FRIState_);
    JointPosition_.read(data.JointPosition_);
    JointVelocity_.read(data.JointVelocity_);
    CartesianWrench_.read(data.CartesianWrench_);
    MassMatrix_.read(data.MassMatrix_);
    JointTorque_.read(data.JointTorque_);
    GravityTorque_.read(data.GravityTorque_);
*/

/*
# leftArmJointPosition
# rightArmJointPosition
float64[7] lArm_q
float64[7] rArm_q

# leftArmJointVelocity
# rightArmJointVelocity
float64[7] lArm_dq
float64[7] rArm_dq

# leftArmJointTorque
# rightArmJointTorque
float64[7] lArm_t
float64[7] rArm_t

# leftArmWrench
# rightArmWrench
float64[6] lArm_w
float64[6] rArm_w

# leftArmMassMatrix
# rightArmMassMatrix
float64[28] lArm_mm
float64[28] rArm_mm

# leftArmGravityTorque
# rightArmGravityTorque
float64[7] lArm_gt
float64[7] rArm_gt


# leftHandStatus
# rightHandStatus
int32 lHand_s
int32 rHand_s

# leftHandJointPosition
# rightHandJointPosition
float64 lHand_q
float64 rHand_q

# RightHandTactile.BHPressureState_OUTPORT
barrett_hand_controller_msgs/BHPressureState rHand_p

# LeftHandOptoforce.force_0_OUTPORT
# LeftHandOptoforce.force_1_OUTPORT
# LeftHandOptoforce.force_2_OUTPORT
geometry_msgs/WrenchStamped[3] lHand_f

# torsoMotorPosition
float64 tMotor_q

# torsoMotorVelocity
float64 tMotor_dq

# headPanMotorPosition
float64 hpMotor_q

# headPanMotorVelocity
float64 hpMotor_dq

# headTiltMotorPosition
float64 htMotor_q

# headTiltMotorVelocity
float64 htMotor_dq


# leftFtSensorRawWrench
# rightFtSensorRawWrench
float64[6] lFTSensor_rw
float64[6] rFTSensor_rw

# leftFtSensorFastFilteredWrench
# rightFtSensorFastFilteredWrench
float64[6] lFTSensor_ffw
float64[6] rFTSensor_ffw

# leftFtSensorSlowFilteredWrench
# rightFtSensorSlowFilteredWrench
float64[6] lFTSensor_sfw
float64[6] rFTSensor_sfw

# leftArmRobotState_OUTPORT_name()           { return string("FRIl.RobotState_OUTPORT"); }
# leftArmFRIState_OUTPORT_name()             { return string("FRIl.FRIState_OUTPORT"); }
# rightArmRobotState_OUTPORT_name()          { return string("FRIr.RobotState_OUTPORT"); }
# rightArmFRIState_OUTPORT_name()            { return string("FRIr.FRIState_OUTPORT"); }
int32 robot_s
*/
}




VelmaLLIStatusOutput::VelmaLLIStatusOutput(RTT::TaskContext &tc) :
    out_(),
    ports_out_(tc)
{
}

void VelmaLLIStatusOutput::writePorts(const velma_low_level_interface_msgs::VelmaLowLevelStatus &status) {
    ports_out_.writePorts(out_);
}

