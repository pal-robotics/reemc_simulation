///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#ifndef REEMC_HARDWARE_GAZEBO_REEMC_HARDWARE_GAZEBO_H
#define REEMC_HARDWARE_GAZEBO_REEMC_HARDWARE_GAZEBO_H

#include <vector>
#include <string>

#include <control_toolbox/pid.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_mode_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <pal_ros_control/pal_actuator_command_interface.h>
#include <gazebo_ros_control/robot_hw_sim.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/ImuSensor.hh>

namespace reemc_hardware_gazebo
{

class ReemcHardwareGazebo : public gazebo_ros_control::RobotHWSim
{
public:

  ReemcHardwareGazebo();

  // Simulation-specific
  bool initSim(const std::string& robot_ns,
               ros::NodeHandle nh,
               gazebo::physics::ModelPtr model,
               const urdf::Model* const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions);
  void readSim(ros::Time time, ros::Duration period);
  void writeSim(ros::Time time, ros::Duration period);

private:

  // Raw data
  unsigned int n_dof_;

  std::vector<double> jnt_pos_;
  std::vector<double> jnt_vel_;
  std::vector<double> jnt_eff_;

  std::vector<double> jnt_pos_cmd_;
  std::vector<double> jnt_eff_cmd_;
  std::vector<int>    jnt_mode_cmd_;
  std::vector<double> jnt_curr_limit_cmd_;
  std::vector<double> jnt_max_effort_;

  double left_force_[3];
  double left_torque_[3];
  double right_force_[3];
  double right_torque_[3];

  double base_orientation_[4];
  double base_ang_vel_[3];
  double base_lin_acc_[3];

  // Simulation-specific
  std::vector<gazebo::physics::JointPtr> sim_joints_;
  gazebo::physics::JointPtr left_ankle_;
  gazebo::physics::JointPtr right_ankle_;
  boost::shared_ptr<gazebo::sensors::ImuSensor> imu_sensor_;

  // Hardware interface: joints
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_cmd_interface_;
  hardware_interface::EffortJointInterface   jnt_eff_cmd_interface_;
  hardware_interface::JointModeInterface     jnt_mode_cmd_interface_;
  hardware_interface::ActuatorStateInterface    act_state_interface_;
  hardware_interface::CurrentLimitActuatorInterface jnt_curr_limit_cmd_interface_;

  // Hardware interface: sensors
  hardware_interface::ForceTorqueSensorInterface ft_sensor_interface_;
  hardware_interface::ImuSensorInterface         imu_sensor_interface_;

  // Joint limits interface
  joint_limits_interface::PositionJointSoftLimitsInterface jnt_limits_interface_;

  // Guess what
  std::map<std::string, hardware_interface::JointCommandModes> joint_modes_;

  // PID controllers
  std::vector<control_toolbox::Pid> pids_;

  void sendPosition(unsigned int j, ros::Duration period);
};

}

#endif // REEMC_HARDWARE_GAZEBO_REEMC_HARDWARE_GAZEBO_H
