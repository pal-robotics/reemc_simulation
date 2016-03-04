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

#include <cassert>
#include <boost/foreach.hpp>

#include <gazebo/sensors/SensorManager.hh>

#include <urdf_parser/urdf_parser.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

#include <joint_limits_interface/joint_limits_urdf.h>

#include <reemc_hardware_gazebo/reemc_hardware_gazebo.h>

using std::vector;
using std::string;

namespace reemc_hardware_gazebo
{

using namespace hardware_interface;

ReemcHardwareGazebo::ReemcHardwareGazebo()
  : gazebo_ros_control::RobotHWSim()
{}

bool ReemcHardwareGazebo::initSim(const std::string& robot_ns,
    ros::NodeHandle nh, gazebo::physics::ModelPtr model,
    const urdf::Model* const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  using gazebo::physics::JointPtr;

  // Wait for robot model to become available
  const string robot_description_name = "robot_description";
  string robot_description;
  while (ros::ok() && !nh.getParam(robot_description_name, robot_description))
  {
    ROS_WARN_STREAM_ONCE("Waiting for robot description: parameter '" << robot_description_name << "' on namespace '" << nh.getNamespace() << "'.");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("Found robot description");

  boost::shared_ptr<urdf::ModelInterface> urdf = urdf::parseURDF(robot_description);
  if (!urdf)
  {
    throw std::runtime_error("Could not load robot description.");
  }
  ROS_DEBUG("Parsed robot description");

  // Cleanup
  sim_joints_.clear();
  jnt_pos_.clear();
  jnt_vel_.clear();
  jnt_eff_.clear();
  jnt_pos_cmd_.clear();
  jnt_curr_limit_cmd_.clear();

  // Simulation joints: All joints to control
  sim_joints_ = model->GetJoints();
  n_dof_ = sim_joints_.size();

  vector<string> jnt_names;
  for (size_t i = 0; i < n_dof_; ++i)
  {
    jnt_names.push_back(sim_joints_[i]->GetName());
  }

  // Raw data
  jnt_pos_.resize(n_dof_);
  jnt_vel_.resize(n_dof_);
  jnt_eff_.resize(n_dof_);
  jnt_pos_cmd_.resize(n_dof_);
  jnt_curr_limit_cmd_.resize(n_dof_, 1.0);
  /// Retrieving max joint effort from urdf because values are not set in sim_joints_
  jnt_max_effort_.resize(n_dof_);
  for(size_t j=0; j< n_dof_; ++j)
    jnt_max_effort_[j] = urdf->getJoint(sim_joints_[j]->GetName())->limits->effort;

  // Hardware interfaces: joints
  for (size_t i = 0; i < n_dof_; ++i)
  {
    jnt_state_interface_.registerHandle(JointStateHandle(jnt_names[i], &jnt_pos_[i], &jnt_vel_[i], &jnt_eff_[i]));
    jnt_pos_cmd_interface_.registerHandle(JointHandle(jnt_state_interface_.getHandle(jnt_names[i]), &jnt_pos_cmd_[i]));
    ROS_DEBUG_STREAM("Registered joint '" << jnt_names[i] << "' in the PositionJointInterface.");

    act_state_interface_.registerHandle(ActuatorStateHandle(jnt_names[i], &jnt_pos_[i], &jnt_vel_[i], &jnt_eff_[i]));
    jnt_curr_limit_cmd_interface_.registerHandle(ActuatorHandle(act_state_interface_.getHandle(jnt_names[i]), &jnt_curr_limit_cmd_[i]));
  }
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_cmd_interface_);
  registerInterface(&jnt_curr_limit_cmd_interface_);

  // Joint limits interface
  vector<string> cmd_handle_names = jnt_pos_cmd_interface_.getNames();
  for (unsigned int i = 0; i < cmd_handle_names.size(); ++i)
  {
    JointHandle cmd_handle = jnt_pos_cmd_interface_.getHandle(cmd_handle_names[i]);
    const string name = cmd_handle.getName();

    using namespace joint_limits_interface;
    boost::shared_ptr<const urdf::Joint> urdf_joint = urdf->getJoint(name);
    JointLimits limits;
    SoftJointLimits soft_limits;
    if (!getJointLimits(urdf_joint, limits) || !getSoftJointLimits(urdf_joint, soft_limits))
    {
      ROS_WARN_STREAM("Joint limits won't be enforced for joint '" << name << "'.");
      continue;
    }
    jnt_limits_interface_.registerHandle(PositionJointSoftLimitsHandle(cmd_handle, limits, soft_limits));

    ROS_DEBUG_STREAM("Joint limits will be enforced for joint '" << name << "'.");
  }

  // Hardware interfaces: Ankle force-torque sensors
  const string left_ankle_name  = "leg_left_6_joint";  // TODO: Make not hardcoded
  const string right_ankle_name = "leg_right_6_joint";
  const string left_wrist_name  = "arm_left_7_joint";  // TODO: TUM check
  const string right_wrist_name = "arm_right_7_joint"; // TODO: TUM check

  left_ankle_  = model->GetJoint(left_ankle_name);
  right_ankle_ = model->GetJoint(right_ankle_name);
  left_wrist_  = model->GetJoint(left_wrist_name);
  right_wrist_ = model->GetJoint(right_wrist_name);

  if (!left_ankle_)
  {
    ROS_ERROR_STREAM("Could not find joint '" << left_ankle_name << "' to which a force-torque sensor is attached.");
    return false;
  }
  if (!right_ankle_)
  {
    ROS_ERROR_STREAM("Could not find joint '" << left_ankle_name << "' to which a force-torque sensor is attached.");
    return false;
  }
  if (!left_wrist_)
  {
    ROS_ERROR_STREAM("Could not find joint '" << left_wrist_name << "' to which a force-torque sensor is attached.");
    return false;
  }
  if (!right_wrist_)
  {
    ROS_ERROR_STREAM("Could not find joint '" << left_wrist_name << "' to which a force-torque sensor is attached.");
    return false;
  }

  ft_sensor_interface_.registerHandle(ForceTorqueSensorHandle("left_ankle_ft",         // TODO: Fetch from elsewhere?
                                                              "left_sole_link", // TODO: Fetch from URDF?
                                                              &l_ankle_force_[0],
                                                              &l_ankle_torque_[0]));

  ft_sensor_interface_.registerHandle(ForceTorqueSensorHandle("right_ankle_ft",         // TODO: Fetch from elsewhere?
                                                              "right_sole_link", // TODO: Fetch from URDF?
                                                              &r_ankle_force_[0],
                                                              &r_ankle_torque_[0]));

  ft_sensor_interface_.registerHandle(ForceTorqueSensorHandle("left_wrist_ft",         // TODO: Fetch from elsewhere?
                                                              "wrist_left_ft_link", // TODO: Fetch from URDF?
                                                              &l_wrist_force_[0],
                                                              &l_wrist_torque_[0]));

  ft_sensor_interface_.registerHandle(ForceTorqueSensorHandle("right_wrist_ft",         // TODO: Fetch from elsewhere?
                                                              "wrist_right_ft_link", // TODO: Fetch from URDF?
                                                              &r_wrist_force_[0],
                                                              &r_wrist_torque_[0]));
  registerInterface(&ft_sensor_interface_);
  ROS_DEBUG_STREAM("Registered ankle force-torque sensors.");

  // Hardware interfaces: Base IMU sensors
  imu_sensor_ =  boost::shared_dynamic_cast<gazebo::sensors::ImuSensor>
      (gazebo::sensors::SensorManager::Instance()->GetSensor("imu_sensor")); // TODO: Fetch from URDF? /*reemc::reemc::base_link::imu_sensor*/
  if (!this->imu_sensor_)
  {
    ROS_ERROR_STREAM("Could not find base IMU sensor.");
    return false;
  }

  ImuSensorHandle::Data data;
  data.name = "base_imu";      // TODO: Fetch from elsewhere?
  data.frame_id = "base_link"; // TODO: Fetch from URDF?
  data.orientation = &base_orientation_[0];
  imu_sensor_interface_.registerHandle(ImuSensorHandle(data));
  registerInterface(&imu_sensor_interface_);
  ROS_DEBUG_STREAM("Registered IMU sensor.");

  // PID controllers
  pids_.resize(n_dof_);
  for (size_t i = 0; i < n_dof_; ++i)
  {
    ros::NodeHandle joint_nh(nh, "gains/" + jnt_names[i]);
    if (!pids_[i].init(joint_nh)) {return false;}
  }

  return true;
}

void ReemcHardwareGazebo::readSim(ros::Time time, ros::Duration period)
{
  // Read joint state
  for(unsigned int j = 0; j < n_dof_; ++j)
  {
    // Gazebo has an interesting API...
    jnt_pos_[j] += angles::shortest_angular_distance
      (jnt_pos_[j], sim_joints_[j]->GetAngle(0u).Radian());
    jnt_vel_[j] = sim_joints_[j]->GetVelocity(0u);
    jnt_eff_[j] = sim_joints_[j]->GetForce(0u);
  }

  // Read force-torque sensors
  // Signs are coherent with FT sensor readings got from REEM-B
  gazebo::physics::JointWrench left_ankle_ft = left_ankle_->GetForceTorque(0u);
  l_ankle_force_[0]  = -left_ankle_ft.body1Force.y;
  l_ankle_force_[1]  = -left_ankle_ft.body1Force.z;
  l_ankle_force_[2]  =  left_ankle_ft.body1Force.x;
  l_ankle_torque_[0] = -left_ankle_ft.body1Torque.x;
  l_ankle_torque_[1] = -left_ankle_ft.body1Torque.z;
  l_ankle_torque_[2] =  left_ankle_ft.body1Torque.y;

  gazebo::physics::JointWrench right_ankle_ft = right_ankle_->GetForceTorque(0u);
  r_ankle_force_[0]  = -right_ankle_ft.body1Force.y;
  r_ankle_force_[1]  = -right_ankle_ft.body1Force.z;
  r_ankle_force_[2]  =  right_ankle_ft.body1Force.x;
  r_ankle_torque_[0] = -right_ankle_ft.body1Torque.x;
  r_ankle_torque_[1] = -right_ankle_ft.body1Torque.z;
  r_ankle_torque_[2] =  right_ankle_ft.body1Torque.y;

  // TODO: TUM, check directions and signs
  gazebo::physics::JointWrench left_wrist_ft = left_wrist_->GetForceTorque(0u);
  l_wrist_force_[0]  = left_wrist_ft.body2Force.x;
  l_wrist_force_[1]  = -left_wrist_ft.body2Force.y;
  l_wrist_force_[2]  = -left_wrist_ft.body2Force.z;
  l_wrist_torque_[0] = left_wrist_ft.body2Torque.x;
  l_wrist_torque_[1] = left_wrist_ft.body2Torque.y;
  l_wrist_torque_[2] = left_wrist_ft.body2Torque.z;

  gazebo::physics::JointWrench right_wrist_ft = right_wrist_->GetForceTorque(0u);
  r_wrist_force_[0]  = -right_wrist_ft.body2Force.x;
  r_wrist_force_[1]  = -right_wrist_ft.body2Force.y;
  r_wrist_force_[2]  =  right_wrist_ft.body2Force.z;
  r_wrist_torque_[0] =  right_wrist_ft.body2Torque.x;
  r_wrist_torque_[1] =  right_wrist_ft.body2Torque.y;
  r_wrist_torque_[2] =  right_wrist_ft.body2Torque.z;

  // Read IMU sensor
  gazebo::math::Quaternion imu_quat = imu_sensor_->GetOrientation();
  base_orientation_[0] = imu_quat.x;
  base_orientation_[1] = imu_quat.y;
  base_orientation_[2] = imu_quat.z;
  base_orientation_[3] = imu_quat.w;

  gazebo::math::Vector3 imu_ang_vel = imu_sensor_->GetAngularVelocity();
  base_ang_vel_[0] = imu_ang_vel.x;
  base_ang_vel_[1] = imu_ang_vel.y;
  base_ang_vel_[2] = imu_ang_vel.z;

  gazebo::math::Vector3 imu_lin_acc = imu_sensor_->GetLinearAcceleration();
  base_lin_acc_[0] =  imu_lin_acc.x;
  base_lin_acc_[1] =  imu_lin_acc.y;
  base_lin_acc_[2] =  imu_lin_acc.z;
}

void ReemcHardwareGazebo::writeSim(ros::Time time, ros::Duration period)
{
  // Enforce joint limits
//   jnt_limits_interface_.enforceLimits(period); // TODO: Tune controllers to make this work?

  // Compute and send effort command
  for(unsigned int j = 0; j < n_dof_; ++j)
  {
    const double error = jnt_pos_cmd_[j] - jnt_pos_[j]; // NOTE: Assumes jnt_pos_ contains most recent value
    const double effort = pids_[j].computeCommand(error, period);

    const double max_effort = jnt_curr_limit_cmd_[j]*jnt_max_effort_[j];
    const double min_effort = -max_effort;
    double effort_modified = (effort - max_effort) > 1e-4  ? max_effort : effort;
    effort_modified = effort_modified - min_effort < -1e-4 ? min_effort : effort_modified;

    // Gazebo has an interesting API...
      sim_joints_[j]->SetForce(0u, effort_modified);
  }

}

} // reemc_hardware_gazebo

PLUGINLIB_EXPORT_CLASS( reemc_hardware_gazebo::ReemcHardwareGazebo, gazebo_ros_control::RobotHWSim)
