
#include <cassert>
#include <boost/foreach.hpp>

#include <pluginlib/class_list_macros.h>

#include <angles/angles.h>

#include <reemc_hardware_gazebo/reemc_hardware_gazebo.h>

// TODO: Remove!
//#include <transmission_interface/simple_transmission.h>
//#include <transmission_interface/differential_transmission.h>
//#include <transmission_interface/four_bar_linkage_transmission.h>

using std::vector;
using std::string;

namespace reemc_hardware_gazebo
{

ReemcHardwareGazebo::ReemcHardwareGazebo()
  : ros_control_gazebo::RobotSim() //hardware_interface::RobotHW(),
{
//  // Transmissions
//  using namespace transmission_interface;
//  TransmissionPtr wheel_1_trans(new SimpleTransmission(100.0));
//  TransmissionPtr wheel_2_trans(new SimpleTransmission(100.0));
//  TransmissionPtr torso_1_trans(new SimpleTransmission(100.0));
//  TransmissionPtr torso_2_trans(new SimpleTransmission(100.0));

//  transmissions_.push_back(wheel_1_trans);
//  transmissions_.push_back(wheel_2_trans);
//  transmissions_.push_back(torso_1_trans);
//  transmissions_.push_back(torso_2_trans);

//  // Transmission interface: actuator->joint state map
//  {
//    ActuatorData act_data;
//    act_data.position.push_back(&act_pos_[0]);
//    act_data.velocity.push_back(&act_vel_[0]);
//    act_data.effort.push_back(&act_eff_[0]);

//    JointData jnt_data;
//    jnt_data.position.push_back(&jnt_pos_[0]);
//    jnt_data.velocity.push_back(&jnt_vel_[0]);
//    jnt_data.effort.push_back(&jnt_eff_[0]);

//    act_to_jnt_state_.registerTransmission(transmission_names_[0], wheel_1_trans.get(), act_data, jnt_data);
//  }
//  {
//    ActuatorData act_data;
//    act_data.position.push_back(&act_pos_[1]);
//    act_data.velocity.push_back(&act_vel_[1]);
//    act_data.effort.push_back(&act_eff_[1]);

//    JointData jnt_data;
//    jnt_data.position.push_back(&jnt_pos_[1]);
//    jnt_data.velocity.push_back(&jnt_vel_[1]);
//    jnt_data.effort.push_back(&jnt_eff_[1]);

//    act_to_jnt_state_.registerTransmission(transmission_names_[1], wheel_2_trans.get(), act_data, jnt_data);
//  }
//  {
//    ActuatorData act_data;
//    act_data.position.push_back(&act_pos_[2]);
//    act_data.velocity.push_back(&act_vel_[2]);
//    act_data.effort.push_back(&act_eff_[2]);

//    JointData jnt_data;
//    jnt_data.position.push_back(&jnt_pos_[2]);
//    jnt_data.velocity.push_back(&jnt_vel_[2]);
//    jnt_data.effort.push_back(&jnt_eff_[2]);

//    act_to_jnt_state_.registerTransmission(transmission_names_[2], torso_1_trans.get(), act_data, jnt_data);
//  }
//  {
//    ActuatorData act_data;
//    act_data.position.push_back(&act_pos_[3]);
//    act_data.velocity.push_back(&act_vel_[3]);
//    act_data.effort.push_back(&act_eff_[3]);

//    JointData jnt_data;
//    jnt_data.position.push_back(&jnt_pos_[3]);
//    jnt_data.velocity.push_back(&jnt_vel_[3]);
//    jnt_data.effort.push_back(&jnt_eff_[3]);

//    act_to_jnt_state_.registerTransmission(transmission_names_[3], torso_2_trans.get(), act_data, jnt_data);
//  }

//  // Transmission interface: joint->actuator map of position commands
//  {
//    ActuatorData act_data;
//    act_data.position.push_back(&act_pos_cmd_[0]); // Velocity and effort vectors are unused

//    JointData jnt_data;
//    jnt_data.position.push_back(&jnt_pos_cmd_[0]); // Velocity and effort vectors are unused

//    jnt_to_act_pos_cmd_.registerTransmission(transmission_names_[2], torso_1_trans.get(), act_data, jnt_data);
//  }
//  {
//    ActuatorData act_data;
//    act_data.position.push_back(&act_pos_cmd_[1]); // Velocity and effort vectors are unused

//    JointData jnt_data;
//    jnt_data.position.push_back(&jnt_pos_cmd_[1]); // Velocity and effort vectors are unused

//    jnt_to_act_pos_cmd_.registerTransmission(transmission_names_[3], torso_2_trans.get(), act_data, jnt_data);
//  }
}

void ReemcHardwareGazebo::read()
{
//  act_to_jnt_state_.propagate();

//     robot_state_->zeroCommands();
//
//     // Restart all running controllers if motors are re-enabled
//     reset_controllers = !robot_state_->isHalted() && motors_previously_halted_;
//     motors_previously_halted_ = robot_state_->isHalted();
}


void ReemcHardwareGazebo::write()
{
//  jnt_to_act_pos_cmd_.propagate();

//     robot_state_->enforceSafety();
//     robot_state_->propagateJointEffortToActuatorEffort();
}

bool ReemcHardwareGazebo::initSim(ros::NodeHandle nh, gazebo::physics::ModelPtr model)
{
  using gazebo::physics::JointPtr;

  // Cleanup
  sim_joints_.clear();
  jnt_pos_.clear();
  jnt_vel_.clear();
  jnt_eff_.clear();
  jnt_pos_cmd_.clear();

  // Simulation joints
  sim_joints_ = model->GetJoints();
  n_dof_ = sim_joints_.size();

  vector<string> jnt_names;
  for (size_t i = 0; i < n_dof_; ++i)
  {
    const string unscoped_name = sim_joints_[i]->GetName().substr(7); // NOTE: Removing extra scoping, TODO: Fix!
    jnt_names.push_back(unscoped_name);
  }

  // Raw data
  jnt_pos_.resize(n_dof_);
  jnt_vel_.resize(n_dof_);
  jnt_eff_.resize(n_dof_);
  jnt_pos_cmd_.resize(n_dof_);

  // Hardware interfaces
  for (size_t i = 0; i < n_dof_; ++i)
  {
    jnt_state_interface_.registerJoint(jnt_names[i], &jnt_pos_[i], &jnt_vel_[i], &jnt_eff_[i]);
    jnt_pos_cmd_interface_.registerJoint(jnt_state_interface_.getJointStateHandle(jnt_names[i]), &jnt_pos_cmd_[i]);

    ROS_DEBUG_STREAM("Registered joint '" << jnt_names[i] << "' in the PositionJointInterface.");
  }
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_cmd_interface_);

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
  for(unsigned int j = 0; j < n_dof_; ++j)
  {
    // Gazebo has an interesting API...
    jnt_pos_[j] += angles::shortest_angular_distance
      (jnt_pos_[j], sim_joints_[j]->GetAngle(0u).Radian());
    jnt_vel_[j] = sim_joints_[j]->GetVelocity(0u);
    jnt_eff_[j] = sim_joints_[j]->GetForce(0u);
  }
}

void ReemcHardwareGazebo::writeSim(ros::Time time, ros::Duration period)
{
  for(unsigned int j = 0; j < n_dof_; ++j)
  {
    const double error = jnt_pos_cmd_[j] - jnt_pos_[j]; // NOTE: Assumes jnt_pos_ contains most recent value
    const double effort = pids_[j].computeCommand(error, period);

    // Gazebo has an interesting API...
    sim_joints_[j]->SetForce(0u, effort);
  }
}

} // reemc_hardware_gazebo

PLUGINLIB_DECLARE_CLASS(reemc_hardware_gazebo, ReemcHardwareGazebo, reemc_hardware_gazebo::ReemcHardwareGazebo, ros_control_gazebo::RobotSim)
