#ifndef REEMC_HARDWARE_GAZEBO_REEMC_HARDWARE_GAZEBO_H
#define REEMC_HARDWARE_GAZEBO_REEMC_HARDWARE_GAZEBO_H

#include <vector>
#include <string>

#include <control_toolbox/pid.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission.h>

#include <ros_control_gazebo/robot_sim.h>

#include <gazebo/physics/physics.hh>

namespace transmission_interface
{
  class Transmission;
}

namespace reemc_hardware_gazebo
{

class ReemcHardwareGazebo : public ros_control_gazebo::RobotSim // hardware_interface::RobotHW
{
public:

  ReemcHardwareGazebo();

//   bool initXml(TiXmlElement* config)
//   {
//     // create robot
//     if (!robot_model_.initXml(config))
//     {
//       ROS_ERROR("Failed to initialize REEM mechanism model");
//       return false;
//     }
//     robot_state_ = new reemc_hardware_gazebo::RobotState(&robot_model_);
//
//     // initialize motor state
//     motors_previously_halted_ = robot_state_->isHalted();
//     reset_controllers = false;
//
//     // register interfaces
//     typedef std::map<std::string, reemc_hardware_gazebo::JointState*> JointStateMap;
//     for(JointStateMap::iterator it = robot_state_->jnt_states_map_.begin();
//     it != robot_state_->jnt_states_map_.end(); ++it)
//     {
//       jnt_state_interface_.registerJoint( it->first,
//                                             &it->second->pos_,
//                                             &it->second->vel_,
//                                             &it->second->measured_effort_);
//       effort_jnt_interface_.registerJoint(jnt_state_interface_.getJointStateHandle(it->first),
//                                             &it->second->commanded_effort_);
//     }
//     registerInterface(&jnt_state_interface_);
//     registerInterface(&effort_jnt_interface_);
//     registerInterface(robot_state_);
//
//     return true;
//   }

  void read();
  void write();

  // Simulation-specific
  bool initSim(ros::NodeHandle nh, gazebo::physics::ModelPtr model);
  void readSim(ros::Time time, ros::Duration period);
  void writeSim(ros::Time time, ros::Duration period);


//  bool reset_controllers;

//   pr2_mechanism_model::Robot robot_model_;
//   pr2_mechanism_model::RobotState* robot_state_;

private:
//   bool motors_previously_halted_;

  // Raw data
  unsigned int n_dof_;

  std::vector<std::string> transmission_names_;

  std::vector<double> act_pos_;
  std::vector<double> act_vel_;
  std::vector<double> act_eff_;

  std::vector<double> jnt_pos_;
  std::vector<double> jnt_vel_;
  std::vector<double> jnt_eff_;

  std::vector<double> act_pos_cmd_;

  std::vector<double> jnt_pos_cmd_;

  // Simulation-specific
  std::vector<gazebo::physics::JointPtr> sim_joints_;


  // Hardware interface: actuators
  hardware_interface::JointStateInterface    act_state_interface_;
  hardware_interface::PositionJointInterface act_pos_cmd_interface_;

  // Hardware interface: joints
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_cmd_interface_;

  // Transmission interface: actuator->joint map
  transmission_interface::ActuatorToJointStateInterface act_to_jnt_state_;

  // Transmission interface: joint->actuator map
  transmission_interface::JointToActuatorPositionInterface jnt_to_act_pos_cmd_;

  // Transmission interface: transmissions container
  typedef boost::shared_ptr<transmission_interface::Transmission> TransmissionPtr;
  std::vector<TransmissionPtr> transmissions_;

  // PID controllers
  std::vector<control_toolbox::Pid> pids_;

};

}

#endif // REEMC_HARDWARE_GAZEBO_REEMC_HARDWARE_GAZEBO_H
