#include <map>
#include <string>

#include "PalModelPlugin.h"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(PalModelPlugin)

#define REEMC_L_FOOT  "l_sole"
#define REEMC_R_FOOT  "r_sole"
#define REEMC_PIN_LINK "torso_2_link"

////////////////////////////////////////////////////////////////////////////////
// Constructor
PalModelPlugin::PalModelPlugin()
{
  /// initial anchor pose
  this->warpRobotWithCmdVel = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
PalModelPlugin::~PalModelPlugin()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueueThread.join();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void PalModelPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // save pointers
  this->model = _parent;
  this->sdf = _sdf;

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(
    boost::bind(&PalModelPlugin::DeferredLoad, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void PalModelPlugin::DeferredLoad()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading vrc plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");


  // this->world->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));
  this->lastUpdateTime = this->model->GetWorld()->GetSimTime().Double();
  this->robotCmdVel = geometry_msgs::Twist();

  // Load Robot
  this->REEMC.Load(this->model, this->sdf);

  // Setup ROS interfaces for robot
  this->LoadRobotROSAPI();

  // Harness the Robot
  // On startup, simulate "virtual harness" by turning gravity off
  // allowing the controllers can initialize without the robot falling
  if (this->REEMC.isInitialized)
  {
    this->SetRobotMode("pinned");
    this->REEMC.startupHarness = true;
    ROS_INFO("Start robot with gravity turned off and harnessed.");
    ROS_INFO("Resume to nominal mode after 10 seconds.");
  }

  // ros callback queue for processing subscription
  this->callbackQueueThread = boost::thread(
    boost::bind(&PalModelPlugin::ROSQueueThread, this));

  // Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
     boost::bind(&PalModelPlugin::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
void PalModelPlugin::SetRobotModeTopic(const std_msgs::String::ConstPtr &_str)
{
  this->SetRobotMode(_str->data);
}

////////////////////////////////////////////////////////////////////////////////
void PalModelPlugin::SetRobotMode(const std::string &_str)
{
  if (_str == "no_gravity")
  {
    // stop warping robot
    this->warpRobotWithCmdVel = false;
    physics::Link_V links = this->REEMC.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }
    if (this->REEMC.pinJoint)
      this->RemoveJoint(this->REEMC.pinJoint);
  }
  else if (_str == "feet")
  {
    // stop warping robot
    this->warpRobotWithCmdVel = false;
    physics::Link_V links = this->REEMC.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      if (links[i]->GetName() == REEMC_L_FOOT || links[i]->GetName() == REEMC_R_FOOT)
        links[i]->SetGravityMode(true);
      else
        links[i]->SetGravityMode(false);
    }
    if (this->REEMC.pinJoint)
      this->RemoveJoint(this->REEMC.pinJoint);
  }
  else if (_str == "pinned")
  {
    // pinning robot, and turning off effect of gravity
    if (!this->REEMC.pinJoint)
      this->REEMC.pinJoint = this->AddJoint(this->REEMC.model,
                                        physics::LinkPtr(),
                                        this->REEMC.pinLink,
                                        "revolute",
                                        math::Vector3(0, 0, 0),
                                        math::Vector3(0, 0, 1),
                                        0.0, 0.0);
    this->REEMC.initialPose = this->REEMC.pinLink->GetWorldPose();

    physics::Link_V links = this->REEMC.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(false);
    }
  }
  else if (_str == "nominal")
  {
    // reinitialize pinning
    this->warpRobotWithCmdVel = false;
    physics::Link_V links = this->REEMC.model->GetLinks();
    for (unsigned int i = 0; i < links.size(); ++i)
    {
      links[i]->SetGravityMode(true);
    }
    if (this->REEMC.pinJoint)
      this->RemoveJoint(this->REEMC.pinJoint);
  }
  else
  {
    ROS_INFO("available modes:no_gravity, feet, pinned, nominal");
  }
}

////////////////////////////////////////////////////////////////////////////////
void PalModelPlugin::SetRobotCmdVel(const geometry_msgs::Twist::ConstPtr &_cmd)
{
  if (_cmd->linear.x == 0 && _cmd->linear.y == 0 && _cmd->angular.z == 0)
  {
    this->warpRobotWithCmdVel = false;
  }
  else
  {
    this->robotCmdVel = *_cmd;
    this->warpRobotWithCmdVel = true;
    this->lastUpdateTime = this->model->GetWorld()->GetSimTime().Double();
  }
}

////////////////////////////////////////////////////////////////////////////////
void PalModelPlugin::SetRobotPose(const geometry_msgs::Pose::ConstPtr &_pose)
{
  math::Pose pose(math::Vector3(_pose->position.x,
                                _pose->position.y,
                                _pose->position.z),
                  math::Quaternion(_pose->orientation.w,
                                   _pose->orientation.x,
                                   _pose->orientation.y,
                                   _pose->orientation.z));
  this->REEMC.model->SetWorldPose(pose);
}

////////////////////////////////////////////////////////////////////////////////
// dynamically add joint between 2 links
physics::JointPtr PalModelPlugin::AddJoint( physics::ModelPtr _model,
                                      physics::LinkPtr _link1,
                                      physics::LinkPtr _link2,
                                      std::string _type,
                                      math::Vector3 _anchor,
                                      math::Vector3 _axis,
                                      double _upper, double _lower)
{
  physics::JointPtr joint = _model->GetWorld()->GetPhysicsEngine()->CreateJoint(
    _type, _model);
  joint->Attach(_link1, _link2);
  // load adds the joint to a vector of shared pointers kept
  // in parent and child links, preventing joint from being destroyed.
  joint->Load(_link1, _link2, math::Pose(_anchor, math::Quaternion()));
  // joint->SetAnchor(0, _anchor);
  joint->SetAxis(0, _axis);
  joint->SetHighStop(0, _upper);
  joint->SetLowStop(0, _lower);

  if (_link1)
    joint->SetName(_link1->GetName() + std::string("_") +
                              _link2->GetName() + std::string("_joint"));
  else
    joint->SetName(std::string("world_") +
                              _link2->GetName() + std::string("_joint"));
  joint->Init();

/*
  // disable collision between the link pair
  if (_link1)
    _link1->SetCollideMode("fixed");
  if (_link2)
    _link2->SetCollideMode("fixed");
*/
  return joint;
}


////////////////////////////////////////////////////////////////////////////////
// remove a joint
void PalModelPlugin::RemoveJoint(physics::JointPtr &_joint)
{
  bool paused = this->model->GetWorld()->IsPaused();
  this->model->GetWorld()->SetPaused(true);
  if (_joint)
  {
    // reenable collision between the link pair
    physics::LinkPtr parent = _joint->GetParent();
    physics::LinkPtr child = _joint->GetChild();
    if (parent)
      parent->SetCollideMode("all");
    if (child)
      child->SetCollideMode("all");

    _joint->Detach();
    _joint.reset();
  }
  this->model->GetWorld()->SetPaused(paused);
}

////////////////////////////////////////////////////////////////////////////////
void PalModelPlugin::Teleport(const physics::LinkPtr &_pinLink,
                         physics::JointPtr &_pinJoint,
                         const math::Pose &_pose,
                         const std::map<std::string, double> &/*_jp*/)
{
  this->Teleport(_pinLink, _pinJoint, _pose);
  /// \todo: use _jp to set robot configuration
}

////////////////////////////////////////////////////////////////////////////////
void PalModelPlugin::Teleport(const physics::LinkPtr &_pinLink,
                         physics::JointPtr &_pinJoint,
                         const math::Pose &_pose)
{
  // pause, break joint, update pose, create new joint, unpause
  bool p = this->model->GetWorld()->IsPaused();
  bool e = this->model->GetWorld()->GetEnablePhysicsEngine();
  this->model->GetWorld()->EnablePhysicsEngine(false);
  this->model->GetWorld()->SetPaused(true);
  if (_pinJoint)
    this->RemoveJoint(_pinJoint);
  _pinLink->GetModel()->SetLinkWorldPose(_pose, _pinLink);
  if (!_pinJoint)
    _pinJoint = this->AddJoint(_pinLink->GetModel(),
                               physics::LinkPtr(),
                               this->REEMC.pinLink,
                               "revolute",
                               math::Vector3(0, 0, 0),
                               math::Vector3(0, 0, 1),
                               0.0, 0.0);
  this->model->GetWorld()->SetPaused(p);
  this->model->GetWorld()->EnablePhysicsEngine(e);
}

////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void PalModelPlugin::UpdateStates()
{
  double curTime = this->model->GetWorld()->GetSimTime().Double();

  if (this->REEMC.isInitialized &&
      this->REEMC.startupHarness && curTime > 10)
  {
    this->SetRobotMode("nominal");
    this->REEMC.startupHarness = false;
  }

  if (curTime > this->lastUpdateTime)
  {
//    this->CheckThreadStart();

    double dt = curTime - this->lastUpdateTime;

    if (this->warpRobotWithCmdVel)
    {
      this->lastUpdateTime = curTime;
      math::Pose cur_pose = this->REEMC.pinLink->GetWorldPose();
      math::Pose new_pose = cur_pose;

      // increment x,y in cur_pose frame
      math::Vector3 cmd(this->robotCmdVel.linear.x,
                        this->robotCmdVel.linear.y, 0);
      cmd = cur_pose.rot.RotateVector(cmd);

      new_pose.pos = cur_pose.pos + cmd * dt;
      // prevent robot from drifting vertically
      new_pose.pos.z = this->REEMC.initialPose.pos.z;

      math::Vector3 rpy = cur_pose.rot.GetAsEuler();
      // decay non-yaw tilts
      rpy.x = 0;
      rpy.y = 0;
      rpy.z = rpy.z + this->robotCmdVel.angular.z * dt;

      new_pose.rot.SetFromEuler(rpy);

      // set this as the new anchor pose of the pin joint
      this->Teleport(this->REEMC.pinLink,
                     this->REEMC.pinJoint,
                     new_pose);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void PalModelPlugin::ROSQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}



////////////////////////////////////////////////////////////////////////////////
void PalModelPlugin::Robot::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->isInitialized = false;
  this->model = _model;
//    ROS_INFO("sdf %s", _sdf->GetFirstElement()->GetName().c_str());

  if (_sdf->HasElement("plugin_params") )
  {
//      ROS_INFO("plugin_params %s",_sdf->GetElement("plugin_params")->GetFirstElement()->GetName().c_str());

      if(_sdf->GetElement("plugin_params")->HasElement("pin_link"))
      {
//          ROS_INFO("pin_link name %s", _sdf->GetElement("plugin_params")->GetValueString("pin_link").c_str());
          physics::Link_V links = this->model->GetLinks();
//          ROS_INFO("model link size %d",links.size());
//          for(unsigned int i=0; i < links.size() ; ++i)
//          {
//              ROS_INFO("link %d is %s", i, links.at(i)->GetName().c_str());
//          }

          this->pinLink = this->model->GetLink(_sdf->GetElement("plugin_params")->GetValueString("pin_link"));
      }
      else
      {
        ROS_INFO("Can't find <REEMC> <plugin_params> <pin_link> param, using default link %s.", REEMC_PIN_LINK);
        this->pinLink = this->model->GetLink(REEMC_PIN_LINK);
      }
  }

  else
  {
    ROS_INFO("Can't find <REEMC> <plugin_params> block, using default params.");
    this->pinLink = this->model->GetLink(REEMC_PIN_LINK);
  }

  if (!this->pinLink)
  {
    ROS_ERROR("REEMC robot pin link not found.");
    return;
  }

  // Note: hardcoded link by name: @todo: make this a pugin param
  this->initialPose = this->pinLink->GetWorldPose();
  this->isInitialized = true;
}


////////////////////////////////////////////////////////////////////////////////
void PalModelPlugin::LoadRobotROSAPI()
{
  // ros subscription
  std::string trajectory_topic_name = "reemc/cmd_vel";
  ros::SubscribeOptions trajectory_so =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(
    trajectory_topic_name, 100,
    boost::bind(&PalModelPlugin::SetRobotCmdVel, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->REEMC.subTrajectory = this->rosNode->subscribe(trajectory_so);

  std::string pose_topic_name = "reemc/set_pose";
  ros::SubscribeOptions pose_so =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
    pose_topic_name, 100,
    boost::bind(&PalModelPlugin::SetRobotPose, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->REEMC.subPose = this->rosNode->subscribe(pose_so);

  std::string configuration_topic_name = "reemc/configuration";
  ros::SubscribeOptions configuration_so =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    configuration_topic_name, 100,
    boost::bind(&PalModelPlugin::SetRobotConfiguration, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->REEMC.subConfiguration =
    this->rosNode->subscribe(configuration_so);

  std::string mode_topic_name = "reemc/mode";
  ros::SubscribeOptions mode_so =
    ros::SubscribeOptions::create<std_msgs::String>(
    mode_topic_name, 100,
    boost::bind(&PalModelPlugin::SetRobotModeTopic, this, _1),
    ros::VoidPtr(), &this->rosQueue);
  this->REEMC.subMode = this->rosNode->subscribe(mode_so);
}

////////////////////////////////////////////////////////////////////////////////
void PalModelPlugin::SetRobotConfiguration(const sensor_msgs::JointState::ConstPtr
  &_cmd)
{
  // This function is planned but not yet implemented.
  ROS_ERROR("The REEMC/configuration handler is not implemented.\n");
/*
  for (unsigned int i = 0; i < _cmd->name.size(); ++i)
  {
    this->REEMC.model->SetJointPositions();
  }
*/
}
}
