#include <algorithm>
#include <assert.h>
#include <stdlib.h>

#include "gazebo_ros_force.h"

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosForce::GazeboRosForce()
{
  this->wrenchMsg.force.x = 0;
  this->wrenchMsg.force.y = 0;
  this->wrenchMsg.force.z = 0;
  this->wrenchMsg.torque.x = 0;
  this->wrenchMsg.torque.y = 0;
  this->wrenchMsg.torque.z = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosForce::~GazeboRosForce()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);

  // Custom Callback Queue
  this->queue.clear();
  this->queue.disable();
  this->rosNode->shutdown();
  this->callbackQueueThread.join();

  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosForce::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world = _model->GetWorld();

  // load parameters
  this->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL("f3d plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->linkName = _sdf->GetElement("bodyName")->GetValueString();

  this->link = _model->GetLink(this->linkName);
  if (!this->link)
  {
    ROS_FATAL("gazebo_ros_f3d plugin error: link named: %s does not exist\n", this->linkName.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL("f3d plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topicName = _sdf->GetElement("topicName")->GetValueString();


  // initialize ros if not done so already
  if (!ros::isInitialized())
  {
    ROS_FATAL("while loading gazebo_ros_force plugin, ros is not initialized, please load a gazebo system plugin that initializes ros (e.g. libgazebo_ros_api_plugin.so from gazebo ros package)\n");
    return;
  }

  this->rosNode = new ros::NodeHandle(this->robotNamespace);

  // Custom Callback Queue
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
    this->topicName,1,
    boost::bind( &GazeboRosForce::UpdateObjectForce,this,_1),
    ros::VoidPtr(), &this->queue);
  this->sub = this->rosNode->subscribe(so);

  // Custom Callback Queue
  this->callbackQueueThread = boost::thread( boost::bind( &GazeboRosForce::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosForce::UpdateChild, this));

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForce::UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr& _msg)
{
  this->wrenchMsg.force.x = _msg->force.x;
  this->wrenchMsg.force.y = _msg->force.y;
  this->wrenchMsg.force.z = _msg->force.z;
  this->wrenchMsg.torque.x = _msg->torque.x;
  this->wrenchMsg.torque.y = _msg->torque.y;
  this->wrenchMsg.torque.z = _msg->torque.z;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForce::UpdateChild()
{
  this->lock.lock();
  math::Vector3 force(this->wrenchMsg.force.x,this->wrenchMsg.force.y,this->wrenchMsg.force.z);
  math::Vector3 torque(this->wrenchMsg.torque.x,this->wrenchMsg.torque.y,this->wrenchMsg.torque.z);
  this->link->AddForce(force);
  this->link->AddTorque(torque);
  this->lock.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosForce::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->queue.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosForce);
}
