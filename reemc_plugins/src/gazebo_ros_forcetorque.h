#ifndef GAZEBO_ROS_FORCETORQUE_H
#define GAZEBO_ROS_FORCETORQUE_H


#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/String.h>

#include <boost/thread.hpp>

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
//#include <gazebo/plugins/ContactPlugin.hh>
#include <gazebo/sensors/Sensor.hh>

//#include <geometry_msgs/WrenchStamped.h>
//#include <sensor_msgs/JointState.h>

namespace gazebo
{
  class FTPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: FTPlugin();

    /// \brief Destructor
    public: virtual ~FTPlugin();

    /// \brief Load the controller
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief connected by updateConnection, called when contact
    private: void UpdateChild();

    /// \brief ROS callback queue thread
    private: void RosQueueThread();

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;
    private: std::string topic_name_;
    private: std::string frame_name_;
    private: double      update_rate_;

    /// Pointer to the update event connections
    private: event::ConnectionPtr updateConnection;

    // Contact sensors
    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr footContactSensor;
    private: ros::Publisher pubFootContact;

    private: boost::thread deferredLoadThread;

    // ROS stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue rosQueue;
    private: boost::thread callbackQueeuThread;

    private: math::Vector3 footForce;
    private: math::Vector3 footTorque;

    // Controls stuff
    private: ros::Time lastUpdateTime;

    // Mutex
    private: boost::mutex mutex_;
  };
}

#endif // GAZEBO_ROS_FORCETORQUE_H
