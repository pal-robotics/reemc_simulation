#ifndef REEMC_PLUGIN_HH
#define REEMC_PLUGIN_HH

#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/Twist.h>
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
#include <gazebo/common/PID.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sensor_msgs/JointState.h>
#include <atlas_msgs/ControllerStatistics.h>
#include <osrf_msgs/JointCommands.h>

namespace gazebo
{
  class REEMCPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: REEMCPlugin();

    /// \brief Destructor
    public: virtual ~REEMCPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);


    /// \brief Update the controller
    private: void UpdateStates();

    /// \brief ROS callback queue thread
    private: void RosQueueThread();

    /// \brief: thread out Load function with
    /// with anything that might be blocking.
    private: void DeferredLoad();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;

    /// Pointer to the update event connections
    private: event::ConnectionPtr updateConnection;

    /// Throttle update rate
    private: common::Time lastControllerStatisticsTime;
    private: double updateRate;

//    // IMU sensor
//    private: boost::shared_ptr<sensors::ImuSensor> imuSensor;
//    private: std::string imuLinkName;
//    private: math::Pose imuOffsetPose;
//    private: physics::LinkPtr imuLink;
//    private: common::Time lastImuTime;
//    private: math::Pose imuReferencePose;
//    private: math::Vector3 imuLastLinearVel;
//    private: ros::Publisher pubImu;

    // deferred loading in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: boost::thread deferredLoadThread;

    // ROS stuff
    private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue rosQueue;
    private: boost::thread callbackQueeuThread;
    private: ros::Publisher pubControllerStatistics;
    private: ros::Publisher pubJointStates;


    private: ros::Subscriber subJointCommands;
    private: void SetJointCommands(
      const osrf_msgs::JointCommands::ConstPtr &_msg);

    private: std::vector<std::string> jointNames;
    private: physics::Joint_V joints;
    private: class ErrorTerms
      {
        double q_p;
        double d_q_p_dt;
        double q_i;
        double qd_p;
        friend class REEMCPlugin;
      };
    private: std::vector<ErrorTerms> errorTerms;

    private: osrf_msgs::JointCommands jointCommands;
    private: sensor_msgs::JointState jointStates;
    private: boost::mutex mutex;

    // Controls stuff
    private: common::Time lastControllerUpdateTime;

    // controls message age measure
    private: atlas_msgs::ControllerStatistics controllerStatistics;
    private: std::vector<double> jointCommandsAgeBuffer;
    private: std::vector<double> jointCommandsAgeDelta2Buffer;
    private: unsigned int jointCommandsAgeBufferIndex;
    private: double jointCommandsAgeBufferDuration;
    private: double jointCommandsAgeMean;
    private: double jointCommandsAgeVariance;
    private: double jointCommandsAge;
  };
}
#endif
