/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <string>

#include "REEMCPlugin.h"

#include "sensor_msgs/Imu.h"

using std::string;

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(REEMCPlugin)

////////////////////////////////////////////////////////////////////////////////
REEMCPlugin::REEMCPlugin()
{

}

////////////////////////////////////////////////////////////////////////////////
REEMCPlugin::~REEMCPlugin()
{
    event::Events::DisconnectWorldUpdateStart(this->updateConnection);
    this->rosNode->shutdown();
    this->rosQueue.clear();
    this->rosQueue.disable();
    this->callbackQueeuThread.join();
    delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
void REEMCPlugin::Load(physics::ModelPtr _parent,
                       sdf::ElementPtr _sdf)
{
    this->model = _parent;

    // Get the world name.
    this->world = this->model->GetWorld();
    this->sdf = _sdf;
    this->lastControllerUpdateTime = this->world->GetSimTime();

    // get joints
    this->jointNames.push_back("l_hip_z_joint");
    this->jointNames.push_back("l_hip_x_joint");
    this->jointNames.push_back("l_hip_y_joint");
    this->jointNames.push_back("l_knee_joint");
    this->jointNames.push_back("l_ankle_y_joint");
    this->jointNames.push_back("l_ankle_x_joint");
    this->jointNames.push_back("r_hip_z_joint");
    this->jointNames.push_back("r_hip_x_joint");
    this->jointNames.push_back("r_hip_y_joint");
    this->jointNames.push_back("r_knee_joint");
    this->jointNames.push_back("r_ankle_y_joint");
    this->jointNames.push_back("r_ankle_x_joint");
    this->jointNames.push_back("torso_1_joint");
    this->jointNames.push_back("torso_2_joint");
    this->jointNames.push_back("head_1_joint");
    this->jointNames.push_back("head_2_joint");
    this->jointNames.push_back("arm_right_1_joint");
    this->jointNames.push_back("arm_right_2_joint");
    this->jointNames.push_back("arm_right_3_joint");
    this->jointNames.push_back("arm_right_4_joint");
    this->jointNames.push_back("arm_right_5_joint");
    this->jointNames.push_back("arm_right_6_joint");
    this->jointNames.push_back("arm_right_7_joint");
    this->jointNames.push_back("arm_left_1_joint");
    this->jointNames.push_back("arm_left_2_joint");
    this->jointNames.push_back("arm_left_3_joint");
    this->jointNames.push_back("arm_left_4_joint");
    this->jointNames.push_back("arm_left_5_joint");
    this->jointNames.push_back("arm_left_6_joint");
    this->jointNames.push_back("arm_left_7_joint");

    this->joints.resize(this->jointNames.size());
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
        this->joints[i] = this->model->GetJoint(this->jointNames[i]);
        if (!this->joints[i])
        {
            ROS_ERROR("REEMC robot expected joint[%s] not present, plugin not loaded",
                      this->jointNames[i].c_str());
            return;
        }
    }

    this->errorTerms.resize(this->joints.size());

    this->jointStates.name.resize(this->joints.size());
    this->jointStates.position.resize(this->joints.size());
    this->jointStates.velocity.resize(this->joints.size());
    this->jointStates.effort.resize(this->joints.size());

    for (unsigned int i = 0; i < this->jointNames.size(); ++i)
        this->jointStates.name[i] = this->jointNames[i];

    this->jointCommands.name.resize(this->joints.size());
    this->jointCommands.position.resize(this->joints.size());
    this->jointCommands.velocity.resize(this->joints.size());
    this->jointCommands.effort.resize(this->joints.size());
    this->jointCommands.kp_position.resize(this->joints.size());
    this->jointCommands.ki_position.resize(this->joints.size());
    this->jointCommands.kd_position.resize(this->joints.size());
    this->jointCommands.kp_velocity.resize(this->joints.size());
    this->jointCommands.i_effort_min.resize(this->joints.size());
    this->jointCommands.i_effort_max.resize(this->joints.size());

    for (unsigned i = 0; i < this->joints.size(); ++i)
    {
        this->errorTerms[i].q_p = 0;
        this->errorTerms[i].d_q_p_dt = 0;
        this->errorTerms[i].q_i = 0;
        this->errorTerms[i].qd_p = 0;
        this->jointCommands.name[i] = this->joints[i]->GetScopedName();
        this->jointCommands.position[i] = 0;
        this->jointCommands.velocity[i] = 0;
        this->jointCommands.effort[i] = 0;
        this->jointCommands.kp_position[i] = 0;
        this->jointCommands.ki_position[i] = 0;
        this->jointCommands.kd_position[i] = 0;
        this->jointCommands.kp_velocity[i] = 0;
        this->jointCommands.i_effort_min[i] = 0;
        this->jointCommands.i_effort_max[i] = 0;
    }

    // ros callback queue for processing subscription
    this->deferredLoadThread = boost::thread(
                boost::bind(&REEMCPlugin::DeferredLoad, this));
}


////////////////////////////////////////////////////////////////////////////////
void REEMCPlugin::SetJointCommands(
        const osrf_msgs::JointCommands::ConstPtr &_msg)
{
    boost::mutex::scoped_lock lock(this->mutex);

    this->jointCommands.header.stamp = _msg->header.stamp;

    if (_msg->position.size() == this->jointCommands.position.size())
        std::copy(_msg->position.begin(), _msg->position.end(),
                  this->jointCommands.position.begin());
    else
        ROS_DEBUG("joint commands message contains different number of"
                  " elements position[%d] than expected[%d]",
                  _msg->position.size(), this->jointCommands.position.size());

    if (_msg->velocity.size() == this->jointCommands.velocity.size())
        std::copy(_msg->velocity.begin(), _msg->velocity.end(),
                  this->jointCommands.velocity.begin());
    else
        ROS_DEBUG("joint commands message contains different number of"
                  " elements velocity[%d] than expected[%d]",
                  _msg->velocity.size(), this->jointCommands.velocity.size());

    if (_msg->effort.size() == this->jointCommands.effort.size())
        std::copy(_msg->effort.begin(), _msg->effort.end(),
                  this->jointCommands.effort.begin());
    else
        ROS_DEBUG("joint commands message contains different number of"
                  " elements effort[%d] than expected[%d]",
                  _msg->effort.size(), this->jointCommands.effort.size());

    if (_msg->kp_position.size() == this->jointCommands.kp_position.size())
        std::copy(_msg->kp_position.begin(), _msg->kp_position.end(),
                  this->jointCommands.kp_position.begin());
    else
        ROS_DEBUG("joint commands message contains different number of"
                  " elements kp_position[%d] than expected[%d]",
                  _msg->kp_position.size(), this->jointCommands.kp_position.size());

    if (_msg->ki_position.size() == this->jointCommands.ki_position.size())
        std::copy(_msg->ki_position.begin(), _msg->ki_position.end(),
                  this->jointCommands.ki_position.begin());
    else
        ROS_DEBUG("joint commands message contains different number of"
                  " elements ki_position[%d] than expected[%d]",
                  _msg->ki_position.size(), this->jointCommands.ki_position.size());

    if (_msg->kd_position.size() == this->jointCommands.kd_position.size())
        std::copy(_msg->kd_position.begin(), _msg->kd_position.end(),
                  this->jointCommands.kd_position.begin());
    else
        ROS_DEBUG("joint commands message contains different number of"
                  " elements kd_position[%d] than expected[%d]",
                  _msg->kd_position.size(), this->jointCommands.kd_position.size());

    if (_msg->kp_velocity.size() == this->jointCommands.kp_velocity.size())
        std::copy(_msg->kp_velocity.begin(), _msg->kp_velocity.end(),
                  this->jointCommands.kp_velocity.begin());
    else
        ROS_DEBUG("joint commands message contains different number of"
                  " elements kp_velocity[%d] than expected[%d]",
                  _msg->kp_velocity.size(), this->jointCommands.kp_velocity.size());

    if (_msg->i_effort_min.size() == this->jointCommands.i_effort_min.size())
        std::copy(_msg->i_effort_min.begin(), _msg->i_effort_min.end(),
                  this->jointCommands.i_effort_min.begin());
    else
        ROS_DEBUG("joint commands message contains different number of"
                  " elements i_effort_min[%d] than expected[%d]",
                  _msg->i_effort_min.size(), this->jointCommands.i_effort_min.size());

    if (_msg->i_effort_max.size() == this->jointCommands.i_effort_max.size())
        std::copy(_msg->i_effort_max.begin(), _msg->i_effort_max.end(),
                  this->jointCommands.i_effort_max.begin());
    else
        ROS_DEBUG("joint commands message contains different number of"
                  " elements i_effort_max[%d] than expected[%d]",
                  _msg->i_effort_max.size(), this->jointCommands.i_effort_max.size());
}

////////////////////////////////////////////////////////////////////////////////
void REEMCPlugin::DeferredLoad()
{
    // initialize ros
    if (!ros::isInitialized())
    {
        gzerr << "Not loading plugin since ROS hasn't been "
              << "properly initialized.  Try starting gazebo with ros plugin:\n"
              << "  gazebo -s libgazebo_ros_api_plugin.so\n";
        return;
    }

    // ros stuff
    this->rosNode = new ros::NodeHandle("");

    // pull down controller parameters
    for (unsigned int joint = 0; joint < this->joints.size(); ++joint)
    {
        char joint_ns[200] = "";
        snprintf(joint_ns, sizeof(joint_ns), "reemc_controller/gains/%s/",
                 this->joints[joint]->GetName().c_str());
        // this is so ugly
        double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0;
        string p_str = string(joint_ns)+"p";
        string i_str = string(joint_ns)+"i";
        string d_str = string(joint_ns)+"d";
        string i_clamp_str = string(joint_ns)+"i_clamp";
        if (!this->rosNode->getParam(p_str, p_val) ||
                !this->rosNode->getParam(i_str, i_val) ||
                !this->rosNode->getParam(d_str, d_val) ||
                !this->rosNode->getParam(i_clamp_str, i_clamp_val))
        {
            ROS_ERROR("couldn't find a param for %s", joint_ns);
            continue;
        }
        this->jointCommands.kp_position[joint]  =  p_val;
        this->jointCommands.ki_position[joint]  =  i_val;
        this->jointCommands.kd_position[joint]  =  d_val;
        this->jointCommands.i_effort_min[joint] = -i_clamp_val;
        this->jointCommands.i_effort_max[joint] =  i_clamp_val;
    }

    // Get window size from ros parameter server (seconds)
    if (!this->rosNode->getParam(
                "reemc_controller/statistics_time_window_size",
                this->jointCommandsAgeBufferDuration))
    {
        this->jointCommandsAgeBufferDuration = 1.0;
        ROS_INFO("controller statistics window size not specified in"
                 " ros parameter server, defaulting to %f sec.",
                 this->jointCommandsAgeBufferDuration);
    }
    double stepSize = this->world->GetPhysicsEngine()->GetStepTime();
    if (math::equal(stepSize, 0.0))
    {
        stepSize = 0.001;
        ROS_WARN("simulation step size is zero, something is wrong,"
                 "  Defaulting to step size of %f sec.", stepSize);
    }
    // document this from
    // http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    // Online algorithm
    // where Delta2 buffer contains delta*(x - mean) line from code block
    unsigned int bufferSize = this->jointCommandsAgeBufferDuration / stepSize;
    this->jointCommandsAgeBuffer.resize(bufferSize);
    this->jointCommandsAgeDelta2Buffer.resize(bufferSize);
    this->jointCommandsAgeBufferIndex = 0;
    this->jointCommandsAgeMean = 0.0;
    this->jointCommandsAgeVariance = 0.0;

    // ROS Controller API
    /// brief broadcasts the robot states
    this->pubJointStates = this->rosNode->advertise<sensor_msgs::JointState>(
                "reemc/joint_states", 1);

    // ros publication / subscription
    this->pubControllerStatistics =
            this->rosNode->advertise<atlas_msgs::ControllerStatistics>(
                "reemc/controller_statistics", 10);


    // ros topic subscribtions
    ros::SubscribeOptions jointCommandsSo =
            ros::SubscribeOptions::create<osrf_msgs::JointCommands>(
                "reemc/joint_commands", 1,
                boost::bind(&REEMCPlugin::SetJointCommands, this, _1),
                ros::VoidPtr(), &this->rosQueue);

    // Because TCP causes bursty communication with high jitter,
    // declare a preference on UDP connections for receiving
    // joint commands, which we want to get at a high rate.
    // Note that we'll still accept TCP connections for this topic
    // (e.g., from rospy nodes, which don't support UDP);
    // we just prefer UDP.
    jointCommandsSo.transport_hints =
            ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

    this->subJointCommands=
            this->rosNode->subscribe(jointCommandsSo);

    // initialize status pub time
    this->lastControllerStatisticsTime = this->world->GetSimTime().Double();
    this->updateRate = 1.0;

    // ros callback queue for processing subscription
    this->callbackQueeuThread = boost::thread(
                boost::bind(&REEMCPlugin::RosQueueThread, this));

    this->updateConnection = event::Events::ConnectWorldUpdateStart(
                boost::bind(&REEMCPlugin::UpdateStates, this));

}

void REEMCPlugin::UpdateStates()
{
    common::Time curTime = this->world->GetSimTime();

    if (curTime > this->lastControllerUpdateTime)
    {

        // populate FromRobot from robot
        this->jointStates.header.stamp = ros::Time(curTime.sec, curTime.nsec);
        for (unsigned int i = 0; i < this->joints.size(); ++i)
        {
            this->jointStates.position[i] = this->joints[i]->GetAngle(0).Radian();
            this->jointStates.velocity[i] = this->joints[i]->GetVelocity(0);
            // better to us e GetForceTorque dot joint axis ??
            this->jointStates.effort[i] = this->joints[i]->GetForce(0);
        }
        this->pubJointStates.publish(this->jointStates);

        double dt = (curTime - this->lastControllerUpdateTime).Double();

        {
            boost::mutex::scoped_lock lock(this->mutex);
            {
                // Keep track of age of jointCommands age in seconds.
                // Note the value is invalid as a moving window average age
                // until the buffer is full.
                this->jointCommandsAge = curTime.Double() -
                        this->jointCommands.header.stamp.toSec();

                double weightedJointCommandsAge = this->jointCommandsAge
                        / this->jointCommandsAgeBuffer.size();

                // for variance calculation, save delta before average is updated.
                double delta = this->jointCommandsAge - this->jointCommandsAgeMean;

                // update average
                this->jointCommandsAgeMean += weightedJointCommandsAge;
                this->jointCommandsAgeMean -=
                        this->jointCommandsAgeBuffer[this->jointCommandsAgeBufferIndex];

                // update variance with new average
                double delta2 = delta *
                        (this->jointCommandsAge - this->jointCommandsAgeMean);
                this->jointCommandsAgeVariance += delta2;
                this->jointCommandsAgeVariance -=
                        this->jointCommandsAgeDelta2Buffer[
                        this->jointCommandsAgeBufferIndex];

                // save weighted average in window
                this->jointCommandsAgeBuffer[this->jointCommandsAgeBufferIndex] =
                        weightedJointCommandsAge;

                // save delta buffer for incremental variance calculation
                this->jointCommandsAgeDelta2Buffer[
                        this->jointCommandsAgeBufferIndex] = delta2;

                this->jointCommandsAgeBufferIndex =
                        (this->jointCommandsAgeBufferIndex + 1) %
                        this->jointCommandsAgeBuffer.size();
            }

            /// update pid with feedforward force
            for (unsigned int i = 0; i < this->joints.size(); ++i)
            {
                // truncate joint position within range of motion
                double positionTarget = math::clamp(
                            this->jointCommands.position[i],
                            this->joints[i]->GetLowStop(0).Radian(),
                            this->joints[i]->GetHighStop(0).Radian());

                double q_p = positionTarget - this->jointStates.position[i];

                if (!math::equal(dt, 0.0))
                    this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / dt;

                this->errorTerms[i].q_p = q_p;

                this->errorTerms[i].qd_p =
                        this->jointCommands.velocity[i] - this->jointStates.velocity[i];

                if (!math::equal(this->jointCommands.ki_position[i], 0.0))
                    this->errorTerms[i].q_i = math::clamp(
                            this->errorTerms[i].q_i + dt * this->errorTerms[i].q_p,
                            static_cast<double>(this->jointCommands.i_effort_min[i]) /
                            this->jointCommands.ki_position[i],
                            static_cast<double>(this->jointCommands.i_effort_max[i]) /
                            this->jointCommands.ki_position[i]);

                // use gain params to compute force cmd
                double force =
                        this->jointCommands.kp_position[i] * this->errorTerms[i].q_p +
                        this->jointCommands.ki_position[i] * this->errorTerms[i].q_i +
                        this->jointCommands.kd_position[i] * this->errorTerms[i].d_q_p_dt +
                        this->jointCommands.kp_velocity[i] * this->errorTerms[i].qd_p +
                        this->jointCommands.effort[i];

                this->joints[i]->SetForce(0, force);
            }
        }
        this->lastControllerUpdateTime = curTime;

        /// controller statistics diagnostics, damages, etc.
        if (this->pubControllerStatistics.getNumSubscribers() > 0)
        {
            if ((curTime - this->lastControllerStatisticsTime).Double() >=
                    1.0/this->updateRate)
            {
                atlas_msgs::ControllerStatistics msg;
                msg.header.stamp = ros::Time(curTime.sec, curTime.nsec);
                msg.command_age = this->jointCommandsAge;
                msg.command_age_mean = this->jointCommandsAgeMean;
                msg.command_age_variance = this->jointCommandsAgeVariance /
                        (this->jointCommandsAgeBuffer.size() - 1);
                msg.command_age_window_size = this->jointCommandsAgeBufferDuration;

                this->pubControllerStatistics.publish(msg);
                this->lastControllerStatisticsTime = curTime;
            }
        }
    }

}

void REEMCPlugin::RosQueueThread()
{
    static const double timeout = 0.01;

    while (this->rosNode->ok())
    {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}
}

