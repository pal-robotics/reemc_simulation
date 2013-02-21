

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "reemc_joints/PIDConfig.h"
#include <boost/noncopyable.hpp>
#include <boost/scoped_ptr.hpp>

class PIDTuner
{
public:

    PIDTuner() : _nh("")
    {
        _jointStatesSub = _nh.subscribe("/reemc/joint_states",1,&PIDTuner::jointStatesCB,this);
        _commandsPub = _nh.advertise<osrf_msgs::JointCommands>("/reemc/joint_commands",1);
    }

    void jointStatesCB(sensor_msgs::JointState const &jointStates)
    {
        unsigned int num_of_joints = jointStates.name.size();
        if(_jointDefaultCfgs.name.size() != num_of_joints)
        {
         _jointDefaultCfgs.name.resize(num_of_joints);
         _jointDefaultCfgs.kp_position.resize(num_of_joints);
         _jointDefaultCfgs.ki_position.resize(num_of_joints);
         _jointDefaultCfgs.kd_position.resize(num_of_joints);
         _jointDefaultCfgs.i_effort_min.resize(num_of_joints);
         _jointDefaultCfgs.i_effort_max.resize(num_of_joints);

        }

        for(unsigned int i=0; i< num_of_joints; ++i)
        {
            _jointDefaultCfgs.name[i] = jointStates.name.at(i);
        }

        // pull down controller parameters
        for (unsigned int joint = 0; joint < num_of_joints; ++joint)
        {
            char joint_ns[200] = "";
            snprintf(joint_ns, sizeof(joint_ns), "reemc_controller/gains/%s/",
                    _jointDefaultCfgs.name[joint].c_str());
            // this is so ugly
            double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0;
            std::string p_str = std::string(joint_ns)+"p";
            std::string i_str = std::string(joint_ns)+"i";
            std::string d_str = std::string(joint_ns)+"d";
            std::string i_clamp_str = std::string(joint_ns)+"i_clamp";
            if (!(this->_nh.getParam(p_str, p_val)) ||
                    !(this->_nh.getParam(i_str, i_val)) ||
                    !(this->_nh.getParam(d_str, d_val)) ||
                    !(this->_nh.getParam(i_clamp_str, i_clamp_val)))
            {
                ROS_ERROR("couldn't find a param for %s", joint_ns);
                continue;
            }
             _jointDefaultCfgs.kp_position[joint]  =  p_val;
             _jointDefaultCfgs.ki_position[joint]  =  i_val;
             _jointDefaultCfgs.kd_position[joint]  =  d_val;
             _jointDefaultCfgs.i_effort_min[joint] = -i_clamp_val;
             _jointDefaultCfgs.i_effort_max[joint] =  i_clamp_val;
             /// _jointDefaultCfgs.kp_velocity[joint]  = ?
        }

        /// Store initial config
        if(_jointCmds.name.empty())
        {
            ROS_INFO("resetting jointCmds for storing %d joints info", num_of_joints);
            _jointCmds.name.resize(num_of_joints);
            std::copy(_jointDefaultCfgs.name.begin(), _jointDefaultCfgs.name.end(), _jointCmds.name.begin());

            _jointCmds.kp_position.resize(num_of_joints);
            std::copy(_jointDefaultCfgs.kp_position.begin(), _jointDefaultCfgs.kp_position.end(), _jointCmds.kp_position.begin());

            _jointCmds.ki_position.resize(num_of_joints);
            std::copy(_jointDefaultCfgs.ki_position.begin(), _jointDefaultCfgs.ki_position.end(), _jointCmds.ki_position.begin());

            _jointCmds.kd_position.resize(num_of_joints);
             std::copy(_jointDefaultCfgs.kd_position.begin(), _jointDefaultCfgs.kd_position.end(), _jointCmds.kd_position.begin());

            _jointCmds.i_effort_min.resize(num_of_joints);
             std::copy(_jointDefaultCfgs.i_effort_min.begin(), _jointDefaultCfgs.i_effort_min.end(), _jointCmds.i_effort_min.begin());

            _jointCmds.i_effort_max.resize(num_of_joints);
             std::copy(_jointDefaultCfgs.i_effort_max.begin(), _jointDefaultCfgs.i_effort_max.end(), _jointCmds.i_effort_max.begin());

             /// Setup dyn configure
             _dynConfigServer.reset( new dynamic_reconfigure::Server<reemc_joints::PIDConfig>(ros::NodeHandle("~")) );
             dynamic_reconfigure::Server<reemc_joints::PIDConfig>::CallbackType cb = boost::bind(&PIDTuner::sendPIDParams, this, _1, _2);
             _dynConfigServer->setCallback(cb);

        }

    }


    bool sendPIDParams(reemc_joints::PIDConfig &config, uint32_t level)
    {
        boost::recursive_mutex::scoped_lock cfl(_configuration_mutex);
        if(config.JointId > (_jointCmds.name.size() - 1) )
        {
            ROS_WARN("JointId %d out of range", config.JointId);
            return false;
        }
        ROS_INFO("Tuning parameters for joint id %d", config.JointId);

        static int jointId = -1;
        if( jointId != config.JointId  )
        {
            ROS_INFO("new joint selected");
            config.Kp = _jointCmds.kp_position[config.JointId];
            config.Kd = _jointCmds.kd_position[config.JointId];
            config.Ki = _jointCmds.ki_position[config.JointId];
            config.I_clamp  = _jointCmds.i_effort_max[config.JointId];

            if(jointId != config.JointId)
                jointId = config.JointId;
        }
        else
        {
            ROS_INFO("Got new params from gui");
            _jointCmds.kp_position[config.JointId]  =  config.Kp;
            _jointCmds.kd_position[config.JointId]  =  config.Kd;
            _jointCmds.ki_position[config.JointId]  =  config.Ki;
            _jointCmds.i_effort_max[config.JointId] =  config.I_clamp;
            _jointCmds.i_effort_min[config.JointId] = -config.I_clamp;
        }

        if(config.restore_defaults)
        {
            ROS_INFO("restoring defaults");
            _jointCmds.kp_position[config.JointId] = _jointDefaultCfgs.kp_position[config.JointId];
            _jointCmds.kd_position[config.JointId] = _jointDefaultCfgs.kd_position[config.JointId];
            _jointCmds.ki_position[config.JointId] = _jointDefaultCfgs.ki_position[config.JointId];
            _jointCmds.i_effort_max[config.JointId] =_jointDefaultCfgs.i_effort_max[config.JointId];
            _jointCmds.i_effort_min[config.JointId] =_jointDefaultCfgs.i_effort_min[config.JointId];

            config.Kp = _jointCmds.kp_position[config.JointId];
            config.Kd = _jointCmds.kd_position[config.JointId];
            config.Ki = _jointCmds.ki_position[config.JointId];
            config.I_clamp  = _jointCmds.i_effort_max[config.JointId];

            //avoid looping
            if(config.restore_defaults)
                config.restore_defaults = false;
        }

        _commandsPub.publish(_jointCmds);

        return true;
    }

private:
    ros::NodeHandle _nh;
    ros::Subscriber _jointStatesSub;
    ros::Publisher  _commandsPub;
    osrf_msgs::JointCommands _jointCmds, _jointDefaultCfgs;

    boost::recursive_mutex _configuration_mutex;
    boost::scoped_ptr<dynamic_reconfigure::Server<reemc_joints::PIDConfig> > _dynConfigServer;
    reemc_joints::PIDConfig default_config_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_tuner");
    PIDTuner pidTuner;
    ros::spin();
    return 0;
}
