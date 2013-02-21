#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/scoped_ptr.hpp>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryActionGoal.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <osrf_msgs/JointCommands.h>

class MoveJoints
{
public:

    typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> JointActionClient;
    MoveJoints()
    {
        ros::NodeHandle nh;
        ros::Duration(1).sleep();
        _jointsRefSub = nh.subscribe("/joints_goal",1,&MoveJoints::jointsReferenceCB, this);
        _controllerStateSub = nh.subscribe("/reemc/joint_states",1,&MoveJoints::jointStatesCB,this);
        _received_reference = false;

        _jointActionClient.reset( new JointActionClient(nh,"/upper_body_controller/joint_trajectory_action/"));
        if(!_jointActionClient->isServerConnected())
            ROS_INFO("Action server for upper body is not connected");
        _commandsPub = nh.advertise<osrf_msgs::JointCommands>("/reemc/joint_commands",1);

    }

    void jointsReferenceCB(sensor_msgs::JointState const &jointsRef)
    {
        for(unsigned int i=0; i< jointsRef.name.size(); ++i)
        {
            for(unsigned int j=0; j < _jointsRef.name.size(); ++j)
            {
                if(_jointsRef.name.at(j) == jointsRef.name.at(i))
                {
                    _jointsRef.position.at(j) = jointsRef.position.at(i);
                    _jointCmds.position.at(j) = jointsRef.position.at(i);
                }
            }
        }
        if(!_jointsRef.position.empty())
            _received_reference = true;
    }

    void jointStatesCB(sensor_msgs::JointState const &jointStates)
    {
        if(!_jointsRef.name.empty())
        {
            ROS_INFO("Joint names received");
            _controllerStateSub.shutdown();
            return;
        }

        for(unsigned int i=0; i< jointStates.name.size(); ++i)
        {
            _jointsRef.name.push_back(jointStates.name.at(i));
            _jointsRef.position.push_back(jointStates.position.at(i));
            _jointCmds.name.push_back(jointStates.name.at(i));
            _jointCmds.position.push_back(jointStates.position.at(i));
        }

    }

    bool sendAction()
    {

        static unsigned int counter = 0;
        counter++;
        std::ostringstream istr;
        istr << counter;
        if(_jointActionClient->isServerConnected())
        {
            pr2_controllers_msgs::JointTrajectoryGoal goal;

            goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
            trajectory_msgs::JointTrajectoryPoint point_position;

            for(unsigned int i=0; i< _jointsRef.name.size(); ++i)
            {
                goal.trajectory.joint_names.push_back(_jointsRef.name.at(i));
                point_position.positions.push_back(_jointsRef.position.at(i));
            }
            point_position.time_from_start = ros::Duration(3.0);
            goal.trajectory.points.push_back(point_position);

            actionlib::SimpleClientGoalState state = _jointActionClient->sendGoalAndWait(goal, ros::Duration(3,0), ros::Duration(4,0));
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
                return true;
            else
                return false;
        }
        ROS_INFO("Action couldn't be sent.");
        return false;
    }

    bool sendCommands()
    {
        _commandsPub.publish(_jointCmds);
        return true;
    }

    void control()
    {
        if(!_received_reference)
        {
            ROS_WARN("waiting for reference positions");
            return;
        }
        if(_jointActionClient->isServerConnected())
            sendAction();
        else
            sendCommands();
    }

private:
    ros::Subscriber _jointsRefSub;
    ros::Subscriber _controllerStateSub;
    boost::scoped_ptr<JointActionClient> _jointActionClient;
    ros::Publisher  _commandsPub;
    sensor_msgs::JointState _jointsRef;
    osrf_msgs::JointCommands _jointCmds;
    bool _received_reference;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_joints");
    MoveJoints moveJoints;
    ros::Rate rate(1.0);
    while(ros::ok())
    {
        ros::spinOnce();
        moveJoints.control();
        rate.sleep();
    }

    return 0;
}
