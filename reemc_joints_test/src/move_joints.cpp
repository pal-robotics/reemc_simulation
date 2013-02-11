#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/scoped_ptr.hpp>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryActionGoal.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

class MoveJoints
{
public:

    typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> JointActionClient;
    MoveJoints()
    {
        ros::NodeHandle nh;
        ros::Duration(1).sleep();
        _jointsRefSub = nh.subscribe("/joints_goal",1,&MoveJoints::jointsReferenceCB, this);
        _controllerStateSub = nh.subscribe("/whole_body_controller/state",1,&MoveJoints::controllerStateCB,this);
        _jointActionClient.reset( new JointActionClient(nh,"/whole_body_controller/joint_trajectory_action/"));
        _received_reference = false;
    }

    void jointsReferenceCB(sensor_msgs::JointState const &jointsRef)
    {
        for(unsigned int i=0; i< jointsRef.name.size(); ++i)
        {
            for(unsigned int j=0; j < _jointsRef.name.size(); ++j)
            {
                if(_jointsRef.name.at(j) == jointsRef.name.at(i))
                    _jointsRef.position.at(j) = jointsRef.position.at(i);
            }
        }
        if(!_jointsRef.position.empty())
            _received_reference = true;
    }

    void controllerStateCB(pr2_controllers_msgs::JointTrajectoryControllerState const &controllerState)
    {
        if(!_jointsRef.name.empty())
        {
            ROS_INFO("Joint names received");
            _controllerStateSub.shutdown();
            return;
        }

        for(unsigned int i=0; i< controllerState.joint_names.size(); ++i)
        {
            _jointsRef.name.push_back(controllerState.joint_names.at(i));
            _jointsRef.position.push_back(controllerState.actual.positions.at(i));
        }

    }

    bool sendAction()
    {
        if(!_received_reference)
        {
            ROS_WARN("waiting for reference positions");
            return false;
        }
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
            point_position.time_from_start = ros::Duration(1.0);
            goal.trajectory.points.push_back(point_position);

            actionlib::SimpleClientGoalState state = _jointActionClient->sendGoalAndWait(goal, ros::Duration(2,0), ros::Duration(3,0));
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
                return true;
            else
                return false;
        }
        ROS_INFO("Action couldn't be sent.");
        return false;
    }

private:
    ros::Subscriber _jointsRefSub;
    ros::Subscriber _controllerStateSub;
    boost::scoped_ptr<JointActionClient> _jointActionClient;
    sensor_msgs::JointState _jointsRef;
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
        moveJoints.sendAction();
        rate.sleep();
    }

    return 0;
}
