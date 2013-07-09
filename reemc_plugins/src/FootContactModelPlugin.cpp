#include "FootContactModelPlugin.h"

#include <string>


using std::string;

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(FootContactModelPlugin)

////////////////////////////////////////////////////////////////////////////////
FootContactModelPlugin::FootContactModelPlugin()
{
    ROS_ERROR("Plugin loaded");
}

////////////////////////////////////////////////////////////////////////////////
FootContactModelPlugin::~FootContactModelPlugin()
{
    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
    this->rosNode->shutdown();
    this->rosQueue.clear();
    this->rosQueue.disable();
    this->callbackQueeuThread.join();
    delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
void FootContactModelPlugin::Load(physics::ModelPtr _parent,
                       sdf::ElementPtr _sdf)
{
    this->model = _parent;

    // Get the world name.
    this->world = this->model->GetWorld();
    this->sdf = _sdf;

    // ros callback queue for processing subscription
    this->deferredLoadThread = boost::thread(
                boost::bind(&FootContactModelPlugin::DeferredLoad, this));

    ROS_ERROR("Load function");
}


////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
void FootContactModelPlugin::DeferredLoad()
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

    // ros callback queue for processing subscription
    this->callbackQueeuThread = boost::thread(
                boost::bind(&FootContactModelPlugin::RosQueueThread, this));

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&FootContactModelPlugin::UpdateStates, this));

    ROS_ERROR("Deferred Load");
}

void FootContactModelPlugin::UpdateStates()
{
    ROS_ERROR("Update");
    gazebo::physics::Collision_V collisions = model->GetLink("reemc::leg_right_6_link")->GetCollisions();

    for (gazebo::physics::Collision_V::iterator iter = collisions.begin(); iter!= collisions.end(); ++iter)
    {
        if (*iter != 0)
        {
            gazebo::physics::SurfaceParamsPtr params = (*iter)->GetSurface();

            ROS_INFO_STREAM("Right Foot Collision params " << std::endl <<
                        "bounce : " << params->bounce << std::endl <<
                        "bounceThreshold : " << params->bounceThreshold  << std::endl <<
                        "cfm : " << params->cfm  << std::endl <<
                        "erp : " << params->erp  << std::endl <<
                        "fdir1 : " << params->fdir1  << std::endl <<
                        "kd : " << params->kd  << std::endl <<
                        "kp : " << params->kp  << std::endl <<
                        "maxVel : " << params->maxVel  << std::endl <<
                        "minDepth : " << params->minDepth  << std::endl <<
                        "mu1 : " << params->mu1  << std::endl <<
                        "mu2 : " << params->mu2  << std::endl <<
                        "slip1 : " << params->slip1  << std::endl <<
                        "slip2 : " << params->slip2  );
        }
    }

}

void FootContactModelPlugin::RosQueueThread()
{
    static const double timeout = 0.01;

    while (this->rosNode->ok())
    {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}
}


