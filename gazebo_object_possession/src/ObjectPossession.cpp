#include "gazebo/ObjectPossession.h"

#include <SystemConfig.h>

#include <ros/ros.h>

#include <math.h>
#include <algorithm>

using std::cerr;
using std::cout;
using std::endl;
using std::string;

namespace gazebo
{

ObjectPossession::ObjectPossession()
    : WorldPlugin()
{
    this->spinner = nullptr;
    this->sc = supplementary::SystemConfig::getInstance();
}

void ObjectPossession::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    this->model = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection =
        event::Events::ConnectWorldUpdateBegin(boost::bind(&ObjectPossession::OnUpdate, this, _1));

    ros::NodeHandle n;
    //this->doorCmdSub = n.subscribe("/DoorCmd", 10, &HingedDoorController::handleDoorCmd, (HingedDoorController *)this);
    this->spinner = new ros::AsyncSpinner(4);
    this->spinner->start();
}

// Called by the world update start event
void ObjectPossession::OnUpdate(const common::UpdateInfo & /*_info*/)
{
    // Apply a small linear velocity to the model.
    // this->model->SetLinearVel(math::Vector3(.03, 0, 0));
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ObjectPossession)
}
