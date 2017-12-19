#include "gazebo/HingedDoorController.h"

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

HingedDoorController::HingedDoorController()
    : WorldPlugin()
{
    this->spinner = nullptr;
    this->sc = supplementary::SystemConfig::getInstance();
}

void HingedDoorController::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    this->model = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection =
        event::Events::ConnectWorldUpdateBegin(boost::bind(&HingedDoorController::OnUpdate, this, _1));

    ros::NodeHandle n;
    this->doorCmdSub = n.subscribe("/DoorCmd", 10, &HingedDoorController::handleDoorCmd, (HingedDoorController *)this);
    this->spinner = new ros::AsyncSpinner(4);
    this->spinner->start();
}

// Called by the world update start event
void HingedDoorController::OnUpdate(const common::UpdateInfo & /*_info*/)
{
    // Apply a small linear velocity to the model.
    // this->model->SetLinearVel(math::Vector3(.03, 0, 0));
}

/**
 * Callback for DoorCmd msgs. Directly sets the position of the referenced door, if it exists.
 */
void HingedDoorController::handleDoorCmd(ttb_msgs::DoorCmdPtr cmd)
{
    // Get the hinge joint of the correct door, according to the given model name
	auto door = this->model->GetModel(cmd->name);
	if (!door)
	{
		std::cerr << "HingedDoorController: Received msg for " << cmd->name << " door, that does NOT EXIST!" << std::endl;
		return;
	}
    auto hingeJoint = door->GetJoint("hinged_door::hinge");

    double setAngle = 0.0; /* <-- 0.0 is the angle for closing a door */

    if (cmd->state == ttb_msgs::DoorCmd::OPEN)
    {
        // Get limits of that joint
    	setAngle = (*sc)["Doors"]->get<double>("Doors", cmd->name.c_str(), "openAngle", NULL);
    }

    hingeJoint->SetPosition(0, setAngle);
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(HingedDoorController)
}
