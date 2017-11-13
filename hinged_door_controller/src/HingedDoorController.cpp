#include "gazebo/HingedDoorController.h"

using std::cout;
using std::endl;
using std::string;

namespace gazebo {

HingedDoorController::HingedDoorController() :
		WorldPlugin() {
	this->spinner = nullptr;
}

void HingedDoorController::Load(physics::WorldPtr _parent,
		sdf::ElementPtr _sdf) {
	// Store the pointer to the model
	this->model = _parent;

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&HingedDoorController::OnUpdate, this, _1));

	ros::NodeHandle n;
	this->doorCmdSub = n.subscribe("/DoorCmd", 10,
			&HingedDoorController::handleDoorCmd, (HingedDoorController*) this);
	this->spinner = new ros::AsyncSpinner(4);
	this->spinner->start();
}

// Called by the world update start event
void HingedDoorController::OnUpdate(const common::UpdateInfo & /*_info*/) {
	// Apply a small linear velocity to the model.
	//this->model->SetLinearVel(math::Vector3(.03, 0, 0));
}

void HingedDoorController::handleDoorCmd(
		hinged_door_controller::DoorCmdPtr cmd) {
	string modelName = cmd->name;
	this->model->GetModel(modelName)->GetJoint("hinged_door::hinge")->SetLowerLimit(2, -M_PI);
	this->model->GetModel(modelName)->GetJoint("hinged_door::hinge")->SetUpperLimit(2, M_PI);
	if(cmd->state == hinged_door_controller::DoorCmd::OPEN)
	{
		this->model->GetModel(modelName)->GetJoint("hinged_door::hinge")->SetPosition(0, M_PI/2);
	}
	else
	{
		this->model->GetModel(modelName)->GetJoint("hinged_door::hinge")->SetPosition(0, 0);
	}
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(HingedDoorController)
}
