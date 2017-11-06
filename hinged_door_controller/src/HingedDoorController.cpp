#include "gazebo/HingedDoorController.h"

namespace gazebo {

HingedDoorController::HingedDoorController() :
		ModelPlugin() {
}

void HingedDoorController::Load(physics::ModelPtr _parent,
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
	this->model->GetJoint("hinge")->SetLowerLimit(2, -M_PI);
	this->model->GetJoint("hinge")->SetUpperLimit(2, M_PI);
	this->model->GetJoint("hinge")->SetPosition(0, M_PI/2);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HingedDoorController)
}
