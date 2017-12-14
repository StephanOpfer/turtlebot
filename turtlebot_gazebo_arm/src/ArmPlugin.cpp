#include <gazebo/common/Plugin.hh>

#include "turtlebot_gazebo_arm/ArmPlugin.h"

#include <cstdlib>
#include <gazebo/physics/Model.hh>
#include <numeric>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace gazebo
{

ArmPlugin::ArmPlugin()
{
    this->spinner = nullptr;
}

ArmPlugin::~ArmPlugin()
{
    ROS_DEBUG_STREAM_NAMED("arm", "Unloaded");
}

void ArmPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    this->model = _parent;
    this->world = this->model->GetWorld();

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ArmPlugin::OnUpdate, this, _1));

    ros::NodeHandle n;
    this->doorCmdSub = n.subscribe("/ArmCmd", 10, &ArmPlugin::onGrabDropObjectCmd, (ArmPlugin *)this);
    this->spinner = new ros::AsyncSpinner(4);
    this->spinner->start();

    ROS_INFO("ArmPlugin PlugIn loaded!");
}

void ArmPlugin::OnUpdate(const common::UpdateInfo &info)
{
    if (this->transportedModel != nullptr)
    {
        //std::cout << "ArmPlugin::OnGrab: " << model->GetName() << std::endl;
        auto ownPos = this->model->GetWorldPose();
        ownPos.pos.z += 1;
        this->transportedModel->SetWorldPose(ownPos);
    }
}

void ArmPlugin::onGrabDropObjectCmd(turtlebot_gazebo_arm::GrabDropObjectPtr msg)
{
    std::cout << "ArmPlugin::OnGrab: " << model->GetName() << std::endl;
    std::cout << "ArmPlugin::OnGrab: Msg Name " << msg->objectName << std::endl;
    this->transportedModel = this->world->GetModel(msg->objectName);
    this->transportedModel->SetStatic(false);
    //    std::cout << "ArmPlugin::OnGrab: Model Name " << modelToGrab->GetName() << std::endl;
    //    auto footprint = this->model->GetLink("base_footprint");
    //    std::cout << "ArmPlugin::OnGrab: Link Name " << footprint->GetName() << std::endl;
    //
    //    modelToGrab->SetStatic(false);
    //    auto parent = modelToGrab->Get
    //
    //
    //    //    auto links = this->model->GetLinks();
    //    //    for (auto& link : links)
    //    //    {
    //    //    	std::cout << "ArmPlugin::OnGrab: " << link->GetName() << std::endl;
    //    //    }
    //
    //    math::Pose pose;
    //    pose.Set(1, 0, 1, 0, 0, 0);
    //
    //    this->model->AttachStaticModel(modelToGrab, pose);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ArmPlugin)
}
