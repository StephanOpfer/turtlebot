#include <gazebo/common/Plugin.hh>

#include "turtlebot_gazebo_arm/ArmPlugin.h"

#include <cstdlib>
#include <gazebo/physics/Model.hh>
#include <numeric>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

namespace gazebo
{

ArmPlugin::ArmPlugin()
{
    this->spinner = nullptr;
    this->transportedModel = nullptr;
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
    std::stringstream ss;
    ss << "/" << this->model->GetName() << "/ArmCmd";
    this->armCmdSub = n.subscribe(ss.str(), 10, &ArmPlugin::onGrabDropObjectCmd, (ArmPlugin *)this);
    this->spinner = new ros::AsyncSpinner(4);
    this->spinner->start();

    ROS_INFO("ArmPlugin PlugIn loaded!");
}

void ArmPlugin::OnUpdate(const common::UpdateInfo &info)
{
    if (this->transportedModel != nullptr)
    {
        // std::cout << "ArmPlugin::OnGrab: " << model->GetName() << std::endl;
        auto ownPos = this->model->GetWorldPose();
        ownPos.pos.z += 1;
        this->transportedModel->SetWorldPose(ownPos);
    }
}

void ArmPlugin::onGrabDropObjectCmd(ttb_msgs::GrabDropObjectPtr msg)
{
    std::cout << "ArmPlugin::OnGrab: Msg Name " << msg->objectName << std::endl;
    if(msg->objectName.empty())
    {
    	return;
    }
    if (msg->action == ttb_msgs::GrabDropObject::GRAB && this->transportedModel == nullptr)
    {
        auto modelToCarry = this->world->GetModel(msg->objectName);
        if (modelToCarry != nullptr)
        {
            auto robotPosition = this->model->GetWorldPose().pos;
            auto transportedModelPose = modelToCarry->GetWorldPose().pos;
            double distance = sqrt(pow(transportedModelPose.x - robotPosition.x, 2) + pow(transportedModelPose.y - robotPosition.y, 2));
            std::cout << "ArmPlugin: distance to model " << msg->objectName << ": " << distance << std::endl;
            if (distance <= armRange)
            {
                this->transportedModel = modelToCarry;
                this->transportedModel->SetStatic(false);
            }
        }
        else
        {
            std::cout << "ArmPlugin::OnGrab: Unknown Object with name: " << msg->objectName << std::endl;
        }
    }
    else if (msg->action == ttb_msgs::GrabDropObject::DROP && this->transportedModel != nullptr)
    {
        if (msg->objectName == this->transportedModel->GetName())
        {
            auto targetPos = this->transportedModel->GetWorldPose();
            // TODO setting x does not work properly
            targetPos.pos.x += 1.0;
            this->transportedModel->SetWorldPose(targetPos);
            this->transportedModel->SetStatic(true);
            // Calling update is neceessary to place an object in x direction
            // y and z work fine ...
            this->transportedModel->Update();
            this->transportedModel = nullptr;
        }
        else
        {
            std::cout << "ArmPlugin::OnDrop: not carrying an object with name: " << msg->objectName << std::endl;
        }
    }
    else
    {
        std::cout << "ArmPlugin::OnGrab: already carrying an object or no object to drop! " << std::endl;
    }

    /**
     * std::cout << "ArmPlugin::OnGrab: Model Name " << modelToGrab->GetName() << std::endl;
        auto footprint = this->model->GetLink("base_footprint");
        std::cout << "ArmPlugin::OnGrab: Link Name " << footprint->GetName() << std::endl;

        modelToGrab->SetStatic(false);
        auto parent = modelToGrab->Get


        //    auto links = this->model->GetLinks();
        //    for (auto& link : links)
        //    {
        //    	std::cout << "ArmPlugin::OnGrab: " << link->GetName() << std::endl;
        //    }

        math::Pose pose;
        pose.Set(1, 0, 1, 0, 0, 0);

        this->model->AttachStaticModel(modelToGrab, pose);
     */
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ArmPlugin)
}
