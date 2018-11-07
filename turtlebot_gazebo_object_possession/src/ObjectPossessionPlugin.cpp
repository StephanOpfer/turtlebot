#include <gazebo/common/Plugin.hh>

#include "turtlebot_gazebo_object_possession/ObjectPossessionPlugin.h"

#include <cstdlib>
#include <gazebo/physics/Model.hh>
#include <numeric>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

namespace gazebo
{

ObjectPossessionPlugin::ObjectPossessionPlugin()
{
    this->spinner = nullptr;
}

ObjectPossessionPlugin::~ObjectPossessionPlugin()
{
    ROS_DEBUG_STREAM_NAMED("ObjectPossessionPlugin", "Unloaded");
}

void ObjectPossessionPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    this->world = _parent;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ObjectPossessionPlugin::OnUpdate, this, _1));

    this->armCmdSub = n.subscribe("/ArmCmd", 10, &ObjectPossessionPlugin::onGrabDropObjectCmd, (ObjectPossessionPlugin*) this);
    this->armCmdPub = this->n.advertise<ttb_msgs::GrabDropObject>("/ArmCmdResponse", 10);
    this->spinner = new ros::AsyncSpinner(4);
    this->spinner->start();

    ROS_INFO("ObjectPossessionPlugin PlugIn loaded!");
}

void ObjectPossessionPlugin::OnUpdate(const common::UpdateInfo& info)
{
    // rosNode->advertise<process_manager::ProcessStats>(this->processStatsTopic, 10);
}

void ObjectPossessionPlugin::onGrabDropObjectCmd(ttb_msgs::GrabDropObjectPtr msg)
{
    std::lock_guard<std::mutex> lock(publisherMutex);

    if (msg->action == msg->GRAB) {
        bool objectIsAvailable = true;
        for (auto pair : this->objectPossession) {
            if (pair.second.compare(msg->objectName) == 0) {
                objectIsAvailable = false;
                break;
            }
        }
        bool robotMayCarry = true;
        if (objectIsAvailable) {
            auto iter = this->objectPossession.find(msg->senderName);
            if (iter == this->objectPossession.end()) {
                this->objectPossession.emplace(msg->senderName, msg->objectName);
            } else {
                if (iter->second.empty()) {
                    iter->second = msg->objectName;
                } else {
                    robotMayCarry = false;
                }
            }
        }

        if (!objectIsAvailable || !robotMayCarry) {
            msg->objectName = "";
            this->armCmdPub.publish(msg);
        }
        if (objectIsAvailable && robotMayCarry) {
            this->armCmdPub.publish(msg);
        }
    } else {
        bool robotMayDrop = true;
        auto iter = this->objectPossession.find(msg->senderName);
        if (iter == this->objectPossession.end()) {
            robotMayDrop = false;
        } else {
            if (iter->second.compare(msg->objectName) == 0) {
                iter->second = "";
            } else {
                robotMayDrop = false;
            }
        }
        if (!robotMayDrop) {
            msg->objectName = "";
        }
        this->armCmdPub.publish(msg);
    }
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ObjectPossessionPlugin)
} // namespace gazebo
