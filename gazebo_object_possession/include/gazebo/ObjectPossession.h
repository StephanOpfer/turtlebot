#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

#include <cstdlib>
#include <functional>
#include <iostream>
#include <stdio.h>

namespace essentials
{
class SystemConfig;
}

namespace gazebo
{
class ObjectPossession : public WorldPlugin
{
public:
    ObjectPossession();
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate(const common::UpdateInfo& /*_info*/);

private:
    // Pointer to the model
    physics::WorldPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    ros::Subscriber doorCmdSub;
    ros::AsyncSpinner* spinner;

    essentials::SystemConfig* sc;
};
} // namespace gazebo
