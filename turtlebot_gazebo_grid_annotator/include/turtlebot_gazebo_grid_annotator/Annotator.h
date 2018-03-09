#pragma once

#include <ttb_msgs/Grid.h>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <ros/ros.h>

namespace gazebo
{

class Annotator : public WorldPlugin
{
  public:
    Annotator();
    virtual ~Annotator();

    /**
	 * Load the sensor plugin.
	 * @param _sensor Pointer to the sensor that loaded this plugin.
	 * @param _sdf SDF element that describes the plugin.
	 */
    virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

  private:
    void onGrid(ttb_msgs::GridPtr grid);
    bool isCloserAndVisible(gazebo::physics::ModelPtr poi, geometry_msgs::Point point, double& minDist);

    ros::NodeHandle n;
    ros::Subscriber gridSubscriber;
    ros::Publisher annotatedGridPublisher;
    ros::AsyncSpinner *spinner;

    physics::WorldPtr world;
	physics::RayShapePtr rayShape;
};

} /* namespace gazebo */
