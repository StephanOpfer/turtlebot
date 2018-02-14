#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <memory>
#include <string>
#include <vector>


#include <ros/ros.h>

//#define LOGICAL_OCCLUSION_DEBUG
//#define LOGICAL_CAMERA_DEBUG
#define LOGICAL_CAMERA_DEBUG_POINTS

namespace supplementary
{
class SystemConfig;
}

namespace gazebo
{
/// \class LogicalOcclusionPlugin LogicalOcclusionPlugin.hh
/// \brief Adds occlusion checking through ray casting filtering out LogicalCameraImage
///        the result is sended as a ROS LogicalCameraImage message
class GAZEBO_VISIBLE LogicalCameraPlugin : public SensorPlugin
{
    /// \brief Constructor
  public:
    LogicalCameraPlugin();

    /// \brief Destructor
  public:
    virtual ~LogicalCameraPlugin();

    // Documentation inherited
  public:
    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    // Documentation inherited
  public:
    virtual void Fini();

    // Callback that receives the contact sensor's update signal
  protected:
    virtual void OnUpdate();

  private:
    struct ConfigModel
    {
        double range;
        /**
         * pair.first = startAngle
         * pair.secong = endAngle
         */
        double publishingRate;
        std::vector<std::pair<double, double>> detectAngles;
        std::string name;
        std::string type;
    };
    /**
    * \brief Models configured as "occluding" will be checked for occlusion throught ray casting
    * @param model model detected by sensor
    */
    bool isDetected(msgs::LogicalCameraImage_Model model, LogicalCameraPlugin::ConfigModel configModel, gazebo::msgs::Pose &outCorrectedPose);
    bool isSensorResponsible(msgs::LogicalCameraImage_Model model);
    bool isInAngleRange(double angle, std::vector<std::pair<double, double>> detectAngles);
    bool isOccluded(gazebo::msgs::Pose &outCorrectedPose, std::string name);
    void publishModel(msgs::LogicalCameraImage_Model model, LogicalCameraPlugin::ConfigModel &configModel, gazebo::msgs::Pose &outCorrectedPose);
    double quaterniumToYaw(double x, double y, double z, double w);
    /**
	 * calculates the angle from the robot to a model
	 */
    double calculateAngle(double x, double y);
    void loadModelsFromConfig();
    void loadOccludingTypes();
    /**
	 * calculates angle of object from quaternium
	 */
    // Sensor ptr
    sensors::LogicalCameraSensorPtr parentSensor;

    // Link between the contact sensor's updated signal and callback.
    event::ConnectionPtr updateConnection;

    // Publisher
    ros::NodeHandle nh;
    ros::Publisher modelPub;

    // Model types that should be considered and occluded
    std::vector<std::string> occludingTypes;
    // Maps model Name to config model
    std::shared_ptr<std::vector<std::string>> modelSectionNames;
    std::map<std::string, ConfigModel> modelMap;

    // Number of LogicalCameraImages transmitted
    uint32_t seq_number;

    // Engine pointer for ray collision initialization
    physics::PhysicsEnginePtr physicsEngine;

    // Collision and shape ptrs
    physics::CollisionPtr laserCollision;
    physics::RayShapePtr rayShape;
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
    bool test;
#endif
    // time points of messages sent, need to determine when to send
    // next message according specified configuration frequency
    std::map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>> lastPublishedMap;

    // Sensor orientation
    double sensorYaw;
    supplementary::SystemConfig *sc;

    std::string robotName;
};
}
