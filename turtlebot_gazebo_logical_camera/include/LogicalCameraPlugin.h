#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <ros/ros.h>

//#define LOGICAL_OCCLUSION_DEBUG
//#define LOGICAL_CAMERA_DEBUG
//#define LOGICAL_CAMERA_DEBUG_POINTS
//#define LOGICAL_CAMERA_RUNTIME_DEBUG

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
    * \brief Models configured as "occluding" will be checked for occlusion through ray casting
    * @param model model detected by sensor
    */
    bool isDetected(msgs::LogicalCameraImage_Model model, LogicalCameraPlugin::ConfigModel configModel, gazebo::math::Pose &outCorrectedPose);
    bool isSensorResponsible(msgs::LogicalCameraImage_Model model);
    bool isInRange(gazebo::math::Vector3 modelPosition, double range);
    bool isInAngleRange(gazebo::math::Pose &pose, std::vector<std::pair<double, double>> detectAngles);
    bool isVisible(gazebo::math::Pose correctedPose, msgs::LogicalCameraImage_Model model);
    void publishModel(msgs::LogicalCameraImage_Model model, LogicalCameraPlugin::ConfigModel &configModel, gazebo::math::Pose outCorrectedPose);
    double quaternionToYaw(double x, double y, double z, double w);

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
    physics::WorldPtr world;

    // Collision and shape ptrs
    physics::CollisionPtr laserCollision;
    physics::RayShapePtr rayShape;
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
    std::string debugName;
#endif
    // time points of messages sent, need to determine when to send
    // next message according specified configuration frequency
    std::map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>> lastPublishedMap;

    // Sensor orientation
    double sensorYaw;
    supplementary::SystemConfig *sc;
    double quadNear;
    double quadFar;

#ifdef LOGICAL_CAMERA_RUNTIME_DEBUG
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    std::chrono::time_point<std::chrono::high_resolution_clock> end;
#endif


    std::string robotName;
    void createDebugPoint(std::string sdfString, std::string positionString, std::string name);
    void moveDebugPoint(std::string name, gazebo::math::Pose& pose);
};
}
