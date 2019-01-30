#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

//#define LOGICAL_OCCLUSION_DEBUG
//#define LOGICAL_CAMERA_DEBUG
//#define LOGICAL_CAMERA_DEBUG_POINTS
//#define LOGICAL_CAMERA_RUNTIME_DEBUG

namespace essentials
{
class SystemConfig;
}

namespace gazebo
{
/// \class LogicalOcclusionPlugin LogicalOcclusionPlugin.hh
/// \brief Adds occlusion checking through ray casting filtering out LogicalCameraImage
///        the result is sended as a ROS LogicalCameraImage message
class GAZEBO_VISIBLE LogicalCameraPlugin : public ModelPlugin
{
    /// \brief Constructor
public:
    LogicalCameraPlugin();

    /// \brief Destructor
public:
    virtual ~LogicalCameraPlugin();

    // Documentation inherited
public:
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Documentation inherited
public:
    virtual void Fini();

    // Callback that receives the contact sensor's update signal
protected:
    virtual void OnUpdate(const common::UpdateInfo& info);

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
    bool isDetected(gazebo::physics::ModelPtr model, LogicalCameraPlugin::ConfigModel configModel, ignition::math::Pose3d& outCorrectedPose);
    //    bool isSensorResponsible(gazebo::physics::ModelPtr model);
    bool isInRange(ignition::math::Vector3d modelPosition, double range);
    bool isInAngleRange(ignition::math::Pose3d& pose, std::vector<std::pair<double, double>> detectAngles);
    bool isVisible(ignition::math::Pose3d correctedPose, gazebo::physics::ModelPtr model);
    void publishModel(gazebo::physics::ModelPtr, LogicalCameraPlugin::ConfigModel& configModel, ignition::math::Pose3d outCorrectedPose);
    double quaternionToYaw(double x, double y, double z, double w);

    void loadModelsFromConfig();
    void loadOccludingTypes();
    void loadParameters();
    /**
     * calculates angle of object from quaternium
     */
    // Sensor ptr
    //    sensors::LogicalCameraSensorPtr parentSensor;

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
    physics::ModelPtr robotModel;

    // Collision and shape ptrs
    physics::CollisionPtr laserCollision;
    physics::RayShapePtr rayShape;

    // time points of messages sent, need to determine when to send
    // next message according specified configuration frequency
    std::map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>> lastPublishedMap;

    // Sensor orientation
    essentials::SystemConfig* sc;
    double quadNear;
    double quadFar;

#ifdef LOGICAL_CAMERA_RUNTIME_DEBUG
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    std::chrono::time_point<std::chrono::high_resolution_clock> end;
#endif

#ifdef LOGICAL_CAMERA_DEBUG_POINTS
    void createDebugPoint(std::string sdfString, std::string positionString, std::string name);
    void moveDebugPoint(std::string name, gazebo::math::Pose& pose);
    std::string debugName;
#endif
};
} // namespace gazebo
