#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include <string>

#include <ros/ros.h>

namespace gazebo
{
    /// \class LogicalOcclusionPlugin LogicalOcclusionPlugin.hh
    /// \brief Adds occlusion checking through ray casting filtering out LogicalCameraImage
    ///        the result is sended as a ROS LogicalCameraImage message
    class GAZEBO_VISIBLE LogicalOcclusionPlugin : public SensorPlugin
    {
      /// \brief Constructor
      public: LogicalOcclusionPlugin();

      /// \brief Destructor
      public: virtual ~LogicalOcclusionPlugin();

      // Documentation inherited
      public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      
      // Documentation inherited
      public: virtual void Fini();

      // Callback that receives the contact sensor's update signal
      protected: virtual void OnUpdate();
      
    private:
    
      /**
      * \brief Models configured as "occluding" will be checked for occlusion throught ray casting
      * @param model model detected by sensor
      */
      bool isDetected(msgs::LogicalCameraImage_Model model);
      
      std::string DetermineModelType(const std::string &modelName);
    
      // Sensor ptr
      sensors::LogicalCameraSensorPtr parentSensor;
      
      // Link between the contact sensor's updated signal and callback.
      event::ConnectionPtr updateConnection;
      
      // Publisher
      ros::NodeHandle nh;
      ros::Publisher occludedPub;
      
      // Model type that should be considered and occluded
      std::string occludingType;
      
      // Number of LogicalCameraImages transmitted
      uint32_t seq_number;
      
      // Engine pointer for ray collision initialization
      physics::PhysicsEnginePtr physicsEngine;
      
      // Collision and shape ptrs
      physics::CollisionPtr laserCollision;
      physics::RayShapePtr rayShape;
    
    };
}
