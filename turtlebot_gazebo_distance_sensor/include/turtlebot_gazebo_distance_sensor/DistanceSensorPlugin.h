#ifndef GAZEBO_ROS_DISTANCE_SENSOR_HH
#define GAZEBO_ROS_DISTANCE_SENSOR_HH

#include <string>

// library for processing camera data for gazebo / ros conversions

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>
#include <SystemConfig.h>
#include <chrono>

//#define LOGICAL_CAMERA_DEBUG

using namespace std;

namespace gazebo
{
	class GazeboRosDistance : public SensorPlugin
	{
	public:
		/**
		 * Constructor
		 */
		GazeboRosDistance();

		/**
		 * Destructor.
		 */
		virtual ~GazeboRosDistance();

		/**
		 * Load the sensor plugin.
		 * @param _sensor Pointer to the sensor that loaded this plugin.
		 * @param _sdf SDF element that describes the plugin.
		 */

		virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

	private:
		/**
		 * Callback that receives the contact sensor's update signal.
		 */
		virtual void OnUpdate();

		struct ConfigModel
		{
			double range;
			/**
			 * pair.first = startAngle
			 * pair.secong = endAngle
			 */
			vector<pair<double, double>> detectAngles;
			string name;
			string type;
			double publishingRate;
			bool alreadyPublished = false;
			chrono::time_point<chrono::high_resolution_clock> lastPublished;
		};

		/**
		 * Connection that maintains a link between the contact sensor's
		 * updated signal and the OnUpdate callback.
		 */

		event::ConnectionPtr updateConnection;
		// ros stuff
		ros::NodeHandle nh;
		ros::Publisher modelPub;
		// Sensor orientation
		double sensorYaw;
		supplementary::SystemConfig* sc;
		// Sensor ptr
		sensors::LogicalCameraSensorPtr parentSensor;
		// Name of Robot
		string robotName;
		// Section in config file
		shared_ptr<vector<string>> modelSectionNames;
		// Maps model Name to config model
		map<string, ConfigModel> modelMap;

		/**
		 * load model parameters from config file
		 */
		void loadModelsFromConfig();
		/**
		 * @param model model detected by sensor
		 * @param configModel config lodaded by systemconf
		 */
		void publishModel(msgs::LogicalCameraImage_Model model, GazeboRosDistance::ConfigModel& configModel);
		/**
		 * @param angle model detected by sensor
		 * @param detectAngles config lodaded by systemconf
		 */
		bool isInAngleRange(double angle, vector<pair<double, double>> detectAngles);
		/**
		 * calculates angle of object from quaternium
		 */
		double quadToTheata(double x, double y, double z, double w);
		/**
		 * @param model model detected by sensor
		 * @param configModel config lodaded by systemconf
		 */
		bool isDetected(msgs::LogicalCameraImage_Model model, GazeboRosDistance::ConfigModel& configModel);

	};
}
#endif
