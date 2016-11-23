#ifndef GAZEBO_ROS_DISTANCE_SENSOR_HH
#define GAZEBO_ROS_DISTANCE_SENSOR_HH

#include <string>

// library for processing camera data for gazebo / ros conversions

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>
#include <SystemConfig.h>

//#define LOGICAL_CAMERA_DEBUG

using namespace std;

namespace gazebo
{
	class GazeboRosDistance : public SensorPlugin
	{
	public:
		/// \brief Constructor
		GazeboRosDistance();

		/// \brief Destructor.
		virtual ~GazeboRosDistance();

		/// \brief Load the sensor plugin.
		/// \param[in] _sensor Pointer to the sensor that loaded this plugin.
		/// \param[in] _sdf SDF element that describes the plugin.
		virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

		/// \brief Callback that receives the contact sensor's update signal.
	private:
		virtual void OnUpdate();

		/// \brief Pointer to the logical camera sensor
	private:
		sensors::LogicalCameraSensorPtr parentSensor;

		/// \brief Connection that maintains a link between the contact sensor's
		/// updated signal and the OnUpdate callback.
		event::ConnectionPtr updateConnection;
		ros::NodeHandle nh;
		ros::Publisher modelPub;
		double sensorYaw;
		supplementary::SystemConfig* sc;
		void loadModelsFromConfig();
		double quadToTheata(double x, double y, double z, double w);
		bool angleRangeCheck(double angle, vector<pair<double, double>> detectAngles);
		shared_ptr<vector<string>> modelNames;

		struct Model
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
		};
		map<string, Model> modelMap;
		bool isDetected(msgs::LogicalCameraImage_Model model, GazeboRosDistance::Model configModel);
		void publishModel(msgs::LogicalCameraImage_Model model, GazeboRosDistance::Model configModel);

	};
}
#endif
