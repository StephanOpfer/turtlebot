#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "turtlebot_gazebo_distance_sensor/DistanceSensorPlugin.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>
#include <vector>
#include <cstdlib>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <ttb_msgs/LogicalCamera.h>

using std::vector;

namespace gazebo
{
	// Register this plugin with the simulator
	GZ_REGISTER_SENSOR_PLUGIN(GazeboRosDistance)

	////////////////////////////////////////////////////////////////////////////////
	// Constructor
	GazeboRosDistance::GazeboRosDistance()
	{
		auto robot = getenv("ROBOT");
		const char *topicName = "logical_camera";
		char topic[100];

		snprintf(topic, 100, "/%s/%s", robot, topicName);

		modelPub = nh.advertise<ttb_msgs::LogicalCamera>(topic, 1000);
		this->sensorYaw = 0;
		this->sc = supplementary::SystemConfig::getInstance();

		ROS_INFO("Loading models form SystemConfig...");
		loadModelsFromConfig();
		this->modelNames = nullptr;
	}

	////////////////////////////////////////////////////////////////////////////////
	// Destructor
	GazeboRosDistance::~GazeboRosDistance()
	{
		ROS_DEBUG_STREAM_NAMED("camera", "Unloaded");
	}

	void GazeboRosDistance::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
	{
		// Make sure the ROS node for Gazebo has already been initialized
		if (!ros::isInitialized())
		{
			ROS_FATAL_STREAM(
					"A ROS node for Gazebo has not been initialized, unable to load plugin. " << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			return;
		}

		// Get the parent sensor.
		this->parentSensor = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(_sensor);

		// Make sure the parent sensor is valid.
		if (!this->parentSensor)
		{
			gzerr << "GazeboRosDistance requires a LogicalCamera Sensor.\n";
			return;
		}

		// Connect to the sensor update event.
		this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&GazeboRosDistance::OnUpdate, this));

		// Make sure the parent sensor is active.
		this->parentSensor->SetActive(true);
		this->sensorYaw = this->parentSensor->Pose().Rot().Yaw();

		ROS_INFO("GazeboRosDistance Plugin loaded!");
	}

	void GazeboRosDistance::OnUpdate()
	{
		bool debug = false;
		vector<msgs::LogicalCameraImage_Model> victimModels;

		// Get all the models in range.
		auto models = this->parentSensor->Image();
		for (int i = 0; i < models.model_size(); i++)
		{
			for (auto& kv : modelMap) {
			    auto m = models.model(i);
			    auto cm = kv.second;
			    if (isDetected(m, cm)) {
			    	publishModel(m, cm);
				}
			}
		}

	}

	bool GazeboRosDistance::isDetected(msgs::LogicalCameraImage_Model model, Model configModel)
	{
		auto modelName = model.name();
		auto gazeboElementName = configModel.type;
		transform(gazeboElementName.begin(), gazeboElementName.end(), gazeboElementName.begin(), ::tolower);
		auto x = model.pose().position().x();
		auto y = model.pose().position().y();
		auto z = model.pose().position().z();
		auto dist = sqrt(x * x + y * y + z * z);

		// Debugging output
//		std::cout << this->parentSensor->Far() << " " << this->parentSensor->Near() << " " << this->parentSensor->HorizontalFOV() << std::endl;
//		printf("Model No. %d with Name %s at (%.1f, %.1f, %.1f) dist: %.2f\n", i, n.c_str(), x, y, z, dist);

		// Model name did not match desired string
		if (modelName.find(gazeboElementName) == std::string::npos)
			return false;

		// TODO: modelMap.at == Model?
		// Model is too far aways
		if (dist > configModel.range)
			return false;

		// TODO: Explain?
		double angle = atan2(y, x) * 180.0 / M_PI;

		// Normalize angles?
		if (this->sensorYaw != 0)
			angle = (angle < 0) ? angle + 180.0 : angle - 180;

//		cout << "GazeboRosDistance: Victim angle: " << angle << endl;

		if (!angleRangeCheck(angle, configModel.detectAngles))
			return false;

		return true;
	}

	void GazeboRosDistance::publishModel(msgs::LogicalCameraImage_Model model, GazeboRosDistance::Model configModel)
	{
		auto x = model.pose().position().x();
		auto y = model.pose().position().y();
		auto z = model.pose().position().z();

		ttb_msgs::LogicalCamera msg;
		msg.modelName = model.name();
		msg.pose.x = x;
		msg.pose.y = y;

		if (true)
			printf("Victim found with Name %s at (%f, %f, %f)\n", model.name().c_str(), x, y, z);

		auto q = model.pose().orientation();
		msg.pose.theta = quadToTheata(q.x(), q.y(), q.z(), q.w());
		msg.timeStamp = ros::Time::now();
		msg.type = configModel.type;

		// TODO: Handle Publishing Rate
		modelPub.publish(msg);
	}

	void GazeboRosDistance::loadModelsFromConfig()
	{
		const char* lc = "LogicalCamera";
		const char* da = "DetectAngles";

		auto config = (*this->sc)[lc];
		this->modelNames = config->getSections(lc, NULL);

		for (auto section : *(this->modelNames))
		{
			const char* sec = section.c_str();
			cout << "GazeboRosDistance: section: " << section << endl;

			Model m;
			// TODO: NULL needed?
			m.range = config->get<double>(lc, sec, "range", NULL);

			auto angleSections = config->getSections(lc, sec, da, NULL);
			for (auto angleSection : *angleSections)
			{
				cout << "GazeboRosDistance: angleSection: " << angleSection << endl;
				auto start = config->get<double>(lc, sec, da, angleSection.c_str(), "startAngle", NULL);
				auto end = config->get<double>(lc, sec, da, angleSection.c_str(), "endAngle", NULL);

				m.detectAngles.push_back(pair<double, double>(start, end));
			}

			m.type = config->get<std::string>("LogicalCamera", sec, "type", NULL);
			m.section = section;
			m.name = section;

			for (auto ang : m.detectAngles)
				printf("%d, %d\n", ang.first, ang.second);

			this->modelMap.emplace(m.section, m);
		}
	}

	bool GazeboRosDistance::angleRangeCheck(double angle, vector<pair<double, double>> detectAngles)
	{
		for (auto pair : detectAngles)
		{
			printf("checking pair %f - %f\n", pair.first, pair.second);
			if (pair.first <= pair.second)
			{
				return pair.first <= angle && angle <= pair.second;
			} else {
				// this is only the case when the angle range crosses over 180°
				return pair.first <= angle || angle <= pair.second;
			}
		}
		return false;
	}

	// See:
	// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	double GazeboRosDistance::quadToTheata(double x, double y, double z, double w)
	{
		double ysqr = y * y;
		double t0 = -2.0f * (ysqr + z * z) + 1.0f;
		double t1 = +2.0f * (x * y - w * z);
//		double t2 = -2.0f * (x * z + w * y);
//		double t3 = +2.0f * (y * z - w * x);
//		double t4 = -2.0f * (x * x + ysqr) + 1.0f;

//		t2 = t2 > 1.0f ? 1.0f : t2;
//		t2 = t2 < -1.0f ? -1.0f : t2;

		//pitch = std::asin(t2);
		//roll = std::atan2(t3, t4);
		double yaw = atan2(t1, t0);

		return yaw;
	}

}
