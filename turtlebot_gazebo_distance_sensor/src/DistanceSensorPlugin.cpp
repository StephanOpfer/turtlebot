#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include "turtlebot_gazebo_distance_sensor/DistanceSensorPlugin.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>
#include <vector>
#include <cstdlib>
#include <numeric>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <ttb_msgs/LogicalCamera.h>

namespace gazebo
{
	// Register this plugin with the simulator
	GZ_REGISTER_SENSOR_PLUGIN(GazeboRosDistance)

	////////////////////////////////////////////////////////////////////////////////
	// Constructor
	GazeboRosDistance::GazeboRosDistance()
	{
		this->sensorYaw = 0;
		this->sc = supplementary::SystemConfig::getInstance();

		ROS_INFO("Loading models form SystemConfig...");
		loadModelsFromConfig();
		this->modelSectionNames = nullptr;
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
					"A ROS node for Gazebo has not been initialized, unable to load plugin. "
					<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
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

		// Extract robot name
		this->robotName = this->parentSensor->ParentName();
		size_t pos = robotName.find(':');
		if (pos == string::npos)
		{
			gzerr << "GazeboRosDistance robot/model name(" << robotName << ") is invalid!";
			return;
		}
		this->robotName = this->robotName.substr(0, pos);

#ifdef LOGICAL_CAMERA_DEBUG
		cout << "GazeboRosDistance: robot: " << this->robotName << endl;
#endif
		// Define topic
		const char *topicName = "logical_camera";
		char topic[100];

		snprintf(topic, 100, "/%s/%s", this->robotName.c_str(), topicName);

		// Publisher
		this->modelPub = this->nh.advertise<ttb_msgs::LogicalCamera>(topic, 1000);

		// Connect to the sensor update event.
		this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&GazeboRosDistance::OnUpdate, this));

		// Make sure the parent sensor is active.
		this->parentSensor->SetActive(true);

		// Get the sensor yaw, which is used to distinguish front and back sensor
		this->sensorYaw = this->parentSensor->Pose().Rot().Yaw();

		ROS_INFO("GazeboRosDistance PlugIn loaded!");
	}

	void GazeboRosDistance::OnUpdate()
	{
		vector<msgs::LogicalCameraImage_Model> victimModels;

		// Get all the models in range.
		auto models = this->parentSensor->Image();
		for (int i = 0; i < models.model_size(); i++)
		{
			for (auto& kv : this->modelMap)
			{
				auto model = models.model(i);
				if (isDetected(model, kv.second))
				{
					publishModel(model, kv.second);
				}
			}
		}

	}

	bool GazeboRosDistance::isDetected(msgs::LogicalCameraImage_Model model, GazeboRosDistance::ConfigModel& configModel)
	{
		auto modelName = model.name();
		auto gazeboElementName = configModel.type;
		transform(gazeboElementName.begin(), gazeboElementName.end(), gazeboElementName.begin(), ::tolower);
		auto x = model.pose().position().x();
		auto y = model.pose().position().y();
		auto z = model.pose().position().z();
		auto dist = sqrt(x * x + y * y + z * z);

		// Model name did not match desired string
		if (modelName.find(gazeboElementName) == std::string::npos)
		{
			return false;
		}

		// Model is too far aways
		if (dist > configModel.range)
		{
			return false;
		}

		/*
		 * angle to model calculated with egocentric coordinates
		 */
		double angle = atan2(y, x) * 180.0 / M_PI;

		/*
		 * change angle of backwards facing sensor get sensor range from 90° to 180°
		 * and from -90° to -180°
		 */
		if (this->sensorYaw != 0)
		{
			angle = (angle < 0) ? angle + 180.0 : angle - 180;
		}

		if (!isInAngleRange(angle, configModel.detectAngles))
		{
			return false;
		}

		return true;
	}

	void GazeboRosDistance::publishModel(msgs::LogicalCameraImage_Model model, GazeboRosDistance::ConfigModel& configModel)
	{
		auto x = model.pose().position().x();
		auto y = model.pose().position().y();
		auto z = model.pose().position().z();

		ttb_msgs::LogicalCamera msg;
		msg.modelName = model.name();
		msg.pose.x = x;
		msg.pose.y = y;

#ifdef LOGICAL_CAMERA_DEBUG
		cout << "Robot " << this->robotName << " found Model with Name " << model.name() << " at ( " << x << ", " << y << ", " << z << ")"
		<< endl;
#endif
		auto q = model.pose().orientation();
		msg.pose.theta = quadToTheata(q.x(), q.y(), q.z(), q.w());
		msg.timeStamp = ros::Time::now();
		msg.type = configModel.type;

		chrono::time_point<chrono::high_resolution_clock> t = chrono::high_resolution_clock::now();

		if (!configModel.alreadyPublished)
		{
			modelPub.publish(msg);
			configModel.lastPublished = t;
			configModel.alreadyPublished = true;
		}
		else
		{
			// Publish message if needed/specified hz from config exceeded
			auto diff = chrono::duration_cast<chrono::milliseconds>(t - configModel.lastPublished);
#ifdef  LOGICAL_CAMERA_DEBUG
			cout << "GazeboRosDistance: diff: " << diff.count()<< endl;
#endif
			if (diff.count() >= (1000.0 / configModel.publishingRate))
			{
				modelPub.publish(msg);
				configModel.lastPublished = t;
			}
		}
	}

	void GazeboRosDistance::loadModelsFromConfig()
	{
		const char* lc = "LogicalCamera";
		const char* da = "DetectAngles";

		auto config = (*this->sc)[lc];
		this->modelSectionNames = config->getSections(lc, NULL);

		// Iterate over all model sections in config file
		for (auto section : *(this->modelSectionNames))
		{
			const char* sec = section.c_str();
#ifdef  LOGICAL_CAMERA_DEBUG
			cout << "GazeboRosDistance: section: " << section << endl;
#endif
			ConfigModel m;
			m.range = config->get<double>(lc, sec, "range", NULL);

			// Add all angles specified in model to detectAngles vector
			auto angleSections = config->getSections(lc, sec, da, NULL);
			for (auto angleSection : *angleSections)
			{
				auto start = config->get<double>(lc, sec, da, angleSection.c_str(), "startAngle", NULL);
				auto end = config->get<double>(lc, sec, da, angleSection.c_str(), "endAngle", NULL);
#ifdef  LOGICAL_CAMERA_DEBUG
				cout << "GazeboRosDistance: angleSection: " << angleSection << " from : " << start << " to: " << end << endl;
#endif
				m.detectAngles.push_back(pair<double, double>(start, end));
			}

			m.type = config->get<string>(lc, sec, "type", NULL);
			m.name = section;
			m.publishingRate = config->get<double>(lc, sec, "publishingRateHz", NULL);
			this->modelMap.emplace(m.name, m);
		}
	}

	bool GazeboRosDistance::isInAngleRange(double angle, vector<pair<double, double>> detectAngles)
	{
		// all angles have to be checked so no return after first pair is checked
		for (auto pair : detectAngles)
		{
#ifdef  LOGICAL_CAMERA_DEBUG
			cout << "GazeboRosDistance: checking pair: (" << pair.first << " : " << pair.second << ")" << endl;
#endif
			if (pair.first <= pair.second)
			{
				if (pair.first <= angle && angle <= pair.second)
				{
					return true;
				}
			}
			else
			{
				// this is only the case when the angle range crosses over 180°
				if (pair.first <= angle || angle <= pair.second)
				{
					return true;
				}
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
		return atan2(t1, t0); // yaw
	}

}