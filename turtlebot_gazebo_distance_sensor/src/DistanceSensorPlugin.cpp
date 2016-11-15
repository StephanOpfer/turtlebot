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
		loadModelsFromConfig();
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

	}



	void GazeboRosDistance::OnUpdate()
	{
		vector<msgs::LogicalCameraImage_Model> victimModels;

		// Get all the models in range.
		auto models = this->parentSensor->Image();
		for (int i = 0; i < models.model_size(); i++)
		{
			auto m = models.model(i);
			auto n = m.name();

			auto x = m.pose().position().x();
			auto y = m.pose().position().y();
			auto z = m.pose().position().z();
			auto dist = sqrt(x * x + y * y + z * z);

			// Debugging output
//			std::cout << this->parentSensor->Far() << " " << this->parentSensor->Near() << " " << this->parentSensor->HorizontalFOV() << std::endl;
			printf("Model No. %d with Name %s at (%.1f, %.1f, %.1f) dist: %.2f\n", i, n.c_str(), x,
					y, z, dist);

			if (n.find("victim") != std::string::npos && dist <= 4)
			{
				victimModels.push_back(m);
			}

		}

		for (auto m : victimModels)
		{
			auto n = m.name();
			auto x = m.pose().position().x();
			auto y = m.pose().position().y();
			auto z = m.pose().position().z();

			printf("Victim found with Name %s at (%f, %f, %f)\n", n.c_str(), x, y, z);

			ttb_msgs::LogicalCamera msg;
			msg.modelName = m.name();
			msg.pose.x = m.pose().position().x();
			msg.pose.y = m.pose().position().y();

			auto q = m.pose().orientation();
			msg.pose.theta = QuadToTheata(q.x(), q.y(), q.z(), q.w());
			msg.timeStamp = ros::Time::now();

			modelPub.publish(msg);
		}

	}

	void GazeboRosDistance::loadModelsFromConfig()
	{
		auto config = (*this->sc)["LogicalCamera"];
		auto sections = config->getSections("LogicalCamera", NULL);
		for(int i = 0; i < sections->size(); i++)
		{
			cout << sections->at(i) << endl;
			Model m;
			m.range = config->get<double>("LogicalCamera", sections->at(i).c_str(), "range", NULL);
			m.startAngle = config->get<double>("LogicalCamera", sections->at(i).c_str(), "startAngle", NULL);
			m.endAngle = config->get<double>("LogicalCamera", sections->at(i).c_str(), "endAngle", NULL);
			m.type = config->get<std::string>("LogicalCamera", sections->at(i).c_str(), "type", NULL);
			m.section = sections->at(i);
			this->models.push_back(m);
		}
	}

	// See:
	// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	double GazeboRosDistance::QuadToTheata(double x, double y, double z, double w)
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
