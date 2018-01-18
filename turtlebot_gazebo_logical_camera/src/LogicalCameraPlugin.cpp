#include "LogicalCameraPlugin.h"

#include <ttb_msgs/LogicalCamera.h>
#include <ttb_msgs/LogicalCameraImage.h>

#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <Configuration.h>
#include <SystemConfig.h>

namespace gazebo
{
//////////////////////////////////////////////////
LogicalCameraPlugin::LogicalCameraPlugin()
    : SensorPlugin()
    , seq_number(-1)
{
    this->sensorYaw = 0;
    this->sc = supplementary::SystemConfig::getInstance();
    loadModelsFromConfig();
    loadOccludingTypes();
}

//////////////////////////////////////////////////
LogicalCameraPlugin::~LogicalCameraPlugin()
{
    this->nh.shutdown();
}

void LogicalCameraPlugin::loadOccludingTypes()
{
    this->occludingTypes = (*this->sc)["LogicalCamera"]->getList<string>("occludingTypes", NULL);
}

void LogicalCameraPlugin::loadModelsFromConfig()
{
    const char *lc = "LogicalCamera";
    const char *da = "DetectAngles";

    auto config = (*this->sc)[lc];
    this->modelSectionNames = config->getSections(lc, NULL);

    // Iterate over all model sections in config file
    for (auto section : *(this->modelSectionNames))
    {
#ifdef LOGICAL_CAMERA_DEBUG
        cout << "LogicalCameraPlugin: section: " << section << endl;
#endif
        const char *sec = section.c_str();
        ConfigModel m;
        m.range = config->get<double>(lc, sec, "range", NULL);

        // Add all angles specified in model to detectAngles vector
        auto angleSections = config->getSections(lc, sec, da, NULL);
        for (auto angleSection : *angleSections)
        {
            auto start = config->get<double>(lc, sec, da, angleSection.c_str(), "startAngle", NULL);
            auto end = config->get<double>(lc, sec, da, angleSection.c_str(), "endAngle", NULL);
#ifdef LOGICAL_CAMERA_DEBUG
            cout << "DistanceSensorPlugin: angleSection: " << angleSection << " from : " << start << " to: " << end << endl;
#endif
            m.detectAngles.push_back(pair<double, double>(start, end));
        }

        m.type = config->get<string>(lc, sec, "type", NULL);
        m.name = section;
        m.publishingRate = config->get<double>(lc, sec, "publishingRateHz", NULL);
        this->modelMap.emplace(m.name, m);
    }

    this->modelSectionNames = nullptr;
}

//////////////////////////////////////////////////
void LogicalCameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    // Get the parent sensor.
    this->parentSensor = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "LogicalCameraPlugin requires a LogicalCamera Sensor.\n";
        return;
    }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&LogicalCameraPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);

    // Initializing physics engine for collision checking
    this->physicsEngine = physics::get_world(this->parentSensor->WorldName())->GetPhysicsEngine();

    GZ_ASSERT(this->physicsEngine != nullptr, "Unable to get a pointer to the physics engine");

    this->laserCollision = this->physicsEngine->CreateCollision("ray", this->parentSensor->ParentName());

    GZ_ASSERT(this->laserCollision != nullptr, "Unable to create a ray collision using the physics engine.");

    this->laserCollision->SetName("logical_occlusion_collision");
    this->laserCollision->SetRelativePose(this->parentSensor->Pose());
    this->laserCollision->SetInitialRelativePose(this->parentSensor->Pose());

    this->rayShape = boost::dynamic_pointer_cast<physics::RayShape>(this->laserCollision->GetShape());

    GZ_ASSERT(this->rayShape != nullptr, "Unable to get the ray shape from the ray collision.");

    this->rayShape->Load(_sdf);
    this->rayShape->Init();

    // Extracting robot name, erasing link
    std::string robotName = this->parentSensor->ParentName();
    size_t pos = robotName.find(':');
    if (pos == std::string::npos)
    {
        gzerr << "LogicalCameraPlugin robot/model name(" << robotName << ") is invalid!";
        return;
    }
    robotName = robotName.substr(0, pos); // Getting only the name

    std::string topicName = "/" + robotName + "/logical_camera";

    this->modelPub = this->nh.advertise<ttb_msgs::LogicalCamera>(topicName, 50);

    ROS_INFO("LogicalCameraPlugin loaded!");

    // Get the sensor yaw, which is used to distinguish front and back sensor
    this->sensorYaw = this->parentSensor->Pose().Rot().Yaw();
}

void LogicalCameraPlugin::Fini()
{
    if (this->laserCollision)
    {
        this->laserCollision->Fini();
        this->laserCollision.reset();
    }

    if (this->rayShape)
    {
        this->rayShape->Fini();
        this->rayShape.reset();
    }
}

//////////////////////////////////////////////////
void LogicalCameraPlugin::OnUpdate()
{
    // Get all the models in range (as gazebo proto message)
    auto models = this->parentSensor->Image();

    for (int i = 0; i < models.model_size(); i++)
    {
        auto model = models.model(i);

        // Is the model for front or back sensor
        if (!isSensorResponsible(model))
        {
            continue;
        }
        for (auto &kv : this->modelMap)
        {
            // Checking occlusion
            if (isDetected(model, kv.second))
            {
                // Translating gazebo Model message to ROS message
                publishModel(model, kv.second);
            }
        }
    }
}

void LogicalCameraPlugin::publishModel(msgs::LogicalCameraImage_Model model, LogicalCameraPlugin::ConfigModel &configModel)
{
    auto x = model.pose().position().x();
    auto y = model.pose().position().y();
    auto z = model.pose().position().z();

    ttb_msgs::LogicalCamera msg;
    msg.modelName = model.name();

    auto q = model.pose().orientation();
    // change coordinate system and rotation for backwards sensor
    if (this->sensorYaw != 0)
    {
        msg.pose.x = -x;
        msg.pose.y = -y;
        //			msg.pose.theta = quadToTheata(q.x(), q.y(), q.z(), q.w());
        auto angle = quadToTheata(q.x(), q.y(), q.z(), q.w());
        msg.pose.theta = -(angle < 0 ? angle + M_PI : angle - M_PI);
    }
    else
    {
        msg.pose.x = x;
        msg.pose.y = y;
        msg.pose.theta = -quadToTheata(q.x(), q.y(), q.z(), q.w());
    }

#ifdef LOGICAL_CAMERA_DEBUG
    cout << "Robot " << this->robotName << " found Model with Name " << model.name() << " at ( " << x << ", " << y << ", " << z << ")" << endl;
#endif

    msg.timeStamp = ros::Time::now();
    msg.type = configModel.type;

    chrono::time_point<chrono::high_resolution_clock> now = chrono::high_resolution_clock::now();

    if (lastPublishedMap.count(msg.modelName) <= 0)
    {
        modelPub.publish(msg);
        lastPublishedMap[msg.modelName] = now;
    }
    else
    {
        // Publish message if needed/specified hz from config exceeded
        auto diff = chrono::duration_cast<chrono::milliseconds>(now - lastPublishedMap[msg.modelName]);
#ifdef LOGICAL_CAMERA_DEBUG
        cout << "DistanceSensorPlugin: diff: " << diff.count() << endl;
#endif
        if (diff.count() >= (1000.0 / configModel.publishingRate))
        {
            modelPub.publish(msg);
            lastPublishedMap[msg.modelName] = now;
        }
    }
}

// See:
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
double LogicalCameraPlugin::quadToTheata(double x, double y, double z, double w)
{
    double ysqr = y * y;
    double t0 = -2.0f * (ysqr + z * z) + 1.0f;
    double t1 = +2.0f * (x * y - w * z);
    return atan2(t1, t0); // yaw
}

bool LogicalCameraPlugin::isDetected(msgs::LogicalCameraImage_Model model, LogicalCameraPlugin::ConfigModel configModel)
{
    // Checking if model type match desired type
    auto gazeboElementName = configModel.type;
    transform(gazeboElementName.begin(), gazeboElementName.end(), gazeboElementName.begin(), ::tolower);
    if (model.name().find(gazeboElementName) == std::string::npos)
    {
        return false;
    }

    // Model is too far away
    auto x = model.pose().position().x();
    auto y = model.pose().position().y();
    auto z = model.pose().position().z();
    auto dist = sqrt(x * x + y * y + z * z);
    if (dist > configModel.range)
    {
        return false;
    }

    /*
     * angle to model calculated with ego-centric coordinates
     */
    double angle = calculateAngle(x, y);
    if (!isInAngleRange(angle, configModel.detectAngles))
    {
        return false;
    }

    if (isOccluded(model))
    {
        return false;
    }

    return true;
}

bool LogicalCameraPlugin::isOccluded(msgs::LogicalCameraImage_Model &model)
{
    // Starting and ending points of the ray, in absolute coordinates relative to the world
    // NOTE: Documentation api saids that should be relative to the sensor, but is not apparently
    //    auto parentSensorPose = this->parentSensor->Pose().Pos();
    //    parentSensorPose = parentSensorPose + (this->parentSensor->Pose().Rot() * ConvertIgn(model.pose().position())).Normalize() * 0.15;
    //    this->rayShape->SetPoints(parentSensorPose, (this->parentSensor->Pose().Rot() * ConvertIgn(model.pose().position())) + parentSensorPose);
    this->rayShape->SetPoints(this->parentSensor->Pose().Pos() * 1.15,
                              (this->parentSensor->Pose().Rot() * ConvertIgn(model.pose().position())) + this->parentSensor->Pose().Pos() * 1.15);

    this->rayShape->Update();

    double dist;
    std::string collided_entity;
    this->rayShape->GetIntersection(dist, collided_entity);

    // TODO: this does not work, because now he can see through walls if there is a table in front

    for(auto occludingType : this->occludingTypes)
    {
    	if(collided_entity.find(occludingType) != std::string::npos)
    	{
    		return true;
    	}
    }

// no door or wall or floor blocking sight to model
#ifdef LOGICAL_OCCLUSION_DEBUG
    cout << "LogicalCameraPlugin: " << (this->sensorYaw != 0 ? "Back" : "Front") << " found model " << model.name() << " at: x=" << model.pose().position().x()
         << ", y= " << model.pose().position().y() << ", z= " << model.pose().position().z() << endl;
#endif
    return false;
}

bool LogicalCameraPlugin::isInAngleRange(double angle, std::vector<std::pair<double, double>> detectAngles)
{
    // all angles have to be checked so no return after first pair is checked
    for (auto pair : detectAngles)
    {
#ifdef LOGICAL_CAMERA_DEBUG
        cout << "DistanceSensorPlugin: checking pair: (" << pair.first << " : " << pair.second << ")" << endl;
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

double LogicalCameraPlugin::calculateAngle(double x, double y)
{
    double angle = atan2(y, x) * 180.0 / M_PI;

    /*
     * change angle of backwards facing sensor get sensor range from 90° to 180°
     * and from -90° to -180°
     */
    if (this->sensorYaw != 0)
    {
        angle = (angle < 0) ? angle + 180.0 : angle - 180;
    }
    return angle;
}

bool LogicalCameraPlugin::isSensorResponsible(msgs::LogicalCameraImage_Model model)
{
    return model.pose().position().x() > 0;
}
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(LogicalCameraPlugin)
}
