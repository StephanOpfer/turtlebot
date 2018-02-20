#include "LogicalCameraPlugin.h"

#include <ttb_msgs/LogicalCamera.h>
#include <ttb_msgs/LogicalCameraImage.h>

#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <SystemConfig.h>

#include <ros/package.h>

namespace gazebo
{

LogicalCameraPlugin::LogicalCameraPlugin()
    : SensorPlugin()
    , seq_number(-1)
{
    this->sensorYaw = 0;
    this->sc = supplementary::SystemConfig::getInstance();
    loadModelsFromConfig();
    loadOccludingTypes();
}

LogicalCameraPlugin::~LogicalCameraPlugin()
{
    this->nh.shutdown();
}

void LogicalCameraPlugin::loadOccludingTypes()
{
    this->occludingTypes = (*this->sc)["LogicalCamera"]->getList<std::string>("occludingTypes", NULL);
}

void LogicalCameraPlugin::loadModelsFromConfig()
{
    auto config = (*this->sc)["LogicalCamera"];
    this->modelSectionNames = config->getSections("LogicalCamera", NULL);

    // Iterate over all model sections in config file
    for (auto section : *(this->modelSectionNames))
    {
#ifdef LOGICAL_CAMERA_DEBUG
        cout << "LogicalCameraPlugin: section: " << section << endl;
#endif
        const char *sec = section.c_str();
        ConfigModel m;
        m.range = config->get<double>("LogicalCamera", sec, "range", NULL);

        // Add all angles specified in model to detectAngles vector
        auto angleSections = config->getSections("LogicalCamera", sec, "DetectAngles", NULL);
        for (auto angleSection : *angleSections)
        {
            auto start =
                config->get<double>("LogicalCamera", sec, "DetectAngles", angleSection.c_str(), "startAngle", NULL);
            auto end =
                config->get<double>("LogicalCamera", sec, "DetectAngles", angleSection.c_str(), "endAngle", NULL);
#ifdef LOGICAL_CAMERA_DEBUG
            cout << "LogicalCameraPlugin: angleSection: " << angleSection << " from : " << start << " to: " << end
                 << endl;
#endif
            m.detectAngles.push_back(pair<double, double>(start, end));
        }

        m.type = section;
        m.name = section;
        m.publishingRate = config->get<double>("LogicalCamera", sec, "publishingRateHz", NULL);
        this->modelMap.emplace(m.name, m);
    }

    this->modelSectionNames = nullptr;
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
    this->debugName = "door_r1411C_r1401";
#endif
}

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

    this->quadFar = this->parentSensor->Far() * this->parentSensor->Far();
    this->quadNear = this->parentSensor->Near() * this->parentSensor->Near();

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&LogicalCameraPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);

    // Initializing physics engine for collision checking
    this->world = physics::get_world(this->parentSensor->WorldName());
    GZ_ASSERT(this->world != nullptr, "Unable to get a pointer to the world");

    this->physicsEngine = this->world->GetPhysicsEngine();
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
    this->robotName = this->parentSensor->ParentName();
    size_t pos = this->robotName.find(':');
    if (pos == std::string::npos)
    {
        gzerr << "LogicalCameraPlugin robot/model name(" << this->robotName << ") is invalid!";
        return;
    }
    this->robotName = this->robotName.substr(0, pos); // Getting only the name

    std::string topicName = "/" + this->robotName + "/logical_camera";

    this->modelPub = this->nh.advertise<ttb_msgs::LogicalCamera>(topicName, 50);

    ROS_INFO("LogicalCameraPlugin loaded!");

    // Get the sensor yaw, which is used to distinguish front and back sensor
    this->sensorYaw = this->parentSensor->Pose().Rot().Yaw();
#ifdef LOGICAL_CAMERA_DEBUG
    std::cout << "LogicalCameraPlugin: Sensor yaw: " << this->sensorYaw << std::endl;
    std::cout << "LogicalCameraPlugin: Sensor far range: " << this->parentSensor->Far() << std::endl;
    std::cout << "LogicalCameraPlugin: Sensor near range: " << this->parentSensor->Near() << std::endl;
#endif

#ifdef LOGICAL_CAMERA_DEBUG_POINTS
    // let only one sensor spawn the debug points
    if (this->sensorYaw != 0)
    {
		std::string path = ros::package::getPath("turtlebot_bringup");
		std::ifstream in(supplementary::FileSystem::combinePaths(path, "/models/debugPoint/debugPoint.sdf"));
		std::string sdfString = std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		this->createDebugPoint(sdfString, "0 0 0 0 0 0", "debugPoint");
		this->createDebugPoint(sdfString, "0 0 0 0 0 0", "debugPoint2");
    }
#endif
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

void LogicalCameraPlugin::OnUpdate()
{
    // Get all the models in range (as gazebo proto message)
//    auto start = chrono::system_clock::now();
    auto models = this->parentSensor->Image();
    //    std::cout << "LogicalCameraPlugin: Image object count: " << models.model_size() << std::endl;

    for (int i = 0; i < models.model_size(); i++)
    {
        auto model = models.model(i);
        //std::cout << "LogicalCameraPlugin: Modelname: " << model.name() << std::endl;
        // Is the model for front or back sensor and model is in far/near distance
        if (!isSensorResponsible(model))
        {
            continue;
        }

        auto mapEntry = this->modelMap.find(model.name().substr(0, model.name().find_first_of('_')));
        if (mapEntry == this->modelMap.end())
        {
            continue;
        }

        gazebo::math::Pose correctedPosition;
        if (isDetected(model, mapEntry->second, correctedPosition))
        {
            // Translating gazebo Model message to ROS message
            publishModel(model, mapEntry->second, correctedPosition);
        }
    }

//    auto end = chrono::system_clock::now();
//    std::cout << "LogicalCameraPlugin: Runtime: "
//              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " units!" << std::endl;
}

void LogicalCameraPlugin::publishModel(msgs::LogicalCameraImage_Model model,
                                       LogicalCameraPlugin::ConfigModel &configModel,
                                       gazebo::math::Pose outCorrectedPose)
{
    ttb_msgs::LogicalCamera msg;
    msg.modelName = model.name();

    // change coordinate system and rotation for backwards sensor
    if (this->sensorYaw != 0)
    {
        msg.pose.x = -outCorrectedPose.pos.x;
        msg.pose.y = -outCorrectedPose.pos.y;
        auto angle = quaternionToYaw(outCorrectedPose.rot.x, outCorrectedPose.rot.y, outCorrectedPose.rot.z,
                                     outCorrectedPose.rot.w);
        msg.pose.theta = -(angle < 0 ? angle + M_PI : angle - M_PI);
    }
    else
    {
        msg.pose.x = outCorrectedPose.pos.x;
        msg.pose.y = outCorrectedPose.pos.y;
        msg.pose.theta = -quaternionToYaw(outCorrectedPose.rot.x, outCorrectedPose.rot.y, outCorrectedPose.rot.z,
                                          outCorrectedPose.rot.w);
    }

    /*
     * Dirty fix for open door angles.
     * See is detected method for detailed description
     */
    if (model.name().find("door_") != std::string::npos)
    {
        msg.pose.theta = outCorrectedPose.rot.z;
    }
#ifdef LOGICAL_CAMERA_DEBUG
    cout << "Robot " << this->robotName << " found Model with Name " << model.name() << " at ( "
         << outCorrectedPose.pos.x << ", " << outCorrectedPose.pos.y << ", " << outCorrectedPose.pos.z << ")" << endl;
#endif

    msg.timeStamp = ros::Time::now();
    msg.type = configModel.type;

    chrono::time_point<chrono::high_resolution_clock> now = chrono::high_resolution_clock::now();

    if (this->lastPublishedMap.find(msg.modelName) != this->lastPublishedMap.end())
    {
        this->modelPub.publish(msg);
        this->lastPublishedMap[msg.modelName] = now;
    }
    else
    {
// Publish message if needed/specified hz from config exceeded
#ifdef LOGICAL_CAMERA_DEBUG
        cout << "LogicalCameraPlugin: diff: " << diff.count() << endl;
#endif
        if (chrono::duration_cast<chrono::milliseconds>(now - this->lastPublishedMap[msg.modelName]).count() >=
            (1000.0 / configModel.publishingRate))
        {
            this->modelPub.publish(msg);
            this->lastPublishedMap[msg.modelName] = now;
        }
    }
}

// See:
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
double LogicalCameraPlugin::quaternionToYaw(double x, double y, double z, double w)
{
    // TODO was (x * y - w * z) before but wikipedia page uses (x * y + w * z)
    return atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z)); // yaw
}

bool LogicalCameraPlugin::isDetected(msgs::LogicalCameraImage_Model model, LogicalCameraPlugin::ConfigModel configModel,
                                     gazebo::math::Pose &outCorrectedPose)
{
    auto objectModel = this->world->GetModel(model.name());

    if (model.name().find("door_") != std::string::npos)
    {
        auto doorLink = objectModel->GetChildLink("door::door");
        outCorrectedPose.pos = doorLink->GetWorldPose().CoordPositionAdd(math::Vector3(0.45, 0.0, 1.0));
        /*
         * Dirty hack to set the correct angle of the door
         * the angle of the door itself does not change when it is opened therefore
         * the rotation on the z-axis is replaced with the angle of the joint between the
         * doorframe and the door leaf
         */
        auto doorJoint = objectModel->GetJoint("door::hinge");
        outCorrectedPose.rot.z = *doorJoint->GetAngle(0);
    }
    else
    {
        outCorrectedPose = objectModel->GetWorldPose();
    }

    // Check the distance to the correctedPose
    if (!this->isInRange(outCorrectedPose.pos, configModel.range))
    {
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
        if (model.name().find(debugName) != string::npos)
        {
            std::cout << "LogicalCameraPlugin::isDetected: out of range " << model.name() << std::endl;
        }
#endif
        return false;
    }
    // Check angle to model calculated with ego-centric coordinates
    if (!this->isInAngleRange(outCorrectedPose, configModel.detectAngles))
    {
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
        if (model.name().find(debugName) != string::npos)
        {
            std::cout << "LogicalCameraPlugin::isDetected: out of angle range" << model.name() << std::endl;
        }
#endif
        return false;
    }
    // Check whether the object is occluded by other things
    if (!this->isVisible(outCorrectedPose, model))
    {
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
        if (model.name().find(debugName) != string::npos)
        {
            std::cout << "LogicalCameraPlugin::isDetected: is occluded " << model.name() << std::endl;
        }
#endif
        return false;
    }
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
    if (model.name().find(debugName) != string::npos)
    {
    	std::cout << "LogicalCameraPlugin: AABB of " << model.name() << " " << objectModel->GetBoundingBox() << std::endl;
        std::cout << "LogicalCameraPlugin: CorrectedPose " << outCorrectedPose << std::endl;
    }
#endif
    return true;
}

bool LogicalCameraPlugin::isVisible(gazebo::math::Pose correctedPose, msgs::LogicalCameraImage_Model model)
{
    // Starting and ending points of the ray
    auto rayEndPoint = correctedPose - this->world->GetModel(this->robotName)->GetWorldPose();
    if (this->sensorYaw != 0)
    {
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
        if (model.name().find(debugName) != string::npos)
        {
            std::cout << "LogicalCameraPlugin: Correcting angle for back sensor!" << std::endl;
        }
#endif
        rayEndPoint.RotatePositionAboutOrigin(math::Quaternion::EulerToQuaternion(math::Vector3(0.0, 0.0, M_PI)));
    }

    // auto rayEndPoint = ConvertIgn(model.pose().position());
    this->rayShape->SetPoints(this->parentSensor->Pose().Pos(), rayEndPoint.pos);

    this->rayShape->Update();

#ifdef LOGICAL_CAMERA_DEBUG_POINTS
    if (model.name().find(this->debugName) != std::string::npos)
    {
        math::Vector3 posA;
        math::Vector3 posB;
        this->rayShape->GetGlobalPoints(posA, posB);

        std::cout << "LogicalCameraPlugin: ray start point: " << posA << " ray end point: " << posB << std::endl;
        auto debugPosA = gazebo::math::Pose(posA.x, posA.y, posA.z, 0, 0, 0);
        auto debugPosB = gazebo::math::Pose(posB.x, posB.y, posB.z, 0, 0, 0);
        this->moveDebugPoint("debugPoint", debugPosA);
        this->moveDebugPoint("debugPoint2", debugPosB);
    }
#endif

    double dist;
    std::string collided_entity;
    this->rayShape->GetIntersection(dist, collided_entity);

    // Things should not occlude themselves (only relevant for things with a collision)
    if (collided_entity.find(model.name()) != std::string::npos)
    {
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
        if (model.name().find(debugName) != string::npos)
        {
            std::cout << "LogicalCameraPlugin: object is occluding itself. Entity: " << collided_entity
                      << " Object name: " << model.name() << std::endl;
        }
#endif
        return true;
    }

    // TODO: this does not work, because now he can see through walls if there is a table in front
    for (auto occludingType : this->occludingTypes)
    {
        if (collided_entity.find(occludingType) != std::string::npos)
        {
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
            if (model.name().find(debugName) != string::npos)
            {
                std::cout << "LogicalCameraPlugin: object is occluded by. Entity: " << collided_entity
                          << " OccludingType: " << occludingType << std::endl;
            }
#endif
            return false;
        }
    }

#ifdef LOGICAL_OCCLUSION_DEBUG
    cout << "LogicalCameraPlugin: " << (this->sensorYaw != 0 ? "Back" : "Front") << " found model " << model.name()
         << " at: x=" << model.pose().position().x() << ", y= " << model.pose().position().y()
         << ", z= " << model.pose().position().z() << endl;
#endif
    return true;
}

bool LogicalCameraPlugin::isInAngleRange(gazebo::math::Pose &pose, std::vector<std::pair<double, double>> detectAngles)
{
    double angle = atan2(pose.pos.y, pose.pos.x) * 180.0 / M_PI;

    /*
     * change angle of backwards facing sensor get sensor range from 90° to 180°
     * and from -90° to -180°
     */
    if (this->sensorYaw != 0)
    {
        angle = (angle < 0) ? angle + 180.0 : angle - 180;
    }

    // all angles have to be checked so no return after first pair is checked
    for (auto pair : detectAngles)
    {
#ifdef LOGICAL_CAMERA_DEBUG
        cout << "LogicalCameraPlugin: checking pair: (" << pair.first << " : " << pair.second << ")" << endl;
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

bool LogicalCameraPlugin::isInRange(gazebo::math::Vector3 modelPosition, double range)
{
    return modelPosition.x * modelPosition.x + modelPosition.y * modelPosition.y + modelPosition.z * modelPosition.z <=
           range * range;
}

bool LogicalCameraPlugin::isSensorResponsible(msgs::LogicalCameraImage_Model model)
{
	return model.pose().position().x() > 0;
}

void LogicalCameraPlugin::createDebugPoint(std::string sdfString, std::string positionString, std::string name)
{
    // first debug point
    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(sdfString);
    // Demonstrate using a custom model name.
    sdf::ElementPtr m = sphereSDF.Root()->GetElement("model");
    m->GetAttribute("name")->SetFromString(name);
    m->GetElement("pose")->Set(positionString);
    this->world->InsertModelSDF(sphereSDF);
}

void LogicalCameraPlugin::moveDebugPoint(std::string name, gazebo::math::Pose &pose)
{
    auto model = this->world->GetModel(name);
    if (model)
    {
        model->SetWorldPose(pose);
        model->Update();
    }
}
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(LogicalCameraPlugin)
}
