#include "LogicalCameraPlugin.h"

#include <ttb_msgs/LogicalCamera.h>
#include <ttb_msgs/LogicalCameraImage.h>

#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <SystemConfig.h>

#ifdef LOGICAL_CAMERA_DEBUG_POINTS
#include <ros/package.h>
#endif

namespace gazebo
{

LogicalCameraPlugin::LogicalCameraPlugin()
    : ModelPlugin()
    , seq_number(-1)
{
    this->sc = supplementary::SystemConfig::getInstance();
    loadModelsFromConfig();
    loadOccludingTypes();
    loadParameters();
}

LogicalCameraPlugin::~LogicalCameraPlugin()
{
    this->nh.shutdown();
}

void LogicalCameraPlugin::loadParameters()
{
    double near = (*this->sc)["LogicalCamera"]->get<double>("LogicalCamera.near", NULL);
    double far = (*this->sc)["LogicalCamera"]->get<double>("LogicalCamera.far", NULL);
    this->quadNear = near * near;
    this->quadFar = far * far;
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
    this->debugName = (*this->sc)["LogicalCamera"]->get<std::string>("LogicalCamera.debugName", NULL);
#endif
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
        std::cout << "LogicalCameraPlugin: section: " << section << std::endl;
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
            std::cout << "LogicalCameraPlugin: angleSection: " << angleSection << " from : " << start << " to: " << end
                      << std::endl;
#endif
            m.detectAngles.push_back(pair<double, double>(start, end));
        }

        m.type = section;
        m.name = section;
        m.publishingRate = config->get<double>("LogicalCamera", sec, "publishingRateHz", NULL);
        this->modelMap.emplace(m.name, m);
    }

    this->modelSectionNames = nullptr;
}

void LogicalCameraPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    this->robotModel = _parent;

    // Connect to the sensor update event.
    this->updateConnection =
        event::Events::ConnectWorldUpdateBegin(boost::bind(&LogicalCameraPlugin::OnUpdate, this, _1));

    // Initializing physics engine for collision checking
    this->world = (this->robotModel->GetWorld());
    GZ_ASSERT(this->world != nullptr, "Unable to get a pointer to the world");

    this->physicsEngine = this->world->GetPhysicsEngine();
    GZ_ASSERT(this->physicsEngine != nullptr, "Unable to get a pointer to the physics engine");

    this->laserCollision = this->physicsEngine->CreateCollision("ray", "base_footprint");
    GZ_ASSERT(this->laserCollision != nullptr, "Unable to create a ray collision using the physics engine.");

    this->laserCollision->SetName("logical_occlusion_collision");
    this->laserCollision->SetRelativePose(this->robotModel->GetRelativePose());
    this->laserCollision->SetInitialRelativePose(this->robotModel->GetRelativePose());

    this->rayShape = boost::dynamic_pointer_cast<physics::RayShape>(this->laserCollision->GetShape());

    GZ_ASSERT(this->rayShape != nullptr, "Unable to get the ray shape from the ray collision.");
    std::cout << "LogicalCameraPlugin: SDF " << _sdf->ToString("FOO") << std::endl;
    this->rayShape->Load(_sdf);
    this->rayShape->Init();
    ROS_INFO("LogicalCameraPlugin loaded!");
    // Extracting robot name, erasing link
    std::string topicName = "/" + this->robotModel->GetName() + "/logical_camera";
    this->modelPub = this->nh.advertise<ttb_msgs::LogicalCamera>(topicName, 50);

    // Get the sensor yaw, which is used to distinguish front and back sensor
    ROS_INFO("LogicalCameraPlugin loaded!");
#ifdef LOGICAL_CAMERA_DEBUG
    std::cout << "LogicalCameraPlugin: Sensor far range: " << sqrt(this->quadFar) << std::endl;
    std::cout << "LogicalCameraPlugin: Sensor near range: " << sqrt(this->quadNear) << std::endl;
#endif

#ifdef LOGICAL_CAMERA_DEBUG_POINTS
    std::string path = ros::package::getPath("turtlebot_bringup");
    std::ifstream in(supplementary::FileSystem::combinePaths(path, "/models/debugPoint/debugPoint.sdf"));
    std::string sdfString = std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    this->createDebugPoint(sdfString, "0 0 0 0 0 0", "debugPoint");
    this->createDebugPoint(sdfString, "0 0 0 0 0 0", "debugPoint2");
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

void LogicalCameraPlugin::OnUpdate(const common::UpdateInfo &info)
{
    this->world->SetPaused(true);
#ifdef LOGICAL_CAMERA_RUNTIME_DEBUG
    start = chrono::high_resolution_clock::now();
#endif
    auto models = this->world->GetModels();
    for (int i = 0; i < models.size(); i++)
    {
        auto model = models.at(i);
        double quadDistance = (model->GetWorldPose().pos.x - this->robotModel->GetWorldPose().pos.x) *
                                  (model->GetWorldPose().pos.x - this->robotModel->GetWorldPose().pos.x) +
                              (model->GetWorldPose().pos.y - this->robotModel->GetWorldPose().pos.y) *
                                  (model->GetWorldPose().pos.y - this->robotModel->GetWorldPose().pos.y) +
                              (model->GetWorldPose().pos.z - this->robotModel->GetWorldPose().pos.z) *
                                  (model->GetWorldPose().pos.z - this->robotModel->GetWorldPose().pos.z);
        if (quadDistance < this->quadNear || quadDistance > this->quadFar)
        {
            continue;
        }

        auto mapEntry = this->modelMap.find(model->GetName().substr(0, model->GetName().find_first_of('_')));
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
#ifdef LOGICAL_CAMERA_RUNTIME_DEBUG
    end = chrono::high_resolution_clock::now();
    std::cout << "LogicalCameraPlugin: Runtime: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " units!" << std::endl;
#endif
    this->world->SetPaused(false);
}

void LogicalCameraPlugin::publishModel(gazebo::physics::ModelPtr model, LogicalCameraPlugin::ConfigModel &configModel,
                                       gazebo::math::Pose outCorrectedPose)
{
    ttb_msgs::LogicalCamera msg;
    msg.modelName = model->GetName();

    // change coordinate system and rotation for backwards sensor
    msg.pose.x = outCorrectedPose.pos.x;
    msg.pose.y = outCorrectedPose.pos.y;
    msg.pose.theta = -quaternionToYaw(outCorrectedPose.rot.x, outCorrectedPose.rot.y, outCorrectedPose.rot.z,
                                      outCorrectedPose.rot.w);

    /*
     * Dirty fix for open door angles.
     * See is detected method for detailed description
     */
    if (model->GetName().find("door_") != std::string::npos)
    {
        msg.pose.theta = outCorrectedPose.rot.z;
    }
#ifdef LOGICAL_CAMERA_DEBUG
    cout << "Robot " << this->robotModel->GetName() << " found Model with Name " << model->GetName() << " at ( "
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

bool LogicalCameraPlugin::isDetected(gazebo::physics::ModelPtr model, LogicalCameraPlugin::ConfigModel configModel,
                                     gazebo::math::Pose &outCorrectedPose)
{

    if (model->GetName().find("door_") != std::string::npos)
    {
        auto doorLink = model->GetChildLink("door::door");
        outCorrectedPose.pos = doorLink->GetWorldPose().CoordPositionAdd(math::Vector3(0.45, 0.0, 1.0));
        /*
         * Dirty hack to set the correct angle of the door
         * the angle of the door itself does not change when it is opened therefore
         * the rotation on the z-axis is replaced with the angle of the joint between the
         * doorframe and the door leaf
         */
        auto doorJoint = model->GetJoint("door::hinge");
        outCorrectedPose.rot.z = *doorJoint->GetAngle(0);
    }
    else
    {
        outCorrectedPose = model->GetWorldPose();
    }

    // Check the distance to the correctedPose
    if (!this->isInRange(outCorrectedPose.pos, configModel.range))
    {
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
        if (model->GetName().find(debugName) != string::npos)
        {
            std::cout << "LogicalCameraPlugin::isDetected: out of range " << model->GetName() << std::endl;
        }
#endif
        return false;
    }
    // Check angle to model calculated with ego-centric coordinates
    if (!this->isInAngleRange(outCorrectedPose, configModel.detectAngles))
    {
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
        if (model->GetName().find(debugName) != string::npos)
        {
            std::cout << "LogicalCameraPlugin::isDetected: out of angle range" << model->GetName() << std::endl;
        }
#endif
        return false;
    }
    // Check whether the object is occluded by other things
    if (!this->isVisible(outCorrectedPose, model))
    {
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
        if (model->GetName().find(debugName) != string::npos)
        {
            std::cout << "LogicalCameraPlugin::isDetected: is occluded " << model->GetName() << std::endl;
        }
#endif
        return false;
    }
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
    if (model->GetName().find(debugName) != string::npos)
    {
        std::cout << "LogicalCameraPlugin: AABB of " << model->GetName() << " " << model->GetBoundingBox() << std::endl;
        std::cout << "LogicalCameraPlugin: CorrectedPose " << outCorrectedPose << std::endl;
    }
#endif
    return true;
}

bool LogicalCameraPlugin::isVisible(gazebo::math::Pose correctedPose, gazebo::physics::ModelPtr model)
{
    if (this->isInRange(model->GetWorldPose().pos, 0.3))
    {
        return true;
    }
    // Starting and ending points of the ray
    auto rayEndPoint = correctedPose - this->robotModel->GetWorldPose();
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
    if (model->GetName().find(debugName) != string::npos)
    {
        std::cout << "LogicalCameraPlugin: Correcting angle for back sensor!" << std::endl;
    }
#endif
    rayEndPoint.RotatePositionAboutOrigin(math::Quaternion::EulerToQuaternion(math::Vector3(0.0, 0.0, M_PI)));

    this->rayShape->SetPoints(gazebo::math::Vector3(0.0, 0.0, 1.15), rayEndPoint.pos);

    this->rayShape->Update();

#ifdef LOGICAL_CAMERA_DEBUG_POINTS
    if (model->GetName().find(this->debugName) != std::string::npos)
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
    if (collided_entity.find(model->GetName()) != std::string::npos)
    {
#ifdef LOGICAL_CAMERA_DEBUG_POINTS
        if (model->GetName().find(debugName) != string::npos)
        {
            std::cout << "LogicalCameraPlugin: object is occluding itself. Entity: " << collided_entity
                      << " Object name: " << model->GetName() << std::endl;
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
            if (model->GetName().find(debugName) != string::npos)
            {
                std::cout << "LogicalCameraPlugin: object is occluded by. Entity: " << collided_entity
                          << " OccludingType: " << occludingType << std::endl;
            }
#endif
            return false;
        }
    }

#ifdef LOGICAL_OCCLUSION_DEBUG
    std::cout << "LogicalCameraPlugin: "
              << " found model " << model->GetName() << " at: x=" << model->GetWorldPose().pos.x
              << ", y= " << model->GetWorldPose().pos.y << ", z= " << model->GetWorldPose().pos.z << std::endl;
#endif
    return true;
}

bool LogicalCameraPlugin::isInAngleRange(gazebo::math::Pose &pose, std::vector<std::pair<double, double>> detectAngles)
{
    double angle = atan2(pose.pos.y, pose.pos.x) * 180.0 / M_PI;

    // all angles have to be checked so no return after first pair is checked
    for (auto pair : detectAngles)
    {
#ifdef LOGICAL_CAMERA_DEBUG
        std::cout << "LogicalCameraPlugin: checking pair: (" << pair.first << " : " << pair.second << ")" << std::endl;
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
    auto robotPosition = this->robotModel->GetWorldPose().pos;
    auto sqDist = (modelPosition.x - robotPosition.x) * (modelPosition.x - robotPosition.x) +
                  (modelPosition.y - robotPosition.y) * (modelPosition.y - robotPosition.y) +
                  (modelPosition.z - robotPosition.z) * (modelPosition.z - robotPosition.z);
    return sqDist <= range * range;
}

#ifdef LOGICAL_CAMERA_DEBUG_POINTS
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
#endif
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LogicalCameraPlugin)
}
