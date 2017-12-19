#include <ttb_msgs/LogicalCameraImage.h>

#include <gazebo/physics/RayShape.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <LogicalOcclusionPlugin.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(LogicalOcclusionPlugin)

//////////////////////////////////////////////////
LogicalOcclusionPlugin::LogicalOcclusionPlugin()
    : SensorPlugin()
    , seq_number(-1)
{
}

//////////////////////////////////////////////////
LogicalOcclusionPlugin::~LogicalOcclusionPlugin()
{
    nh.shutdown();
}

//////////////////////////////////////////////////
void LogicalOcclusionPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    // Get the parent sensor.
    parentSensor = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "LogicalOcclusionPlugin requires a LogicalCamera Sensor.\n";
        return;
    }

    // TODO: Change ConnectUpdated for suscription to gazebo image topic!
    // Connect to the sensor update event.
    updateConnection = parentSensor->ConnectUpdated(std::bind(&LogicalOcclusionPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    parentSensor->SetActive(true);

    // Initializing physics engine for collision checking
    physicsEngine = physics::get_world(parentSensor->WorldName())->GetPhysicsEngine();

    GZ_ASSERT(physicsEngine != nullptr, "Unable to get a pointer to the physics engine");

    laserCollision = physicsEngine->CreateCollision("ray", parentSensor->ParentName());

    GZ_ASSERT(laserCollision != nullptr, "Unable to create a ray collision using the physics engine.");

    laserCollision->SetName("logical_occlusion_collision");
    laserCollision->SetRelativePose(parentSensor->Pose());
    laserCollision->SetInitialRelativePose(parentSensor->Pose());

    rayShape = boost::dynamic_pointer_cast<physics::RayShape>(laserCollision->GetShape());

    GZ_ASSERT(rayShape != nullptr, "Unable to get the ray shape from the ray collision.");

    rayShape->Load(_sdf);
    rayShape->Init();

    // Extracting robot name, erasing link
    std::string robotName = this->parentSensor->ParentName();
    size_t pos = robotName.find(':');
    if (pos == std::string::npos)
    {
        gzerr << "LogicalOcclusionPlugin robot/model name(" << robotName << ") is invalid!";
        return;
    }
    robotName = robotName.substr(0, pos); // Getting only the name

    std::string topicName = robotName + "/" + parentSensor->Name();
    if (_sdf->HasElement("ros_topic"))
        topicName = _sdf->Get<std::string>("ros_topic");

    topicName = topicName + "/occluded";

    occludedPub = nh.advertise<ttb_msgs::LogicalCameraImage>(topicName, 50);

    occludingType = "map_point"; // default occluding type
    if (_sdf->HasElement("occluding_model"))
        occludingType = _sdf->Get<std::string>("occluding_model");

    ROS_DEBUG("LogicalOcclusionPlugin loaded!");
}

void LogicalOcclusionPlugin::Fini()
{
    if (laserCollision)
    {
        laserCollision->Fini();
        laserCollision.reset();
    }

    if (rayShape)
    {
        rayShape->Fini();
        rayShape.reset();
    }
}

//////////////////////////////////////////////////
void LogicalOcclusionPlugin::OnUpdate()
{
    // ROS message to be sent
	ttb_msgs::LogicalCameraImage occludedImageROSMsg;

    occludedImageROSMsg.header.seq = ++seq_number;
    occludedImageROSMsg.header.stamp = ros::Time::now();
    //Was getName() before, but is deprecated
    occludedImageROSMsg.header.frame_id = this->parentSensor->Name();

    // Get all the models in range (as gazebo proto message)
    auto logicalImageMsg = this->parentSensor->Image();

    // Translating gazebo message to ROS message
    msgs::Vector3d cameraPosition = logicalImageMsg.pose().position();
    msgs::Quaternion cameraOrientation = logicalImageMsg.pose().orientation();

    occludedImageROSMsg.pose.position.x = cameraPosition.x();
    occludedImageROSMsg.pose.position.y = cameraPosition.y();
    occludedImageROSMsg.pose.position.z = cameraPosition.z();
    occludedImageROSMsg.pose.orientation.x = cameraOrientation.x();
    occludedImageROSMsg.pose.orientation.z = cameraOrientation.z();
    occludedImageROSMsg.pose.orientation.w = cameraOrientation.w();

    for (int i = 0; i < logicalImageMsg.model_size(); i++)
    {
        auto modelMsg = logicalImageMsg.model(i);

        // Checking occlusion
        if (isDetected(modelMsg))
        {

            // Translating gazebo Model message to ROS message
        	ttb_msgs::Model modelROSMsg;

            modelROSMsg.name = modelMsg.name();
            modelROSMsg.type = DetermineModelType(modelMsg.name());

            msgs::Vector3d position = modelMsg.pose().position();
            msgs::Quaternion orientation = modelMsg.pose().orientation();
            modelROSMsg.pose.position.x = position.x();
            modelROSMsg.pose.position.y = position.y();
            modelROSMsg.pose.position.z = position.z();
            modelROSMsg.pose.orientation.x = orientation.x();
            modelROSMsg.pose.orientation.y = orientation.y();
            modelROSMsg.pose.orientation.z = orientation.z();
            modelROSMsg.pose.orientation.w = orientation.w();

            occludedImageROSMsg.models.push_back(modelROSMsg);
        }
    }

    occludedPub.publish(occludedImageROSMsg);
}

std::string LogicalOcclusionPlugin::DetermineModelType(const std::string &modelName)
{
    std::string modelType(modelName);

    // Trim namespaces
    size_t index = modelType.find_last_of('|');
    modelType = modelType.substr(index + 1);

    // Trim trailing underscore and number caused by inserting multiple of the same model
    size_t end_index = modelType.find_last_not_of("0123456789");
    size_t start_index = modelType.find_last_of(":");

    if (modelType[end_index] == '_' && end_index > 1)
        modelType = modelType.substr(0, end_index);

    if (start_index > 1)
        modelType = modelType.substr(start_index + 1);

    return modelType;
}

bool LogicalOcclusionPlugin::isDetected(msgs::LogicalCameraImage_Model model)
{
    auto modelType = DetermineModelType(model.name());

    transform(occludingType.begin(), occludingType.end(), occludingType.begin(), ::tolower);

    // Checking if model type match desired type
    if (modelType.find(occludingType) == std::string::npos)
        return false;

    // Starting and ending points of the ray, in absolute coordinates relative to the world
    // NOTE: Documentation api saids that should be relative to the sensor, but is not apparently
    rayShape->SetPoints(parentSensor->Pose().Pos(), (parentSensor->Pose().Rot() * ConvertIgn(model.pose().position())) + parentSensor->Pose().Pos());

    rayShape->Update();

    double dist;
    std::string collided_entity;
    rayShape->GetIntersection(dist, collided_entity);

    /*
    math::Vector3 start, end;
    rayShape->GetGlobalPoints(start, end);

    ROS_INFO_STREAM(" Start: " << start << " End: " << end );
    ROS_INFO_STREAM(" Checking Model: " << model.name() << " Colliding: " << collided_entity );
    */

    // true if the model name appears on the collided entity
    return collided_entity.find(model.name()) != std::string::npos;
}
