#include <gazebo/common/Plugin.hh>

#include "turtlebot_gazebo_grid_annotator/Annotator.h"

#include <SystemConfig.h>
#include <limits>
#include <string>
#include <ttb_msgs/AnnotatedGrid.h>
#include <ttb_msgs/Grid.h>
#include <vector>

#ifdef ANNOTATOR_DEBUG_POINTS
#include <ros/package.h>
#endif

namespace gazebo
{

Annotator::Annotator()
{
    this->spinner = nullptr;
#ifdef ANNOTATOR_DEBUG_POINTS
    auto sc = supplementary::SystemConfig::getInstance();
    this->debugName = (*sc)["LogicalCamera"]->get<std::string>("LogicalCamera.debugName", NULL);
#endif
}

Annotator::~Annotator()
{
    delete this->spinner;
}

void Annotator::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    // Initializing physics engine for collision checking
    this->world = _parent;

    // Connect to the sensor update event.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Annotator::OnUpdate, this, _1));

    auto physicsEngine = this->world->Physics();
    physicsEngine->InitForThread();
    this->rayShape =
        boost::dynamic_pointer_cast<physics::RayShape>(physicsEngine->CreateShape("ray", physics::CollisionPtr()));
    if (!rayShape)
    {
        std::cerr << "Annotator: No RayShape available! " << std::endl;
    }
    else
    {
        std::cout << "Annotator: RayShape available! " << std::endl;
    }

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        std::cerr << "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                  << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)"
                  << std::endl;
        return;
    }

    auto sc = supplementary::SystemConfig::getInstance();
    std::string annotatedGridTopic =
        (*sc)["TTBWorldModel"]->get<std::string>("Processing.AnnotatedGrid.annotatedGridTopic", NULL);
    std::string gridTopic = (*sc)["TTBWorldModel"]->get<std::string>("Processing.AnnotatedGrid.gridTopic", NULL);
    ros::NodeHandle n;
    this->annotatedGridPublisher = n.advertise<ttb_msgs::AnnotatedGrid>(annotatedGridTopic, 5, false);
    this->gridSubscriber = n.subscribe(gridTopic, 10, &Annotator::onGrid, (Annotator *)this);
    this->spinner = new ros::AsyncSpinner(4);
    this->spinner->start();

    std::cout << "GridAnnotator PlugIn loaded!" << std::endl;

#ifdef ANNOTATOR_DEBUG_POINTS
    std::string path = ros::package::getPath("turtlebot_bringup");
    std::ifstream in(supplementary::FileSystem::combinePaths(path, "/models/debugPoint/debugPoint.sdf"));
    std::string sdfString = std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    this->createDebugPoint(sdfString, "0 0 0 0 0 0", "debugPoint");
    this->createDebugPoint(sdfString, "0 0 0 0 0 0", "debugPoint2");
#endif
}

void Annotator::OnUpdate(const common::UpdateInfo &info)
{
    auto models = this->world->Models();
    gazebo::physics::Model_V pois;
    for (auto model : models)
    {
        if (model->GetName().find_first_of("poi_") != 0)
        {
            // all POIs start with "poi_" so skip this one
            continue;
        }
        else
        {
            pois.push_back(model);
        }
    }

    if (this->gridMsgBuffer.empty())
    {
    	return;
    }
    auto grid = this->gridMsgBuffer.front();
	this->gridMsgBuffer.pop();

    ttb_msgs::AnnotatedGrid annotatedGrid;
    double minDist = std::numeric_limits<double>::max();
    gazebo::physics::ModelPtr closestPOI = nullptr;
    for (auto point : grid->points)
    {
        for (auto poi : pois)
        {
            if (this->isCloserAndVisible(poi, point, minDist))
            {
                closestPOI = poi;
            }
        }

        // points without any line to a POI are filtered (e.g. points in walls)
        if (closestPOI)
        {
            annotatedGrid.points.push_back(point);
            annotatedGrid.annotatedPOIs.push_back(closestPOI->GetName());
        }

        closestPOI = nullptr;
        minDist = std::numeric_limits<double>::max();
    }

    this->annotatedGridPublisher.publish(annotatedGrid);
}

void Annotator::onGrid(ttb_msgs::GridPtr grid)
{
	this->gridMsgBuffer.push(grid);
}

bool Annotator::isCloserAndVisible(gazebo::physics::ModelPtr poi, geometry_msgs::Point point, double &minDist)
{
    // check whether the poi is closer than any other checked before
    double tmpDist = sqrt(((poi->WorldPose().Pos().X() - point.x) * (poi->WorldPose().Pos().X() - point.x)) +
                          ((poi->WorldPose().Pos().Y() - point.y) * (poi->WorldPose().Pos().Y() - point.y)) +
                          ((poi->WorldPose().Pos().Z() - point.z) * (poi->WorldPose().Pos().Z() - point.z)));
    if (minDist < tmpDist)
    {
        return false;
    }

    // check whether the poi is visible from the point of view
    this->rayShape->SetPoints(ignition::math::Vector3d(point.x, point.y, 0.3), poi->WorldPose().Pos());

#ifdef ANNOTATOR_DEBUG_POINTS
    if (poi->GetName().find(this->debugName) != std::string::npos)
    {
        math::Vector3 posA;
        math::Vector3 posB;
        this->rayShape->GetGlobalPoints(posA, posB);

        std::cout << "Annotator: ray start point: " << posA << " ray end point: " << posB << std::endl;
        auto debugPosA = gazebo::math::Pose(posA.x, posA.y, posA.z, 0, 0, 0);
        auto debugPosB = gazebo::math::Pose(posB.x, posB.y, posB.z, 0, 0, 0);
        this->moveDebugPoint("debugPoint", debugPosA);
        this->moveDebugPoint("debugPoint2", debugPosB);
    }
#endif

    double collisionDist = std::numeric_limits<double>::max();
    std::string collisionEntity;
    this->rayShape->GetIntersection(collisionDist, collisionEntity);
    if (tmpDist > collisionDist && collisionEntity != "")
    {
        return false;
    }

    // closer and visible poi found, so write the distance into the out parameter
    minDist = tmpDist;
    return true;
}

#ifdef ANNOTATOR_DEBUG_POINTS
void Annotator::createDebugPoint(std::string sdfString, std::string positionString, std::string name)
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

void Annotator::moveDebugPoint(std::string name, gazebo::math::Pose &pose)
{
    auto model = this->world->GetModel(name);
    if (model)
    {
        model->SetWorldPose(pose);
        model->Update();
    }
}
#endif

GZ_REGISTER_WORLD_PLUGIN(Annotator)
} /* namespace gazebo */
