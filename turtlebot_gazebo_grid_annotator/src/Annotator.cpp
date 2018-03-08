#include <gazebo/common/Plugin.hh>

#include "turtlebot_gazebo_grid_annotator/Annotator.h"

#include <SystemConfig.h>
#include <limits>
#include <string>
#include <ttb_msgs/AnnotatedGrid.h>
#include <ttb_msgs/Grid.h>
#include <vector>

namespace gazebo
{

Annotator::Annotator()
{
    this->spinner = nullptr;
}

Annotator::~Annotator()
{
    delete this->spinner;
}

void Annotator::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        std::cerr << "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                  << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)"
                  << std::endl;
        return;
    }

    this->world = _parent;

    auto sc = supplementary::SystemConfig::getInstance();
    std::string annotatedGridTopic = (*sc)["TTBWorldModel"]->get<std::string>("AnnotatedGrid.annotatedGridTopic", NULL);
    std::string gridTopic = (*sc)["TTBWorldModel"]->get<std::string>("AnnotatedGrid.gridTopic", NULL);
    ros::NodeHandle n;
    this->annotatedGridPublisher = n.advertise<ttb_msgs::AnnotatedGrid>(annotatedGridTopic, 5, false);
    this->gridSubscriber = n.subscribe(gridTopic, 10, &Annotator::onGrid, (Annotator *)this);
    this->spinner = new ros::AsyncSpinner(4);
    this->spinner->start();

    std::cout << "GridAnnotator PlugIn loaded!" << std::endl;
}

void Annotator::onGrid(ttb_msgs::GridPtr grid)
{
    auto models = this->world->GetModels();
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

    ttb_msgs::AnnotatedGrid annotatedGrid;

    double minDist = numeric_limits<double>::max();
    for (auto point : grid->points)
    {
        for (auto poi : pois)
        {
            if (this->isCloserAndVisible(poi, point, minDist))
            {
            	annotatedGrid.points.push_back(point);
            	annotatedGrid.annotatedRooms.push_back(poi->GetName());
            }
        }
        minDist = numeric_limits<double>::max();
    }

    this->annotatedGridPublisher.publish(annotatedGrid);
}

bool Annotator::isCloserAndVisible(gazebo::physics::ModelPtr poi, geometry_msgs::Point point, double &minDist)
{
    double tmpDist = sqrt((poi->GetWorldPose().pos.x - point.x) * (poi->GetWorldPose().pos.y - point.y) *
                          (poi->GetWorldPose().pos.z - point.z));
    if (minDist < tmpDist)
    {
        return false;
    }

    // TODO make ray check
    if (true/* ray check does not match*/)
    {
    	return false;
    }

    minDist = tmpDist;
    return true;
}

GZ_REGISTER_WORLD_PLUGIN(Annotator)
} /* namespace gazebo */
