#include "poi_spawner/POISpawnerPlugin.h"

#include <SystemConfig.h>

#include <gazebo/physics/World.hh>

#include <string>

namespace gazebo
{

POISpawnerPlugin::POISpawnerPlugin()
{
}

POISpawnerPlugin::~POISpawnerPlugin()
{
}

void POISpawnerPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    this->world = _parent;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&POISpawnerPlugin::OnUpdate, this, _1));

    // Read rooms with its connected areas and pois from config
    supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();
    auto roomNames = (*sc)["TopologicalModel"]->getSections("DistributedSystems.Rooms", NULL);
    for (auto &roomName : *roomNames)
    {
        std::shared_ptr<std::vector<std::string>> poisIDStrings;
        try
        {
            poisIDStrings =
                (*sc)["TopologicalModel"]->getSections("DistributedSystems.Rooms", roomName.c_str(), "POIs", NULL);
        }
        catch (exception &e)
        {
            continue;
        }
        for (auto poiIDString : *poisIDStrings)
        {
            double x = (*sc)["TopologicalModel"]->get<double>("DistributedSystems.Rooms", roomName.c_str(), "POIs",
                                                              poiIDString.c_str(), "X", NULL);
            double y = (*sc)["TopologicalModel"]->get<double>("DistributedSystems.Rooms", roomName.c_str(), "POIs",
                                                              poiIDString.c_str(), "Y", NULL);
            // spawn poi model

        }
    }
}

void POISpawnerPlugin::OnUpdate(const common::UpdateInfo &info)
{
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(POISpawnerPlugin)
}
