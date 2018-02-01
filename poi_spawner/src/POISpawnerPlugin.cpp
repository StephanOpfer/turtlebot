#include "poi_spawner/POISpawnerPlugin.h"

#include <SystemConfig.h>

#include <gazebo/common/common.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/rendering/rendering.hh>
#include <ros/package.h>

#include <memory>
#include <vector>

namespace gazebo
{

POISpawnerPlugin::POISpawnerPlugin() : WorldPlugin()
{
    string path = ros::package::getPath("turtlebot_bringup");
    ifstream in(supplementary::FileSystem::combinePaths(path, "/models/poi/poi.sdf"));
    this->poi_model_xml = string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    spawned = false;
}

POISpawnerPlugin::~POISpawnerPlugin()
{

}

void POISpawnerPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    this->world = _parent;

    this->updateConnections.push_back(event::Events::ConnectWorldUpdateBegin(boost::bind(&POISpawnerPlugin::OnModelUpdate, this, _1)));
    this->updateConnections.push_back(event::Events::ConnectPreRender(std::bind(&POISpawnerPlugin::OnUpdate, this)));

        // Create a new transport node
        node = boost::make_shared<transport::Node>();

    // Initialize the node with the world name
    node->Init(this->world->GetName());

    // Create a publisher on the ~/factory topic
    this->factoryPub = node->Advertise<msgs::Factory>("~/factory");

    std::cout << "Info: POISpawnerPlugin PlugIn loaded!" << std::endl;
}

void POISpawnerPlugin::spawnPOI(string name, double x, double y)
{
    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(this->poi_model_xml);
    // Demonstrate using a custom model name.
    sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
    string modelName = "poi_" + name;
    model->GetAttribute("name")->SetFromString(modelName);
    model->GetElement("pose")->Set(to_string(x) + " " + to_string(y) + " 0 0 0 0");
    world->InsertModelSDF(sphereSDF);
    this->poiTextMap.emplace(modelName, false);
}

void POISpawnerPlugin::OnUpdate()
{
    for (auto &entry : this->poiTextMap)
    {
        if (!entry.second)
        {
            this->setText(entry.first);
        }
    }
}

void POISpawnerPlugin::setText(std::string name)
{
    scene = rendering::get_scene();
    if (!scene || !scene->Initialized())
    {
        return;
    }
    auto vis = scene->GetVisual(name);

    // Visual is not in the scene yet
    if (!vis)
    {
        return;
    }

    std::string textName = name + "__TEXT__";
    auto modelBox = vis->GetBoundingBox().Ign();
    std::cout << "POISpawner: " << name << std::endl;
    // Create text
    auto text = new rendering::MovableText();
//    text->Load(textName, name, "Arial", 0.1, common::Color::Blue);
    text->SetText(name);
    text->SetFontName("Arial");
    text->SetCharHeight(1.0);
    text->SetColor(common::Color::Blue);
    text->SetBaseline(modelBox.Max().Z() + 1);
//    text->SetShowOnTop(true);
//    text->setVisible(true);
    //
    //    // Attach modelName to the visual's node
    auto textNode = vis->GetSceneNode()->createChildSceneNode(textName + "__NODE__");
    textNode->attachObject(text);
    textNode->setInheritScale(false);
    this->poiTextMap[name] = true;
    std::cout << "POISpawner: SetText Done!" << std::endl;
}

void POISpawnerPlugin::OnModelUpdate(const common::UpdateInfo &info)
{
    if (!spawned)
    {
        // Read rooms with its connected areas and pois from config
        supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();
        auto roomNames = (*sc)["TopologicalModel"]->getSections("DistributedSystems.Rooms", NULL);
        for (auto &roomName : (*roomNames))
        {
            try
            {
                auto poisIDStrings = (*sc)["TopologicalModel"]->getSections("DistributedSystems.Rooms", roomName.c_str(), "POIs", NULL);
                for (auto poiIDString : (*poisIDStrings))
                {
                    double x = (*sc)["TopologicalModel"]->get<double>("DistributedSystems.Rooms", roomName.c_str(), "POIs", poiIDString.c_str(), "X", NULL);
                    double y = (*sc)["TopologicalModel"]->get<double>("DistributedSystems.Rooms", roomName.c_str(), "POIs", poiIDString.c_str(), "Y", NULL);
                    this->spawnPOI(poiIDString, x, y);
                }
            }
            catch (exception &e)
            {
                continue;
            }
        }
        spawned = true;
    }
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(POISpawnerPlugin)
}
