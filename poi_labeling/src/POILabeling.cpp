#include "poi_labeling/POILabeling.h"
#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gui/GuiEvents.hh>

namespace gazebo
{
POILabeling::POILabeling()
    : GUIPlugin()
{
    // Position and resize this widget
    this->move(10, 10);
    this->resize(0, 0);

    // Listen to event indicating a new model has been inserted
    this->connections.push_back(gazebo::gui::Events::ConnectModelUpdate(std::bind(&POILabeling::OnModelUpdate, this, std::placeholders::_1)));

    // Callback called on pre-render
    this->connections.push_back(event::Events::ConnectPreRender(std::bind(&POILabeling::Update, this)));
}

POILabeling::~POILabeling()
{
    this->models.clear();
    this->connections.clear();
}

void POILabeling::OnModelUpdate(const msgs::Model& msg)
{
    // Add new model to the list and it will be processed on Update
    std::lock_guard<std::mutex> lock(this->mutex);
    if (msg.name().find("poi_") != std::string::npos && this->models.find(msg.name()) == this->models.end())
    {
        this->models[msg.name()] = false;
    }
}

void POILabeling::Update()
{
    // Get scene pointer
    auto scene = rendering::get_scene();
    if (!scene)
    {
        return;
    }

    // Go through all models
    std::lock_guard<std::mutex> lock(this->mutex);
    for (auto &model : this->models)
    {
        // Skip if model has already been processed
        if (model.second)
        {
            continue;
        }

        std::string modelName = model.first;

        auto vis = scene->GetVisual(modelName);

        // Visual is not in the scene yet
        if (!vis)
        {
            continue;
        }
        int pos = modelName.find("_");
        std::string visualText = modelName.substr(pos + 1, modelName.size() - pos - 1);
        std::string textName = modelName + "_TEXT_";
        auto modelBox = vis->BoundingBox();

        // Create text
        auto text = new rendering::MovableText;
        text->Load(textName, visualText, "Arial", 0.3, common::Color::Red);
        text->SetBaseline(modelBox.Center().Z());
        text->SetTextAlignment(rendering::MovableText::HorizAlign::H_CENTER, rendering::MovableText::VertAlign::V_ABOVE);
        text->SetShowOnTop(true);

        // Attach modelName to the visual's node
        auto textNode = vis->GetSceneNode()->createChildSceneNode(textName + "_NODE_");
        textNode->attachObject(text);
        textNode->setInheritScale(false);

        // Model has been processed
        model.second = true;
    }
}
// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(POILabeling)
}
