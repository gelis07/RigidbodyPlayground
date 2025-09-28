#pragma once
#include "Utilities.h"
#include "Entity.h"
#include "cRigidbody.h"
#include "cRenderer.h"
#include <json.hpp>
using json = nlohmann::json;

class Scene
{
    public:

        void Load(const std::string& path);
        void Save(const std::string& path);
        void Delete();

        std::vector<Entity> entities;
        std::vector<cRenderer*> renderers;
        std::vector<cRigidBody*> rbs;
        float width, height;
};