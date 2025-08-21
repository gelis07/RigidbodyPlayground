#pragma once
#include "Utilities.h"
#include "Entity.h"
#include "cRigidbody.h"
#include "cRenderer.h"
class Scene
{
    public:
        std::vector<Entity> entities;
        std::vector<cRenderer*> renderers;
        std::vector<cRigidBody*> rbs;
        float width, height;
};