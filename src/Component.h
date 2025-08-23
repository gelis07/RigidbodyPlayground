#pragma once
#include "Utilities.h"


class Entity;


struct InspectorVarData
{
    std::string name;
    VARIABLE_TYPE type;
    void* data;
    float speed = 1.0f; // Speed of the imgui slider
    float min = -INFINITY;
    float max = INFINITY;
};

class Component
{
    public:
        Component(Entity* entity);
        Entity* GetMasterEntity() {return mMasterEntity;}
        std::vector<InspectorVarData> InspectorVariables;
        std::string name;
    private:
        Entity* mMasterEntity;
};