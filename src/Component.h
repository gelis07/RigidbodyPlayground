#pragma once
#include "Utilities.h"
#include <imgui.h>
#include <sol.hpp>
#include <json.hpp>
#include <variant>
using json = nlohmann::json;

class Entity;

struct LuaFunc
{
    std::string name;
    std::function<void()> func;
};

struct InspectorVarData
{
    std::string name;
    VARIABLE_TYPE type;
    void* data = nullptr;
    float speed = 1.0f; // Speed of the imgui slider
    float min = -INFINITY;
    float max = INFINITY;

    sol::state* LuaState = nullptr;

    void Render();
    void Save(json& jsonData);
    void Load(json& jsonData);
    void LoadLuaVar(const sol::state& state, const std::string& name);

    private:
        void RenderLua();
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


class LuaComponent : public Component
{
    public:
        LuaComponent(Entity* entity, const std::string& path);
        void Update();
    private:
        sol::state lua;
};




