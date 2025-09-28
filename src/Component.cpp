#include "Component.h"
#include "Entity.h"
#include "cRigidbody.h"
#include <GLFW/glfw3.h>

Component::Component(Entity* entity)
{
    mMasterEntity = entity;
}

//Necessary mapping GLFW enums to strings for lua
#include <GLFW/glfw3.h>
#include <unordered_map>
#include <string>

static const std::unordered_map<std::string, int> StringToKey {
    {"SPACE", GLFW_KEY_SPACE},
    {"'", GLFW_KEY_APOSTROPHE},
    {",", GLFW_KEY_COMMA},
    {"-", GLFW_KEY_MINUS},
    {".", GLFW_KEY_PERIOD},
    {"/", GLFW_KEY_SLASH},

    {"0", GLFW_KEY_0},
    {"1", GLFW_KEY_1},
    {"2", GLFW_KEY_2},
    {"3", GLFW_KEY_3},
    {"4", GLFW_KEY_4},
    {"5", GLFW_KEY_5},
    {"6", GLFW_KEY_6},
    {"7", GLFW_KEY_7},
    {"8", GLFW_KEY_8},
    {"9", GLFW_KEY_9},

    {";", GLFW_KEY_SEMICOLON},
    {"=", GLFW_KEY_EQUAL},

    {"A", GLFW_KEY_A},
    {"B", GLFW_KEY_B},
    {"C", GLFW_KEY_C},
    {"D", GLFW_KEY_D},
    {"E", GLFW_KEY_E},
    {"F", GLFW_KEY_F},
    {"G", GLFW_KEY_G},
    {"H", GLFW_KEY_H},
    {"I", GLFW_KEY_I},
    {"J", GLFW_KEY_J},
    {"K", GLFW_KEY_K},
    {"L", GLFW_KEY_L},
    {"M", GLFW_KEY_M},
    {"N", GLFW_KEY_N},
    {"O", GLFW_KEY_O},
    {"P", GLFW_KEY_P},
    {"Q", GLFW_KEY_Q},
    {"R", GLFW_KEY_R},
    {"S", GLFW_KEY_S},
    {"T", GLFW_KEY_T},
    {"U", GLFW_KEY_U},
    {"V", GLFW_KEY_V},
    {"W", GLFW_KEY_W},
    {"X", GLFW_KEY_X},
    {"Y", GLFW_KEY_Y},
    {"Z", GLFW_KEY_Z},

    {"[", GLFW_KEY_LEFT_BRACKET},
    {"\\", GLFW_KEY_BACKSLASH},
    {"]", GLFW_KEY_RIGHT_BRACKET},
    {"`", GLFW_KEY_GRAVE_ACCENT},
    {"WORLD_1", GLFW_KEY_WORLD_1},
    {"WORLD_2", GLFW_KEY_WORLD_2},

    {"ESCAPE", GLFW_KEY_ESCAPE},
    {"ENTER", GLFW_KEY_ENTER},
    {"TAB", GLFW_KEY_TAB},
    {"BACKSPACE", GLFW_KEY_BACKSPACE},
    {"INSERT", GLFW_KEY_INSERT},
    {"DELETE", GLFW_KEY_DELETE},
    {"RIGHT", GLFW_KEY_RIGHT},
    {"LEFT", GLFW_KEY_LEFT},
    {"DOWN", GLFW_KEY_DOWN},
    {"UP", GLFW_KEY_UP},
    {"PAGE_UP", GLFW_KEY_PAGE_UP},
    {"PAGE_DOWN", GLFW_KEY_PAGE_DOWN},
    {"HOME", GLFW_KEY_HOME},
    {"END", GLFW_KEY_END},
    {"CAPS_LOCK", GLFW_KEY_CAPS_LOCK},
    {"SCROLL_LOCK", GLFW_KEY_SCROLL_LOCK},
    {"NUM_LOCK", GLFW_KEY_NUM_LOCK},
    {"PRINT_SCREEN", GLFW_KEY_PRINT_SCREEN},
    {"PAUSE", GLFW_KEY_PAUSE},

    {"F1", GLFW_KEY_F1}, {"F2", GLFW_KEY_F2}, {"F3", GLFW_KEY_F3},
    {"F4", GLFW_KEY_F4}, {"F5", GLFW_KEY_F5}, {"F6", GLFW_KEY_F6},
    {"F7", GLFW_KEY_F7}, {"F8", GLFW_KEY_F8}, {"F9", GLFW_KEY_F9},
    {"F10", GLFW_KEY_F10}, {"F11", GLFW_KEY_F11}, {"F12", GLFW_KEY_F12},
    {"F13", GLFW_KEY_F13}, {"F14", GLFW_KEY_F14}, {"F15", GLFW_KEY_F15},
    {"F16", GLFW_KEY_F16}, {"F17", GLFW_KEY_F17}, {"F18", GLFW_KEY_F18},
    {"F19", GLFW_KEY_F19}, {"F20", GLFW_KEY_F20}, {"F21", GLFW_KEY_F21},
    {"F22", GLFW_KEY_F22}, {"F23", GLFW_KEY_F23}, {"F24", GLFW_KEY_F24},
    {"F25", GLFW_KEY_F25},

    {"KP_0", GLFW_KEY_KP_0}, {"KP_1", GLFW_KEY_KP_1}, {"KP_2", GLFW_KEY_KP_2},
    {"KP_3", GLFW_KEY_KP_3}, {"KP_4", GLFW_KEY_KP_4}, {"KP_5", GLFW_KEY_KP_5},
    {"KP_6", GLFW_KEY_KP_6}, {"KP_7", GLFW_KEY_KP_7}, {"KP_8", GLFW_KEY_KP_8},
    {"KP_9", GLFW_KEY_KP_9},
    {"KP_DECIMAL", GLFW_KEY_KP_DECIMAL},
    {"KP_DIVIDE", GLFW_KEY_KP_DIVIDE},
    {"KP_MULTIPLY", GLFW_KEY_KP_MULTIPLY},
    {"KP_SUBTRACT", GLFW_KEY_KP_SUBTRACT},
    {"KP_ADD", GLFW_KEY_KP_ADD},
    {"KP_ENTER", GLFW_KEY_KP_ENTER},
    {"KP_EQUAL", GLFW_KEY_KP_EQUAL},

    {"LEFT_SHIFT", GLFW_KEY_LEFT_SHIFT},
    {"LEFT_CONTROL", GLFW_KEY_LEFT_CONTROL},
    {"LEFT_ALT", GLFW_KEY_LEFT_ALT},
    {"LEFT_SUPER", GLFW_KEY_LEFT_SUPER},
    {"RIGHT_SHIFT", GLFW_KEY_RIGHT_SHIFT},
    {"RIGHT_CONTROL", GLFW_KEY_RIGHT_CONTROL},
    {"RIGHT_ALT", GLFW_KEY_RIGHT_ALT},
    {"RIGHT_SUPER", GLFW_KEY_RIGHT_SUPER},
    {"MENU", GLFW_KEY_MENU}
};

int StringToKeyFunc(const std::string& name) {
    auto it = StringToKey.find(name);
    if (it != StringToKey.end())
        return it->second;
    return GLFW_KEY_UNKNOWN;
}

void InspectorVarData::Render()
{
    if(LuaState != nullptr)
    {
        RenderLua();
        return;
    }
    switch (type)
    {
        case BOOL:
        {
            ImGui::Checkbox(name.c_str(), reinterpret_cast<bool*>(data));
            break;
        }
        case FLOAT:
        {
            ImGui::DragFloat(name.c_str(), reinterpret_cast<float*>(data), speed, min, max);
            break;
        }
        case VEC2:
        {
            ImGui::DragFloat2(name.c_str(), reinterpret_cast<float*>(data), speed, min, max);
            break;
        }
        case VEC3:
        {
            ImGui::DragFloat3(name.c_str(), reinterpret_cast<float*>(data), speed, min, max);
            break;
        }
        case VEC4:
        {
            ImGui::DragFloat4(name.c_str(), reinterpret_cast<float*>(data), speed, min, max);
            break;
        }
    }
}


void InspectorVarData::RenderLua()
{
    sol::object obj = (*LuaState)["p"][name];
    switch (obj.get_type())
    {
        case sol::type::nil:
            fmt::println("Can't render nill variable");
            break;
        case sol::type::number:
        {
            float x = obj.as<float>();
            if(ImGui::DragFloat(name.c_str(), &x, speed, min, max))
            {
                (*LuaState)["p"][name] = x;
            }
            break;
        }
        case sol::type::boolean:
        {
            bool x = obj.as<bool>();
            if(ImGui::Checkbox(name.c_str(), &x))
            {
                (*LuaState)["p"][name] = x;
            }
            break;
        }
        default:
            fmt::println("What is that type of variable?");
            break;
    }
}
void InspectorVarData::SaveLuavVar(json& jsonData)
{
    sol::object obj = (*LuaState)["p"][name];
    switch (obj.get_type())
    {
        case sol::type::nil:
            fmt::println("Can't save nill variable");
            break;
        case sol::type::number:
        {
            jsonData[name] = obj.as<float>();
            break;
        }
        case sol::type::boolean:
        {
            jsonData[name] = obj.as<bool>();
            break;
        }
        default:
            fmt::println("What is that type of variable?");
            break;
    }
}
void InspectorVarData::LoadLuaVar(json& jsonData)
{
    sol::object obj = (*LuaState)["p"][name];
    switch (obj.get_type())
    {
        case sol::type::nil:
            fmt::println("Can't load nill variable");
            break;
        case sol::type::number:
        {
            (*LuaState)["p"][name] = jsonData[name].get<float>();
            break;
        }
        case sol::type::boolean:
        {
            (*LuaState)["p"][name] = jsonData[name].get<bool>();
            break;
        }
        default:
            fmt::println("What is that type of variable?");
            break;
    }
}


void InspectorVarData::Save(json& jsonData)
{
    if(LuaState != nullptr)
    {
        SaveLuavVar(jsonData);
        return;
    }
    switch (type)
    {
        case BOOL:
        {
            jsonData[name] = *(bool*)(data);
            break;
        }
        case FLOAT:
        {
            jsonData[name] = *(float*)(data);
            break;
        }
        case VEC2:
        {
            jsonData[name]["x"] = (*(glm::vec2*)(data)).x;
            jsonData[name]["y"] = (*(glm::vec2*)(data)).y;
            break;
        }
        case VEC3:
        {
            jsonData[name]["x"] = (*(glm::vec3*)(data)).x;
            jsonData[name]["y"] = (*(glm::vec3*)(data)).y;
            jsonData[name]["z"] = (*(glm::vec3*)(data)).z;
            break;
        }
        case VEC4:
        {
            jsonData[name]["x"] = (*(glm::vec4*)(data)).x;
            jsonData[name]["y"] = (*(glm::vec4*)(data)).y;
            jsonData[name]["z"] = (*(glm::vec4*)(data)).z;
            jsonData[name]["w"] = (*(glm::vec4*)(data)).w;
            break;
        }
    }
}
void InspectorVarData::Load(json& jsonData)
{
    if(LuaState != nullptr)
    {
        LoadLuaVar(jsonData);
        return;
    }
    switch (type)
    {
        case BOOL:
        {
            *(bool*)(data) = jsonData[name];
            break;
        }
        case FLOAT:
        {
            *(float*)(data) = jsonData[name];
            break;
        }
        case VEC2:
        {
            (*(glm::vec2*)(data)).x = jsonData[name]["x"];
            (*(glm::vec2*)(data)).y = jsonData[name]["y"];
            break;
        }
        case VEC3:
        {
            (*(glm::vec3*)(data)).x = jsonData[name]["x"];
            (*(glm::vec3*)(data)).y = jsonData[name]["y"];
            (*(glm::vec3*)(data)).z = jsonData[name]["z"];
            break;
        }
        case VEC4:
        {
            (*(glm::vec4*)(data)).x = jsonData[name]["x"];
            (*(glm::vec4*)(data)).y = jsonData[name]["y"];
            (*(glm::vec4*)(data)).z = jsonData[name]["z"];
            (*(glm::vec4*)(data)).w = jsonData[name]["w"];
            break;
        }
    }
}


LuaComponent::LuaComponent(Entity* entity, const std::string& path) : Component(entity)
{
    name = path;
    lua.open_libraries(sol::lib::base, sol::lib::math);
    try
    {
        lua.safe_script_file(path);
    }
    catch(const sol::error& e)
    {
        fmt::println(std::string(e.what()));
    }

    sol::optional<sol::table> globals = lua["p"];
    if(globals)
    {
        int i = 0;
        for (auto& kv : globals.value()) 
        {
            InspectorVarData data;
            data.name = kv.first.as<std::string>();
            data.max = INFINITY;
            data.min = -INFINITY;
            data.speed = 1.0f;
            data.LuaState = &lua;
            InspectorVariables.push_back(data);
            i++;
        }
    }

    lua["Move"] = [this](float x, float y)
    {
        GetMasterEntity()->GetTransformPointer()->position = glm::vec3(x,y,0.0f);
    };
    lua["TransformX"] = [this]()
    {
        return GetMasterEntity()->GetTransform().position.x;
    };
    lua["TransformY"] = [this]()
    {
        return GetMasterEntity()->GetTransform().position.y;
    };
    lua["Input"] = [](std::string key)
    {
        int state = glfwGetKey(glfwGetCurrentContext(), StringToKeyFunc(key));
        return state == GLFW_PRESS || state == GLFW_REPEAT;
    };
    if(GetMasterEntity()->GetComponent<cRigidBody>() != nullptr)
    {
        cRigidBody* rb = GetMasterEntity()->GetComponent<cRigidBody>();
        lua["SetVelocity"] = [rb](float x, float y)
        {
            rb->velocity = glm::vec3(x,y, 0);
        };
        lua["VelocityX"] = [rb]()
        {
            return rb->velocity.x;
        };
        lua["VelocityY"] = [rb]()
        {
            return rb->velocity.y;
        };
    }
}


void LuaComponent::Update()
{
    try {
        lua["Update"]();
    }
    catch (const sol::error& e) {
        fmt::println("Error in {} : \n{}", name, e.what());
    }
}