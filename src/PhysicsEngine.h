#pragma once
#include "Utilities.h"
#include "cRigidbody.h"



struct ImpulseData
{
    glm::vec3 dir;
    glm::vec3 point;

    size_t DminLimit = -1;
    size_t DmaxLimit = -1;

    // if DminLimit or DmaxLimit exists then these variables are multipliers for the dependend limit.
    float CminLimit = -INFINITY;  
    float CmaxLimit = INFINITY; 
};

struct ImpulseActData
{
    size_t id;
    /*
    Basically this exists to follow the newtons third law, if A collides with B then B should get the negative of the impulse and with this 
    I can keep track if its object B in a collision that should get the negative impulse. So its 1 if its A so it doesnt effect the end result and -1 
    if its B so when multiplied it negates the value.
    */
    int mult = 1;
};

struct CollisionObject
{
    std::vector<ImpulseActData> ImpulsesActed;
};
struct Collision
{
    std::vector<size_t> objects;
    glm::vec3 point;
    glm::vec3 normal;
    float constant = 0.0f;
};

class PhysicsEngine
{
    public:
        void Init();
        void Update(const std::vector<cRigidBody*>& rigidbodies, float time);
        void DebugUI();
        void CollisionResolution(const std::vector<cRigidBody*>& rigidbodies,size_t A, size_t B, const CollisionData& data, float dt);
        void SolveLinearSystem(std::vector<float>* output, const std::vector<std::vector<float>>& matrix
        , const std::vector<float>& constants);
        void SolveLinearSystemF(std::vector<float>* output, const std::vector<std::vector<float>>& matrix
        , const std::vector<float>& constants);
        int PhysicsEngineIterations = 1;
        int GaussSeidelIterations = 15;
        bool IntersectingAABB(AABB a, AABB b);
    private:
        std::vector<float> impulses = std::vector<float>(0);
        // std::vector<float> frictionImpulses = std::vector<float>(0);
        std::vector<ImpulseData> mImpulses;
        std::vector<ImpulseData> mFrictionImpulses;
        std::vector<Collision> mCollisions;
        std::vector<CollisionObject> mColObjects;
        bool enableFriction = true;
};