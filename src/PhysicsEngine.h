#pragma once
#include "Utilities.h"
#include "cRigidbody.h"



struct ImpulseData
{
    glm::vec3 ra;
    glm::vec3 rb;
    float j;
};

class PhysicsEngine
{
    public:
        void Init();
        void Update(const std::vector<cRigidBody*>& rigidbodies, float time);

        void CollisionResolution(cRigidBody* A, cRigidBody* B, const CollisionData& data);
};