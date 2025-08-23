#pragma once
#include "Utilities.h"
#include "Component.h"
#include "Entity.h"

#define GConst -1000.0f
#define DC 1.0f

struct CollisionData
{
    bool collided;
    glm::vec3 normal;
    float displacement;
    std::vector<glm::vec3> contactPoints;
};

struct Force
{
    glm::vec3 force;
    glm::vec3 point;
};
struct Edge
{
    glm::vec3 v1,v2,v3, max;
    glm::vec3 edge() {return v2-v1;}
};

class cRigidBody : public Component
{
    public:
        cRigidBody(Entity* entity);
        void SetVertices(const std::vector<glm::vec3>& aVertices);
        Edge Best(glm::vec3 normal, const std::vector<glm::vec3>& vertices);
        std::vector<glm::vec3> Clip(glm::vec3 v1, glm::vec3 v2, glm::vec3 n, float o);
        void Update(float deltaTime);
        void Init();
        
        
        CollisionData CheckCollisionsGJK(cRigidBody* obj);
        CollisionData CheckCollisionsSAT(cRigidBody* obj);
        std::vector<glm::vec3> GetCollisionPoints(const std::vector<glm::vec3>& verticesA, const std::vector<glm::vec3>& verticesB, glm::vec3 normal);


        Transform GetTransform() {return *mTransform;}
        void AddForce(Force force) {mForces.push_back(force);}
        const std::vector<glm::vec3> GetWorldCoordinates();
        void SetStatic(bool aStatic);
        bool GetStatic() {return mStatic;}
        void SetVelocity(glm::vec3 aVel) {velocity = aVel;}
        float GetInvMass() {return mInvMass;}
        float GetInvInertia() {return InvInertiaCenter;}
        glm::vec3 velocity = glm::vec3(0);
        glm::vec3 AngVelocity = glm::vec3(0);

        void CalculateMassIner();

        float GetSf() {return StaticFriction;}
        float GetDf() {return DynamicFriction;}
    private:
        bool AlmostEqual(float a, float b);

        glm::vec3 ToWorldCoordinates(glm::vec3 point);

        bool mStatic = false;

        float InertiaCenter = 0.0f;
        float InvInertiaCenter = 0.0f;
        float mMass = 1.0f;
        float mInvMass = 1.0f;
        float StaticFriction = 0.1f;
        float DynamicFriction = 0.2f;
        Transform* mTransform;
        std::vector<glm::vec3> mVertices;
        std::vector<Force> mForces;
        std::vector<glm::vec3> Hinges;
        glm::vec2 Project(const std::vector<glm::vec3>& vertices, glm::vec2 axis);
};