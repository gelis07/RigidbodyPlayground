#pragma once
#include "Utilities.h"
#include "Component.h"
#include "Entity.h"
#include <array>
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
struct Face {
    std::vector<glm::vec3> vertices;
    glm::vec3 normal;
};

struct ContactData {
    std::vector<glm::vec3> points;
};


struct Simplex
{
    private:
        std::array<glm::vec3, 4> mPoints;
        unsigned mSize;
    public:
        Simplex() : mPoints({glm::vec3(0), glm::vec3(0), glm::vec3(0), glm::vec3(0)}), mSize(0)
        {}
        Simplex& operator=(std::initializer_list<glm::vec3> list)
        {
            for(auto v = list.begin(); v != list.end(); v++)
            {
                mPoints[std::distance(list.begin(), v)] = *v;
            }
            mSize = list.size();
            return *this;
        }

        void PushFront(glm::vec3 point)
        {
            mPoints = {point, mPoints[0], mPoints[1], mPoints[2]};
            mSize = std::min(mSize + 1, 4u);
        }

        glm::vec3& operator[](unsigned i) {return mPoints[i];}
        unsigned size() const {return mSize;}
        auto begin() const {return mPoints.begin();}
        auto end() const {return mPoints.end() - (4 - mSize);}
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
        
        
        CollisionData CheckCollisionsGJK3D(cRigidBody* obj);
        CollisionData CheckCollisionsGJK2D(cRigidBody* obj);
        glm::vec3 Support(const std::vector<glm::vec3>& verticesA, const std::vector<glm::vec3>& verticesB, glm::vec3 d);
        glm::vec3 GetFarthestPoint(const std::vector<glm::vec3>& vertices, const glm::vec3& d);
        bool NextSimplex3D(Simplex& simplex, glm::vec3& d);
        bool NextSimplex2D(Simplex& simplex, glm::vec3& d);
        CollisionData EPA(const Simplex& simplex, const std::vector<glm::vec3>& verticesA, const std::vector<glm::vec3>& verticesB);
        std::pair<std::vector<glm::vec4>, size_t> GetFaceNormals(const std::vector<glm::vec3>& polytope,const std::vector<size_t>& faces);
        void AddIfUniqueEdge(std::vector<std::pair<size_t, size_t>>& edges, const std::vector<size_t>& faces, size_t a, size_t b);

        bool Line2D(Simplex& simplex, glm::vec3& d);
        bool Line3D(Simplex& simplex, glm::vec3& d);
        bool Triangle3D(Simplex& simplex, glm::vec3& d);
        bool Triangle2D(Simplex& simplex, glm::vec3& d);
        bool Tetrahedron(Simplex& simplex, glm::vec3& d);
        bool SameDirection(const glm::vec3& d, const glm::vec3& ao);
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