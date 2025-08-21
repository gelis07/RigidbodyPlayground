#include "PhysicsEngine.h"

void PhysicsEngine::Init()
{
}

void PhysicsEngine::Update(const std::vector<cRigidBody*>& rigidbodies, float time)
{
    float dt = 0.02f;
    for(int a = 0; a < rigidbodies.size(); a++)
    {
        //b=a+1 because that way it doesn't check already checked collisions (and with itself).
        //for example object A will check a collision with B but not B with A.
        for (int b = a+1; b < rigidbodies.size(); b++) 
        {
            CollisionData data = rigidbodies[a]->CheckCollisions(rigidbodies[b]);
            if(data.collided)
            {
                CollisionResolution(rigidbodies[a], rigidbodies[b], data);
            }
        }
        rigidbodies[a]->Update(dt);
    }
}


void PhysicsEngine::CollisionResolution(cRigidBody* A, cRigidBody* B, const CollisionData& data)
{
    //These variables need to be calculated once per iteration.
    A->CalculateMassIner();
    B->CalculateMassIner();
    glm::vec3 n = data.normal;
    float e = 0.0f;
    const int iterations = 20;
    std::vector<glm::vec3> ra;
    ra.resize(data.contactPoints.size());
    std::vector<glm::vec3> rat;
    rat.resize(data.contactPoints.size());
    std::vector<glm::vec3> rb;
    rb.resize(data.contactPoints.size());
    std::vector<glm::vec3> rbt;
    rbt.resize(data.contactPoints.size());
    std::vector<float> raPerpDotN;
    raPerpDotN.resize(data.contactPoints.size());
    std::vector<float> rbPerpDotN;
    rbPerpDotN.resize(data.contactPoints.size());
    for(int iter = 0; iter < iterations; iter++)
    {
        // std::vector<ImpulseData> ImpData;
        // ImpData.resize(data.contactPoints.size());
        for (int p = 0; p < data.contactPoints.size(); p++)
        {
            if(iter == 0)
            {
                ra[p] = data.contactPoints[p] - A->GetTransform().position;
                rb[p] = data.contactPoints[p] - B->GetTransform().position;
                
                rat[p] = glm::vec3(-ra[p].y, ra[p].x, 0.0f);
                rbt[p] = glm::vec3(-rb[p].y, rb[p].x, 0.0f);

                raPerpDotN[p] = glm::dot(rat[p], n);
                rbPerpDotN[p] = glm::dot(rbt[p], n);
            }
            glm::vec3 angLinVelA = rat[p] * A->AngVelocity.z;
            glm::vec3 angLinVelB = rbt[p] * B->AngVelocity.z;
            glm::vec3 Vp = A->velocity + angLinVelA - B->velocity - angLinVelB;
            float PVelMagn = glm::dot(Vp, n);
            float denom = A->GetInvMass() + B->GetInvMass() 
            + (raPerpDotN[p] * raPerpDotN[p]) * A->GetInvInertia()
            + (rbPerpDotN[p] * rbPerpDotN[p]) * B->GetInvInertia();


            float Impulse = -(1.0f+e) * glm::dot(Vp, n) / denom;
            std::cout << Impulse << '\n';
            // ImpData[p] = {ra[p], rb[p], Impulse};
            A->velocity += Impulse * n * A->GetInvMass();
            A->AngVelocity += glm::cross(ra[p], Impulse * n) * A->GetInvInertia();

            B->velocity += -Impulse * n * B->GetInvMass();
            B->AngVelocity += -glm::cross(rb[p], Impulse * n) * B->GetInvInertia();
        }
        // for (ImpulseData impulse : ImpData)
        // {
        //     A->velocity += impulse.j * n * A->GetInvMass();
        //     A->AngVelocity += glm::cross(impulse.ra, impulse.j * n) * A->GetInvInertia();
        //     B->velocity += -impulse.j * n * B->GetInvMass();
        //     B->AngVelocity += glm::cross(impulse.rb, impulse.j * n) * B->GetInvInertia();
        // }
    }
}