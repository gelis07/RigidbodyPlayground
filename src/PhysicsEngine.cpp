#include "PhysicsEngine.h"

void PhysicsEngine::Init()
{
}

void PhysicsEngine::Update(const std::vector<cRigidBody*>& rigidbodies, float dt)
{
    for(int i = 0; i < rigidbodies.size(); i++)
    {
        rigidbodies[i]->Update(dt);
    }
    for(int a = 0; a < rigidbodies.size(); a++)
    {
        //b=a+1 because that way it doesn't check already checked collisions (and with itself).
        //for example object A will check a collision with B but not B with A.
        for (int b = a+1; b < rigidbodies.size(); b++) 
        {
            CollisionData data = rigidbodies[a]->CheckCollisionsSAT(rigidbodies[b]);
            if(data.collided)
            {
                CollisionResolution(rigidbodies[a], rigidbodies[b], data);
            }
        }
    }
}


void PhysicsEngine::CollisionResolution(cRigidBody* A, cRigidBody* B, const CollisionData& data)
{
    //These variables need to be calculated once per iteration.
    A->CalculateMassIner();
    B->CalculateMassIner();
    glm::vec3 n = data.normal;
    float e = 0.0f;
    const int iterations = 15;
    float sf = (A->GetSf() + B->GetSf()) * 0.5f;
    float df = (A->GetDf() + B->GetDf()) * 0.5f;
    for(int iter = 0; iter < iterations; iter++)
    {
        std::vector<ImpulseData> ImpData;
        ImpData.resize(data.contactPoints.size());
        std::vector<glm::vec3> FrictionImpulses;
        FrictionImpulses.resize(data.contactPoints.size());
        for (int p = 0; p < data.contactPoints.size(); p++)
        {
            // Utilities::print(data.contactPoints[p]);
            glm::vec3 ra = data.contactPoints[p] - A->GetTransform().position;
            glm::vec3 rb = data.contactPoints[p] - B->GetTransform().position;
            
            glm::vec3 rat = glm::vec3(-ra.y, ra.x, 0.0f);
            glm::vec3 rbt = glm::vec3(-rb.y, rb.x, 0.0f);

            float raPerpDotN = glm::dot(rat, n);
            float rbPerpDotN = glm::dot(rbt, n);
            glm::vec3 angLinVelA = rat * A->AngVelocity.z;
            glm::vec3 angLinVelB = rbt * B->AngVelocity.z;
            glm::vec3 Vp = A->velocity + angLinVelA - B->velocity - angLinVelB;
            float PVelMagn = glm::dot(Vp, n);
            float denom = A->GetInvMass() + B->GetInvMass() 
            + (raPerpDotN * raPerpDotN) * A->GetInvInertia()
            + (rbPerpDotN * rbPerpDotN) * B->GetInvInertia();

            float Impulse = -(1.0f+e) * glm::dot(Vp, n) / (denom * data.contactPoints.size());
            ImpData[p] = {ra, rb, Impulse};
            //*Gauss-seidel method
            A->velocity += Impulse * n * A->GetInvMass();
            A->AngVelocity += glm::cross(ra, Impulse * n) * A->GetInvInertia();
            B->velocity += -Impulse * n * B->GetInvMass();
            B->AngVelocity += -glm::cross(rb, Impulse * n) * B->GetInvInertia();
        }
        //*Jacobi method
        // for (ImpulseData impulse : ImpData)
        // {
        //     A->velocity += impulse.j * n * A->GetInvMass();
        //     A->AngVelocity += glm::cross(impulse.ra, impulse.j * n) * A->GetInvInertia();

        //     B->velocity += -impulse.j * n * B->GetInvMass();
        //     B->AngVelocity += -glm::cross(impulse.rb, impulse.j * n) * B->GetInvInertia();
        // }

        // *Friction
        for (int p = 0; p < data.contactPoints.size(); p++)
        {
            glm::vec3 ra = data.contactPoints[p] - A->GetTransform().position;
            glm::vec3 rb = data.contactPoints[p] - B->GetTransform().position;
            
            glm::vec3 rat = glm::vec3(-ra.y, ra.x, 0.0f);
            glm::vec3 rbt = glm::vec3(-rb.y, rb.x, 0.0f);

            glm::vec3 angLinVelA = rat * A->AngVelocity.z;
            glm::vec3 angLinVelB = rbt * B->AngVelocity.z;
            glm::vec3 Vp = A->velocity + angLinVelA - B->velocity - angLinVelB;
            
            glm::vec3 tangent = Vp - glm::dot(Vp, n) * n;
            if(Utilities::AlmostEqual(glm::length2(tangent), 0.0f))
            {
                continue;
            }else{
                tangent = glm::normalize(tangent);
            }
            float raPerpDotT = glm::dot(rat, tangent);
            float rbPerpDotT = glm::dot(rbt, tangent);

            float denom = A->GetInvMass() + B->GetInvMass() 
            + (raPerpDotT * raPerpDotT) * A->GetInvInertia()
            + (rbPerpDotT * rbPerpDotT) * B->GetInvInertia();

            float ImpulseFriction = 0.0f;
            ImpulseFriction = -glm::dot(Vp, tangent) / (denom * data.contactPoints.size());
            glm::vec3 Friction(0);
            if(glm::abs(ImpulseFriction) <= ImpData[p].j * sf)
            {
                Friction = ImpulseFriction * tangent;
            }else
            {
                Friction = -ImpData[p].j * tangent * df;
            }
            
            FrictionImpulses[p] = Friction;
            //*Gauss-seidel method
            A->velocity += Friction * A->GetInvMass();
            A->AngVelocity += glm::cross(ImpData[p].ra, Friction) * A->GetInvInertia();

            B->velocity += -Friction * B->GetInvMass();
            B->AngVelocity += -glm::cross(ImpData[p].rb, Friction) * B->GetInvInertia();
        }
        //*Jacobi method
        // for (int i = 0; i < ImpData.size(); i++)
        // {
        //     ImpulseData impulse = ImpData[i];
        //     glm::vec3 FrictionImpulse = FrictionImpulses[i];

        //     A->velocity += FrictionImpulse * A->GetInvMass();
        //     A->AngVelocity += glm::cross(impulse.ra, FrictionImpulse) * A->GetInvInertia();

        //     B->velocity += -FrictionImpulse * B->GetInvMass();
        //     B->AngVelocity += -glm::cross(impulse.rb, FrictionImpulse) * B->GetInvInertia();
        // }
    }

}