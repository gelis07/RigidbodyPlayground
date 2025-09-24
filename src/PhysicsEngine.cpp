#include "PhysicsEngine.h"
#include <imgui.h>
#include "Timings.h"
void PhysicsEngine::Init()
{

}


void PhysicsEngine::DebugUI()
{
    ImGui::Begin("Physics Settings");
    ImGui::Checkbox("Enable Friction", &enableFriction);
    ImGui::End();
}
float e = 0.0f;
void PhysicsEngine::Update(const std::vector<cRigidBody*>& rigidbodies, float dt)
{
    mColObjects.resize(rigidbodies.size());
    for(int i = 0; i < rigidbodies.size(); i++)
    {
        rigidbodies[i]->Update(dt);
    }
    Timings::StartTimer("Collision Detection");
    for(int a = 0; a < rigidbodies.size(); a++)
    {
        //b=a+1 because that way it doesn't check already checked collisions (and with itself).
        //for example object A will check a collision with B but not B with A.
        for (int b = a+1; b < rigidbodies.size(); b++) 
        {
            CollisionData data = rigidbodies[a]->CheckCollisionsSAT(rigidbodies[b]);
            if(data.collided)
            {
                CollisionResolution(rigidbodies, a, b, data, dt);
            }
        }
    }
    Timings::EndTimer();
    if(mCollisions.size() > 0)
    {
        Timings::StartTimer("Setting up for solving");
        std::vector<std::vector<float>> matrix(mCollisions.size(), std::vector<float>(mCollisions.size(), 0.0f));
        std::vector<float> c(mCollisions.size());
        impulses.resize(c.size());
        for(int i = 0; i < mCollisions.size(); i++)
        {
            for(int j = 0; j < mCollisions[i].objects.size(); j++)
            {
                float objMult = 1.0f;
                if(j == 1)
                    objMult *= -1.0f;
                for (int k = 0; k < mColObjects[mCollisions[i].objects[j]].ImpulsesActed.size(); k++)
                {
                    cRigidBody* rb = rigidbodies[mCollisions[i].objects[j]];
                    rb->CalculateMassIner();

                    ImpulseActData impActData = mColObjects[mCollisions[i].objects[j]].ImpulsesActed[k];

                    glm::vec3 ri = mImpulses[impActData.id].point - rb->GetTransform().position;
                    glm::vec3 rit(-ri.y, ri.x, 0.0f);
                    
                    glm::vec3 rp = mCollisions[i].point - rb->GetTransform().position;
                    glm::vec3 rpt(-rp.y, rp.x, 0.0f);
                    glm::vec3 impDir = mImpulses[impActData.id].dir * (float)impActData.mult;
                    matrix[i][impActData.id] += objMult * 
                    (glm::dot(impDir, mCollisions[i].normal) * rb->GetInvMass() + 
                    glm::dot(rit, impDir) * glm::dot(rpt, mCollisions[i].normal) * rb->GetInvInertia());
                }
            }
            c[i] = mCollisions[i].constant;
        }
        Timings::EndTimer();
        
        Timings::StartTimer("Solving the matrix");    
        std::vector<float> preImpulses(impulses);
        SolveLinearSystem(&impulses, matrix, c);
        Timings::EndTimer();
        // for (size_t i = 0; i < impulses.size(); i++)
        // {
            // fmt::println("previous impulse: {}, current impulse: {}", preImpulses[i], impulses[i]);
        // }
        // fmt::println("");
        for (int i = 0; i < mColObjects.size(); i++)
        {
            for (int j = 0; j < mColObjects[i].ImpulsesActed.size(); j++)
            {
                cRigidBody* rb = rigidbodies[i];
                ImpulseActData imp = mColObjects[i].ImpulsesActed[j];
                ImpulseData data = mImpulses[imp.id];
                glm::vec3 r = data.point - rb->GetTransform().position;
                rb->velocity += imp.mult * impulses[imp.id] * data.dir * rb->GetInvMass();  
                rb->AngVelocity += (float)imp.mult * glm::cross(r, impulses[imp.id] * data.dir) * rb->GetInvInertia();  
            }
        }
    }

    mImpulses.clear();
    mCollisions.clear();
    mColObjects.clear();
}
void PhysicsEngine::SolveLinearSystem(std::vector<float>* output, const std::vector<std::vector<float>>& matrix
, const std::vector<float>& constants)
{
    if(matrix.size() != constants.size())
    {
        fmt::println("cant do it sorry");
        return;
    }
    const size_t n = constants.size();
    const int max_iters = 15;

    for (int iter = 0; iter < max_iters; iter++) {
        for (int i = 0; i < n; i++) {
            double sum = 0.0;
            for (int j = 0; j < n; j++) {
                if (j != i) {
                    sum += matrix[i][j] * (*output)[j];
                }
            }
            if (std::abs(matrix[i][i]) < 1e-8) {
                // std::cerr << "Zero diagonal at row " << i << "\n";
                continue;
            }
            double min, max;

            if(mImpulses[i].DmaxLimit == -1)
                max = mImpulses[i].CmaxLimit;
            else
                max = impulses[mImpulses[i].DmaxLimit] * mImpulses[i].CmaxLimit;

            if(mImpulses[i].DminLimit == -1)
                min = mImpulses[i].CminLimit;
            else
                min = impulses[mImpulses[i].DminLimit] * mImpulses[i].CminLimit;

            (*output)[i] = glm::clamp((constants[i] - sum) / matrix[i][i], min, max);
        }
    }
}



void PhysicsEngine::CollisionResolution(const std::vector<cRigidBody*>& rigidbodies, size_t A, size_t B, const CollisionData& data, float dt)
{
    for (int i =0; i < data.contactPoints.size(); i++)
    {
        cRigidBody* bA = rigidbodies[A];
        cRigidBody* bB = rigidbodies[B];
        glm::vec3 ra = data.contactPoints[i] - bA->GetTransform().position;
        glm::vec3 rbp = data.contactPoints[i] - bB->GetTransform().position;
        glm::vec3 rat = glm::vec3(-ra.y , ra.x, 0);
        glm::vec3 rbt = glm::vec3(-rbp.y , rbp.x, 0);
        glm::vec3 angLinVelA = rat * bA->AngVelocity.z;
        glm::vec3 angLinVelB = rbt * bB->AngVelocity.z;
        glm::vec3 Vp = bA->velocity + angLinVelA - bB->velocity - angLinVelB;
        glm::vec3 t(0);
        glm::vec3 tangentVel = Vp - glm::dot(Vp, data.normal) * data.normal;
        if (glm::length2(tangentVel) > 1e-8f)
            t = glm::normalize(tangentVel);
        else
            t = glm::vec3(0);
        if(!utilities::AlmostEqual(glm::length2(t), 0.0f))
            t = glm::normalize(t);
        else
            t = glm::vec3(0);

        mImpulses.push_back({data.normal, data.contactPoints[i], (size_t)-1, (size_t)-1, 0.0f, INFINITY});
        mImpulses.push_back({t, data.contactPoints[i], (size_t)mImpulses.size()-1, (size_t)mImpulses.size()-1, -0.6f, 0.6f});
        
        mColObjects[A].ImpulsesActed.push_back({mImpulses.size()-1,  1});
        mColObjects[B].ImpulsesActed.push_back({mImpulses.size()-1, -1});

        mColObjects[A].ImpulsesActed.push_back({mImpulses.size()-2,  1});
        mColObjects[B].ImpulsesActed.push_back({mImpulses.size()-2, -1});
        mCollisions.push_back({{A, B}, data.contactPoints[i], data.normal, -(e+1.0f) * glm::dot(Vp, data.normal)});
        mCollisions.push_back({{A, B}, data.contactPoints[i], t, -glm::dot(Vp, t)});


    }
}