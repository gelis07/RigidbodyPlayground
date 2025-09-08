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
    A->CalculateMassIner();
    B->CalculateMassIner();
    glm::vec3 n = data.normal;
    float e = 0.0f;
    float sf = (A->GetSf() + B->GetSf()) * 0.5f;
    float df = (A->GetDf() + B->GetDf()) * 0.5f;
    std::vector<ImpulseData> ImpData(data.contactPoints.size());
    if(data.contactPoints.size() == 1)
    {
        glm::vec3 ra = data.contactPoints[0] - A->GetTransform().position;
        glm::vec3 rb = data.contactPoints[0] - B->GetTransform().position;
        glm::vec3 rat = glm::vec3(-ra.y , ra.x, 0);
        glm::vec3 rbt = glm::vec3(-rb.y , rb.x, 0);


        float M11 = A->GetInvMass() + B->GetInvMass() + 
        glm::dot(rat, n) * glm::dot(rat, n) * A->GetInvInertia() +
        glm::dot(rbt, n) * glm::dot(rbt, n) * B->GetInvInertia();
        glm::vec3 angLinVelA = rat * A->AngVelocity.z;
        glm::vec3 angLinVelB = rbt * B->AngVelocity.z;
        glm::vec3 Vp = A->velocity + angLinVelA - B->velocity - angLinVelB;
        float j = glm::max(0.0f, -(e+1)*glm::dot(Vp,n) / M11);
        glm::vec3 impulse = j * n;
        A->velocity += impulse * A->GetInvMass();
        A->AngVelocity += glm::cross(ra, impulse) * A->GetInvInertia();
        
        B->velocity += -impulse * B->GetInvMass();
        B->AngVelocity += -glm::cross(rb, impulse) * B->GetInvInertia();

        //Update do new velocities.
        Vp = A->velocity + angLinVelA - B->velocity - angLinVelB;
        glm::vec3 t = Vp - glm::dot(Vp, n) * n;
        if(!Utilities::AlmostEqual(glm::length2(t), 0.0f))
            t = glm::normalize(t);
        else
            return;

        float MF = A->GetInvMass() + B->GetInvMass() +
        glm::dot(t, rat) * glm::dot(t, rat) * A->GetInvInertia() + 
        glm::dot(t, rbt) * glm::dot(t, rbt) * B->GetInvInertia();

        float StopVel = -glm::dot(Vp, t);
        float jf = StopVel / MF;
        glm::vec3 friction(0);
        if(glm::abs(jf) <= j * df)
            friction = jf * t;
        else
            friction = j * df * -t;

        A->velocity += friction * A->GetInvMass();
        A->AngVelocity += glm::cross(ra, friction) * A->GetInvInertia();
        
        B->velocity += -friction * B->GetInvMass();
        B->AngVelocity += -glm::cross(rb, friction) * B->GetInvInertia();
    }
    else if(data.contactPoints.size() == 2)
    {
        std::vector<glm::vec3> ras(data.contactPoints.size());
        std::vector<glm::vec3> rbs(data.contactPoints.size());
        std::vector<glm::vec3> rats(data.contactPoints.size());
        std::vector<glm::vec3> rbts(data.contactPoints.size());

        for(int i = 0; i < data.contactPoints.size(); i++)
        {
            ras[i] = data.contactPoints[i] - A->GetTransform().position;
            rbs[i] = data.contactPoints[i] - B->GetTransform().position;
            rats[i] = glm::vec3(-ras[i].y , ras[i].x, 0);
            rbs[i] = glm::vec3(-rbs[i].y , rbs[i].x, 0);
        }

        glm::mat2x2 M;
        for(int i = 0; i < data.contactPoints.size(); i++)
        {
            for(int j = 0; j < data.contactPoints.size(); j++)
            {
                M[i][j] = A->GetInvMass() + B->GetInvMass() + 
                glm::dot(rats[i], n) * glm::dot(rats[j], n) * A->GetInvInertia() +
                glm::dot(rbts[i], n) * glm::dot(rbts[j], n) * B->GetInvInertia();
            }
        }

        std::vector<glm::vec3> Vps(data.contactPoints.size());
        for (int i = 0; i < data.contactPoints.size(); i++)
        {
            glm::vec3 angLinVelA = rats[i] * A->AngVelocity.z;
            glm::vec3 angLinVelB = rbts[i] * B->AngVelocity.z;
            Vps[i] = A->velocity + angLinVelA - B->velocity - angLinVelB;
        } 
        /*
            There's going to be a maximum of two collision points so I just solved the equations algebraically:
            a = j1 * M11 + j2 * M21 (1)
            b = j2 * M12 + j2 * M22 (2)
            by hand coming at the solutions that are in the code:
            Mij are the coefficients for each impulse that basically measure "how much" each impulse contributes.
            From the fact that j (j is going to be the sum of the impulses so for each point j = j1 + j2 if there are only two points)
            is the change in momentum ΔP and so j/m = uf - ui = Δu (uf : u final, ui : u initial)
            and that the fact that
            ufp * n = -e * uip * n => uip * n + Δup * n = -e * uip * n => Δup * n = -(e+1) uip* n
            that equation holds for each point (ufp is the final position of the collision point p same for uip)
            you can come to the above system of equations (1),(2) with some algebra.
        */
        //final velocities for each collision point 
        float NewCV1 = -(e+1) * glm::dot(Vps[0], n); 
        float NewCV2 = -(e+1) * glm::dot(Vps[1], n); 
        
        float j1 = glm::max(0.0f, (M[0][1] * NewCV2 - M[1][1]*NewCV1) / (M[0][1] * M[0][1] - M[0][0] * M[1][1]));
        float j2 = glm::max(0.0f, (NewCV2 * M[0][0] - NewCV1 * M[0][1]) / (M[1][1] * M[0][0] - M[1][0] * M[0][1]));

        /*
            One of the impulses is negative or doesnt exist, so the other impulse has to be 
            calculated based on the fact that one is negative so 
            its trying to "pull" it into the object, which can't happen on a normal force 
            (if two objects collide, the objects wont ever pull eachother they only
            excert a force so they don't penetrate eachother)
        */
        if(j1 == 0.0f)
        {
            j2 = NewCV1 / M[0][1];
        }else if(j2 == 0.0f)
        {
            j1 = NewCV1 / M[0][0];
        }

        A->velocity += (j1 + j2) * n * A->GetInvMass();
        A->AngVelocity += Utilities::RoundToDecimal((glm::cross(ras[0], j1 * n) + glm::cross(ras[1], j2 * n)) * A->GetInvInertia(), 4);

        B->velocity += -(j1+j2) * n * B->GetInvMass();
        B->AngVelocity += -(glm::cross(rbs[0], j1*n) + glm::cross(rbs[1], j2*n)) * B->GetInvInertia();

        for (int i = 0; i < data.contactPoints.size(); i++)
        {
            glm::vec3 angLinVelA = rats[i] * A->AngVelocity.z;
            glm::vec3 angLinVelB = rbts[i] * B->AngVelocity.z;
            Vps[i] = A->velocity + angLinVelA - B->velocity - angLinVelB;
        } 

        std::vector<glm::vec3> ts(data.contactPoints.size());
        for(int i = 0; i < data.contactPoints.size(); i++)
        {
            if(!Utilities::AlmostEqual(glm::length2(Vps[i] - glm::dot(Vps[i], n) * n), 0.0f))
                ts[i] = glm::normalize(Vps[i] - glm::dot(Vps[i], n) * n);
            else
                continue;
        }
        glm::mat2x2 MF;

        for(int i = 0; i < data.contactPoints.size(); i++)
        {
            for(int j = 0; j < data.contactPoints.size(); j++)
            {
                MF[i][j] = glm::dot(ts[i], ts[j]) * A->GetInvMass() + glm::dot(ts[i], ts[j]) * B->GetInvMass() +
                glm::dot(ts[i], rats[i]) * glm::dot(ts[j], rats[j]) * A->GetInvInertia() +
                glm::dot(ts[i], rbts[i]) * glm::dot(ts[j], rbts[j]) * B->GetInvInertia();
            }
        }
        float StopV1 = -glm::dot(Vps[0], ts[0]);
        float StopV2 = -glm::dot(Vps[1], ts[1]);

        float jf1 = 0.0f;
        float jf2 = 0.0f;

        if(Utilities::AlmostEqual(MF[0][1] * MF[1][0] - MF[0][0] * MF[1][1], 0.0f) || Utilities::AlmostEqual(MF[1][1] * MF[0][0] - MF[1][0] * MF[1][0], 0.0f))
        {
            //! add a comment explaining here
            jf1 = StopV1 / (2.0f * MF[0][0]);
            jf2 = jf1;
        }else{
            jf1 = (StopV2 * MF[1][0] - StopV1*MF[1][1]) / (MF[0][1] * MF[1][0] - MF[0][0] * MF[1][1]);
            jf2 = (StopV2 * MF[0][0] - StopV1*MF[1][0]) / (MF[1][1] * MF[0][0] - MF[1][0] * MF[1][0]);
        }
        glm::vec3 f1(0);
        glm::vec3 f2(0);
        if(jf1 == 0.0f)
        {
            jf2 = StopV1 / MF[1][0];
        }else if(jf2 == 0.0f)
        {
            jf1 = StopV1 / MF[0][0];
        }
        if(glm::abs(jf1) <= df * j1)
        {
            f1 = jf1 * ts[0];
        }else
        {
            f1 = df * j1 * -ts[0];
        }
        
        if(glm::abs(jf2) <= df * j2)
        {
            f2 = jf2 * ts[1];
        }else
        {
            f2 = df * j2 * -ts[1];
        }

        A->velocity += (f1 + f2) * A->GetInvMass();
        A->AngVelocity += Utilities::RoundToDecimal((glm::cross(ras[0], f1) + glm::cross(ras[1], f2)) * A->GetInvInertia(), 4);

        B->velocity += -(f1+f2) * B->GetInvMass();
        B->AngVelocity += -(glm::cross(rbs[0], f1) + glm::cross(rbs[1], f2)) * B->GetInvInertia();
    }

}