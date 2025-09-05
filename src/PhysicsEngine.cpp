#include "PhysicsEngine.h"

void PhysicsEngine::Init()
{
}

void PhysicsEngine::Update(const std::vector<cRigidBody*>& rigidbodies, float dt)
{
    //comment test
    //commit test 2
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
        std::cout << j << '\n';
        glm::vec3 impulse = j * n;
        A->velocity += impulse * A->GetInvMass();
        A->AngVelocity += glm::cross(ra, impulse) * A->GetInvInertia();
        
        B->velocity += -impulse * B->GetInvMass();
        B->AngVelocity += -glm::cross(rb, impulse) * B->GetInvInertia();
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
        float a = -(e+1) * glm::dot(Vps[0], n); 
        float b = -(e+1) * glm::dot(Vps[1], n); 
        
        float j1 = glm::max(0.0f, (M[0][1] * b - M[1][1]*a) / (M[0][1] * M[0][1] - M[0][0] * M[1][1]));
        float j2 = glm::max(0.0f, (b * M[0][0] - a * M[0][1]) / (M[1][1] * M[0][0] - M[1][0] * M[0][1]));

        /*
            One of the impulses is negative or doesnt exist, so the other impulse has to be 
            calculated based on the fact that one is negative so 
            its trying to "pull" it into the object, which can't happen on a normal force 
            (if two objects collide, the objects wont ever pull eachother they only
            excert a force so they don't penetrate eachother)
        */
        if(j1 == 0.0f)
        {
            j2 = a / M[0][1];
        }else if(j2 == 0.0f)
        {
            j1 = a / M[0][0];
        }

        glm::vec3 tangent1 = Vps[0] - glm::dot(Vps[0], n) * n;
        glm::vec3 tangent2 = Vps[1] - glm::dot(Vps[1], n) * n;


        A->velocity += (j1 + j2) * n * A->GetInvMass();
        A->AngVelocity += Utilities::RoundToDecimal((glm::cross(ras[0], j1 * n) + glm::cross(ras[1], j2 * n)) * A->GetInvInertia(), 4);

        B->velocity += -(j1+j2) * n * B->GetInvMass();
        B->AngVelocity += -(glm::cross(rbs[0], j1*n) + glm::cross(rbs[1], j2*n)) * B->GetInvInertia();
    }

}