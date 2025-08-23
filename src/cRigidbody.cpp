#include "cRigidbody.h"
void cRigidBody::SetVertices(const std::vector<glm::vec3>& aVertices)
{
    mVertices = aVertices;
}

cRigidBody::cRigidBody(Entity* entity) : Component(entity)
{
    mTransform = GetMasterEntity()->GetTransformPointer();

    InspectorVariables.push_back({"velocity", VEC3, glm::value_ptr(velocity)});
    InspectorVariables.push_back({"Angular Velocity", VEC3, glm::value_ptr(AngVelocity)});
    InspectorVariables.push_back({"mass", FLOAT, reinterpret_cast<void*>(&mMass)});
    InspectorVariables.push_back({"InvInertia", FLOAT, reinterpret_cast<void*>(&InvInertiaCenter)});
    InspectorVariables.push_back({"static", BOOL, reinterpret_cast<void*>(&mStatic)});
    InspectorVariables.push_back({"static friction", FLOAT, reinterpret_cast<void*>(&StaticFriction), 0.01f, 0.0f, 1.0f});
    InspectorVariables.push_back({"dynamic friction", FLOAT, reinterpret_cast<void*>(&DynamicFriction), 0.01f, 0.0f, 1.0f});
    name = "Rigidbody";
}

void cRigidBody::Init()
{
    InertiaCenter = (1.0f/12.0f) * mMass * (mTransform->scale.x * mTransform->scale.x + mTransform->scale.y * mTransform->scale.y);
    mInvMass = 1.0f / mMass;
    InvInertiaCenter = 1.0f / InertiaCenter;
}

void cRigidBody::CalculateMassIner()
{
    if(mStatic)
    {
        mInvMass = 0.0f;
        InvInertiaCenter = 0.0f;
    }
    else if(!mStatic && mMass != 0.0f)
    {
        InertiaCenter = (1.0f/12.0f) * mMass * (mTransform->scale.x * mTransform->scale.x + mTransform->scale.y * mTransform->scale.y);
        mInvMass = 1.0f / mMass;
        InvInertiaCenter = 1.0f / InertiaCenter;
    }
}
void cRigidBody::SetStatic(bool aStatic)
{
    mStatic = aStatic;

}

void cRigidBody::Update(float deltaTime)
{    

    if(mStatic) return;


    glm::vec3 RotationPoint = mTransform->position;


    glm::vec3 gravity(0,GConst*mMass, 0);
    glm::vec3 forceSum(0);
    mForces.push_back({gravity, mTransform->position});
    for(Force force : mForces)
    {
        forceSum += force.force;
    }
    // glm::vec3 DampingForce = -DC * (velocity + glm::cross(RotVelocity, mTransform->position - RotationPoint));
    // mForces.push_back({DampingForce, mTransform->position});
    float PointInertia = InertiaCenter + mMass * glm::length2(mTransform->position - RotationPoint);
    glm::vec3 TorqueSum(0);
    for(Force force : mForces)
    {
        TorqueSum += glm::cross(force.force, RotationPoint - force.point);
    }
    // TorqueSum += -DC * 100.0f * RotVelocity; // Resistance for rotation
    glm::vec3 acceleration = forceSum / mMass;
    velocity += acceleration * deltaTime;
    mTransform->position += velocity*deltaTime;


    glm::vec3 AngAccelaration = TorqueSum / PointInertia;
    AngVelocity += AngAccelaration * deltaTime;
    mTransform->rotation += AngVelocity * deltaTime;
    // float angle = RotVelocity.z * deltaTime;
    // float xr = mTransform->position.x - RotationPoint.x;
    // float yr = mTransform->position.y - RotationPoint.y;
    // mTransform->position.x = xr*glm::cos(angle) - yr*glm::sin(angle) + RotationPoint.x;
    // mTransform->position.y = xr*glm::sin(angle) + yr*glm::cos(angle) + RotationPoint.y;
    mForces.clear();
}


glm::vec2  ProjectShape(glm::vec3 axis, const std::vector<glm::vec3>& vertices)
{
    float min = glm::dot(axis, vertices[0]);
    float max = min;

    for (int i = 0; i < vertices.size(); i++) 
    {
        float p = glm::dot(axis,vertices[i]);
        if(p < min)
            min = p;
        else if(p > max)
            max = p;
    }
    return glm::vec2(min, max);
}
CollisionData cRigidBody::CheckCollisionsSAT(cRigidBody* obj)
{

    //Basic collision using the SAT 
    //https://dyn4j.org/2010/01/sat/
    std::vector<glm::vec3> verticesA = GetWorldCoordinates();
    std::vector<glm::vec3> verticesB = obj->GetWorldCoordinates();

    std::vector<glm::vec3> axes;

    for (int i = 0; i < verticesA.size(); i++) 
    {
        glm::vec3 p1 = verticesA[i];
        glm::vec3 p2 = verticesA[i+1==verticesA.size() ? 0 : i + 1];

        glm::vec3 edge = p1 - p2;
        glm::vec3 normal(-edge.y, edge.x, 0.0f);
        axes.push_back(glm::normalize(normal));
    }
    for (int i = 0; i < verticesB.size(); i++) 
    {
        glm::vec3 p1 = verticesB[i];
        glm::vec3 p2 = verticesB[i+1==verticesB.size() ? 0 : i + 1];

        glm::vec3 edge = p1 - p2;
        glm::vec3 normal(-edge.y, edge.x, 0.0f);
        axes.push_back(glm::normalize(normal));
    }
    float MinOverlap = INFINITY;
    glm::vec3 smallestAxis;
    for (int i = 0; i < axes.size(); i++)
    {
        glm::vec3 axis = axes[i];
        glm::vec2 p1 = ProjectShape(axis, verticesA);
        glm::vec2 p2 = ProjectShape(axis, verticesB);
        if(!(p1.y >= p2.x && p2.y >= p1.x))
        {
            //One axis is not overlapping so the SAT guarantees that there isn't a collision.
            return {false, glm::vec3(0), {}};
        }
        else
        {
            float overlap = glm::min(p1.y, p2.y) - glm::max(p1.x, p2.x);
            if(MinOverlap > overlap)
            {
                MinOverlap = overlap;
                smallestAxis = axis;
            }
        }
    }
    glm::vec3 normal = glm::normalize(smallestAxis);
    glm::vec3 displacement = -normal * MinOverlap;
    bool flipObj = false;
    if(glm::dot(obj->GetTransform().position - GetTransform().position, normal) < 0)
    {
        displacement *= -1.0f;
        flipObj = true;
    }

    if(obj->GetStatic())
    {
        mTransform->position += displacement;
    }
    else if(GetStatic())
    {
        obj->mTransform->position += -displacement;
    }
    else{
        mTransform->position += displacement * 0.5f;
        obj->mTransform->position += -displacement * 0.5f;
    }

    //Getting the collision points uing the clipping method 
    // https://dyn4j.org/2011/11/contact-points-using-clipping/
    if(!flipObj)
    {
        verticesA = GetWorldCoordinates();
        verticesB = obj->GetWorldCoordinates();
    }else
    {
        verticesA = obj->GetWorldCoordinates();
        verticesB = GetWorldCoordinates();
    }
    std::vector<glm::vec3> contactPoints = GetCollisionPoints(verticesA, verticesB, normal);
    if(flipObj)
        normal *= -1.0f;

    return {true, -normal, MinOverlap, contactPoints};
}



CollisionData cRigidBody::CheckCollisionsGJK(cRigidBody* obj)
{
    return {};
}

std::vector<glm::vec3> cRigidBody::GetCollisionPoints(const std::vector<glm::vec3>& verticesA, const std::vector<glm::vec3>& verticesB, glm::vec3 normal)
{
    Edge e1 = Best(normal, verticesA);
    Edge e2 = Best(-normal, verticesB);
    
    Edge ref,inc;
    bool flip = false;
    if(glm::abs(glm::dot(e1.edge(), normal)) <= glm::abs(glm::dot(e2.edge(), normal)))
    {
        ref = e1;
        inc = e2;
    }else{
        ref = e2;
        inc = e1;
        flip = true;
    }
    glm::vec3 refNorm(-ref.edge().y, ref.edge().x, 0.0f);


    glm::vec3 refv = glm::normalize(ref.edge());
    float o1 =glm::dot(refv,ref.v1);
    std::vector<glm::vec3> cp1 = Clip(inc.v1, inc.v2, refv, o1);


    if(cp1.size() < 2)
        return {};

    float o2 = glm::dot(refv, ref.v2);
    std::vector<glm::vec3> cp2 = Clip(cp1[0], cp1[1], -refv, -o2);
    if(cp2.size() < 2)
        return {};


    if(flip)
        refNorm *= -1.0f;

    float max = glm::dot(refNorm, ref.max);
    if (glm::dot(refNorm, cp2[0]) - max < 0.0f) {
        cp2.erase(cp2.begin());
    }
    if (glm::dot(refNorm, cp2[1]) - max < 0.0f) {
        cp2.erase(cp2.begin() + 1);
    }
    return cp2;
}
std::vector<glm::vec3> cRigidBody::Clip(glm::vec3 v1, glm::vec3 v2, glm::vec3 n, float o)
{
    std::vector<glm::vec3> cp;
    float d1 = glm::dot(n,v1) - o;
    float d2 = glm::dot(n,v2) - o;
    if(d1 >= 0.0f) cp.push_back(v1);
    if(d2 >= 0.0f) cp.push_back(v2);

    if(d1*d2<0.0f)
    {
        glm::vec3 e = v2 - v1;
        float u = d1 / (d1 - d2);
        e *= u;
        e += v1;
        cp.push_back(e); 
    }
    return cp;
}


Edge cRigidBody::Best(glm::vec3 normal, const std::vector<glm::vec3>& vertices)
{
    float max = -INFINITY;
    int index;
    for(int i = 0; i < vertices.size(); i++)
    {
        float proj = glm::dot(normal, vertices[i]);
        if(proj > max)
        {
            max = proj;
            index = i;
        }
    }

    glm::vec3 v = vertices[index];
    glm::vec3 v1 = vertices[index + 1 == vertices.size() ? 0 : index + 1];
    glm::vec3 v0 = vertices[index - 1 == -1 ? vertices.size()-1 : index - 1];
    glm::vec3 l = glm::normalize(v-v1);
    glm::vec3 r = glm::normalize(v-v0);
    if(glm::dot(r,normal) < glm::dot(l,normal))
    {
        Edge e {v, v0, v};
        e.max = (glm::dot(v, normal) > glm::dot(v0, normal)) ? v : v0;
        return e;
    }else
    {
        Edge e { v, v1, v };
        e.max = (glm::dot(v, normal) > glm::dot(v1, normal)) ? v : v1;
        return e;
    }
}


const std::vector<glm::vec3> cRigidBody::GetWorldCoordinates()
{
    std::vector<glm::vec3> NewVertices;

    glm::mat4 model(1.0f);
    model = glm::translate(model, mTransform->position);
    model = glm::rotate(model, mTransform->rotation.x, glm::vec3(1, 0, 0));
    model = glm::rotate(model, mTransform->rotation.y, glm::vec3(0, 1, 0));
    model = glm::rotate(model, mTransform->rotation.z, glm::vec3(0, 0, 1));
    model = glm::scale(model, mTransform->scale);

    for (glm::vec3 vertex : mVertices) 
    {
        //Not using the function because it would be wasteful to calculate the model each time.
        NewVertices.push_back(model * glm::vec4(vertex,1.0f)); 
    }
    return NewVertices;
}
glm::vec3 cRigidBody::ToWorldCoordinates(glm::vec3 point)
{
    glm::mat4 model(1.0f);
    model = glm::translate(model, mTransform->position);
    model = glm::rotate(model, mTransform->rotation.x, glm::vec3(1, 0, 0));
    model = glm::rotate(model, mTransform->rotation.y, glm::vec3(0, 1, 0));
    model = glm::rotate(model, mTransform->rotation.z, glm::vec3(0, 0, 1));

    return glm::vec3(model * glm::vec4(point,1.0f));
}

bool cRigidBody::AlmostEqual(float a, float b)
{
    return glm::abs(a-b) < 0.0001f;
}