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
    for (glm::vec3& contactPoint : contactPoints) 
    {
        contactPoint += displacement;
    }
    if(flipObj)
        normal *= -1.0f;

    return {true, -normal, MinOverlap, contactPoints};
}


glm::vec3 cRigidBody::GetFarthestPoint(const std::vector<glm::vec3>& vertices,const glm::vec3& d)
{
    glm::vec3 farthestPoint;
    float farthestDistance = -INFINITY;
    for(int i = 0; i < vertices.size(); i++)
    {
        float distance = glm::dot(d, vertices[i]);
        if (farthestDistance < distance) 
        {
            farthestDistance = distance;
            farthestPoint = vertices[i];
        }
    }
    return farthestPoint;
}

glm::vec3 cRigidBody::Support(const std::vector<glm::vec3>& verticesA, const std::vector<glm::vec3>& verticesB, glm::vec3 d)
{
    glm::vec3 p1 = GetFarthestPoint(verticesA, d);
    glm::vec3 p2 = GetFarthestPoint(verticesB, -d);

    glm::vec3 p3 = p1 - p2;
    return p3;
}

bool cRigidBody::SameDirection(const glm::vec3& d, const glm::vec3& ao)
{
    return glm::dot(d, ao) > 0;
}
bool cRigidBody::Line3D(Simplex& simplex, glm::vec3& d)
{
    glm::vec3 a = simplex[0];
    glm::vec3 b = simplex[1];
    glm::vec3 ab = b - a;
    glm::vec3 ao = -a;

    if(SameDirection(ab, ao))
    {
        d = glm::cross(glm::cross(ab, ao), ab);
    }
    else
    {
        simplex = { a };
        d = ao; 
    }
    return false;
}
bool cRigidBody::Line2D(Simplex& simplex, glm::vec3& d)
{
    glm::vec3 a = simplex[0];
    glm::vec3 b = simplex[1];
    glm::vec3 ab = b - a;
    glm::vec3 ao = -a;
    glm::vec3 abp = glm::cross(glm::cross(ab, ao), ab);
    d = abp;
    return false;
}
bool cRigidBody::Triangle3D(Simplex& simplex, glm::vec3& d)
{
    glm::vec3 a = simplex[0];
    glm::vec3 b = simplex[1];
    glm::vec3 c = simplex[2];

    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ao = -a;
    glm::vec3 abc = glm::cross(ab, ac);
    if(SameDirection(glm::cross(abc, ac), ao))
    {
        if(SameDirection(ac, ao))
        {
            simplex = {a , c};
            d = glm::cross(glm::cross(ac, ao), ac);
        }
        else
        {
            simplex =  {a, b};
            return Line3D(simplex, d);
        }
    }
    else
    {
        if(SameDirection(glm::cross(ab, abc), ao))
        {
            simplex = {a,b};
            return Line3D(simplex, d);
        }else
        {
            if(SameDirection(abc, ao))
            {
                d = abc;
            }else
            {
                simplex = {a , c, b};
                d = -abc;
            }
        }
    }
    return false;
}
bool cRigidBody::Triangle2D(Simplex& simplex, glm::vec3& d)
{
    glm::vec3 a = simplex[0];
    glm::vec3 b = simplex[1];
    glm::vec3 c = simplex[2];

    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ao = -a;
    glm::vec3 acp = glm::cross(glm::cross(ab, ac), ac);
    glm::vec3 abp = glm::cross(glm::cross(ac, ab), ab);
    if(SameDirection(abp, ao))
    {
        simplex =  {a, b};
        d = abp;
        return false;
    }
    else if(SameDirection(acp, ao))
    {
        simplex = {a, c};
        d = acp;
        return false;
    }
    return true;
}
bool cRigidBody::Tetrahedron(Simplex& simplex, glm::vec3& dir)
{
    glm::vec3 a = simplex[0];
    glm::vec3 b = simplex[1];
    glm::vec3 c = simplex[2];
    glm::vec3 d = simplex[3];

    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 ad = d - a;
    glm::vec3 ao = -a;

    glm::vec3 abc = glm::cross(ab, ac);
    glm::vec3 acd = glm::cross(ac, ad);
    glm::vec3 adb = glm::cross(ad, ab);
    if(SameDirection(abc, ao))
    {
        simplex = {a,b,c};
        return Triangle3D(simplex, dir);
    }
    if(SameDirection(acd, ao))
    {
        simplex = {a,c,d};
        return Triangle3D(simplex, dir);
    }
    if(SameDirection(adb, ao))
    {
        simplex = {a,d,b};
        return Triangle3D(simplex, dir);
    }
    return true;
}

bool cRigidBody::NextSimplex3D(Simplex& simplex, glm::vec3& d)
{
    switch(simplex.size())
    {
        case 2: return Line3D(simplex, d);
        case 3: return Triangle3D(simplex, d);
        case 4: return Tetrahedron(simplex, d);
    }
    return false;
}
bool cRigidBody::NextSimplex2D(Simplex& simplex, glm::vec3& d)
{
    switch(simplex.size())
    {
        case 2: return Line2D(simplex, d);
        case 3: return Triangle2D(simplex, d);
        case 4: return Tetrahedron(simplex, d);
    }
    return false;
}
//Only because of these videos I managed to understand this algorithm. Thanks a ton!
//https://www.youtube.com/watch?v=MDusDn8oTSE
//https://www.youtube.com/watch?v=ajv46BSqcK4
CollisionData cRigidBody::CheckCollisionsGJK3D(cRigidBody* obj)
{
    const std::vector<glm::vec3>& verticesA = GetWorldCoordinates();
    const std::vector<glm::vec3>& verticesB = obj->GetWorldCoordinates();
    glm::vec3 d(1, 0, 0);
    Simplex Simplex;
    glm::vec3 support = Support(verticesA, verticesB, d);
    Simplex.PushFront(support);
    d = -support;
    while(true)
    {
        support = Support(verticesA, verticesB, d);
        if(glm::dot(support, d) <= 0)
        {
            return {false, glm::vec3(0), {}};
        }
        Simplex.PushFront(support);
        if(NextSimplex3D(Simplex, d))
        {
            return {true};
        }
    }
    return {false};
}
CollisionData cRigidBody::CheckCollisionsGJK2D(cRigidBody* obj)
{
    const std::vector<glm::vec3>& verticesA = GetWorldCoordinates();
    const std::vector<glm::vec3>& verticesB = obj->GetWorldCoordinates();
    glm::vec3 d(1, 0, 0);
    Simplex Simplex;
    glm::vec3 support = Support(verticesA, verticesB, d);
    Simplex.PushFront(support);
    d = -support;
    while(true)
    {
        support = Support(verticesA, verticesB, d);
        if(glm::dot(support, d) <= 0)
        {
            return {false, glm::vec3(0), {}};
        }
        Simplex.PushFront(support);
        if(NextSimplex2D(Simplex, d))
        {
            return {true};
        }
    }
    return {false};
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
    std::vector<glm::vec3> kept;
    for(glm::vec3& p : cp2)
    {
        if(glm::dot(refNorm, p) - max >= 0.0f)
        {
            kept.push_back(p);
        }
    }
    return kept;
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