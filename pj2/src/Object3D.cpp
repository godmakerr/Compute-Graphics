#include "Object3D.h"

bool Sphere::intersect(const Ray &r, float tmin, Hit &h) const
{
    // BEGIN STARTER

    // We provide sphere intersection code for you.
    // You should model other intersection implementations after this one.

    // Locate intersection point ( 2 pts )
    const Vector3f &rayOrigin = r.getOrigin(); //Ray origin in the world coordinate
    const Vector3f &dir = r.getDirection();

    Vector3f origin = rayOrigin - _center;      //Ray origin in the sphere coordinate

    float a = dir.absSquared();
    float b = 2 * Vector3f::dot(dir, origin);
    float c = origin.absSquared() - _radius * _radius;

    // no intersection
    if (b * b - 4 * a * c < 0) {
        return false;
    }

    float d = sqrt(b * b - 4 * a * c);

    float tplus = (-b + d) / (2.0f*a);
    float tminus = (-b - d) / (2.0f*a);

    // the two intersections are at the camera back
    if ((tplus < tmin) && (tminus < tmin)) {
        return false;
    }

    float t = 10000;
    // the two intersections are at the camera front
    if (tminus > tmin) {
        t = tminus;
    }

    // one intersection at the front. one at the back 
    if ((tplus > tmin) && (tminus < tmin)) {
        t = tplus;
    }

    if (t < h.getT()) {
        Vector3f normal = r.pointAtParameter(t) - _center;
        normal = normal.normalized();
        h.set(t, this->material, normal);
        return true;
    }
    // END STARTER
    return false;
}

// Add object to group
void Group::addObject(Object3D *obj) {
    m_members.push_back(obj);
}

// Return number of objects in group
int Group::getGroupSize() const {
    return (int)m_members.size();
}

bool Group::intersect(const Ray &r, float tmin, Hit &h) const
{
    // BEGIN STARTER
    // we implemented this for you
    bool hit = false;
    for (Object3D* o : m_members) {
        if (o->intersect(r, tmin, h)) {
            hit = true;
        }
    }
    return hit;
    // END STARTER
}


Plane::Plane(const Vector3f &normal, float d, Material *m) : Object3D(m) {
    // TODO implement Plane constructor
    _normal = normal; 		
    _d = d;
    material = m;
}
bool Plane::intersect(const Ray &r, float tmin, Hit &h) const
{
    // TODO implement
    const Vector3f &rayOrigin = r.getOrigin(); //Ray origin in the world coordinate
    const Vector3f &dir = r.getDirection();

    Vector3f T = _d * _normal - rayOrigin;
    float t = Vector3f::dot(T, _normal) / Vector3f::dot(dir, _normal); // <= dot(rayOrigin + t * dir - P0, n) = 0 <= dot(P - P0, n) = 0

    if (t < tmin)
    {
        return false;
    }

    if (t < h.getT())
    {
        h.set(t, this->material, _normal);
        return true;
    }

    return false;
}
bool Triangle::intersect(const Ray &r, float tmin, Hit &h) const 
{
    // TODO implement
    Matrix3f T(_v[1] - _v[0], _v[2] - _v[0], -r.getDirection()); // 系数矩阵T[V1 - V0, V2 - V0, -D]
    Vector3f vec = T.inverse() * (r.getOrigin() - _v[0]); // [β, γ, t]^T = T^(-1) * (O - V0)

    if (vec[0] > 0 && vec[1] > 0 && vec[0] + vec[1] < 1 && vec[2] >= tmin && vec[2] < h.getT()) // β > 0, γ > 0, β+γ < 1, t >= tmin, t < h.getT()
    {
        Vector3f normal = (1 - vec[0] - vec[1]) * _normals[0] + vec[0] * _normals[1] + vec[1] * _normals[2];
        normal = normal.normalized(); // 重心坐标插值 normal = (1-β-γ)*N0 + β*N1 + γ*N2
        h.set(vec[2], this->material, normal);
        return true;
    }
    return false;
}


Transform::Transform(const Matrix4f &m,
    Object3D *obj) : _object(obj) {
    // TODO implement Transform constructor
    _matrix = m;
}
bool Transform::intersect(const Ray &r, float tmin, Hit &h) const
{
    // TODO implement
    Matrix4f _m_inverse = _matrix.inverse(); // 逆矩阵用于将光线变换到物体的局部坐标系
    Vector3f TransformD((_m_inverse * Vector4f(r.getDirection(), 0)).xyz()); // 变换后方向向量
    Ray TransformRay((_m_inverse * Vector4f(r.getOrigin(), 1)).xyz(), TransformD); // 变换后光线

    if (_object->intersect(TransformRay, tmin * TransformD.abs(), h)) // 光线与物体是否相交？
    {
        Vector3f normal = (_m_inverse.transposed() * Vector4f(h.getNormal(), 0)).xyz();
        normal = normal.normalized();
        h.set(h.getT(), h.getMaterial(), normal); // 更新法向量
        return true;
    }
    return false;
}