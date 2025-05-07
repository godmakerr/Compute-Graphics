#include "Material.h"
#include <math.h>
#include <algorithm>

Vector3f Material::shade(const Ray &ray,
    const Hit &hit,
    const Vector3f &dirToLight,
    const Vector3f &lightIntensity)
{
    // TODO implement Diffuse and Specular phong terms
    float LN = std::max(0.0f, Vector3f::dot(dirToLight, hit.getNormal()));
    Vector3f diffuse = LN * lightIntensity * _diffuseColor;
    float LR = std::max(0.0f, Vector3f::dot(dirToLight, 2.0 * Vector3f::dot(-ray.getDirection(), hit.getNormal()) * hit.getNormal() + ray.getDirection()));
    Vector3f specular = pow(LR, _shininess) * lightIntensity * _specularColor;

    return diffuse + specular;
}
