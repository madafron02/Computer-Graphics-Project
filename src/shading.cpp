#include "texture.h"
#include <cmath>
#include <glm/geometric.hpp>
#include <shading.h>
#include <iostream>

const glm::vec3 computeShading(const glm::vec3& lightPosition, const glm::vec3& lightColor, const Features& features, Ray ray, HitInfo hitInfo)
{
    // TODO: implement the Phong shading model.
    glm::vec3 result = hitInfo.material.kd * lightColor * glm::max(0.0f, glm::dot(hitInfo.normal, normalize(lightPosition - (ray.origin + ray.t*ray.direction))));
    return result;
}


const Ray computeReflectionRay (Ray ray, HitInfo hitInfo)
{
    // Do NOT use glm::reflect!! write your own code.
    Ray reflectionRay = ray;
    // TODO: implement the reflection ray computation.
    reflectionRay.direction = ray.direction - 2 * glm::dot(ray.direction, hitInfo.normal) * hitInfo.normal;
    return reflectionRay;
}