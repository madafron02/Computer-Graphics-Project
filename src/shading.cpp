#include "texture.h"
#include <cmath>
#include <glm/geometric.hpp>
#include <shading.h>
#include <iostream>

const glm::vec3 computeShading(const glm::vec3& lightPosition, const glm::vec3& lightColor, const Features& features, Ray ray, HitInfo hitInfo)
{
    // TODO: implement the Phong shading model.
    glm::vec3 lightVec = normalize(lightPosition - (ray.origin + ray.t * ray.direction));
    glm::vec3 diffuse = lightColor * hitInfo.material.kd * glm::max(0.0f, glm::dot(lightVec, hitInfo.normal));

    glm::vec3 viewVec = normalize(ray.origin - (ray.origin + ray.t * ray.direction));
    glm::vec3 reflected = normalize(glm::reflect(glm::vec3 { -1, -1, -1 }  *lightVec, hitInfo.normal));
    glm::vec3 specular = lightColor * hitInfo.material.ks * pow(glm::max(0.0f, glm::dot(reflected, viewVec)), hitInfo.material.shininess);

    return diffuse + specular;
}


const Ray computeReflectionRay (Ray ray, HitInfo hitInfo)
{
    // Do NOT use glm::reflect!! write your own code.
    Ray reflectionRay = ray;
    // TODO: implement the reflection ray computation.
    reflectionRay.direction = ray.direction - 2 * glm::dot(ray.direction, hitInfo.normal) * hitInfo.normal;
    return reflectionRay;
}