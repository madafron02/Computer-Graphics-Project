#include "texture.h"
#include <cmath>
#include <glm/geometric.hpp>
#include <shading.h>

const glm::vec3 computeShading(const glm::vec3& lightPosition, const glm::vec3& lightColor, const Features& features, Ray ray, HitInfo hitInfo)
{
    // TODO: implement the Phong shading model.
    glm::vec3 cameraToPoint = ray.origin + ray.t * ray.direction;
    glm::vec3 cameraToLight = lightPosition;

    glm::vec3 pointToLight = cameraToLight - cameraToPoint;
    glm::vec3 diffuse = lightColor * hitInfo.material.kd * glm::max(0.0f, glm::dot(hitInfo.normal, normalize(pointToLight)));

    glm::vec3 viewVec = cameraToPoint - ray.origin;
    glm::vec3 lightVec = lightPosition - cameraToPoint;
    glm::vec3 reflected = normalize(- normalize(lightVec) + 2 * (glm::dot(normalize(lightVec), hitInfo.normal)) * hitInfo.normal);
    glm::vec3 specular = lightColor * hitInfo.material.ks * pow(glm::max(0.0f, glm::dot(normalize(viewVec), reflected)), hitInfo.material.shininess);

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