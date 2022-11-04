#include "texture.h"
#include <cmath>
#include <glm/geometric.hpp>
#include <shading.h>


//Diffuse and specular component calulated based on the formula from from “Fundamentals of Computer Graphics” Chapter 4.5 Ray Tracing - Shading
//L = kd I max(0, n · l) + ks I max(0, n · h) ^ p 
const glm::vec3 computeShading(const glm::vec3& lightPosition, const glm::vec3& lightColor, const Features& features, Ray ray, HitInfo hitInfo)
{

    // Adding textures
    if (features.enableTextureMapping && hitInfo.material.kdTexture) {
        hitInfo.material.kd = acquireTexel(*hitInfo.material.kdTexture, hitInfo.texCoord, features);
    }

    glm::vec3 cameraToPoint = ray.origin + ray.t * ray.direction;
    glm::vec3 cameraToLight = lightPosition;

    // Check if light and camera are facing the same direction
    if (glm::dot(ray.direction, hitInfo.normal) * glm::dot(cameraToPoint - cameraToLight, hitInfo.normal) <= 0) {
        return { 0, 0, 0 };
    }

    // If no diffuse component (eg mirror) return black
    if (hitInfo.material.kd == glm::vec3 { 0, 0, 0 }) {
        return { 0, 0, 0 };
    }

    glm::vec3 pointToLight = cameraToLight - cameraToPoint;
    glm::vec3 diffuse = lightColor * hitInfo.material.kd * glm::abs(glm::dot(hitInfo.normal, normalize(pointToLight)));

    glm::vec3 viewVec = cameraToPoint - ray.origin;
    glm::vec3 lightVec = lightPosition - cameraToPoint;
    glm::vec3 reflected = normalize(-normalize(lightVec) + 2 * (glm::dot(normalize(lightVec), hitInfo.normal)) * hitInfo.normal);
    glm::vec3 specular = lightColor * hitInfo.material.ks * pow(glm::max(0.0f, glm::dot(normalize(viewVec), reflected)), hitInfo.material.shininess);

    // Phong model = diffuse + specular
    return diffuse + specular;
}


//Formula from additional material video about reflection ray
//https://brightspace.tudelft.nl/d2l/le/content/499418/viewContent/3010973/View
const Ray computeReflectionRay(Ray ray, HitInfo hitInfo)
{
    Ray reflectionRay = ray;
    // Compute reflection direction (formula from slides) v - 2(dot(N,L) * N
    reflectionRay.direction = normalize(normalize(ray.direction * ray.t) - 2 * glm::dot(normalize(ray.direction * ray.t), hitInfo.normal) * hitInfo.normal);
    // Set the origin of a reflected ray to the point of intersection of a ray
    reflectionRay.origin = ray.origin + (ray.t - 0.0001f) * ray.direction;
    return reflectionRay;
}
