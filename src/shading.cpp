#include "draw.h"
#include "texture.h"
#include <cmath>
#include <cstdlib>
#include <glm/geometric.hpp>
#include <random>
#include <shading.h>

int SAMPLES_NUMBER;

namespace Generator {
    std::mt19937 mt {std::random_device {}()};

    float get(float min, float max) {
        std::uniform_real_distribution die { min, std::nextafter(max, FLT_MAX) };

        return die(mt);
    }
}

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
const Ray computeReflectionRay(Ray ray, HitInfo hitInfo, const Features& features)
{
    Ray reflectionRay = ray;
    // Compute reflection direction (formula from slides) v - 2(dot(N,L) * N
    reflectionRay.direction = normalize(normalize(ray.direction * ray.t) - 2 * glm::dot(normalize(ray.direction * ray.t), hitInfo.normal) * hitInfo.normal);
    // Set the origin of a reflected ray to the point of intersection of a ray
    reflectionRay.origin = ray.origin + (ray.t - 0.0001f) * ray.direction;

    /*
    * GLOSSY REFLECTIONS
    */
    if (features.extra.enableGlossyReflection && hitInfo.material.ks != glm::vec3{0.0f}) {
        glm::vec3 cameraToPoint = ray.origin + ray.t * ray.direction;
        drawRay({ cameraToPoint, reflectionRay.direction, 0.1f }, { 0.0f, 0.0f, 1.0f });

        reflectionRay.direction = distortRayDirection(reflectionRay, hitInfo.material.shininess, SAMPLES_NUMBER);
        drawRay({ cameraToPoint, reflectionRay.direction, 0.1f }, { 1.0f, 1.0f, 0.0f });
    }

    return reflectionRay;
}

glm::vec3 distortRayDirection(const Ray& ray, float shininess, int samples)
{
    float a = 1 / shininess;

    // 1. Find the orthonormal basis:
    // Formula from: "Fundamentals of Computer Graphics" (Steve Marschner & Peter Shirley, 4th edition, 2016)

    glm::vec3 w_vec = glm::normalize(ray.direction);
    // u_vec is supposed to be not collinear with w: to find it
    // we set the smallest u_vec component to 1 (2.4.6 book)
    glm::vec3 u_vec = w_vec;
    if (u_vec.x < u_vec.y && u_vec.x < u_vec.z)
        u_vec.x = 1;
    else if (u_vec.y < u_vec.x && u_vec.y < u_vec.z)
        u_vec.y = 1;
    else
        u_vec.z = 1;
    u_vec = glm::normalize(u_vec);

    glm::vec3 v_vec = glm::normalize(glm::cross(w_vec, u_vec));

    glm::vec3 direction {};

    for (int i = 0; i < samples; ++i) {
        // 2. Create 2 random points in [0,1]:
        float ran1 = Generator::get(0.0f, 1.0f);
        float ran2 = Generator::get(0.0f, 1.0f);

        // 3. Calculate vector coefficients:
        float u = -a / 2 + ran1 * a;
        float v = -a / 2 + ran2 * a;

        // 4. Add sample to direction
        direction += ray.direction + u * u_vec + v * v_vec;
    }

    // 5. Return distorted direction
    return ray.direction + (direction / glm::vec3 { (float)samples });
}
