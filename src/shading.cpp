#include "draw.h"
#include "texture.h"
#include <cmath>
#include <cstdlib>
#include <glm/geometric.hpp>
#include <random>
#include <shading.h>

#include <iostream>

namespace Generator {
    std::mt19937 mt {std::random_device {}()};

    float get(float min, float max) {
        std::uniform_real_distribution die { min, std::nextafter(max, FLT_MAX) };

        return die(mt);
    }
}

const glm::vec3 computeShading(const glm::vec3& lightPosition, const glm::vec3& lightColor, const Features& features, Ray ray, HitInfo hitInfo)
{
    // TODO: implement the Phong shading model.
    // if(length(hitInfo.material.kd) <= 0.0000001f) return hitInfo.material.kd;

    // check for mirrors !!!

    if (features.enableTextureMapping && hitInfo.material.kdTexture) {
        hitInfo.material.kd = acquireTexel(*hitInfo.material.kdTexture, hitInfo.texCoord, features);
    }

    glm::vec3 cameraToPoint = ray.origin + ray.t * ray.direction;
    glm::vec3 cameraToLight = lightPosition;

    if (glm::dot(ray.direction, hitInfo.normal) * glm::dot(cameraToPoint - cameraToLight, hitInfo.normal) <= 0) {
        return { 0, 0, 0 };
    }
    if (hitInfo.material.kd == glm::vec3 { 0, 0, 0 }) {
        return { 0, 0, 0 };
    }

    glm::vec3 pointToLight = cameraToLight - cameraToPoint;
    glm::vec3 diffuse = lightColor * hitInfo.material.kd * glm::abs(glm::dot(hitInfo.normal, normalize(pointToLight)));

    glm::vec3 viewVec = cameraToPoint - ray.origin;
    glm::vec3 lightVec = lightPosition - cameraToPoint;
    glm::vec3 reflected = normalize(-normalize(lightVec) + 2 * (glm::dot(normalize(lightVec), hitInfo.normal)) * hitInfo.normal);
    /*
    * GLOSSY REFLECTIONS
    */
    //if (features.extra.enableGlossyReflection) {
    //    float a = 1 / ((hitInfo.material.shininess) * (hitInfo.material.shininess));

    //    drawRay({ cameraToPoint, reflected, 0.1f }, { 0.0f, 0.0f, 1.0f });

    //    // 1. Find the orthonormal basis:

    //    glm::vec3 w_vec = glm::normalize(reflected);
    //    // u_vec is supposed to be not collinear with w: to find it
    //    // we set the smallest u_vec component to 1 (2.4.6 book)
    //    glm::vec3 u_vec = w_vec;
    //    if (u_vec.x < u_vec.y && u_vec.x < u_vec.z)
    //        u_vec.x = 1;
    //    else if (u_vec.y < u_vec.x && u_vec.y < u_vec.z)
    //        u_vec.y = 1;
    //    else
    //        u_vec.z = 1;
    //    u_vec = glm::normalize(u_vec);

    //    glm::vec3 v_vec = glm::normalize(glm::cross(w_vec, u_vec));

    //    // 2. Create 2 random points in [0,1]:
    //    float ran1 = Generator::get(0.0f, 1.0f);
    //    float ran2 = Generator::get(0.0f, 1.0f);

    //    // 3. Calculate vector coefficients:
    //    float u = -a / 2 + ran1 * a;
    //    float v = -a / 2 + ran2 * a;

    //    // 4. Replace reflected with a perturbed reflected vector
    //    reflected = reflected + u * u_vec + v * v_vec;
    //    std::cout << "u: " << u << ", v: " << v << '\n';

    //    drawRay({ cameraToPoint, reflected, 0.1f }, { 1.0f, 0.0f, 1.0f });
    //}
  
    glm::vec3 specular = lightColor * hitInfo.material.ks * pow(glm::max(0.0f, glm::dot(normalize(viewVec), reflected)), hitInfo.material.shininess);

    return diffuse + specular;
}

const Ray computeReflectionRay(Ray ray, HitInfo hitInfo, const Features& features)
{
    // Do NOT use glm::reflect!! write your own code.
    Ray reflectionRay = ray;
    // TODO: implement the reflection ray computation.
    reflectionRay.direction = normalize(normalize(ray.direction * ray.t) - 2 * glm::dot(normalize(ray.direction * ray.t), hitInfo.normal) * hitInfo.normal);
    reflectionRay.origin = ray.origin + (ray.t - 0.0001f) * ray.direction;

    /*
    * GLOSSY REFLECTIONS
    */
    if (features.extra.enableGlossyReflection && hitInfo.material.ks != glm::vec3{0.0f}) {
        glm::vec3 cameraToPoint = ray.origin + ray.t * ray.direction;

        float a = 1 / (hitInfo.material.shininess);

        drawRay({ cameraToPoint, reflectionRay.direction, 0.1f }, { 0.0f, 0.0f, 1.0f });

        // 1. Find the orthonormal basis:

        glm::vec3 w_vec = glm::normalize(reflectionRay.direction);
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

        constexpr int NUM_OF_SAMPLES = 100;
        glm::vec3 direction {};

        for (int i = 0; i < NUM_OF_SAMPLES; ++i) {
            // 2. Create 2 random points in [0,1]:
            float ran1 = Generator::get(0.0f, 1.0f);
            float ran2 = Generator::get(0.0f, 1.0f);

            // 3. Calculate vector coefficients:
            float u = -a / 2 + ran1 * a;
            float v = -a / 2 + ran2 * a;

            // 4. Replace reflected with a perturbed reflected vector
            direction += reflectionRay.direction + u * u_vec + v * v_vec;
        }

        reflectionRay.direction = direction / glm::vec3 { NUM_OF_SAMPLES };

        drawRay({ cameraToPoint, reflectionRay.direction, 0.1f }, { 1.0f, 1.0f, 0.0f });
    }

    return reflectionRay;
}
