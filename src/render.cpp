#include "render.h"
#include "intersect.h"
#include "light.h"
#include "screen.h"
#include "texture.h"
#include <framework/trackball.h>
#ifdef NDEBUG
#include <omp.h>
#endif
#include <iostream>
#include <bounding_volume_hierarchy.h>

void drawShadowRays(Scene scene, Ray ray, BvhInterface bvh, HitInfo hitInfo, Features features)
{
    if (features.enableHardShadow) {
        for (const auto& light : scene.lights) {
            bvh.intersect(ray, hitInfo, features);
            glm::vec3 pointHit = ray.origin + ray.direction * (ray.t - 0.001f);

            if (std::holds_alternative<PointLight>(light)) {
                const PointLight pointLight = std::get<PointLight>(light);
                // Perform your calculations for a point light.

                glm::vec3 shadowRayDirection = glm::normalize(pointLight.position - pointHit);
                Ray shadowRay = { pointHit, shadowRayDirection, glm::length(pointLight.position - pointHit) };

                if (testVisibilityLightSample(pointLight.position, pointLight.color, bvh, features, ray, hitInfo) == 0.0) {
                    drawRay(shadowRay, glm::vec3 { 1.0f, 0.0f, 0.0f });
                } else {
                    drawRay(shadowRay, pointLight.color);
                }
            }
        }
    }
}

glm::vec3 getFinalColor(const Scene& scene, const BvhInterface& bvh, Ray ray, const Features& features, int rayDepth)
{
    HitInfo hitInfo;
    if(chosenRayDepth == rayDepth)
        debugNotVisited = true;
    else debugNotVisited = false;

    if (bvh.intersect(ray, hitInfo, features)) {

        glm::vec3 Lo = computeLightContribution(scene, bvh, features, ray, hitInfo);

         if (features.extra.enableTransparency) {
            drawRay(ray, Lo);
        } else if (features.enableShading) {
            drawRay(ray, Lo);
            drawShadowRays(scene, ray, bvh, hitInfo, features);
        } else {
            drawRay(ray, glm::vec3 { 1.0f });
            drawShadowRays(scene, ray, bvh, hitInfo, features);
        }

        if (features.enableTextureMapping && hitInfo.material.kdTexture && !features.enableShading) {
            return acquireTexel(*hitInfo.material.kdTexture, hitInfo.texCoord, features);
        }

        if (features.extra.enableTransparency && rayDepth < 10) {
            Ray helper = Ray { ray.origin + ray.direction * ray.t, ray.direction };
            helper.origin += helper.direction * std::numeric_limits<float>::epsilon();

            return hitInfo.material.transparency * Lo + (1 - hitInfo.material.transparency) * getFinalColor(scene, bvh, helper, features, rayDepth + 1);
        }

        if (features.enableRecursive && rayDepth < 6 && (hitInfo.material.ks != glm::vec3 { 0, 0, 0 })) {

            Ray reflected = computeReflectionRay(ray, hitInfo);
            reflected.origin += reflected.direction * std::numeric_limits<float>::epsilon();
            glm::vec3 reflectColor = getFinalColor(scene, bvh, reflected, features, rayDepth + 1);


            return Lo + reflectColor;
        }

        return Lo;
    } else {
        // Draw a red debug ray if the ray missed.
        drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
        // Set the color of the pixel to black if the ray misses.
        return glm::vec3(0.0f);
    }
}

void renderRayTracing(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features)
{
    glm::ivec2 windowResolution = screen.resolution();
    // Enable multi threading in Release mode
#ifdef NDEBUG
#pragma omp parallel for schedule(guided)
#endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            // NOTE: (-1, -1) at the bottom left of the screen, (+1, +1) at the top right of the screen.
            const glm::vec2 normalizedPixelPos {
                float(x) / float(windowResolution.x) * 2.0f - 1.0f,
                float(y) / float(windowResolution.y) * 2.0f - 1.0f
            };
            const Ray cameraRay = camera.generateRay(normalizedPixelPos);
            screen.setPixel(x, y, getFinalColor(scene, bvh, cameraRay, features));
        }
    }
}