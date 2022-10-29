#include "light.h"
#include "config.h"
#include "draw.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>


// samples a segment light source
// you should fill in the vectors position and color with the sampled position and color
void sampleSegmentLight(const SegmentLight& segmentLight, glm::vec3& position, glm::vec3& color)
{
    glm::vec3 segDir = normalize(segmentLight.endpoint1 - segmentLight.endpoint0);
    float segLen = length(segmentLight.endpoint1 - segmentLight.endpoint0);

    float t = (float)rand() / (RAND_MAX / segLen);
    position = segmentLight.endpoint0 + t * segDir;

    float leftToPoint = length(position - segmentLight.endpoint0);
    float leftToRight = length(segmentLight.endpoint1 - segmentLight.endpoint0);

    float ratioOfLeft = leftToPoint / leftToRight;
    float ratioOfRight = 1 - ratioOfLeft;

    color = segmentLight.color0 * ratioOfRight + segmentLight.color1 * ratioOfLeft;
}

// samples a parallelogram light source
// you should fill in the vectors position and color with the sampled position and color
void sampleParallelogramLight(const ParallelogramLight& parallelogramLight, glm::vec3& position, glm::vec3& color)
{
    glm::vec3 segDir1 = normalize(parallelogramLight.edge01);
    glm::vec3 segDir2 = normalize(parallelogramLight.edge02);
    float segLen1 = length(parallelogramLight.edge01);
    float segLen2 = length(parallelogramLight.edge02);

    float t1 = rand() / (RAND_MAX / segLen1);
    float t2 = rand() / (RAND_MAX / segLen2);
    //std::cout << t1 << " "  << t2 << "\n";

    position = parallelogramLight.v0 + t1 * segDir1 + t2 * segDir2;

    float alpha = length(t2 * segDir2);
    float beta = length(t1 * segDir1);

    float area = segLen1 * segLen2;

    color = parallelogramLight.color3 * (alpha * beta / area)
        + parallelogramLight.color2 * (alpha * (segLen1 - beta) / area)
        + parallelogramLight.color1 * ((segLen2 - alpha) * beta / area)
        + parallelogramLight.color0 * ((segLen2 - alpha) * (segLen1 - beta) / area);

    //std::cout << color.x + color.y + color.z << "\n";
}

// test the visibility at a given light sample
// returns 1.0 if sample is visible, 0.0 otherwise
float testVisibilityLightSample(const glm::vec3& samplePos, const glm::vec3& debugColor, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo)
{
    // TODO: implement this function.
    glm::vec3 pointHit = ray.origin + ray.direction * (ray.t - 0.001f);
    glm::vec3 pointToLight = glm::normalize(samplePos - pointHit);

    float expectedT = glm::length(samplePos - pointHit);
    Ray toLight = {pointHit, pointToLight, expectedT};
    bvh.intersect(toLight, hitInfo, features);

    if(abs(expectedT - toLight.t) > 0.001) return 0.0;
    return 1.0;
}

// given an intersection, computes the contribution from all light sources at the intersection point
// in this method you should cycle the light sources and for each one compute their contribution
// don't forget to check for visibility (shadows!)

// Lights are stored in a single array (scene.lights) where each item can be either a PointLight, SegmentLight or ParallelogramLight.
// You can check whether a light at index i is a PointLight using std::holds_alternative:
// std::holds_alternative<PointLight>(scene.lights[i])
//
// If it is indeed a point light, you can "convert" it to the correct type using std::get:
// PointLight pointLight = std::get<PointLight>(scene.lights[i]);
//
//
// The code to iterate over the lights thus looks like this:
// for (const auto& light : scene.lights) {
//     if (std::holds_alternative<PointLight>(light)) {
//         const PointLight pointLight = std::get<PointLight>(light);
//         // Perform your calculations for a point light.
//     } else if (std::holds_alternative<SegmentLight>(light)) {
//         const SegmentLight segmentLight = std::get<SegmentLight>(light);
//         // Perform your calculations for a segment light.
//     } else if (std::holds_alternative<ParallelogramLight>(light)) {
//         const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
//         // Perform your calculations for a parallelogram light.
//     }
// }
//
// Regarding the soft shadows for **other** light sources **extra** feature:
// To add a new light source, define your new light struct in scene.h and modify the Scene struct (also in scene.h)
// by adding your new custom light type to the lights std::variant. For example:
// std::vector<std::variant<PointLight, SegmentLight, ParallelogramLight, MyCustomLightType>> lights;
//
// You can add the light sources programmatically by creating a custom scene (modify the Custom case in the
// loadScene function in scene.cpp). Custom lights will not be visible in rasterization view.
glm::vec3 computeLightContribution(const Scene& scene, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo)
{
    glm::vec3 lightSum{0.0f};
    bvh.intersect(ray, hitInfo, features);
    glm::vec3 pointHit = ray.origin + (ray.t - 0.001f) * ray.direction;


    if (features.enableShading || features.extra.enableTransparency) {
        // If shading is enabled, compute the contribution from all lights.
        for (const auto& light : scene.lights) {
            bvh.intersect(ray, hitInfo, features);

            if (std::holds_alternative<PointLight>(light)) {
                const PointLight pointLight = std::get<PointLight>(light);

                glm::vec3 color = computeShading(pointLight.position, pointLight.color, features, ray, hitInfo);
                if(features.enableHardShadow && testVisibilityLightSample(pointLight.position, color, bvh, features, ray, hitInfo) == 0.0) {
                    lightSum += glm::vec3{0.0f};
                } else {
                    lightSum += color;
                }

            } else if (std::holds_alternative<SegmentLight>(light)) {
                const SegmentLight segmentLight = std::get<SegmentLight>(light);
                glm::vec3 result{0.0f};

                if(features.enableSoftShadow) {
                    for(int i = 1; i <= 100; i++) {
                        glm::vec3 position;
                        glm::vec3 color;
                        sampleSegmentLight(segmentLight, position, color);

                        Ray softDebug = {pointHit, normalize(position - pointHit), length(position - pointHit)};
                        if(testVisibilityLightSample(position, color, bvh, features, ray, hitInfo) != 0.0) {
                            drawRay(softDebug, color);
                            result += computeShading(position, color, features, ray, hitInfo);
                        } else {
                            Ray prev = softDebug; HitInfo hi = hitInfo;
                            bvh.intersect(softDebug, hitInfo, features);
                            drawRay(softDebug, glm::vec3{1.0f, 0.0f, 0.0f});
                            softDebug = prev; hitInfo = hi;
                        }
                    }
                    lightSum += result / 100.0f;
                } else {
                    lightSum += hitInfo.material.kd;
                }
            } else if (std::holds_alternative<ParallelogramLight>(light)) {
                const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
                glm::vec3 result{0.0f};

                if(features.enableSoftShadow) {
                    for(int i = 1; i <= 100; i++) {
                        glm::vec3 position;
                        glm::vec3 color;
                        sampleParallelogramLight(parallelogramLight, position, color);

                        Ray softDebug = {pointHit, normalize(position - pointHit), length(position - pointHit)};
                        if(testVisibilityLightSample(position, color, bvh, features, ray, hitInfo) != 0.0) {
                            drawRay(softDebug, color);
                            result += computeShading(position, color, features, ray, hitInfo);
                        } else {
                            Ray prev = softDebug; HitInfo hi = hitInfo;
                            bvh.intersect(softDebug, hitInfo, features);
                            drawRay(softDebug, glm::vec3{1.0f, 0.0f, 0.0f});
                            softDebug = prev; hitInfo = hi;
                        }
                    }
                    lightSum += result / 100.0f;
                } else {
                    lightSum += hitInfo.material.kd;
                }
            }
        }
    } else {
        for (const auto& light : scene.lights) {
            bvh.intersect(ray, hitInfo, features);

        if (std::holds_alternative<PointLight>(light)) {
                const PointLight pointLight = std::get<PointLight>(light);

                if (features.enableHardShadow && testVisibilityLightSample(pointLight.position, pointLight.color, bvh, features, ray, hitInfo) == 0.0) {
                    lightSum += glm::vec3 { 0.0f };
                } else {
                    lightSum += hitInfo.material.kd;
                }

            } else if (std::holds_alternative<SegmentLight>(light)) {
                const SegmentLight segmentLight = std::get<SegmentLight>(light);
                glm::vec3 result{0.0f};

                if(features.enableSoftShadow) {
                    for(int i = 1; i <= 100; i++) {
                        glm::vec3 position;
                        glm::vec3 color;
                        sampleSegmentLight(segmentLight, position, color);

                        Ray softDebug = {pointHit, normalize(position - pointHit), length(position - pointHit)};
                        if(testVisibilityLightSample(position, color, bvh, features, ray, hitInfo) != 0.0) {
                            drawRay(softDebug, color);
                            result += hitInfo.material.kd;
                        } else {
                            Ray prev = softDebug; HitInfo hi = hitInfo;
                            bvh.intersect(softDebug, hitInfo, features);
                            drawRay(softDebug, glm::vec3{1.0f, 0.0f, 0.0f});
                            softDebug = prev; hitInfo = hi;
                        }
                    }
                    lightSum += result / 100.0f;
                } else {
                    lightSum += hitInfo.material.kd;
                }

            } else if (std::holds_alternative<ParallelogramLight>(light)) {
                const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
                glm::vec3 result{0.0f};

                if(features.enableSoftShadow) {
                    for(int i = 1; i <= 100; i++) {
                        glm::vec3 position;
                        glm::vec3 color;
                        sampleParallelogramLight(parallelogramLight, position, color);

                        Ray softDebug = {pointHit, normalize(position - pointHit), length(position - pointHit)};
                        if(testVisibilityLightSample(position, color, bvh, features, ray, hitInfo) != 0.0) {
                            drawRay(softDebug, color);
                            result += hitInfo.material.kd;
                        } else {
                            Ray prev = softDebug; HitInfo hi = hitInfo;
                            bvh.intersect(softDebug, hitInfo, features);
                            drawRay(softDebug, glm::vec3{1.0f, 0.0f, 0.0f});
                            softDebug = prev; hitInfo = hi;
                        }
                    }
                    lightSum += result / 100.0f;
                } else {
                    lightSum += hitInfo.material.kd;
                }
            }
        }
    }

    return  lightSum;
}
