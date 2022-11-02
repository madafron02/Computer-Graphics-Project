#include "render.h"
#include "light.h"
#include "screen.h"
#include "texture.h"
#include <framework/trackball.h>
#ifdef NDEBUG
#include <omp.h>
#endif
#include <bounding_volume_hierarchy.h>
#include <random>

float aperture = 0;
float focalLength = 0.15f;

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
        debugIntersected = true;
    else
        debugIntersected = false;

    if (bvh.intersect(ray, hitInfo, features)) {
        glm::vec3 Lo = computeLightContribution(scene, bvh, features, ray, hitInfo);

        if(features.extra.enableDepthOfField && rayDepth < 1) {
            // for DOF visual debug, only call this method to draw the secondary rays
            getPixelColorDOF(ray, scene, bvh, features);
        }

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
            if (features.extra.enableTransparency && rayDepth < 10) {
                Ray helper = Ray { ray.origin + ray.direction * ray.t, ray.direction };
                helper.origin += helper.direction * std::numeric_limits<float>::epsilon();
                drawRay(ray, hitInfo.material.transparency* acquireTexel(*hitInfo.material.kdTexture, hitInfo.texCoord, features) + (1 - hitInfo.material.transparency) * getFinalColor(scene, bvh, helper, features, rayDepth + 1));


                return hitInfo.material.transparency * acquireTexel(*hitInfo.material.kdTexture, hitInfo.texCoord, features) + (1 - hitInfo.material.transparency) * getFinalColor(scene, bvh, helper, features, rayDepth + 1);
            }
            return acquireTexel(*hitInfo.material.kdTexture, hitInfo.texCoord, features);
        }

        if (features.extra.enableTransparency && rayDepth < 10) {
            Ray helper = Ray { ray.origin + ray.direction * ray.t, ray.direction };
            helper.origin += helper.direction * std::numeric_limits<float>::epsilon();
            drawRay(ray, hitInfo.material.transparency * Lo + (1 - hitInfo.material.transparency) * getFinalColor(scene, bvh, helper, features, rayDepth + 1));

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
        if (features.extra.enableEnvironmentMapping) {
            return getEnvironmentTexel(*scene.envMap, ray.direction);
        }
        // Draw a red debug ray if the ray missed.
        drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
        // Set the color of the pixel to black if the ray misses.
        return glm::vec3(0.0f);
    }
}

glm::vec3 getPixelColorDOF(Ray cameraRay, const Scene &scene, const BvhInterface &bvh, const Features &features) {
    /*
     * get focal point: P = camRay.o + camRay.dir * focalLength
     * move origin randomly according to aperture: camRay.o + rand displacement
     * generate 20 rays towards the focal point with direction D = P - (camRay.o + rand displacement)
     * average getFinalColor called on each of them
     *
     * visual debug: use drawSampleRay as a flag which
     */
    HitInfo hitInfo;
    glm::vec3 finalColor{0.0f};
    glm::vec3 focalPoint = cameraRay.origin + cameraRay.direction * focalLength;

    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<float> distribution(-0.5f, 0.5f);

    for(int i = 0; i < 20; i++) {
        float randomXCoord = distribution(generator) * aperture;
        float randomYCoord = distribution(generator) * aperture;
        float randomZCoord = distribution(generator) * aperture;

        glm::vec3 randomOffset = {randomXCoord, randomYCoord, randomZCoord};
        glm::vec3 originWithOffset = cameraRay.origin + randomOffset;

        glm::vec3 sampleVector = focalPoint - originWithOffset;
        Ray sampleSecondaryRay = {originWithOffset, normalize(sampleVector)};
        glm::vec3 sampleColor = getFinalColor(scene, bvh, sampleSecondaryRay, features, 1);

        if(enableDebugDraw) {
            bvh.intersect(sampleSecondaryRay, hitInfo, features);
            drawRay(sampleSecondaryRay, sampleColor);
        }

        finalColor += sampleColor;
    }

    return finalColor /= 20.0f;
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

            if(features.extra.enableMultipleRaysPerPixel) {
                glm::vec3 color{0.0f};
                // test with 4x4 grid in each pixel
                for(int i = 0; i < 15; i++) {
                    for(int j = 0; j < 15; j++) {
                        float r = ((double) rand() / RAND_MAX);
                        //std::cout<< r << "\n";
                        float sampleX = x + (float)(i + r) / 15.0f;
                        float sampleY = y + (float)(j + r) / 15.0f;

                        const glm::vec2 normalizedPixelPosSample {
                            float(sampleX) / float(windowResolution.x) * 2.0f - 1.0f,
                            float(sampleY) / float(windowResolution.y) * 2.0f - 1.0f
                        };

                        Ray sampleRay = camera.generateRay(normalizedPixelPosSample);
                        if(features.extra.enableDepthOfField) {
                            color = color + getPixelColorDOF(sampleRay, scene, bvh, features);
                        } else {
                            color = color + getFinalColor(scene, bvh, sampleRay, features);
                        }
                    }
                }
                screen.setPixel(x, y, color / 225.0f);
            } else if(features.extra.enableDepthOfField) {
                glm::vec3 averagedColor = getPixelColorDOF(cameraRay, scene, bvh, features);
                screen.setPixel(x, y, averagedColor);
            } else {
                screen.setPixel(x, y, getFinalColor(scene, bvh, cameraRay, features));
            }
        }
    }
}