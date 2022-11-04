#pragma once
#include "common.h"
#include <framework/ray.h>

extern int SAMPLES_NUMBER;

// Compute the shading at the intersection point using the Phong model.
const glm::vec3 computeShading (const glm::vec3& lightPosition, const glm::vec3& lightColor, const Features& features, Ray ray, HitInfo hitInfo);

// Given a ray and a normal (in hitInfo), compute the reflected ray in the specular direction (mirror direction).
const Ray computeReflectionRay(Ray ray, HitInfo hitInfo, const Features& features);

glm::vec3 distortRayDirection(const Ray& ray, float shininess, int samples);