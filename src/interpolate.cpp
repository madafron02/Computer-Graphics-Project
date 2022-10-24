#include "interpolate.h"
#include <glm/geometric.hpp>

glm::vec3 computeBarycentricCoord (const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& p)
{
    // TODO: implement this function.
    float area = glm::length(glm::cross(v0 - v1, v0 - v2)) / 2;
    float a = glm::length(glm::cross(v1 - p, v1 - v2)) / (2 * area);
    float b = glm::length(glm::cross(v2 - p, v2 - v0)) / (2 * area);
    float c = area - a - b;
    return glm::vec3 {a,b,c};
}

glm::vec3 interpolateNormal (const glm::vec3& n0, const glm::vec3& n1, const glm::vec3& n2, const glm::vec3 barycentricCoord)
{
    // TODO: implement this function.
    return n0 * barycentricCoord.x + n1 * barycentricCoord.y + n2 * barycentricCoord.z;
}

glm::vec2 interpolateTexCoord (const glm::vec2& t0, const glm::vec2& t1, const glm::vec2& t2, const glm::vec3 barycentricCoord)
{
// TODO: implement this function.
    return t0 * barycentricCoord.x + t1 * barycentricCoord.y + t2 * barycentricCoord.z;
}
