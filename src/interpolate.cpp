#include "interpolate.h"
#include <glm/geometric.hpp>

glm::vec3 computeBarycentricCoord(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& p)
{
    glm::vec3 edge0 = v1 - v0;
    glm::vec3 edge1 = v2 - v1;
    glm::vec3 edge2 = v0 - v2;

    glm::vec3 p0 = p - v0;
    glm::vec3 p1 = p - v1;
    glm::vec3 p2 = p - v2;

    float totalArea = glm::length(abs(glm::cross(v1 - v0, v2 - v1))) / 2;

    float a = glm::length(abs(glm::cross(v2 - v1, p - v1))) / 2 / totalArea;

    float b = glm::length(abs(glm::cross(v0 - v2, p - v2))) / 2 / totalArea;

    float c = glm::length(abs(glm::cross(v1 - v0, p - v0))) / 2 / totalArea;

    return glm::vec3(a, b, c);
}

glm::vec3 interpolateNormal(const glm::vec3& n0, const glm::vec3& n1, const glm::vec3& n2, const glm::vec3 barycentricCoord)
{
    // TODO: implement this function.
    return n0 * barycentricCoord.x + n1 * barycentricCoord.y + n2 * barycentricCoord.z;
}

glm::vec2 interpolateTexCoord(const glm::vec2& t0, const glm::vec2& t1, const glm::vec2& t2, const glm::vec3 barycentricCoord)
{
    // TODO: implement this function.
    // Pami¹tka po janku :)))))))
    return t0 * barycentricCoord.x + t1 * barycentricCoord.y + t2 * barycentricCoord.z;
}
