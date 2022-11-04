#include "interpolate.h"
#include <glm/geometric.hpp>


//Method taken from slides lecture 6
glm::vec3 computeBarycentricCoord(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& p)
{
    //Compute total area of the triangle and three smaller ones inside it
    float totalArea = glm::length(abs(glm::cross(v1 - v0, v2 - v1))) / 2;

    float a = glm::length(abs(glm::cross(v2 - v1, p - v1))) / 2 / totalArea;
    float b = glm::length(abs(glm::cross(v0 - v2, p - v2))) / 2 / totalArea;
    float c = glm::length(abs(glm::cross(v1 - v0, p - v0))) / 2 / totalArea;

    return glm::vec3(a, b, c);
}

glm::vec3 interpolateNormal(const glm::vec3& n0, const glm::vec3& n1, const glm::vec3& n2, const glm::vec3 barycentricCoord)
{
    return n0 * barycentricCoord.x + n1 * barycentricCoord.y + n2 * barycentricCoord.z;
}

glm::vec2 interpolateTexCoord(const glm::vec2& t0, const glm::vec2& t1, const glm::vec2& t2, const glm::vec3 barycentricCoord)
{
    return t0 * barycentricCoord.x + t1 * barycentricCoord.y + t2 * barycentricCoord.z;
}
