#include "interpolate.h"
#include <glm/geometric.hpp>

glm::vec3 computeBarycentricCoord (const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& p)
{
    // TODO: implement this function.
    //float area = glm::length(glm::cross(v0 - v1, v0 - v2)) / 2;
    //float a = glm::length(glm::cross(v1 - p, v1 - v2)) / (2 * area);
    //float b = glm::length(glm::cross(v2 - p, v2 - v0)) / (2 * area);
    //float c = area - a - b;
    //return glm::vec3 {a,b,c};

    // Compute the edges of the triangles. Edge 0 - triangle edge starting at vertex 0.
    glm::vec3 edge0 = v1 - v0;
    glm::vec3 edge1 = v2 - v1;
    glm::vec3 edge2 = v0 - v2;

    // Compute vectors between vertices and the point. p0 - vector between v0 and p.
    glm::vec3 p0 = p - v0;
    glm::vec3 p1 = p - v1;
    glm::vec3 p2 = p - v2;

    // Baricentric coordinate of a vertice is equal to  ration of half of the area of parallelogram between edge and point-vertex vector to the total area.
    float totalArea = glm::length(abs(glm::cross(edge0, edge1))) / 2;

    float coor0 = glm::length(abs(glm::cross(edge1, p1))) / 2;
    coor0 = coor0 / totalArea;

    float coor1 = glm::length(abs(glm::cross(edge2, p2))) / 2;
    coor1 = coor1 / totalArea;

    float coor2 = glm::length(abs(glm::cross(edge0, p0))) / 2;
    coor2 = coor2 / totalArea;

    // Result
    glm::vec3 baricentric = glm::vec3(coor0, coor1, coor2);



    return baricentric;
}

glm::vec3 interpolateNormal (const glm::vec3& n0, const glm::vec3& n1, const glm::vec3& n2, const glm::vec3 barycentricCoord)
{
    // TODO: implement this function.
    return n0 * barycentricCoord.x + n1 * barycentricCoord.y + n2 * barycentricCoord.z;
}

glm::vec2 interpolateTexCoord (const glm::vec2& t0, const glm::vec2& t1, const glm::vec2& t2, const glm::vec3 barycentricCoord)
{
// TODO: implement this function.
    //Pami¹tka po janku :))))))) 
    return t0 * barycentricCoord.x + t1 * barycentricCoord.y + t2 * barycentricCoord.z;
}
