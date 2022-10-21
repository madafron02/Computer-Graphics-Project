#pragma once
#include "common.h"
#include <array>
#include <framework/ray.h>
#include <vector>

// Forward declaration.
struct Scene;

class BoundingVolumeHierarchy {
public:
    // Constructor. Receives the scene and builds the bounding volume hierarchy.
    BoundingVolumeHierarchy(Scene* pScene);

    // Return how many levels there are in the tree that you have constructed.
    [[nodiscard]] int numLevels() const;

    // Return how many leaf nodes there are in the tree that you have constructed.
    [[nodiscard]] int numLeaves() const;

    // Visual Debug 1: Draw the bounding boxes of the nodes at the selected level.
    void debugDrawLevel(int level);

    // Visual Debug 2: Draw the triangles of the i-th leaf
    void debugDrawLeaf(int leafIdx);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo, const Features& features) const;

private:
    Vertex computeCentroid(int mesh, glm::uvec3 triangle);
    std::vector<Vertex> getTriangleVertices(int mesh, glm::uvec3 triangle);

    struct Node {
        glm::vec3 min = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
        glm::vec3 max = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
        std::vector<std::tuple<int, int>> indexes;
        int divisionAxis = 0; // x = 0, y = 1, z = 2
        int level = 0;
        bool isLeaf = false;
        int leafNumber = -1;
    };

    std::vector<Node> createdNodes;
    int m_numLevels { 0 };
    int m_numLeaves { 0 };
    Scene* m_pScene;
};