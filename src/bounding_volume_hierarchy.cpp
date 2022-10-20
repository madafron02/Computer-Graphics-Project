#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "interpolate.h"
#include "intersect.h"
#include "scene.h"
#include "texture.h"
#include <queue>
#include <glm/glm.hpp>

struct Node {
    glm::vec3 min;
    glm::vec3 max;
    std::vector<std::tuple<int, int>> indexes;
    int divisionAxis = 0; // x = 0, y = 1, z = 2
    int level = -1;
    bool isLeaf;
};

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    int desiredLevel = 5;
    std::vector<Node> createdNodes;
    Node root;
    createdNodes.emplace_back(root);
    // distribute world triangles
    for (int i = 0; i < pScene->meshes.size(); ++i) {
        for (int j = 0; j < pScene->meshes.at(i).vertices.size(); ++j) {
            root.indexes.push_back(std::make_tuple(i, j));
        }
    }
    std::vector<std::tuple<int, int>> triangles;

    std::queue<Node> toDivide;
    toDivide.push(root);

    int divisionAxis = 0;
    while (!toDivide.empty()) {
        Node n = toDivide.front();
        toDivide.pop();

        if (n.level >= desiredLevel || n.indexes.size() < 2) {
            n.isLeaf = true;
            continue;
        }

        // The node is not a leaf
        std::vector<float> axisCoords;
        for (auto t : n.indexes) {
            auto coords = pScene->meshes.at(std::get<0>(t)).vertices.at(std::get<1>(t));
            if (coords.position.x < n.min.x)
                n.min.x = coords.position.x;
            else if (coords.position.x > n.max.x)
                n.max.x = coords.position.x;

            if (coords.position.y < n.min.y)
                n.min.y = coords.position.y;
            else if (coords.position.y > n.max.y)
                n.max.y = coords.position.y;

            if (coords.position.z < n.min.z)
                n.min.z = coords.position.z;
            else if (coords.position.z > n.max.z)
                n.max.z = coords.position.z;

            // find division median
            axisCoords.push_back(coords.position[n.divisionAxis*3]);
        }

        // find min max
        std::sort(axisCoords.begin(), axisCoords.end());
        float median = 0;
        if (axisCoords.size() % 2 == 1)
            median = axisCoords[(axisCoords.size() - 1) / 2];
        else
            median = axisCoords[axisCoords.size() / 2];

        // if level not too big then we divide
        std::vector<std::tuple<int, int>> indexesLeft;
        std::vector<std::tuple<int, int>> indexesRight;
        for (auto t : triangles) {
            auto triangle = pScene->meshes.at(std::get<0>(t)).vertices.at(std::get<1>(t));
            if (triangle.position[n.divisionAxis*3] <= median)
                indexesLeft.push_back(t);
            else
                indexesRight.push_back(t);
        }

        // create new nodes
        divisionAxis = (divisionAxis + 1) % 2;
        Node left = { glm::vec3 {}, glm::vec3 {}, indexesLeft, divisionAxis, n.level + 1 };
        Node right = { glm::vec3 {}, glm::vec3 {}, indexesRight, divisionAxis, n.level + 1 };

        // store them in the list and update current node's index
        toDivide.push(left);
        toDivide.push(right);
        n.indexes.clear();
        n.indexes.emplace_back(std::make_tuple(createdNodes.size(), createdNodes.size() + 1));
        createdNodes.emplace_back(left);
        createdNodes.emplace_back(right);
    }
}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 1.
int BoundingVolumeHierarchy::numLevels() const
{
    return 1;
}

// Return the number of leaf nodes in the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 2.
int BoundingVolumeHierarchy::numLeaves() const
{
    return 1;
}

// Use this function to visualize your BVH. This is useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDrawLevel(int level)
{
    // Draw the AABB as a transparent green box.
    //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);

    // Draw the AABB as a (white) wireframe box.
    AxisAlignedBox aabb { glm::vec3(0.0f), glm::vec3(0.0f, 1.05f, 1.05f) };
    //drawAABB(aabb, DrawMode::Wireframe);
    drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);
}

// Use this function to visualize your leaf nodes. This is useful for debugging. The function
// receives the leaf node to be draw (think of the ith leaf node). Draw the AABB of the leaf node and all contained triangles.
// You can draw the triangles with different colors. NoteL leafIdx is not the index in the node vector, it is the
// i-th leaf node in the vector.
void BoundingVolumeHierarchy::debugDrawLeaf(int leafIdx)
{
    // Draw the AABB as a transparent green box.
    //AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    //drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);

    // Draw the AABB as a (white) wireframe box.
    AxisAlignedBox aabb { glm::vec3(0.0f), glm::vec3(0.0f, 1.05f, 1.05f) };
    //drawAABB(aabb, DrawMode::Wireframe);
    drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);

    // once you find the leaf node, you can use the function drawTriangle (from draw.h) to draw the contained primitives
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h.
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo, const Features& features) const
{
    // If BVH is not enabled, use the naive implementation.
    if (!features.enableAccelStructure) {
        bool hit = false;
        // Intersect with all triangles of all meshes.
        for (const auto& mesh : m_pScene->meshes) {
            for (const auto& tri : mesh.triangles) {
                const auto v0 = mesh.vertices[tri[0]];
                const auto v1 = mesh.vertices[tri[1]];
                const auto v2 = mesh.vertices[tri[2]];
                if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo)) {
                    hitInfo.material = mesh.material;
                    hit = true;
                }
            }
        }
        // Intersect with spheres.
        for (const auto& sphere : m_pScene->spheres)
            hit |= intersectRayWithShape(sphere, ray, hitInfo);
        return hit;
    } else {
        // TODO: implement here the bounding volume hierarchy traversal.
        // Please note that you should use `features.enableNormalInterp` and `features.enableTextureMapping`
        // to isolate the code that is only needed for the normal interpolation and texture mapping features.
        return false;
    }
}