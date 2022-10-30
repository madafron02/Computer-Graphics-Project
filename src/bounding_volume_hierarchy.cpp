#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "interpolate.h"
#include "intersect.h"
#include "scene.h"
#include "texture.h"
#include <chrono>
#include <limits>
#include <iostream>
#include <queue>
#include <glm/glm.hpp>

const std::vector<float> BoundingVolumeHierarchy::splitBins { 0.25, 0.5, 0.75 };

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene, const Features& features)
    : m_pScene(pScene)
{
    constexpr int MAX_LEVEL = 6;
    constexpr int MIN_TRIANGLES_IN_LEAF = 6;

    using clock = std::chrono::high_resolution_clock;
    const auto start = clock::now();
    
    Node root;
    // distribute world triangles
    for (int i = 0; i < m_pScene->meshes.size(); ++i) {
        for (int j = 0; j < m_pScene->meshes.at(i).triangles.size(); ++j) {
            root.indexes.push_back(std::make_tuple(i, j));
        }
    }

    std::queue<int> toDivide;
    createdNodes.emplace_back(root);
    toDivide.push(0);

    while (!toDivide.empty()) {
        int n_idx = toDivide.front();
        toDivide.pop();

        Node& n = createdNodes.at(n_idx);

        n.bounds = getAABBFromTriangles(n.indexes);
        if (n.level >= MAX_LEVEL || n.indexes.size() <= MIN_TRIANGLES_IN_LEAF) {
            // We make this node a leaf
            n.isLeaf = true;
            ++m_numLeaves;
            n.leafNumber = m_numLeaves;
            continue;
        }
         
        Node left;
        Node right;
        left.level = n.level + 1;
        right.level = n.level + 1;
        if (!features.extra.enableBvhSahBinning) {
            int divisionAxis = (n.divisionAxis + 1) % 3;
            left.divisionAxis = divisionAxis;
            right.divisionAxis = divisionAxis;

            getBestSplit(n, { n.divisionAxis }, { findTrianglesAxisMedian(n.indexes, n.divisionAxis) }, left, right);
        } else {
            std::vector<int> axises { 0, 1, 2 };
            getBestSplit(n, axises, calcAABBthresholds(n.bounds, axises, splitBins), left, right);
        }
       
        // Store them in the list and update current node's index
        int left_idx = -1;
        int right_idx = -1;

        if (left.indexes.size() > 0) {
            left_idx = createdNodes.size();
            toDivide.push(left_idx);
            createdNodes.emplace_back(left);
        }
        if (right.indexes.size() > 0) {
            right_idx = createdNodes.size();
            toDivide.push(right_idx);
            createdNodes.emplace_back(right);
        }
       
        // IMPORTANT: make sure n is still referencing the same node
        // to do that we create n_copy
        Node& n_copy = createdNodes.at(n_idx);
        n_copy.indexes.clear();
        n_copy.indexes.push_back(std::make_tuple(left_idx, right_idx));
    }

    const auto end = clock::now();
    std::cout << "Time to generate BVH: " << std::chrono::duration<float, std::milli>(end - start).count() << " milliseconds" << std::endl;
}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 1.
int BoundingVolumeHierarchy::numLevels() const
{
    return createdNodes.back().level + 1;
}

// Return the number of leaf nodes in the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 2.
int BoundingVolumeHierarchy::numLeaves() const
{
    return m_numLeaves;
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
    for (const auto& n : createdNodes) {
        if (n.level != level)
            continue;

        drawAABB(n.bounds, DrawMode::Wireframe);
    }
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
    //AxisAlignedBox aabb { glm::vec3(0.0f), glm::vec3(0.0f, 1.05f, 1.05f) };
    //drawAABB(aabb, DrawMode::Wireframe);
    //drawAABB(aabb, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);

    for (const auto& n : createdNodes) {
        if (n.isLeaf && n.leafNumber == leafIdx) {
            drawAABB(n.bounds, DrawMode::Wireframe);

            for (const std::tuple<int, int>& t : n.indexes) {
                int mesh_idx = std::get<0>(t);

                const auto& triangle = m_pScene->meshes.at(mesh_idx).triangles.at(std::get<1>(t));
                auto vertices = getTriangleVertices(mesh_idx, triangle);
                drawTriangle(vertices.at(0), vertices.at(1), vertices.at(2));
            }
            break;
        }
    }
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
                    hitInfo.normal = normalize(glm::cross(v1.position - v0.position, v2.position - v0.position));
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

Vertex BoundingVolumeHierarchy::computeCentroid(int mesh, glm::uvec3 triangle)
{
    Vertex A = m_pScene->meshes.at(mesh).vertices.at(triangle.x);
    Vertex B = m_pScene->meshes.at(mesh).vertices.at(triangle.y);
    Vertex C = m_pScene->meshes.at(mesh).vertices.at(triangle.z);

    glm::vec3 position { (A.position.x + B.position.x + C.position.x) / 3.0f,
                         (A.position.y + B.position.y + C.position.y) / 3.0f,
                         (A.position.z + B.position.z + C.position.z) / 3.0f };
    return { position, glm::vec3 {}, glm::vec2 {} };
}

std::vector<Vertex> BoundingVolumeHierarchy::getTriangleVertices(int mesh, glm::uvec3 triangle)
{
    Vertex A = m_pScene->meshes.at(mesh).vertices.at(triangle.x);
    Vertex B = m_pScene->meshes.at(mesh).vertices.at(triangle.y);
    Vertex C = m_pScene->meshes.at(mesh).vertices.at(triangle.z);

    std::vector<Vertex> answer;
    answer.emplace_back(A);
    answer.emplace_back(B);
    answer.emplace_back(C);

    return answer;
}

AxisAlignedBox BoundingVolumeHierarchy::getAABBFromTriangles(const IndexTuple& indexes)
{
    AxisAlignedBox answer;
    answer.lower = VEC_OF_MAXS;
    answer.upper = VEC_OF_MINS;
    
    for (const auto& t : indexes) {
        int mesh_idx = std::get<0>(t);
        const auto& triangle = m_pScene->meshes.at(mesh_idx).triangles.at(std::get<1>(t));

        // TODO: potential optimization:
        // get all vertices before looping over them, so that calculating the same vertex
        // multiple times can be avoided
        auto vertices = getTriangleVertices(mesh_idx, triangle);
        for (const auto& v : vertices) {
            if (v.position.x < answer.lower.x)
                answer.lower.x = v.position.x;
            if (v.position.x > answer.upper.x)
                answer.upper.x = v.position.x;

            if (v.position.y < answer.lower.y)
                answer.lower.y = v.position.y;
            if (v.position.y > answer.upper.y)
                answer.upper.y = v.position.y;

            if (v.position.z < answer.lower.z)
                answer.lower.z = v.position.z;
            if (v.position.z > answer.upper.z)
                answer.upper.z = v.position.z;
        }
    }

    return answer;
}

float BoundingVolumeHierarchy::findTrianglesAxisMedian(const IndexTuple& indexes, int axis)
{
    if (indexes.size() < 1)
        return 0.0f;

    std::vector<float> coords;

    for (const auto& t : indexes) {
        int mesh_idx = std::get<0>(t);
        const auto& triangle = m_pScene->meshes.at(mesh_idx).triangles.at(std::get<1>(t));

        Vertex centroid = computeCentroid(std::get<0>(t), triangle);
        coords.push_back(centroid.position[axis]);
    }

    if (coords.size() == 1)
        return coords.at(0);

    std::sort(coords.begin(), coords.end());
    float median = 0;
    {
        int len = coords.size();
        if (len % 2 == 1)
            median = coords[(int)(len / 2)];
        else
            median = coords[(len / 2) - 1];
    }

    return median;
}

void BoundingVolumeHierarchy::splitTrianglesByAxisAndThreshold(const IndexTuple& indexes, int axis, float threshold, IndexTuple& left, IndexTuple& right)
{
    for (const auto& t : indexes) {
        const auto& triangle = m_pScene->meshes.at(std::get<0>(t)).triangles.at(std::get<1>(t));
        Vertex coords = computeCentroid(std::get<0>(t), triangle);

        if (coords.position[axis] <= threshold)
            left.push_back(t);
        else
            right.push_back(t);
    }
}

void BoundingVolumeHierarchy::getBestSplit(const Node& parent, const std::vector<int>& axises, std::vector<float> thresholds, Node& left, Node& right)
{
    float bestCost = FLOAT_MAX;
    float parentVolume = calcAABBvolume(parent.bounds);
    int thresholds_per_axis = thresholds.size() / axises.size();
    for (int i = 0; i < axises.size(); ++i) {
        for (int j = 0; j < thresholds_per_axis; ++j) {
            IndexTuple indexesLeft;
            IndexTuple indexesRight;

            // thresholds.at(i * thresholds_per_axis + j) retrieves a threshold from
            // a two-dimensional array, where we store n thresholds for each axis
            splitTrianglesByAxisAndThreshold(parent.indexes, axises.at(i), thresholds.at(i * thresholds_per_axis + j), 
                indexesLeft, indexesRight);

            float cost_left = calcSplitCost(indexesLeft);
            float cost_right = calcSplitCost(indexesRight);

            // We want to penalize a division, where we keep all triangles in one child node
            if (indexesLeft.size() == 0 || indexesRight.size() == 0)
                cost_left += FLOAT_MAX / 2 - 1;

            if (cost_left + cost_right < bestCost) {
                bestCost = cost_left + cost_right;
                // Save best setting
                left.indexes = indexesLeft;
                right.indexes = indexesRight;
            }
        }
    }
}

float BoundingVolumeHierarchy::calcSplitCost(const IndexTuple& indexes)
{
    float cost = 0;

    if (indexes.size() > 0) {
        float leftVolume = calcAABBvolume(getAABBFromTriangles(indexes));

        cost = leftVolume * indexes.size();
    }
    return cost;
}

float BoundingVolumeHierarchy::calcAABBvolume(const AxisAlignedBox& a)
{
    float answer = (a.upper.x - a.lower.x) * (a.upper.y - a.lower.y) * (a.upper.z - a.lower.z);
    if (answer < 0)
        answer *= -1;
    return answer;
}

std::vector<float> BoundingVolumeHierarchy::calcAABBthresholds(const AxisAlignedBox& aabb, const std::vector<int>& axises, const std::vector<float>& thresholds)
{
    /*
        This function produces a two-dimensional array of n thresholds (specified in thresholds vector) 
        in AABB coordinates for each axis from the axises vector.
    */
    std::vector<float> answer;
    for (int axis : axises) {
        // For each axis:
        float len = aabb.upper[axis] - aabb.lower[axis];
        float start = aabb.lower[axis];

        for (float threshold : thresholds) {
            answer.emplace_back(threshold * len + start);
        }
    }
    return answer;
}
