#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "interpolate.h"
#include "intersect.h"
#include "scene.h"
#include "texture.h"
#include <limits>
#include <queue>
#include <glm/glm.hpp>



BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene, const Features& features)
    : m_pScene(pScene)
    , splitBins {0.25, 0.5, 0.75}
{
    int desiredLevel = 10;
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
        // if it's a leaf
        if (n.level >= desiredLevel || n.indexes.size() < 2) {
            // we quit calculations at this point
            n.isLeaf = true;
            n.leafNumber = m_numLeaves;
            ++m_numLeaves;
            continue;
        }

        // The node is not a leaf
        float median = findTrianglesAxisMedian(n.indexes, n.divisionAxis);
        
        // if level not too big then we divide
        IndexTuple indexesLeft;
        IndexTuple indexesRight;
        splitTrianglesByAxisAndThreshold(n.indexes, n.divisionAxis, median, indexesLeft, indexesRight);

        // create new nodes
        int divisionAxis = (n.divisionAxis + 1) % 3;
        Node left = { VEC_OF_MAXS, VEC_OF_MINS, indexesLeft, divisionAxis, n.level + 1 };
        Node right = { VEC_OF_MAXS, VEC_OF_MINS, indexesRight, divisionAxis, n.level + 1 };

        // store them in the list and update current node's index
        int left_idx = -1;
        int right_idx = -1;
        if (indexesLeft.size() > 0) {
            left_idx = createdNodes.size();
            toDivide.push(left_idx);
            createdNodes.emplace_back(left);
        }
        if (indexesRight.size() > 0) {
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

    for (auto n : createdNodes) {
        if (n.isLeaf && n.leafNumber == leafIdx) {
            drawAABB(n.bounds, DrawMode::Wireframe);

            for (const std::tuple<int, int>& t : n.indexes) {
                int mesh_idx = std::get<0>(t);

                auto triangle = m_pScene->meshes.at(mesh_idx).triangles.at(std::get<1>(t));
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

    for (auto t : indexes) {
        int mesh_idx = std::get<0>(t);
        auto triangle = m_pScene->meshes.at(mesh_idx).triangles.at(std::get<1>(t));

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
    for (auto t : indexes) {
        auto triangle = m_pScene->meshes.at(std::get<0>(t)).triangles.at(std::get<1>(t));
        Vertex coords = computeCentroid(std::get<0>(t), triangle);

        if (coords.position[axis] <= threshold)
            left.push_back(t);
        else
            right.push_back(t);
    }
}

void BoundingVolumeHierarchy::getBestSplit(const Node& parent, std::vector<int> axises, std::vector<float> thresholds, Node& left, Node& right)
{
    float bestCost = FLOAT_MAX;
    for (auto axis : axises) {
        for (auto threshold : thresholds) {
            IndexTuple indexesLeft;
            IndexTuple indexesRight;
            splitTrianglesByAxisAndThreshold(parent.indexes, axis, threshold, indexesLeft, indexesRight);

            float parentVolume = calcAABBvolume(parent.bounds);

            float cost_left = 0;
            AxisAlignedBox AABB_left;
            if (indexesLeft.size() > 0) {
                AABB_left = getAABBFromTriangles(indexesLeft);
                float leftVolume = calcAABBvolume(AABB_left);

                cost_left = (leftVolume / parentVolume) * indexesLeft.size();
            }

            float cost_right = 0;
            AxisAlignedBox AABB_right;
            if (indexesRight.size() > 0) {
                AABB_right = getAABBFromTriangles(indexesRight);
                float rightVolume = calcAABBvolume(AABB_right);

                cost_right = (rightVolume / parentVolume) * indexesRight.size();
            }
           
            if (cost_left + cost_right < bestCost) {
                bestCost = cost_left + cost_right;
                // save best setting
            }
        }
    }
}

float BoundingVolumeHierarchy::calcAABBvolume(const AxisAlignedBox& a)
{
    float answer = (a.upper.x - a.lower.x) * (a.upper.y - a.lower.y) * (a.upper.z - a.lower.z);
    if (answer < 0)
        answer *= -1;
    return answer;
}
