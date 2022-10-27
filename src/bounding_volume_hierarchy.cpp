#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "interpolate.h"
#include "intersect.h"
#include "scene.h"
#include "texture.h"
#include <limits>
#include <queue>
#include <stack>
#include <glm/glm.hpp>
#include <iostream>

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
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

        std::vector<float> axisCoords;
        for (auto t : n.indexes) {
            int mesh_idx = std::get<0>(t);
            auto triangle = m_pScene->meshes.at(mesh_idx).triangles.at(std::get<1>(t));
            
            // TODO: potential optimization:
            // get all vertices before looping over them, so that calculating the same vertex
            // multiple times can be avoided
            auto vertices = getTriangleVertices(mesh_idx, triangle);
            for (auto v : vertices) {
                if (v.position.x < n.bounds.lower.x)
                    n.bounds.lower.x = v.position.x;
                if (v.position.x > n.bounds.upper.x)
                    n.bounds.upper.x = v.position.x;

                if (v.position.y < n.bounds.lower.y)
                    n.bounds.lower.y = v.position.y;
                if (v.position.y > n.bounds.upper.y)
                    n.bounds.upper.y = v.position.y;

                if (v.position.z < n.bounds.lower.z)
                    n.bounds.lower.z = v.position.z;
                if (v.position.z > n.bounds.upper.z)
                    n.bounds.upper.z = v.position.z;
            }
            
            // find division median
            Vertex coords = computeCentroid(std::get<0>(t), triangle);
            axisCoords.push_back(coords.position[n.divisionAxis]);
        }

        // if it's a leaf
        if (n.level >= desiredLevel || n.indexes.size() < 2) {
            // we quit calculations at this point
            n.isLeaf = true;
            n.leafNumber = m_numLeaves;
            ++m_numLeaves;
            continue;
        }

        // The node is not a leaf
        // find min max
        std::sort(axisCoords.begin(), axisCoords.end());
        float median = 0;
        {
            int len = axisCoords.size();
            if (len % 2 == 1)
                median = axisCoords[(int)(len / 2)];
            else
                median = axisCoords[(len / 2) - 1];
        }
        
        // if level not too big then we divide
        std::vector<std::tuple<int, int>> indexesLeft;
        std::vector<std::tuple<int, int>> indexesRight;
        for (auto t : n.indexes) {
            auto triangle = m_pScene->meshes.at(std::get<0>(t)).triangles.at(std::get<1>(t));
            Vertex coords = computeCentroid(std::get<0>(t), triangle);

            if (coords.position[n.divisionAxis] <= median)
                indexesLeft.push_back(t);
            else
                indexesRight.push_back(t);
        }

        // create new nodes
        int divisionAxis = (n.divisionAxis + 1) % 3;
        glm::vec3 n_min = { FLOAT_MAX, FLOAT_MAX, FLOAT_MAX };
        glm::vec3 n_max = { FLOAT_MIN, FLOAT_MIN, FLOAT_MIN };
        Node left = { n_min, n_max, indexesLeft, divisionAxis, n.level + 1 };
        Node right = { n_min, n_max, indexesRight, divisionAxis, n.level + 1 };

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
    typedef std::pair<float, Node> pair;
    struct Comparator
    {
        bool operator() (const pair &lhs, const pair &rhs) const
        {
            return lhs.first < rhs.first;
        }
    };

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
        bool hit = false;
        std::priority_queue<pair, std::vector<pair>, Comparator> intersections;

        // We start with the first AABB
        Node root = createdNodes.at(0);

        float initial_t = ray.t;
        bool check = intersectRayWithShape(root.bounds, ray);
        float t = ray.t;
        ray.t = initial_t;

        if(check) {
            if(t >= 0.0f) {
                intersections.push(std::make_pair( t, root));
            } else {
                intersections.push(std::make_pair( 0.0f, root));
            }

            while (!intersections.empty()) {
                pair current_pair = intersections.top();
                Node current = current_pair.second;
                intersections.pop();

                // We need to call functions on copies of the ray or otherwise the small ray.t will make the
                // intersect functions always fail.
                Ray initial_ray = ray;
                if (!intersectRayWithShape(current.bounds, initial_ray))
                    continue;

                // We know the current AABB is intersected by the ray: we need to check its child nodes or its triangles
                // (if it's a leaf):
                if (!current.isLeaf) {
                    int leaf1_idx = std::get<0>(current.indexes.at(0));
                    int leaf2_idx = std::get<1>(current.indexes.at(0));

                    if(leaf1_idx != -1) {
                        Node node1 = createdNodes.at(leaf1_idx);
                        initial_t = ray.t;
                        bool check1 = intersectRayWithShape(node1.bounds, ray);
                        float t1 = ray.t;
                        ray.t = initial_t;
                        if (check1) {
                            if (t1 >= 0.0f) {
                                intersections.push(std::make_pair(t1, node1));
                            } else {
                                intersections.push(std::make_pair(0.0f, node1));
                            }
                        }
                    }

                    if(leaf2_idx != -1) {
                        Node node2 = createdNodes.at(leaf2_idx);
                        initial_t = ray.t;
                        bool check2 = intersectRayWithShape(node2.bounds, ray);
                        float t2 = ray.t;
                        ray.t = initial_t;
                        if(check2) {
                            if(t2 >= 0.0f) {
                                intersections.push(std::make_pair(t2, node2));
                            } else {
                                intersections.push(std::make_pair(0.0f, node2));
                            }
                        }
                    }
                } else {
                    // This is a leaf: we check if any of the triangles is closer than any other
                    for (const auto& idx : current.indexes) {
                        const auto& mesh = m_pScene->meshes.at(std::get<0>(idx));
                        const auto& triangle = mesh.triangles.at(std::get<1>(idx));
                        const auto v0 = mesh.vertices[triangle[0]];
                        const auto v1 = mesh.vertices[triangle[1]];
                        const auto v2 = mesh.vertices[triangle[2]];

                        if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo)) {
                            // TODO: somewhere here we need to consider the `features.enableNormalInterp`
                            // and `features.enableTextureMapping` options
                            initial_t = ray.t;
                            hitInfo.material = mesh.material;
                            hitInfo.normal = normalize(glm::cross(v1.position - v0.position, v2.position - v0.position));
                            hit = true;
                        }
                    }
                }
            }
        }
        // TODO: repeat the above loop for the recursive ray-tracing

        // Intersect with spheres which is not supported by the BVH
        for (const auto& sphere : m_pScene->spheres)
            hit |= intersectRayWithShape(sphere, ray, hitInfo);
        return hit;
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
