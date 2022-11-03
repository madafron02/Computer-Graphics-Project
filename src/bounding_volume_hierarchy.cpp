#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "interpolate.h"
#include "intersect.h"
#include "scene.h"
#include "texture.h"
#include <chrono>
#include <glm/glm.hpp>
#include <iostream>
#include <limits>
#include <queue>
#include <stack>

const std::vector<float> BoundingVolumeHierarchy::splitBins { 0.25, 0.5, 0.75 };
bool debugIntersected = false;
int chosenRayDepth = -1;

BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene, const Features& features)
    : m_pScene(pScene)
{
    constexpr int MAX_LEVEL = 6;
    constexpr int MIN_TRIANGLES_IN_LEAF = 6;

    //using clock = std::chrono::high_resolution_clock;
    //const auto start = clock::now();

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

    //const auto end = clock::now();
    //std::cout << "Time to generate BVH: " << std::chrono::duration<float, std::milli>(end - start).count() << " milliseconds" << std::endl;
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

        /*
            Visual debug: draw a 'threshold' plane for nodes that are not leaves:
        */
        if (n.isLeaf)
            continue;

        if (n.divisionAxis < 0 || n.divisionAxis > 2) {
            std::cout << "[!!!] Incorrect division axis in debugDrawLevel(): " << n.divisionAxis << '\n';
            continue;
        }

        glm::vec3 planeColor = { 0.0, 1.0, 0.0 };
        AxisAlignedBox copy = n.bounds;
        copy.lower[n.divisionAxis] = n.divisionThreshold;
        copy.upper[n.divisionAxis] = n.divisionThreshold;

        drawAABB(copy, DrawMode::Filled, { 0.0f, 1.0f, 0.0f }, 0.2);
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

void BoundingVolumeHierarchy::debugDrawNotVisited(std::vector<AxisAlignedBox> notVisited) const
{
    // Draw the AABB as an orange wireframe box.
    for (auto n : notVisited) {
        drawAABB(n, DrawMode::Wireframe, glm::vec3 { 0.67f, 0.33f, 0.0f });
    }
}

void BoundingVolumeHierarchy::debugDrawAllIntersected(std::vector<AxisAlignedBox> allIntersected, std::vector<AxisAlignedBox> notVisited) const
{
    // Draw the AABB as a blue wireframe box.
    for (auto n : allIntersected) {
        if (!checkContainsAABB(notVisited, n))
            drawAABB(n, DrawMode::Wireframe, glm::vec3 { 0.0f, 0.5f, 0.5f });
    }
}

bool BoundingVolumeHierarchy::checkContainsAABB(std::vector<AxisAlignedBox> v, AxisAlignedBox b) const
{
    for (AxisAlignedBox a : v) {
        if (a.upper == b.upper && a.lower == b.lower)
            return true;
    }
    return false;
}

bool BoundingVolumeHierarchy::checkRayOriginInsideAABB(AxisAlignedBox aabb, Ray ray) const
{
    glm::vec3 origin = ray.origin;
    if (origin.x >= aabb.lower.x && origin.y >= aabb.lower.y && origin.z >= aabb.lower.z
        && origin.x <= aabb.upper.x && origin.y <= aabb.upper.y && origin.z <= aabb.upper.z)
        return true;
    return false;
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h.
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo, const Features& features) const
{
    typedef std::pair<float, Node> pair;
    struct Comparator {
        bool operator()(const pair& lhs, const pair& rhs) const
        {
            return lhs.first < rhs.first;
        }
    };

    Vertex vf0, vf1, vf2;

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

                    hitInfo.barycentricCoord = computeBarycentricCoord(v0.position, v1.position, v2.position, ray.origin + ray.t * ray.direction);

                    hitInfo.material = mesh.material;
                    hitInfo.normal = normalize(glm::cross(v1.position - v0.position, v2.position - v0.position));
                    hit = true;

                    if (features.enableTextureMapping) {
                        hitInfo.texCoord = interpolateTexCoord(v0.texCoord, v1.texCoord, v2.texCoord, hitInfo.barycentricCoord);
                    }

                    if (features.enableNormalInterp) {
                        glm::vec3 interpolatNormal = interpolateNormal(v0.normal, v1.normal, v2.normal, hitInfo.barycentricCoord);
                        Ray normal0 = { v0.position,
                            normalize(v0.normal),
                            1 };

                        Ray normal1 = { v1.position,
                            normalize(v1.normal),
                            1 };

                        Ray normal2 = { v2.position,
                            normalize(v2.normal),
                            1 };

                        Ray interpolated = { ray.origin + ray.t * ray.direction,
                            normalize(interpolatNormal),
                            1 };

                        drawRay(normal0, { 0, 1, 0 });
                        drawRay(normal1, { 0, 1, 0 });
                        drawRay(normal2, { 0, 1, 0 });
                        drawRay(interpolated, { 1, 0, 0 });
                    }
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

        // DEBUG DETAILS:
        // -> all intersected AABB's in blue
        // -> all intersected but not visited AABB's in orange
        // -> final triangle in green

        bool hit = false;
        float last_primitive_t = MAXFLOAT;
        std::priority_queue<pair, std::vector<pair>, Comparator> intersections;
        std::vector<AxisAlignedBox> notVisited;
        std::vector<AxisAlignedBox> allIntersected;

        // We start with the first AABB
        Node root = createdNodes.at(0);

        Ray ray_copy = ray;
        bool check = intersectRayWithShape(root.bounds, ray_copy);
        if (checkRayOriginInsideAABB(root.bounds, ray_copy))
            check = true;

        if (check) {
            allIntersected.emplace_back(root.bounds);
            if (ray_copy.t >= 0.0f) {
                intersections.push(std::make_pair(ray_copy.t, root));
            } else {
                intersections.push(std::make_pair(0.0f, root));
            }

            while (!intersections.empty()) {
                pair current_pair = intersections.top();
                Node current = current_pair.second;
                intersections.pop();

                ray_copy = ray;
                if (!intersectRayWithShape(current.bounds, ray_copy) && !checkRayOriginInsideAABB(root.bounds, ray_copy)) {
                    continue;
                } else if (last_primitive_t != -1.0f && ray_copy.t >= last_primitive_t) {
                    notVisited.emplace_back(current.bounds);
                    continue;
                }

                // We know the current AABB is intersected by the ray: we need to check its child nodes or its triangles
                // (if it's a leaf):
                if (!current.isLeaf) {
                    int leaf1_idx = std::get<0>(current.indexes.at(0));
                    int leaf2_idx = std::get<1>(current.indexes.at(0));

                    if (leaf1_idx != -1) {
                        Node node1 = createdNodes.at(leaf1_idx);
                        ray_copy = ray;
                        bool check1 = intersectRayWithShape(node1.bounds, ray_copy);
                        if (checkRayOriginInsideAABB(node1.bounds, ray_copy))
                            check1 = true;

                        if (check1) {
                            allIntersected.emplace_back(node1.bounds);
                            if (ray_copy.t >= 0.0f) {
                                intersections.push(std::make_pair(ray_copy.t, node1));
                            } else {
                                intersections.push(std::make_pair(0.0f, node1));
                            }
                        }
                    }

                    if (leaf2_idx != -1) {
                        Node node2 = createdNodes.at(leaf2_idx);
                        ray_copy = ray;
                        bool check2 = intersectRayWithShape(node2.bounds, ray_copy);
                        if (checkRayOriginInsideAABB(node2.bounds, ray_copy))
                            check2 = true;

                        if (check2) {
                            allIntersected.emplace_back(node2.bounds);
                            if (ray_copy.t >= 0.0f) {
                                intersections.push(std::make_pair(ray_copy.t, node2));
                            } else {
                                intersections.push(std::make_pair(0.0f, node2));
                            }
                        }
                    }
                } else {
                    // This is a leaf: we check if any of the triangles is closer than any other
                    ray_copy = ray;

                    for (const auto& idx : current.indexes) {
                        const auto& mesh = m_pScene->meshes.at(std::get<0>(idx));
                        const auto& triangle = mesh.triangles.at(std::get<1>(idx));
                        const auto v0 = mesh.vertices[triangle[0]];
                        const auto v1 = mesh.vertices[triangle[1]];
                        const auto v2 = mesh.vertices[triangle[2]];

                        if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray_copy, hitInfo)) {
                            if(ray_copy.t < last_primitive_t) {
                                last_primitive_t = ray_copy.t;
                                vf0 = v0;
                                vf1 = v1;
                                vf2 = v2;

                                hitInfo.material = mesh.material;
                                hitInfo.normal = normalize(glm::cross(v1.position - v0.position, v2.position - v0.position));
                                hit = true;

                                hitInfo.barycentricCoord = computeBarycentricCoord(v0.position, v1.position, v2.position, ray_copy.origin + ray_copy.t * ray_copy.direction);

                                if (features.enableTextureMapping) {
                                    hitInfo.texCoord = interpolateTexCoord(v0.texCoord, v1.texCoord, v2.texCoord, hitInfo.barycentricCoord);
                                }
                            }
                        }
                    }
                }
            }

            if (hit)
                ray.t = last_primitive_t;
        }

        if (enableDebugDraw && hit) {
            drawColoredTriangle(vf0, vf1, vf2, glm::vec3 { 0.0f, 1.0f, 0.0f });
            if (debugIntersected) {
                debugDrawNotVisited(notVisited);
                debugDrawAllIntersected(allIntersected, notVisited);
                debugIntersected = false;
            }

            if (features.enableNormalInterp) {
                glm::vec3 interpolatedNormal = interpolateNormal(vf0.normal, vf1.normal, vf2.normal, hitInfo.barycentricCoord);
                Ray normal0 = { vf0.position,
                    normalize(vf0.normal),
                    1 };

                Ray normal1 = { vf1.position,
                    normalize(vf1.normal),
                    1 };

                Ray normal2 = { vf2.position,
                    normalize(vf2.normal),
                    1 };

                Ray interpolated = { ray.origin + ray.t * ray.direction,
                    normalize(interpolatedNormal),
                    1 };

                drawRay(normal0, { 0, 1, 0 });
                drawRay(normal1, { 0, 1, 0 });
                drawRay(normal2, { 0, 1, 0 });
                drawRay(interpolated, { 1, 0, 0 });
            }
        }

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

void BoundingVolumeHierarchy::getBestSplit(Node& parent, const std::vector<int>& axises, std::vector<float> thresholds, Node& left, Node& right)
{
    float bestCost = FLOAT_MAX;
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
                // save chosen threshold & axis for visual debug
                parent.divisionAxis = axises.at(i);
                parent.divisionThreshold = thresholds.at(i * thresholds_per_axis + j);
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
