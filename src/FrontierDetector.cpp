//
// Created by john on 25.1.8.
//

#include "../include/FrontierDetector.h"

#include <queue>
#include <set>


namespace ORB_SLAM3 {
    std::vector<std::shared_ptr<Frontier>>* FrontierDetector::DetectFrontiers(const OccGrid* pOG) {
        auto *frontiers = new std::vector<std::shared_ptr<Frontier>>();
        const OcTree *tree = pOG->GetOcTree();

        if (!tree) {
            return nullptr;
        }

        std::set<octomap::OcTreeKey, OcTreeKeyComparator> visited;  // Keeps track of visited nodes

        // Iterate through all free voxels in the OctoMap
        for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
            if (it->getOccupancy() < 0.5) {  // Free voxel
                const octomap::OcTreeKey& key = it.getKey();

                // Check if this voxel is a frontier and hasn't been visited
                if (visited.find(key) == visited.end() && isFrontier(tree, &key)) {
                    // Perform BFS to gather all frontier points in this cluster
                    auto frontier = clusterFrontier(tree, key, visited);
                    frontiers->push_back(std::make_shared<Frontier>(frontier));
                }
            }
        }

        return frontiers;
    }


    // Checks if a voxel is a frontier (free voxel with at least one unknown neighbor)
    bool FrontierDetector::isFrontier(const octomap::OcTree* tree, const octomap::OcTreeKey* key) {
        if (!tree || !key) return false;

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;  // Skip self

                    // Calculate neighbor key
                    octomap::OcTreeKey neighborKey = *key;
                    neighborKey.k[0] += dx;
                    neighborKey.k[1] += dy;
                    neighborKey.k[2] += dz;

                    // Check if the neighbor is unknown
                    if (!tree->search(neighborKey)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }


    Frontier FrontierDetector::clusterFrontier(const octomap::OcTree* tree,
                                               const octomap::OcTreeKey& startKey,
                                               std::set<octomap::OcTreeKey, OcTreeKeyComparator>& visited) {
        Frontier frontier;
        std::queue<octomap::OcTreeKey> toVisit;

        toVisit.push(startKey);

        while (!toVisit.empty()) {
            octomap::OcTreeKey currentKey = toVisit.front();
            toVisit.pop();

            if (visited.find(currentKey) != visited.end()) continue;  // Skip if already visited
            visited.insert(currentKey);

            octomap::point3d point = tree->keyToCoord(currentKey);
            frontier.points.push_back(point);

            // Check neighbors for potential frontier voxels
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;  // Skip self

                        octomap::OcTreeKey neighborKey = currentKey;
                        neighborKey.k[0] += dx;
                        neighborKey.k[1] += dy;
                        neighborKey.k[2] += dz;

                        // Check if the neighbor is unvisited and a frontier
                        if (visited.find(neighborKey) == visited.end()) {
                            octomap::OcTreeNode* neighborNode = tree->search(neighborKey);
                            if (neighborNode && neighborNode->getOccupancy() < 0.5 && isFrontier(tree, &neighborKey)) {
                                toVisit.push(neighborKey);
                            }
                        }
                    }
                }
            }
        }

        // Compute the centroid of the frontier
        octomap::point3d centroid(0.0, 0.0, 0.0);
        for (const auto& p : frontier.points) {
            centroid += p;
        }
        if (!frontier.points.empty()) {
            centroid /= static_cast<float>(frontier.points.size());
        }
        frontier.center = centroid;

        return frontier;
    }
}