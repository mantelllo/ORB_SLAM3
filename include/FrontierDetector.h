//
// Created by john on 25.1.8.
//

#ifndef FRONTIERDETECTOR_H
#define FRONTIERDETECTOR_H
#include <set>
#include <octomap/OcTree.h>
#include <tr1/memory>

#include "OccGrid.h"

namespace ORB_SLAM3 {
    typedef struct Frontier {
        std::vector<octomap::point3d> points;
        octomap::point3d center;
    } Frontier;

    struct OcTreeKeyComparator {
        bool operator()(const octomap::OcTreeKey& lhs, const octomap::OcTreeKey& rhs) const {
            if (lhs[0] != rhs[0]) {
                return lhs[0] < rhs[0];  // Compare x values first
            }
            if (lhs[1] != rhs[1]) {
                return lhs[1] < rhs[1];  // Compare y values next
            }
            return lhs[2] < rhs[2];  // Finally, compare z values
        }
    };

    class FrontierDetector {
    public:
        static std::vector<std::shared_ptr<Frontier>>* DetectFrontiers(const shared_ptr<OccGrid>& pOG);

    private:
        static bool isFrontier(const OcTree* tree, const octomap::OcTreeKey* key);
        static Frontier clusterFrontier(const OcTree* tree,
                                        const OcTreeKey& startKey,
                                        std::set<octomap::OcTreeKey, OcTreeKeyComparator>& visited);

    };
}



#endif //FRONTIERDETECTOR_H