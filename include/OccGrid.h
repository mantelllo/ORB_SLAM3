//
// Created by john on 12/28/24.
//

#ifndef ORB_SLAM3_OCCGRID_H
#define ORB_SLAM3_OCCGRID_H


#include <octomap/OcTree.h>
#include "Atlas.h"

using namespace octomap;


namespace ORB_SLAM3
{


class OccGrid
{
    public:
        OccGrid(Atlas *pAtlas, float resolution, int n_mappoint_obs_min = 7, int n_mappoint_max_dst = 10);
        explicit OccGrid(OcTree *pOT);
        explicit OccGrid(string treePath);
        void SaveToFile(const std::string& filename);
        double Entropy() const;
        double InformationGainOver(const OccGrid* otherOG) const;
        shared_ptr<vector<point3d>> GetPointCloud() const;

        void GetNodes(std::vector<std::vector<float>>& nodeOccupation) const;
        const OcTree* GetOcTree() const;

        unsigned int nTotalPoints{};
        unsigned int nTotalKeyFrames{};
        double nAverageDistance{};

    protected:
        OcTree* pOT;
        std::time_t ts{};

};

} // namespace ORB_SLAM3

#endif //ORB_SLAM3_OCCGRID_H
