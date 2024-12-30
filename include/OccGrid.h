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
        OccGrid(Atlas *pAtlas, float resolution);
        void SaveToFile(std::string filename);
        double Entropy() const;
        double InformationGainOver(OccGrid* otherOG) const;

        unsigned int nTotalPoints;
        unsigned int nTotalKeyFrames;
        double nAverageDistance;

    protected:
        OcTree* pOT;
        std::time_t ts;

};

} // namespace ORB_SLAM3

#endif //ORB_SLAM3_OCCGRID_H
