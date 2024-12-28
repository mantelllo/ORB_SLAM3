//
// Created by john on 12/28/24.
//

#include "OccGrid.h"

namespace ORB_SLAM3
{


OccGrid::OccGrid(Atlas* pAtlas, float resolution) {
    ts = std::time(nullptr);
    pOT = new OcTree(resolution);

    std::vector<KeyFrame*> mvpKeyFrames = pAtlas->GetAllKeyFrames();
    Pointcloud pPointcloud;

    const int N_MAPPOINT_OBS_MIN = 3;
    const int N_MAPPOINT_MAX_DST = 10;

    int totalPoints = 0;

    //Iterate over KeyFrames
    for(size_t i=0, iend=mvpKeyFrames.size(); i<iend; i++) {
        KeyFrame* pKF = mvpKeyFrames[i];
        Sophus::Vector3f positionKF = pKF->GetPoseInverse().translation();

        vector<MapPoint*> mvpMapPoints = pKF->GetMapPointMatches();
        pPointcloud.clear();

        for(size_t j=0, jend=mvpMapPoints.size(); j<jend; j++) {
            MapPoint* pMP = mvpMapPoints[j];
            if (!pMP
                || pMP->isBad()
                || pMP->Observations() < N_MAPPOINT_OBS_MIN
                || pMP->GetMaxDistanceInvariance() > N_MAPPOINT_MAX_DST
                )
                continue;

            Sophus::Vector3f positionMP = pMP->GetWorldPos();
            pPointcloud.push_back((float)positionMP.x(),
                                  (float)positionMP.y(),
                                  (float)positionMP.z());
            totalPoints++;
        }

        point3d p (positionKF.x(), positionKF.y(), positionKF.z());
        pOT->insertPointCloud(pPointcloud, p);
    }

    nTotalPoints = totalPoints;
    nTotalKeyFrames = mvpKeyFrames.size();
};


double OccGrid::Entropy() const {
    double entropy = 0.0;

    for (auto it = pOT->begin_leafs(), end = pOT->end_leafs(); it != end; ++it) {
        double size = it.getSize();          // Node size
        double volume = size * size * size;  // Node volume
        double p = it->getOccupancy();       // Node probability

        // Ensure probabilities are within valid range
        if (p > 0 && p < 1) {
            double nodeEntropy = -volume * (p * std::log2(p) + (1 - p) * std::log2(1 - p));
            entropy += nodeEntropy;
        }
    }

    return entropy;
}


double OccGrid::InformationGainOver(OccGrid* otherOG) const {
    double e1 = Entropy();
    double e2 = otherOG->Entropy();
    return e1 - e2;
}


void OccGrid::SaveToFile(std::string filename) {
    bool bOK = pOT->writeBinaryConst(filename);
    if (!bOK) {
        cout << "Storing of Poincloud to disk failed!!" << endl;
    }
}



} // namespace ORB_SLAM3