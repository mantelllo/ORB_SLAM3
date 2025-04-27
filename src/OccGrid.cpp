//
// Created by john on 12/28/24.
//

#include "OccGrid.h"

#include <utility>

namespace ORB_SLAM3
{


OccGrid::OccGrid(Atlas* pAtlas, float resolution, const int n_mappoint_obs_min,
                 const int n_mappoint_max_dst) {
    ts = std::time(nullptr);
    pOT = new OcTree(resolution);

    std::vector<KeyFrame*> mvpKeyFrames = pAtlas->GetAllKeyFrames();
    Pointcloud pPointcloud;

    int totalPoints = 0, lastKFTotalPoints = 0;
    double totalDistance = 0, lastKFTotalDistance = 0;

    //Iterate over KeyFrames
    for(size_t i=0, iend=mvpKeyFrames.size(); i<iend; i++) {
        KeyFrame* pKF = mvpKeyFrames[i];
        Sophus::Vector3f positionKF = pKF->GetPoseInverse().translation();

        vector<MapPoint*> mvpMapPoints = pKF->GetMapPointMatches();
        pPointcloud.clear();

        for(size_t j=0, jend=mvpMapPoints.size(); j<jend; j++) {
            MapPoint* pMP = mvpMapPoints[j];
            if (!pMP) continue;

            Sophus::Vector3f positionMP = pMP->GetWorldPos();
            double nDistance = sqrt(
                pow(positionKF.x() - positionMP.x(), 2) +
                pow(positionKF.y() - positionMP.y(), 2) +
                pow(positionKF.z() - positionMP.z(), 2)
            );
            if (pMP->isBad()
                || pMP->Observations() < n_mappoint_obs_min
                || nDistance > n_mappoint_max_dst
                )
                continue;


            Eigen::Vector3f pMPNed = Converter::cvPinholeToNED(positionMP);
            pPointcloud.push_back((float)pMPNed.x(), (float)pMPNed.y(), (float)pMPNed.z());
            totalPoints++;
            totalDistance += nDistance;

            if (i == mvpKeyFrames.size() - 1) {
                lastKFTotalPoints++;
                lastKFTotalDistance += nDistance;
            }
        }

        Eigen::Vector3f pKFNED = Converter::cvPinholeToNED(positionKF);
        point3d p (pKFNED.x(), pKFNED.y(), pKFNED.z());
        pOT->insertPointCloud(pPointcloud, p, -1, true, false);
        //pOT->updateNode(p, true);
    }
    pOT->updateInnerOccupancy();

    // add reference map points
    Map* pActiveMap = pAtlas->GetCurrentMap();
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
    for(auto vpMP : vpRefMPs)
    {
        if(vpMP->isBad() || vpMP->nObs < n_mappoint_obs_min) continue;
        Sophus::Vector3f point = vpMP->GetWorldPos();
        Sophus::Vector3f pNED = Converter::cvPinholeToNED(point);
        point3d p (pNED.x(), pNED.y(), pNED.z());
        pOT->updateNode(p, true);
    }


    double lastKFAvgDistance = lastKFTotalDistance / lastKFTotalPoints;
    // cout << "lastKFAvgDistance " << lastKFAvgDistance << endl;

    nTotalPoints = totalPoints;
    nAverageDistance = totalDistance / totalPoints;
    nTotalKeyFrames = mvpKeyFrames.size();
}


OccGrid::OccGrid(OcTree *pOT) {
    this->pOT = pOT;
}

OccGrid::OccGrid(string treePath) {
    auto pOT = new OcTree(std::move(treePath));
    this->pOT = pOT;
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


double OccGrid::InformationGainOver(const OccGrid* otherOG) const {
    double e1 = Entropy();
    double e2 = otherOG->Entropy();
    return e1 - e2;
}

shared_ptr<vector<point3d>> OccGrid::GetPointCloud() const {
    auto vPoints = make_shared<vector<point3d>>();

    for (auto it=pOT->begin_leafs(), end=pOT->end_leafs();
            it != end; ++it) {
        point3d p;
        if (it->getOccupancy() > 0.5) {
            p = it.getCoordinate();
            vPoints->push_back(p);
        };
    }

    return vPoints;
}


void OccGrid::SaveToFile(const std::string& filename) {
    bool bOK = pOT->writeBinaryConst(filename);
    if (!bOK) {
        cout << "Storing of Poincloud to disk failed!!" << endl;
    }
}


void OccGrid::GetNodes(std::vector<std::vector<float>>& nodeOccupation) const {
    octomap::OcTree::leaf_iterator it = pOT->begin_leafs(), end = pOT->end_leafs();
    for (; it != end; ++it) {
        if (it->getOccupancy() > 0.5) {
            nodeOccupation.push_back({(float)it.getX(), (float)it.getY(), (float)it.getZ()});
        }
    }
}


const OcTree * OccGrid::GetOcTree() const {
    return this->pOT;
}
} // namespace ORB_SLAM3