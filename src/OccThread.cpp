//
// Created by john on 12/28/24.
//

#include "OccThread.h"
#include "OccGrid.h"

namespace ORB_SLAM3 {


void OccupancyThread::Run() {
    cout << "OccupancyThread STARTED" << endl;

    while (1) {
        // cout << endl << "-- New Loop --" << endl;
        // pAtlas->GenerateNewOccupancyGrid(octree_resolution);
        //
        // //Print
        // auto mvpOccupancyGrids = pAtlas->GetOccupancyGrids();
        // const auto& lastOG = mvpOccupancyGrids[mvpOccupancyGrids.size()-1];
        // // cout << "  H = " << lastOG->Entropy();
        // if (mvpOccupancyGrids.size() > 1) {
        //     auto prelastOG = mvpOccupancyGrids[mvpOccupancyGrids.size()-2];
        //     // cout << " IG = " << lastOG->InformationGainOver(prelastOG);
        // }
        // // cout << endl;
        //
        // lastOG->SaveToFile("OCTREE.bt");

        usleep(1000000);
    }
}




} // ORB_SLAM3