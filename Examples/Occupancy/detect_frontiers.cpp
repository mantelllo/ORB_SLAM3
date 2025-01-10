//
// Created by john on 25.1.10.
//

#include <iostream>
#include <ostream>

#include "FrontierDetector.h"
#include "OccGrid.h"
using namespace std;
using namespace ORB_SLAM3;


int main(int argc, char **argv) {
    if (argc != 2) {
        cerr << "Usage: " << argv[0] << " filename" << endl;
        return 1;
    }

    const string filename = argv[1];
    OccGrid grid(filename);

    std::vector<std::shared_ptr<Frontier>> frontiers;
    FrontierDetector::DetectFrontiers(&grid, &frontiers);

    // Output results
    std::cout << "Detected " << frontiers.size() << " frontiers." << std::endl;

    // Optionally: Print coordinates of detected frontiers
    for (const auto& frontier : frontiers) {
        std::cout << "Frontier size: " << frontier->points.size() << std::endl;
        std::cout << "Frontier center: " << frontier->center << std::endl;
        std::cout << "Frontier points: ";
        for (const auto& point : frontier->points) {
            std::cout << point << " ";
        }
        std::cout << std::endl;
    }
}