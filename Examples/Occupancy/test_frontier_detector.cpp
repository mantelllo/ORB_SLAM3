//
// Created by john on 25.1.8.
//

#include "FrontierDetector.h"
#include <octomap/OcTree.h>
#include <iostream>


using namespace ORB_SLAM3;


int main() {
    // Create an OctoMap object
    double resolution = 0.5;  // Adjust the resolution based on your map scale
    octomap::OcTree tree(resolution);

    // Set up a simple map (example: add some occupied and free nodes)
    octomap::point3d p1(0.5, 0.5, 0.5);
    octomap::point3d p2(1.0, 0.5, 0.5);
    octomap::point3d p3(1.5, 0.5, 0.5);

    octomap::point3d p4(2.5, 0.5, 0.5);
    octomap::point3d p5(3.0, 0.5, 0.5);
    octomap::point3d p6(3.5, 0.5, 0.5);

    // Mark some nodes as occupied and others as free
    tree.updateNode(p1, false);
    tree.updateNode(p2, false);
    tree.updateNode(p3, false);

    tree.updateNode(p4, false);
    tree.updateNode(p5, false);
    tree.updateNode(p6, true);
    OccGrid pOG(&tree);

    // Now test FrontierDetector
    std::vector<std::shared_ptr<Frontier>> frontiers;
    FrontierDetector::DetectFrontiers(&pOG, &frontiers);

    // Output results
    std::cout << "Detected " << frontiers.size() << " frontiers." << std::endl;

    // Optionally: Print coordinates of detected frontiers
    for (const auto& frontier : frontiers) {
        std::cout << "Frontier center: " << frontier->center << std::endl;
        std::cout << "Frontier points: ";
        for (const auto& point : frontier->points) {
            std::cout << point << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
