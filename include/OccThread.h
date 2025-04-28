//
// Created by john on 12/28/24.
//

#ifndef ORB_SLAM3_OCCTHREAD_H
#define ORB_SLAM3_OCCTHREAD_H

#include <Atlas.h>


namespace ORB_SLAM3
{

class OccupancyThread
{
public:
    explicit OccupancyThread(Atlas* pAtlas, const float octree_resolution):
        pAtlas(pAtlas), octree_resolution(octree_resolution) {};
    void Run();

protected:
    Atlas* pAtlas;
    const float octree_resolution;
};

} // ORB_SLAM3

#endif //ORB_SLAM3_OCCTHREAD_H
