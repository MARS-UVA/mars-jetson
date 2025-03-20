#ifndef SLAM_H
#define SLAM_H

#include <vector>
#include <cmath>
#include <utility>
#include "models/realsense.h"
#include "models/obstacle_clustering_tree.h"
#include "realsense_capture.h"

class SLAM_Graph
{
private:
    std::vector<Coordinate *> poses;
    std::vector<Coordinate *> landmarks;
    std::vector<Coordinate *> odom_edges; // Odometry edges
    std::vector<Coordinate *> loop_edges; // Loop closures

public:
    Coordinate currentRobotPos = Coordinate(0, 0, 0);
    Coordinate currentRobotVel = Coordinate(0, 0, 0);
}
#endif