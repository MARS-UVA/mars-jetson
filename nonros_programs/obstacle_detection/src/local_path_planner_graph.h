#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include <cmath>
#include <utility>
#include "models/realsense.h"
#include "models/obstacle_clustering_tree.h"
#include "realsense_capture.h"

struct PathPoint
{
    std::pair<int, int> indices;
    PathPoint *parent;
    Coordinate *coordinate;
    std::pair<float, float> costs; // local cost, global cost
    PathPoint(PathPoint *parent, Coordinate *coordinate, int i, int j, float localCost, float globalCost) : parent(parent),
                                                                                                            coordinate(coordinate), indices({i, j}), costs({localCost, globalCost}) {}
};

class AStarPathPlanner
{
private:
    ObstacleClusteringTree &obstacleTree;
    std::vector<PathPoint *> costArray;
    Vertex goal;

public:
    AStarPathPlanner(ObstacleClusteringTree &tree, Vertex&& goal)
        : obstacleTree(tree), goal(goal) {}
    ~AStarPathPlanner()
    {
        for (auto point : costArray)
        {
            delete point;
        }
        costArray.clear();
    }

    std::vector<Vertex> retracePath(PathPoint *current);

    std::vector<Vertex> planPath(std::vector<std::vector<Coordinate>> &actualCoordinates, Vertex &start, std::pair<int, int> &startIndices);

    float hValue(float x, float y);

    bool isDestination(PathPoint *current);
};
#endif