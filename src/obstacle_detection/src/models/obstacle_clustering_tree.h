#ifndef OBSTACLECLUSTERING_H
#define OBSTACLECLUSTERING_H

#include "realsense.h"

class ObstacleNode
{
private:
    Vertex vertex;
    ObstacleNode *left;
    ObstacleNode *right;
    int dims;

public:
    ObstacleNode() : left(nullptr), right(nullptr), dims(2) {}
    ObstacleNode(Vertex &vertex) : left(nullptr), right(nullptr), vertex(vertex), dims(2) {}
    ObstacleNode(Vertex &vertex, ObstacleNode *left, ObstacleNode *right) : vertex(vertex), left(left), right(right), dims(2) {}

    ObstacleNode *getLeft()
    {
        return this->left;
    }
    ObstacleNode *getRight()
    {
        return this->right;
    }

    Vertex &getVertex()
    {
        return this->vertex;
    }

    void add(ObstacleNode *node);

    void add(ObstacleNode *node, int depth);

    float get(int dim);

    int size();
};

class ObstacleClusteringTree
{
private:
    ObstacleNode *root;
    int dims;

public:
    ObstacleClusteringTree(int nDims) : root(nullptr), dims(nDims) {}
    ObstacleClusteringTree(Vertex &vertex) : root(new ObstacleNode(vertex)), dims(2) {}

    void add(Vertex &vertex);

    ObstacleNode *findNearestObstacle(Vertex &subject);

    ObstacleNode *findNearestObstacle(ObstacleNode &subject, ObstacleNode *node, int depth);

    double distanceSquared(ObstacleNode &subject, ObstacleNode *node);

    ObstacleNode *closestNode(ObstacleNode *node, ObstacleNode *best, ObstacleNode &subject);
};

#endif
