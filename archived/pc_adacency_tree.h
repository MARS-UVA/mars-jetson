#pragma once
#include <vector>
#include <cmath>
#include "realsense.h"

#ifndef POINTCLOUDTREE_H
#define POINTCLOUDTREE_H

/* Quadrant indices */
#define FL 0 // Left Front
#define FR 1 // Right Front
#define BL 2 // Left Back
#define BR 3 // Right Back

struct Quadrant
{
    int pos;
    float middle_x, middle_y;
};

struct Point
{
    float x, y;
    Point() : x(0), y(0) {}
    Point(float x, float y) : x(x), y(y) {}

    bool operator==(const Point &other) const
    {
        return x == other.x && y == other.y;
    }
};

struct Node
{
    Point pos;
    float height;
    float gradient;
    bool isLeaf;
    Node(Point pos, float height)
    {
        this->pos = pos;
        this->height = height;
        isLeaf = true;
    }
    Node()
    {
        height = 0;
        isLeaf = false;
    }
};

class PointcloudTree
{
public:
    PointcloudTree(Point &topLeft, Point &bottomRight);
    PointcloudTree();
    PointcloudTree(const PointcloudTree &other);
    ~PointcloudTree();
    PointcloudTree &operator=(const PointcloudTree &other);
    PointcloudTree(PointcloudTree &&other) noexcept;

    void clear();
    int getQuadrant(Node *node, float middle_x, float middle_y);
    void add(Vertex *vertex);
    void add(Node *newNode);
    Node *find(Node *node);
    Node *find(Point &pos, Node *node);
    void extractAllNodes(std::vector<Vertex> &vertices);
    std::vector<Vertex> extractAllNodes();

    void print(int depth);
    void exportToPly();

    std::vector<PointcloudTree *> getChildTrees() { return childTrees; }
    Node *getRoot() { return root; }

    void extractLeafNodesAtDepth(int targetDepth, std::vector<std::vector<Vertex>> &quadrantVertices);
    void extractLeafNodesRecursive(int currentDepth, int targetDepth, std::vector<std::vector<Vertex>> &quadrantVertices);
    void collectLeafNodes(PointcloudTree *tree, int quadrant, std::vector<std::vector<Vertex>> &quadrantVertices);

    void gradientify();
    void gradientify(PointcloudTree *parent, PointcloudTree *grandparent, int quadrant);
    // void gradientify_parallel();
    // void gradientify_cuda();

private:
    Node *root;
    Point topLeft, bottomRight;
    std::vector<PointcloudTree *> childTrees;

    void deepCopy(const PointcloudTree &other);
};

#endif // POINTCLOUDTREE_H
