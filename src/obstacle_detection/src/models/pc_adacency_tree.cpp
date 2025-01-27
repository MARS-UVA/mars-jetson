#include "realsense.h"
#include "pc_adacency_tree.h"
#include <iostream>
#include "../realsense_capture.h"

void PointcloudTree::deepCopy(const PointcloudTree &other)
{
    root = other.root ? new Node(*other.root) : nullptr;
    topLeft = other.topLeft;
    bottomRight = other.bottomRight;

    childTrees.clear();
    for (auto child : other.childTrees)
    {
        childTrees.push_back(child ? new PointcloudTree(*child) : nullptr);
    }
}

/* COnstructors: */
PointcloudTree::PointcloudTree(Point &topLeft, Point &bottomRight)
{
    this->root = nullptr;
    this->topLeft = topLeft;
    this->bottomRight = bottomRight;
    childTrees.assign(4, nullptr);
}

PointcloudTree::PointcloudTree()
{
    this->root = NULL;
    this->topLeft = Point();
    this->bottomRight = Point();
    childTrees.assign(4, nullptr);
}

PointcloudTree::PointcloudTree(const PointcloudTree &other)
{
    deepCopy(other);
}

PointcloudTree::~PointcloudTree()
{
    clear();
}

PointcloudTree &PointcloudTree::operator=(const PointcloudTree &other)
{
    if (this != &other)
    {
        clear();
        deepCopy(other);
    }
    return *this;
}

PointcloudTree::PointcloudTree(PointcloudTree &&other) noexcept
{
    root = other.root;
    topLeft = other.topLeft;
    bottomRight = other.bottomRight;
    childTrees = std::move(other.childTrees);

    other.root = nullptr;
    other.childTrees.clear();
}

/* Methods: */
void PointcloudTree::clear()
{
    delete root;
    root = nullptr;

    for (auto &child : childTrees)
    {
        if (child)
        {
            delete child;
            child = nullptr;
        }
    }
}

int PointcloudTree::getQuadrant(Node *node, float middle_x, float middle_y)
{
    int quadrant = -1;
    if (node->pos.x <= middle_x)
    {
        if (node->pos.y <= middle_y)
        {
            quadrant = FL;
        }
        else
        {
            quadrant = BL;
        }
    }
    else
    {
        if (node->pos.y <= middle_y)
        {
            quadrant = FR;
        }
        else
        {
            quadrant = BR;
        }
    }

    return quadrant;
}
void PointcloudTree::add(Vertex *vertex)
{
    if (vertex == nullptr)
        return;
    Node *newNode = new Node(Point(vertex->x, vertex->y), vertex->z);
    add(newNode);
}

void PointcloudTree::add(Node *newNode)
{
    if (newNode == nullptr)
        return;

    // If root is empty, add here regardless of size
    if (root == nullptr)
    {
        root = newNode;
        return;
    }

    float middle_x = (topLeft.x + bottomRight.x) / 2;
    float middle_y = (topLeft.y + bottomRight.y) / 2;

    int currQuadrant = getQuadrant(newNode, middle_x, middle_y);
    if (currQuadrant == -1)
        return;

    // Create child tree if it doesn't exist
    if (childTrees[currQuadrant] == nullptr)
    {
        Point newTopLeft, newBottomRight;
        switch (currQuadrant)
        {
        case FL:
            newTopLeft = Point(topLeft.x, topLeft.y);
            newBottomRight = Point(middle_x, middle_y);
            break;
        case FR:
            newTopLeft = Point(middle_x, topLeft.y);
            newBottomRight = Point(bottomRight.x, middle_y);
            break;
        case BL:
            newTopLeft = Point(topLeft.x, middle_y);
            newBottomRight = Point(middle_x, bottomRight.y);
            break;
        case BR:
            newTopLeft = Point(middle_x, middle_y);
            newBottomRight = Point(bottomRight.x, bottomRight.y);
            break;
        }
        childTrees[currQuadrant] = new PointcloudTree(newTopLeft, newBottomRight);
    }

    childTrees[currQuadrant]->add(newNode);
}

Node *PointcloudTree::find(Node *node)
{
    return find(node->pos, node);
}

Node *PointcloudTree::find(Point &pos, Node *node)
{
    if (root != nullptr && root->pos == node->pos)
    {
        return root;
    }

    float middle_x = (topLeft.x + bottomRight.x) / 2;
    float middle_y = (topLeft.y + bottomRight.y) / 2;

    int currQuad = getQuadrant(node, middle_x, middle_y);

    if (currQuad == -1)
        return nullptr;
    if (childTrees[currQuad] == nullptr)
        return nullptr;
    return childTrees[currQuad]->find(pos, node);
}

void PointcloudTree::print(int depth)
{
    // Print indentation based on depth
    for (int i = 0; i < depth; ++i)
        std::cout << "  ";

    // Print tree boundaries
    std::cout << "Tree: ["
              << topLeft.x << "," << topLeft.y << "] to ["
              << bottomRight.x << "," << bottomRight.y << "] ";

    // Print root if exists
    if (root)
        std::cout << "Root: (" << root->pos.x << "," << root->pos.y << ")";

    std::cout << std::endl;

    // Recursively print child trees
    for (int i = 0; i < 4; ++i)
    {
        if (childTrees[i])
        {
            for (int j = 0; j < depth; ++j)
                std::cout << "  ";
            std::cout << "Quadrant " << i << ":" << std::endl;
            childTrees[i]->print(depth + 1);
        }
    }
}

void PointcloudTree::exportToPly()
{
    std::vector<Vertex> vertices;
    extractAllNodes(vertices);
    save_to_ply(vertices, "pc_tree_out.ply");
}

std::vector<Vertex> PointcloudTree::extractAllNodes()
{
    std::vector<Vertex> vertices;
    extractAllNodes(vertices);
    return vertices;
}

void PointcloudTree::extractAllNodes(std::vector<Vertex> &vertices)
{
    if (root)
    {
        vertices.push_back(Vertex(root->pos.x, root->pos.y, root->height));
    }

    for (auto &child : childTrees)
    {
        if (child)
        {
            child->extractAllNodes(vertices);
        }
    }
}

void PointcloudTree::extractLeafNodesAtDepth(int targetDepth, std::vector<std::vector<Vertex>> &quadrantVertices)
{
    quadrantVertices.resize(4);

    extractLeafNodesRecursive(0, targetDepth, quadrantVertices);
    std::cout << "num vertices in 1st qyuadrant: " << quadrantVertices[0].size() << std::endl;
    std::cout << "num vertices in 2nd qyuadrant: " << quadrantVertices[1].size() << std::endl;
    std::cout << "num vertices in 3rd qyuadrant: " << quadrantVertices[2].size() << std::endl;
    std::cout << "num vertices in 4th qyuadrant: " << quadrantVertices[2].size() << std::endl;
}

void PointcloudTree::extractLeafNodesRecursive(int currentDepth, int targetDepth,
                                               std::vector<std::vector<Vertex>> &quadrantVertices)
{
    // At target depth, check all child trees for leaf nodes
    if (currentDepth == targetDepth)
    {
        for (int i = 0; i < 4; i++)
        {
            if (childTrees[i] != nullptr)
            {
                collectLeafNodes(childTrees[i], i, quadrantVertices);
            }
        }
        return;
    }

    // Continue traversing until target depth
    for (int i = 0; i < 4; i++)
    {
        if (childTrees[i] != nullptr)
        {
            childTrees[i]->extractLeafNodesRecursive(currentDepth + 1, targetDepth, quadrantVertices);
        }
    }
}

void PointcloudTree::collectLeafNodes(PointcloudTree *tree, int quadrant, std::vector<std::vector<Vertex>> &quadrantVertices)
{
    if (tree->root != nullptr)
    {
        quadrantVertices[quadrant].push_back(Vertex(tree->root->pos.x, tree->root->pos.y, tree->root->height));
    }

    for (int i = 0; i < 4; i++)
    {
        if (tree->childTrees[i] != nullptr)
        {
            collectLeafNodes(tree->childTrees[i], quadrant, quadrantVertices);
        }
    }
}

// void PointcloudTree::extractLeafNodesRecursive(int currentDepth, int targetDepth, int quadIndex,
//                                                std::vector<std::vector<Vertex>> &quadrantVertices)
// {
//     if (root != nullptr && quadIndex > -1)
//     {
//         quadrantVertices[quadIndex].push_back(Vertex(root->pos.x, root->pos.y, root->height));
//         return;
//     }
//     if (currentDepth == targetDepth)
//     {
//         for (int i = 0; i < 4; i++)
//         {
//             auto child = childTrees[i];
//             child->extractLeafNodesRecursive(currentDepth + 1, targetDepth, i, quadrantVertices);
//         }
//     }
//     else
//     {
//         for (int i = 0; i < 4; i++)
//         {
//             auto child = childTrees[i];
//             child->extractLeafNodesRecursive(currentDepth + 1, targetDepth, quadIndex, quadrantVertices);
//         }
//     }
// }

// int main()
// {
//     PointcloudTree tree = PointcloudTree(Point(-5, 10), Point(5, 0));
//     Vertex *v = new Vertex(0, 0, 0);
//     tree.add(v);
//     Node a(Point(1.0, 1.0), 1);
//     Node b(Point(2, 5), 2);
//     Node c(Point(7, 6), 3);
//     tree.add(&a);
//     tree.add(&b);
//     tree.add(&c);
//     return 0;
// }