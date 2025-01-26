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
    // std::cout << "Adding node at " << newNode->pos.x << ", " << newNode->pos.y << std::endl;
    if (newNode == nullptr)
    {
        // std::cout << "Node is null" << std::endl;
        return;
    }
    if (find(newNode) != nullptr)
    {
        // std::cout << "Node already exists and is " << newNode->pos.x << ", " << newNode->pos.y << std::endl;
        return;
    }

    if (std::abs(topLeft.x - bottomRight.x) <= 1 && std::abs(topLeft.y - bottomRight.y) <= 1)
    {
        if (this->root == NULL)
        {
            this->root = newNode;
            std::cout << "Root is added as " << newNode->pos.x << ", " << newNode->pos.y << std::endl;
        }
        return;
    }

    float middle_x = (topLeft.x + bottomRight.x) / 2;
    float middle_y = (topLeft.y + bottomRight.y) / 2;

    int currQuadrant = getQuadrant(newNode, middle_x, middle_y);

    if (childTrees[currQuadrant] == nullptr)
    {
        switch (currQuadrant)
        {
        case FL:
            childTrees[currQuadrant] = new PointcloudTree(Point(topLeft.x, middle_y), Point(middle_x, bottomRight.y));
            break;
        case FR:
            childTrees[currQuadrant] = new PointcloudTree(Point(middle_x, middle_y), bottomRight);
            break;
        case BL:
            childTrees[currQuadrant] = new PointcloudTree(topLeft, Point(middle_x, middle_y));
            break;
        case BR:
            childTrees[currQuadrant] = new PointcloudTree(Point(middle_x, topLeft.y), Point(bottomRight.x, middle_y));
            break;
        default:
            // std::cout << "Invalid quadrant" << std::endl;
            return;
        }
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
    std::cout << "num vertices in first qyuadrant: " << quadrantVertices[0].size() << std::endl;
}

void PointcloudTree::extractLeafNodesRecursive(int currentDepth, int targetDepth,
                                               std::vector<std::vector<Vertex>> &quadrantVertices)
{
    // If at target depth and this is a leaf node (no children)
    if (currentDepth == targetDepth)
    {
        // Check if this subtree contains a root node
        if (root)
        {
            // Determine which quadrant this belongs to
            float middle_x = (topLeft.x + bottomRight.x) / 2;
            float middle_y = (topLeft.y + bottomRight.y) / 2;

            int quadrant = -1;
            if (root->pos.x <= middle_x)
            {
                quadrant = (root->pos.y <= middle_y) ? 0 : 1;
            }
            else
            {
                quadrant = (root->pos.y <= middle_y) ? 2 : 3;
            }

            if (quadrant != -1)
            {
                quadrantVertices[quadrant].push_back(
                    Vertex(root->pos.x, root->pos.y, root->height));
            }
        }
        return;
    }

    // Recursively traverse child quadrants
    if (currentDepth < targetDepth)
    {
        for (auto &child : childTrees)
        {
            if (child)
            {
                child->extractLeafNodesRecursive(currentDepth + 1, targetDepth, quadrantVertices);
            }
        }
    }
    std::cout << "num vertices in first qyuadrant: " << quadrantVertices[0].size() << std::endl;
}

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