#include "obstacle_clustering_tree.h"

void ObstacleNode::add(ObstacleNode *node)
{
    this->add(node, 0);
}
void ObstacleNode::add(ObstacleNode *node, int depth)
{
    if (node->get(depth) < this->get(depth))
    {
        if (this->left == nullptr)
        {
            this->left = node;
        }
        else
        {
            this->left->add(node, depth + 1);
        }
    }
    else
    {
        if (this->right == nullptr)
        {
            this->right = node;
        }
        else
        {
            this->right->add(node, depth + 1);
        }
    }
}

float ObstacleNode::get(int dim)
{
    {
        if (dim % 2 == 0)
        {
            return this->vertex.x;
        }
        else
        {
            return this->vertex.y;
        }
    }
}

int ObstacleNode::size()
{
    {
        int size = 1;
        if (this->left != nullptr)
        {
            size += this->left->size();
        }
        if (this->right != nullptr)
        {
            size += this->right->size();
        }
        return size;
    }
}

void ObstacleClusteringTree::add(Vertex &vertex)
{
    // std::cout << "Adding vertex: " << vertex.x << ", " << vertex.y << std::endl;
    if (this->root == nullptr)
    {
        this->root = new ObstacleNode(vertex);
    }
    else
    {
        this->root->add(new ObstacleNode(vertex));
    }
}
ObstacleNode *ObstacleClusteringTree::findNearestObstacle(Vertex &subject)
{
    return this->findNearestObstacle(ObstacleNode(subject), this->root, 0);
}

ObstacleNode *ObstacleClusteringTree::findNearestObstacle(ObstacleNode &subject, ObstacleNode *node, int depth)
{
    if (node == nullptr)
    {
        return nullptr;
    }
    // else
    // {
    //     std::cout << "Node: " << node->getVertex().x << ", " << node->getVertex().y << std::endl;
    // }

    ObstacleNode *nextBranch;
    ObstacleNode *oppositeBranch;

    if (subject.get(depth) < node->get(depth))
    {
        nextBranch = node->getLeft();
        oppositeBranch = node->getRight();
        // std::cout << "Next branch: " << nextBranch->getVertex().x << ", " << nextBranch->getVertex().y << std::endl;
        // std::cout << "Opposite branch: " << oppositeBranch->getVertex().x << ", " << oppositeBranch->getVertex().y << std::endl;
    }
    else
    {
        nextBranch = node->getRight();
        oppositeBranch = node->getLeft();
        // std::cout << "Next branch: " << nextBranch->getVertex().x << ", " << nextBranch->getVertex().y << std::endl;
        // std::cout << "Opposite branch: " << oppositeBranch->getVertex().x << ", " << oppositeBranch->getVertex().y << std::endl;
    }

    // if (nextBranch != nullptr)
    // {
    //     std::cout << "Next branch: " << nextBranch->getVertex().x << ", " << nextBranch->getVertex().y << std::endl;
    // }
    // if (oppositeBranch != nullptr)
    // {
    //     // std::cout << "Opposite branch: " << oppositeBranch->getVertex().x << ", " << oppositeBranch->getVertex().y << std::endl;
    // }

    ObstacleNode *temp = this->findNearestObstacle(subject, nextBranch, depth + 1);
    ObstacleNode *best = this->closestNode(temp, node, subject);

    double radiusSquared = this->distanceSquared(subject, best);
    double distanceSquared = pow(subject.get(depth) - node->get(depth), 2);

    if (distanceSquared <= radiusSquared)
    {
        temp = this->findNearestObstacle(subject, oppositeBranch, depth + 1);
        best = this->closestNode(temp, best, subject);
    }
    return best;
}

double ObstacleClusteringTree::distanceSquared(ObstacleNode &subject, ObstacleNode *node)
{
    double total = 0;
    total = pow(std::abs(subject.get(0) - node->get(0)), 2) + pow(std::abs(subject.get(1) - node->get(1)), 2);
    return total;
}

ObstacleNode *ObstacleClusteringTree::closestNode(ObstacleNode *node, ObstacleNode *best, ObstacleNode &subject)
{
    if (node == nullptr)
    {
        return best;
    }
    if (best == nullptr)
    {
        best = node;
    }

    double distanceBtwSubjectAndBest = this->distanceSquared(subject, best);
    double distanceBtwSubjectAndNode = this->distanceSquared(subject, node);

    if (distanceBtwSubjectAndNode < distanceBtwSubjectAndBest)
    {
        best = node;
    }

    return best;
}