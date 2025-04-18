#include "realsense.h"

/* Octants */
#define TLF 0 // Top Left Front
#define TRF 1 // Top Right Front
#define BRF 2 // Bottom Right Front
#define BLF 3 // Bottom Left Front
#define TLB 4 // Top Left Back
#define TRB 5 // Top Right Back
#define BRB 6 // Bottom Right Back
#define BLB 7 // Bottom Left Back

struct
{
    int pos;
    float middle_x, middle_y, middle_z;
} typedef Octant;

class PointcloudTreee
{
    Vertex *root;
    Vertex *topLeftFront, *bottomRightBack;
    std::vector<PointcloudTreee *> childTrees;

public:
    PointcloudTreee(Vertex *root, Vertex *topLeftFront, Vertex *bottomRightBack)
    {
        this->root = root;
        this->topLeftFront = topLeftFront;
        this->bottomRightBack = bottomRightBack;
    }

    PointcloudTreee(Vertex *root)
    {
        this->root = root;
    }

    PointcloudTreee()
    {
        this->root = new Vertex();
    }

    PointcloudTreee(Vertex *v1, Vertex *v2)
    {
        if (v2->x < v1->y || v2->y < v1->y || v2->z < v1->z)
            return;

        root = nullptr;
        topLeftFront = new Vertex(v1->x, v1->y, v1->z);
        bottomRightBack = new Vertex(v2->x, v2->y, v2->z);

        childTrees.assign(8, nullptr);
        for (int i = TLF; i <= BLB; ++i)
        {
            childTrees[i] = new PointcloudTreee();
        }
    }

    PointcloudTreee(const PointcloudTreee &other)
    {
        this->root = new Vertex(*other.root);
        this->topLeftFront = new Vertex(*other.topLeftFront);
        this->bottomRightBack = new Vertex(*other.bottomRightBack);
        for (auto child : other.childTrees)
        {
            this->childTrees.push_back(new PointcloudTreee(*child));
        }
    }

    ~PointcloudTreee()
    {
        delete root;
        delete topLeftFront;
        delete bottomRightBack;
        for (auto child : childTrees)
        {
            delete child;
        }
    }

    Octant getOctant(Vertex *vertex)
    {
        Octant result;
        float middle_x = (topLeftFront->x + bottomRightBack->x) / 2;
        float middle_y = (topLeftFront->y + bottomRightBack->y) / 2;
        float middle_z = (topLeftFront->z + bottomRightBack->z) / 2;

        int pos = -1;

        // Find the position of the octant in the current tree
        if (vertex->x <= middle_x)
        {
            if (vertex->y <= middle_y)
            {
                if (vertex->z <= middle_z)
                {
                    pos = BLF;
                }
                else
                {
                    pos = TLF;
                }
            }
            else
            {
                if (vertex->z <= middle_z)
                {
                    pos = BLB;
                }
                else
                {
                    pos = TLB;
                }
            }
        }
        else
        {
            if (vertex->y <= middle_y)
            {
                if (vertex->z <= middle_z)
                {
                    pos = BRF;
                }
                else
                {
                    pos = TRF;
                }
            }
            else
            {
                if (vertex->z <= middle_z)
                {
                    pos = BRB;
                }
                else
                {
                    pos = TRB;
                }
            }
        }

        result.pos = pos;
        result.middle_x = middle_x;
        result.middle_y = middle_y;
        result.middle_z = middle_z;
        return result;
    }

    void insert(Vertex *vertex)
    {
        if (find(vertex))
        {
            return;
        }

        Octant currOctant = getOctant(vertex);
        int pos = currOctant.pos;
        float middle_x = currOctant.middle_x;
        float middle_y = currOctant.middle_y;
        float middle_z = currOctant.middle_z;

        if (childTrees[pos]->root == nullptr)
        {
            // encountered an internal node meaning that the point is not in the outer tree
            childTrees[pos]->insert(vertex);
            return;
        }
        else if (childTrees[pos]->root->x == -1 && childTrees[pos]->root->y == -1 && childTrees[pos]->root->z == -1)
        {
            // encountered a leaf node that has not been initialized so create a new tree for it with the vertex as the root
            delete childTrees[pos];
            childTrees[pos] = new PointcloudTreee(vertex);
            return;
        }
        else
        {
            // int x_ = childTrees[pos]->root->x,
            //     y_ = childTrees[pos]->root->y,
            //     z_ = childTrees[pos]->point->z;
            Vertex *oldVertex = childTrees[pos]->root;
            delete childTrees[pos];
            childTrees[pos] = nullptr;
            if (pos == TLF)
            {
                childTrees[pos] = new PointcloudTreee(topLeftFront, new Vertex(middle_x, middle_y, middle_z));
            }
            else if (pos == TRF)
            {
                childTrees[pos] = new PointcloudTreee(new Vertex(middle_x + 1, topLeftFront->y, topLeftFront->z),
                                                      new Vertex(bottomRightBack->x, middle_y, middle_z));
            }
            else if (pos == BRF)
            {
                childTrees[pos] = new PointcloudTreee(new Vertex(middle_x + 1, middle_y + 1, topLeftFront->z),
                                                      new Vertex(bottomRightBack->x, bottomRightBack->y, middle_z));
            }
            else if (pos == BLF)
            {
                childTrees[pos] = new PointcloudTreee(new Vertex(topLeftFront->x, middle_y + 1, topLeftFront->z),
                                                      new Vertex(middle_x, bottomRightBack->y, middle_z));
            }
            else if (pos == TLB)
            {
                childTrees[pos] = new PointcloudTreee(new Vertex(topLeftFront->x, topLeftFront->y, middle_z + 1),
                                                      new Vertex(middle_x, middle_y, bottomRightBack->z));
            }
            else if (pos == TRB)
            {
                childTrees[pos] = new PointcloudTreee(new Vertex(middle_x + 1, topLeftFront->y, middle_z + 1),
                                                      new Vertex(bottomRightBack->x, middle_y, bottomRightBack->z));
            }
            else if (pos == BRB)
            {
                childTrees[pos] = new PointcloudTreee(new Vertex(middle_x + 1, middle_y + 1, middle_z + 1),
                                                      new Vertex(bottomRightBack->x, bottomRightBack->y, bottomRightBack->z));
            }
            else if (pos == BLB)
            {
                childTrees[pos] = new PointcloudTreee(new Vertex(topLeftFront->x, middle_y + 1, middle_z + 1),
                                                      new Vertex(middle_x, bottomRightBack->y, bottomRightBack->z));
            }
            childTrees[pos]->insert(oldVertex);
            childTrees[pos]->insert(vertex);
        }
    }

    Vertex *find(Vertex *vertex)
    {
        if (root == nullptr)
        {
            return false;
        }

        Octant currOctant = getOctant(vertex);
        int pos = currOctant.pos;

        if (childTrees[pos]->root == nullptr)
        {
            return childTrees[pos]->find(vertex);
        }
        else if (childTrees[pos]->root->x == -1 && childTrees[pos]->root->y == -1 && childTrees[pos]->root->z == -1)
        {
            return nullptr;
        }
        else
        {
            return childTrees[pos]->root;
        }
    }
}