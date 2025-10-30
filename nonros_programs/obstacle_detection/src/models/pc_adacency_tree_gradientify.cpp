#include "realsense.h"
#include "pc_adacency_tree.h"
#include <iostream>
#include <cmath>

std::vector<Vertex> PointcloudTree::gradientify()
{
    std::vector<Vertex> gradientVertices;
    root = new Node();
    for (int i = 0; i < 4; i++)
    {
        if (childTrees[i])
        {
            childTrees[i]->gradientify(gradientVertices);
            root->gradient += std::abs(childTrees[i]->root->gradient);
        }
    }
    std::cout << "Root gradient: " << root->gradient << std::endl;
    return gradientVertices;
}

// void PointcloudTree::gradientify(std::vector<Vertex> &gradientVertices)
// {
//     if (root)
//     {
//         float totalGradient = 0.0f;
//         // int pointCount = root->pos.size();

//         for (int i = 0; i < root->pos.size(); ++i)
//         {
//             Point &p1 = root->pos[i];
//             for (int j = i + 1; j < root->pos.size(); ++j)
//             {
//                 Point &p2 = root->pos[j];
//                 float dz = std::abs(p1.height - p2.height);
//                 float dy = std::abs(p1.y - p2.y);
//                 float dx = std::abs(p1.x - p2.x);

//                 float grad = std::sqrt(pow(dz / dx, 2) + pow(dz / dy, 2));
//                 totalGradient += grad;
//             }
//         }

//         // Calculate the center point of the current node
//         float centerX = 0.0f;
//         float centerY = 0.0f;

//         for (const auto &point : root->pos)
//         {
//             centerX += point.x;
//             centerY += point.y;
//         }

//         centerX /= root->pos.size();
//         centerY /= root->pos.size();

//         root->gradient = totalGradient / (root->pos.size() * (root->pos.size() - 1) / 2);
//         gradientVertices.push_back(Vertex(centerX, centerY, root->gradient));
//     }
//     else if (root == nullptr)
//     {
//         root = new Node();
//         for (int i = 0; i < 4; i++)
//         {
//             if (childTrees[i])
//             {
//                 childTrees[i]->gradientify(gradientVertices);
//                 root->gradient += std::abs(childTrees[i]->root->gradient);
//             }
//         }
//     }
// }

void PointcloudTree::gradientify(std::vector<Vertex> &gradientVertices)
{
    if (root && root->pos.size() > 0)
    {
        float totalGradient = 0.0f;
        int validGradientCount = 0;

        for (int i = 0; i < root->pos.size(); ++i)
        {
            Point &p1 = root->pos[i];
            for (int j = i + 1; j < root->pos.size(); ++j)
            {
                Point &p2 = root->pos[j];
                float dz = std::abs(p1.height - p2.height);
                float dy = std::abs(p1.y - p2.y);
                float dx = std::abs(p1.x - p2.x);

                // Only calculate gradient if we have valid denominators
                if (dx > 1e-6 || dy > 1e-6) // Small epsilon to avoid near-zero divisions
                {
                    float dx_grad = (dx > 1e-6) ? (dz / dx) : 0.0f;
                    float dy_grad = (dy > 1e-6) ? (dz / dy) : 0.0f;
                    float grad = std::sqrt(dx_grad * dx_grad + dy_grad * dy_grad);

                    // Only add if the gradient is finite and reasonable
                    if (std::isfinite(grad) && grad < 1000.0f) // Add reasonable upper bound
                    {
                        totalGradient += grad;
                        validGradientCount++;
                    }
                }
            }
        }

        // Calculate the center point of the current node
        float centerX = 0.0f;
        float centerY = 0.0f;

        for (const auto &point : root->pos)
        {
            centerX += point.x;
            centerY += point.y;
        }

        centerX /= root->pos.size();
        centerY /= root->pos.size();

        // Only set gradient if we have valid calculations
        if (validGradientCount > 0)
        {
            root->gradient = totalGradient / validGradientCount;
            gradientVertices.push_back(Vertex(centerX, centerY, root->gradient));
        }
        else
        {
            root->gradient = 0.0f; // Default value when no valid gradients
            gradientVertices.push_back(Vertex(centerX, centerY, 0.0f));
        }
    }
    else if (root == nullptr)
    {
        root = new Node();
        float totalChildGradient = 0.0f;
        int validChildCount = 0;

        for (int i = 0; i < 4; i++)
        {
            if (childTrees[i])
            {
                childTrees[i]->gradientify(gradientVertices);
                float childGradient = childTrees[i]->root->gradient;
                if (std::isfinite(childGradient))
                {
                    totalChildGradient += std::abs(childGradient);
                    validChildCount++;
                }
            }
        }

        root->gradient = (validChildCount > 0) ? (totalChildGradient / validChildCount) : 0.0f;
    }
}
