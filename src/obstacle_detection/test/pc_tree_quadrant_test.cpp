#include <iostream>
#include "../src/models/pc_adacency_tree.h"
#include "../src/realsense_capture.h"

int main()
{
    std::vector<Vertex> vertices;
    PointcloudTree *tree = new PointcloudTree(Point(-5, 10), Point(5, 0));
    capture_depth_matrix(tree, vertices);
    // In main or test function
    // tree->print(0);
    std::vector<std::vector<Vertex>> quadrantVertices;
    std::cout << "Extracting leaf nodes at depth 1..." << std::endl;
    tree->extractLeafNodesAtDepth(1, quadrantVertices);

    // Save each quadrant's vertices to a separate PLY
    for (size_t i = 0; i < quadrantVertices.size(); i++)
    {
        std::string filename = "quadrant_" + std::to_string(i) + ".ply";
        save_to_ply(quadrantVertices[i], filename);
        std::cout << "Saved " << quadrantVertices[i].size() << " vertices to " << filename << std::endl;
    }
    tree->clear();
    delete tree;
    capture_depth_matrix(nullptr, vertices);
    return 0;
}