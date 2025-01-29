#include <iostream>
#include "../src/models/pc_adacency_tree.h"
#include "../src/realsense_capture.h"

int main()
{
    std::vector<Vertex> vertices;
    PointcloudTree *tree = new PointcloudTree(Point(-5, 10), Point(5, 0));
    capture_depth_matrix(vertices);

    std::vector<Vertex> gradientVertices = tree->gradientify();
    std::cout << "Extracted " << gradientVertices.size() << " gradient vertices from tree." << std::endl;
    save_to_ply(gradientVertices, "gradient_tree_out.ply");

    tree->clear();
    delete tree;
    capture_depth_matrix(vertices);
    return 0;
}