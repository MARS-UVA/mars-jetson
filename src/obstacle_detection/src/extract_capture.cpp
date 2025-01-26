#include "models/realsense.h"
#include "models/pc_adacency_tree.h"
#include "realsense_capture.h"
#include <vector>

int main(int argc, char *argv[])
{
    char capture_format = 0;
    if (argc > 1)
    {
        capture_format = std::stoi(argv[1]);
    }
    std::vector<Vertex> vertices;
    if (capture_format)
    {
        capture_depth_matrix(nullptr, vertices);
        save_to_ply(vertices, "out.ply");
        std::cout << "PLY file saved successfullyy!" << std::endl;
    }
    else
    {
        PointcloudTree *tree = new PointcloudTree(Point(-5, 10), Point(5, 0));
        capture_depth_matrix(tree, vertices);
        std::cout << "Pointcloud tree created!" << std::endl;
        delete tree;
    }
    std::cout << "Finished!" << std::endl;
}