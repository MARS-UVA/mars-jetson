#include "models/realsense.h"
#include "models/pc_adacency_tree.h"
#include "realsense_capture.h"
#include <vector>
#include "extract_capture.h"

std::shared_ptr<Matrices> runMatrixCollector(std::vector<Vertex> &vertices)
{
    std::shared_ptr<Matrices> matrices = capture_depth_matrix(nullptr, vertices);
    save_to_ply(vertices, "vertices_out.ply");
    std::cout << "PLY file saved successfullyy!" << std::endl;
    return matrices;
}

void runPcTreeCollector(PointcloudTree *tree, std::vector<Vertex> &vertices)
{
    capture_depth_matrix(tree, vertices);
    std::cout << "Pointcloud tree created!" << std::endl;
}