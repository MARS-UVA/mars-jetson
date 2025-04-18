#include "models/realsense.h"
#include "models/pc_adacency_tree.h"
#include "realsense_capture.h"
#include <vector>
#include "extract_capture.h"

std::shared_ptr<Matrices> runMatrixCollector(std::vector<Vertex> &vertices, int decimationKernelSize)
{
    rs2::pipeline pipe;
    std::optional<std::vector<Vertex> *> v = &vertices;
    std::shared_ptr<Matrices> matrices = capture_depth_matrix(v, decimationKernelSize, pipe);
    // save_to_ply(v.value(), "vertices_out.ply");
    // std::cout << "PLY file saved successfullyy!" << std::endl;
    return matrices;
}

void runPcTreeCollector(PointcloudTree *tree, std::vector<Vertex> &vertices, int decimationKernelSize)
{
    rs2::pipeline pipe;
    std::optional<std::vector<Vertex> *> v = &vertices;
    capture_depth_matrix(v, decimationKernelSize, pipe);
    std::cout << "Pointcloud tree created!" << std::endl;
}