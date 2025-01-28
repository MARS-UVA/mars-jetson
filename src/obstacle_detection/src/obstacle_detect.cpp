#include "gradient_map.h"
#include "obstacle_detect.h"

std::vector<std::vector<float>> processGradients(const std::vector<std::vector<float>> &heights,
                                                 const std::vector<std::vector<Coordinate>> &actualCoordinates)
{
    std::vector<Vertex> obstacleVertices;
    std::vector<std::vector<float>> gradientMap = ParallelGradientCalculator::calculateGradientsParallel(heights, actualCoordinates, 4, obstacleVertices);
    save_to_ply(obstacleVertices, "obstacle_vertices.ply");
}