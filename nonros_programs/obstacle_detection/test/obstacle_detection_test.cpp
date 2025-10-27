#include <iostream>
#include "../src/gradient_map.h"
#include "../src/realsense_capture.h"
#include "../src/extract_capture.h"
#include "../src/models/obstacle_clustering_tree.h"
#include <chrono>
#include <cassert>

int main()
{
    std::vector<Vertex> vertices;
    std::shared_ptr<Matrices> matrices = runMatrixCollector(vertices, 4);
    std::vector<Vertex> obstacleVertices;
    ObstacleClusteringTree obstacleTree(2);
    std::vector<std::vector<float>> gradients = ParallelGradientCalculator::calculateGradientsParallel(matrices->heights, matrices->actualCoordinates, 3, obstacleVertices, obstacleTree);
    std::vector<Vertex> gradientVertices;
    for (size_t i = 0; i < gradients.size(); i++)
    {
        for (size_t j = 0; j < gradients[i].size(); j++)
        {
            if (matrices->actualCoordinates[i][j].valid)
            {
                float x = matrices->actualCoordinates[i][j].x;
                float y = matrices->actualCoordinates[i][j].y;
                gradientVertices.push_back(Vertex(x, y, gradients[i][j]));
            }
        }
    }

    save_to_ply(vertices, "vertices.ply");
    save_to_ply(gradientVertices, "gradients.ply");
    save_to_ply(obstacleVertices, "obstacles.ply");

    Vertex v(-0.07, 0.5, -0.2455);
    auto obstacle = obstacleTree.findNearestObstacle(v);
    bool isNearObstacle = false;
    if (obstacle != nullptr)
    {
        if (sqrt(pow(obstacle->getVertex().x - (-0.07), 2) + pow(obstacle->getVertex().y - 0.5, 2)) < 0.05)
            isNearObstacle = true;
    }

    std::cout << "Is near obstacle: " << isNearObstacle << std::endl;

    v = Vertex(0.164, 0.614, 0);
    auto obstacle1 = obstacleTree.findNearestObstacle(v);
    isNearObstacle = false;
    if (obstacle1 != nullptr)
    {
        std::cout << "Obstacle 1: " << obstacle1->getVertex().x << ", " << obstacle1->getVertex().y << ", " << obstacle1->getVertex().z << std::endl;
        if (sqrt(pow(obstacle1->getVertex().x - (0.164), 2) + pow(obstacle1->getVertex().y - 0.614, 2)) < 0.1)
            isNearObstacle = true;
    }

    std::cout << "Is near obstacle: " << isNearObstacle << std::endl;

    v = Vertex(-0.494, 0.5, 0);
    auto obstacle2 = obstacleTree.findNearestObstacle(v);
    isNearObstacle = false;
    if (obstacle2 != nullptr)
    {
        if (sqrt(pow(obstacle2->getVertex().x - (-0.494), 2) + pow(obstacle2->getVertex().y - 0.5, 2)) < 0.1)
            isNearObstacle = true;
    }

    std::cout << "Is near obstacle: " << isNearObstacle << std::endl;

    // std::string filename = "../../../test/assets/matrix_benchmarks/matrix_benchmark_2025-01-30-21-15-38";
    // std::shared_ptr<Matrices> retMatrices = load_matrices_from_txt(filename + ".txt");

    // std::vector<Vertex> obstacleVertices2;
    // ObstacleClusteringTree obstacleTree2(2);
    // std::cout << "retMatrices->heights.size(): " << retMatrices->heights.size() << std::endl;
    // std::cout << "retMatrices->heights[0].size(): " << retMatrices->heights[0].size() << std::endl;
    // std::cout << "retMatrices->actualCoordinates.size(): " << retMatrices->actualCoordinates.size() << std::endl;
    // std::cout << "retMatrices->actualCoordinates[0].size(): " << retMatrices->actualCoordinates[0].size() << std::endl;
    // std::cout << "obstacleVertices2.size(): " << obstacleVertices2.size() << std::endl;
    // std::vector<std::vector<float>> gradients2 = ParallelGradientCalculator::calculateGradientsParallel(retMatrices->heights, retMatrices->actualCoordinates, 3, obstacleVertices2, obstacleTree2);
    // std::cout << "gradients2.size(): " << gradients2.size() << std::endl;
    // std::cout << "gradients2[0].size(): " << gradients2[0].size() << std::endl;

    // std::string test_cases = "../../../test/assets/matrix_benchmarks/test_points.case.txt";
    // std::ifstream in(test_cases);
    // if (!in.is_open())
    // {
    //     std::cerr << "Error: Could not open file " << test_cases << std::endl;
    //     return 1;
    // }
    // std::string line;
    // bool first = true;
    // while (std::getline(in, line))
    // {
    //     if (first)
    //     {
    //         first = false;
    //         continue;
    //     }
    //     float x, y;
    //     int expected;
    //     const char *lineChar = line.c_str();
    //     sscanf(lineChar, "%f, %f, %d", &x, &y, &expected);
    //     std::cout << "Testing point: " << x << ", " << y << std::endl;
    //     auto obstacle = obstacleTree2.findNearestObstacle(Vertex(x, y, -0.2455));
    //     bool isNearObstacle = false;
    //     if (obstacle != nullptr)
    //     {
    //         if (sqrt(pow(obstacle->getVertex().x - x, 2) + pow(obstacle->getVertex().y - y, 2)) < 0.05)
    //             isNearObstacle = true;
    //         // assert(isNearObstacle == expected);
    //         std::cout << "Is near point: " << obstacle->getVertex().x << ", " << obstacle->getVertex().y << ", " << obstacle->getVertex().z << " but is " << isNearObstacle << std::endl;
    //     }
    // }

    return 0;
}