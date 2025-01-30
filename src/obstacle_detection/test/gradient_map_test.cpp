#include <iostream>
#include "../src/gradient_map.h"
#include "../src/realsense_capture.h"
#include "../src/extract_capture.h"
#include <chrono>
#include <cassert>

int main()
{
    std::vector<Vertex> vertices;
    std::shared_ptr<Matrices> matrices = runMatrixCollector(vertices, 4);
    std::vector<Vertex> obstacleVertices;
    std::vector<std::vector<float>> gradients = ParallelGradientCalculator::calculateGradientsParallel(matrices->heights, matrices->actualCoordinates, 3, obstacleVertices);
    std::cout << "Gradint size: " << gradients.size() << std::endl;
    assert(gradients.size() == matrices->heights.size());

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

    /* Measure runtime of single thread :*/
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<float>> gradientsSingle = ParallelGradientCalculator::calculateGradientsParallel(matrices->heights, matrices->actualCoordinates, 1, obstacleVertices);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Single thread runtime: " << elapsed.count() << "s" << std::endl;
    assert(elapsed.count() > 0 && elapsed.count() < 1);

    start = std::chrono::high_resolution_clock::now();
    gradientsSingle = ParallelGradientCalculator::calculateGradientsParallel(matrices->heights, matrices->actualCoordinates, 2, obstacleVertices);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "2 threads runtime: " << elapsed.count() << "s" << std::endl;
    assert(elapsed.count() > 0 && elapsed.count() < 0.6);

    start = std::chrono::high_resolution_clock::now();
    gradientsSingle = ParallelGradientCalculator::calculateGradientsParallel(matrices->heights, matrices->actualCoordinates, 3, obstacleVertices);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "3 threads runtime: " << elapsed.count() << "s" << std::endl;
    assert(elapsed.count() > 0 && elapsed.count() < 0.35);

    start = std::chrono::high_resolution_clock::now();
    gradientsSingle = ParallelGradientCalculator::calculateGradientsParallel(matrices->heights, matrices->actualCoordinates, 4, obstacleVertices);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "4 threads runtime: " << elapsed.count() << "s" << std::endl;
    assert(elapsed.count() > 0 && elapsed.count() < 0.4);

    start = std::chrono::high_resolution_clock::now();
    gradientsSingle = ParallelGradientCalculator::calculateGradientsParallel(matrices->heights, matrices->actualCoordinates, 5, obstacleVertices);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "5 threads runtime: " << elapsed.count() << "s" << std::endl;
    assert(elapsed.count() > 0 && elapsed.count() < 0.4);

    start = std::chrono::high_resolution_clock::now();
    gradientsSingle = ParallelGradientCalculator::calculateGradientsParallel(matrices->heights, matrices->actualCoordinates, 6, obstacleVertices);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "6 threads runtime: " << elapsed.count() << "s" << std::endl;
    assert(elapsed.count() > 0 && elapsed.count() < 0.4);

    return 0;
}