#include <iostream>
#include "../src/gradient_map.h"
#include "../src/realsense_capture.h"
#include "../src/extract_capture.h"

int main()
{
    std::vector<Vertex> vertices;
    std::shared_ptr<Matrices> matrices = runMatrixCollector(vertices);
    std::vector<std::vector<float>> gradients = ParallelGradientCalculator::calculateGradientsParallel(matrices->heights, matrices->actualCoordinates, 4);

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

    save_to_ply(gradientVertices, "gradient_map_out.ply");
    return 0;
}