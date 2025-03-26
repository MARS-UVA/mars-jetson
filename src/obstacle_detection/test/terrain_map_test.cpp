#include <iostream>
#include "../src/gradient_map.h"
#include "../src/realsense_capture.h"
#include "../src/extract_capture.h"
#include <chrono>
#include <cassert>

int main()
{
    std::vector<Vertex> vertices;
    std::shared_ptr<Matrices> matricesDec0 = runMatrixCollector(vertices, 0);
    std::shared_ptr<Matrices> matricesDec1 = runMatrixCollector(vertices, 1);
    std::shared_ptr<Matrices> matricesDec4 = runMatrixCollector(vertices, 4);
}