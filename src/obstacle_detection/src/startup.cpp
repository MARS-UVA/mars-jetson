#include "extract_capture.h"

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
        runMatrixCollector(vertices);
    }
    else
    {
        PointcloudTree *tree = new PointcloudTree(Point(-5, 10), Point(5, 0));
        runPcTreeCollector(tree, vertices);
        delete tree;
    }
    std::cout << "Finished!" << std::endl;
}