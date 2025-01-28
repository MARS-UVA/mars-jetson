#ifndef GRADIENTMAP_H
#define GRADIENTMAP_H

#include <vector>
#include <cmath>
#include "models/realsense.h"
#include "realsense_capture.h"
#include "obstacle_detect.h"

#define OVERLAP 1

struct Tile
{
    int sRow, sCol;
    int rows, cols;
    std::vector<std::vector<float>> data;
    std::vector<std::vector<std::shared_ptr<Coordinate>>> actualCoors;
    float tileSum;
    float gradientSum;
    int numValidPoints;

    Tile(int sRow, int sCol, int r, int c)
        : sRow(sRow), sCol(sCol), rows(r), cols(c),
          data(r, std::vector<float>(c)), actualCoors(r, std::vector<std::shared_ptr<Coordinate>>(c)),
          tileSum(0.0f), numValidPoints(0), gradientSum(0.0f)
    {
    }
};

struct Stats
{
    float mean;
    float stdDev;
    float gradMean;
    float gradStdDev;
};

float magnitude(float x, float y);

class ParallelGradientCalculator
{
public:
    static std::vector<std::vector<float>> calculateGradientsParallel(
        const std::vector<std::vector<float>> &heights,
        const std::vector<std::vector<Coordinate>> &actualCoordinates,
        int numThreads, std::vector<Vertex> &obstacleVertices);

private:
    static Tile createTileWithOverlap(
        const std::vector<std::vector<float>> &heights,
        const std::vector<std::vector<Coordinate>> &actualCoordinates,
        int startRow, int startCol,
        int rows, int cols);

    static std::vector<std::vector<float>> calculateTileGradients(
        Tile &tile);

    static void copyTileResults(
        std::vector<std::vector<float>> &result,
        const std::vector<std::vector<float>> &tileGradients,
        const Tile &tile);

    static float calculatePartialX(
        const std::vector<std::vector<float>> &heights,
        const std::vector<std::vector<std::shared_ptr<Coordinate>>> &actualCoordinates,
        int i, int j, int cols);

    static float calculatePartialY(
        const std::vector<std::vector<float>> &heights,
        const std::vector<std::vector<std::shared_ptr<Coordinate>>> &actualCoordinates,
        int i, int j, int rows);
};

#endif