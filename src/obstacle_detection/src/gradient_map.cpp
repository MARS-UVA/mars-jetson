#include <librealsense2/rs.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include "realsense.h"

#define OVERLAP 1

struct Tile
{
  int sRow, sCol;
  int rows, cols;
  std::vector<std::vector<double>> data;

  Tile(int sRow, int sCol, int r, int c)
      : sRow(sRow), sCol(sCol), rows(r), cols(c),
        data(r, std::vector<double>(c)) {}
};

/* Depending on Autonomous travel decision, can use parallel processes or Jetson CUDA cores to create gradient map */
// heights is basically the depths in the depth matix but because he pointcloud has bn rotated about the X-axis, the depths are now the heights
class ParallelGradientCalculator
{
public:
  static std::vector<std::vector<Vertex>> calculateGradientsParallel(
      const std::vector<std::vector<double>> &heights,
      int numThreads,
      double dx = 1.0,
      double dy = 1.0)
  {
    int fullRows = heights.size();
    int fullCols = heights[0].size();

    std::vector<std::vector<Vec3>> result(
        fullRows, std::vector<Vec3>(fullCols));
    int tileRows = (fullRows + numThreads - 1) / numThreads;
    std::vector<std::thread> threads;
    std::mutex resultMutex;

    for (int tileStart = 0; tileStart < fullRows; tileStart += tileRows)
    {
      threads.emplace_back([&, tileStart]() { // Notd: & is used to capture all variables by reference
        int actualTileRows = std::min(tileRows, fullRows - tileStart);

        Tile tile = createTileWithOverlap(
            heights, tileStart, 0,
            actualTileRows, fullCols);

        auto tileGradients = calculateTileGradients(tile, dx, dy);

        std::lock_guard<std::mutex> lock(resultMutex);
        copyTileResults(result, tileGradients, tile);
      });
    }
    for (auto &thread : threads)
    {
      thread.join();
    }

    return result;
  }

private:
  static Tile createTile(
      const std::vector<std::vector<double>> &heights,
      int startRow, int startCol,
      int rows, int cols)
  {
    int overlapStartRow = std::max(0, startRow - OVERLAP);
    int overlapStartCol = std::max(0, startCol - OVERLAP);
    int overlapEndRow = std::min((int)heights.size(), startRow + rows + OVERLAP);
    int overlapEndCol = std::min((int)heights[0].size(), startCol + cols + OVERLAP);

    Tile tile(startRow, startCol,
              overlapEndRow - overlapStartRow,
              overlapEndCol - overlapStartCol);

    for (int i = overlapStartRow; i < overlapEndRow; ++i)
    {
      for (int j = overlapStartCol; j < overlapEndCol; ++j)
      {
        tile.data[i - overlapStartRow][j - overlapStartCol] = heights[i][j];
      }
    }

    return tile;
  }

  static std::vector<std::vector<Vertex>> calculateTileGradients(
      const Tile &tile,
      double dx,
      double dy)
  {
    std::vector<std::vector<Vertex>> gradients(
        tile.rows, std::vector<Vec3>(tile.cols));

    for (int i = 0; i < tile.rows; ++i)
    {
      for (int j = 0; j < tile.cols; ++j)
      {
        double dzdx = calculatePartialX(tile.data, i, j, dx, tile.cols);
        double dzdy = calculatePartialY(tile.data, i, j, dy, tile.rows);
        gradients[i][j] = Vec3(dzdx, dzdy, 1.0);
      }
    }

    return gradients;
  }

  static void copyTileResults(
      std::vector<std::vector<Vertex>> &result,
      const std::vector<std::vector<Vertex>> &tileGradients,
      const Tile &tile)
  {
    for (int i = OVERLAP; i < tile.rows - OVERLAP; ++i)
    {
      for (int j = OVERLAP; j < tile.cols - OVERLAP; ++j)
      {
        int globalRow = tile.startRow + (i - OVERLAP);
        int globalCol = tile.startCol + (j - OVERLAP);
        if (globalRow < result.size() && globalCol < result[0].size())
        {
          result[globalRow][globalCol] = tileGradients[i][j];
        }
      }
    }
  }

  static double calculatePartialX(
      const std::vector<std::vector<double>> &heights,
      int i, int j, double dx, int cols)
  {
    if (j == 0)
    {
      return (heights[i][j + 1] - heights[i][j]) / dx;
    }
    else if (j == cols - 1)
    {
      return (heights[i][j] - heights[i][j - 1]) / dx;
    }
    else
    {
      return (heights[i][j + 1] - heights[i][j - 1]) / (2.0 * dx);
    }
  }

  static double calculatePartialY(
      const std::vector<std::vector<double>> &heights,
      int i, int j, double dy, int rows)
  {
    if (i == 0)
    {
      return (heights[i + 1][j] - heights[i][j]) / dy;
    }
    else if (i == rows - 1)
    {
      return (heights[i][j] - heights[i - 1][j]) / dy;
    }
    else
    {
      return (heights[i + 1][j] - heights[i - 1][j]) / (2.0 * dy);
    }
  }
};