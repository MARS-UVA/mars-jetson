#include <librealsense2/rs.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include "models/realsense.h"
#include "realsense_capture.h"
#include "gradient_map.h"
#include <atomic>

std::mutex sumMutex;

float magnitude(float x, float y)
{
  return std::sqrt((x * x) + (y * y));
}

/* Depending on Autonomous travel decision, can use parallel processes or Jetson CUDA cores to create gradient map */
// heights is basically the depths in the depth matix but because he pointcloud has bn rotated about the X-axis, the depths are now the heights
std::vector<std::vector<float>> ParallelGradientCalculator::calculateGradientsParallel(
    const std::vector<std::vector<float>> &heights,
    const std::vector<std::vector<Coordinate>> &actualCoordinates,
    int numThreads, std::vector<Vertex> &obstacleVertices)
{
  Stats globalStats;
  std::vector<Stats> localStats(numThreads);
  int fullRows = heights.size();
  int fullCols = heights[0].size();

  // Initialize result matrix
  std::vector<std::vector<float>> result(fullRows, std::vector<float>(fullCols));

  // Calculate tile dimensions
  int tileRows = (fullRows + numThreads - 1) / numThreads;
  std::vector<std::thread> threads;
  std::mutex resultMutex;

  // Create and process tiles
  std::atomic<float> globalSum(0.0f);
  std::atomic<float> globalGradSum(0.0f);
  std::atomic<int> totalValidPoints(0);
  int n = 0;
  for (int tileStart = 0; tileStart < fullRows; tileStart += tileRows)
  {
    threads.emplace_back([&, tileStart, n]()
                         {
                // Calculate actual tile dimensions
                int actualTileRows = std::min(tileRows, fullRows - tileStart);
                
                // Create tile with overlap
                Tile tile = createTileWithOverlap(
                    heights, actualCoordinates, tileStart, 0, 
                    actualTileRows, fullCols
                );
                {
                  std::lock_guard<std::mutex> lock(sumMutex);
                  globalSum.store(globalSum.load() + tile.tileSum);
                  totalValidPoints.store(totalValidPoints.load() + tile.numValidPoints);
                }
                
                // Calculate gradients for tile
                auto tileGradients = calculateTileGradients(tile);
                {
                  std::lock_guard<std::mutex> lock(sumMutex);
                  globalGradSum.store(globalGradSum.load() + tile.gradientSum);
                }
                
                // Copy results back to main matrix, excluding overlap
                {
                  std::lock_guard<std::mutex> lock(resultMutex);
                  copyTileResults(result, tileGradients, tile);
                }
                
                // Calculate local stats
                localStats[n].mean = tile.tileSum / tile.numValidPoints;
                localStats[n].stdDev = 0.0f;
                for (int i = 0; i < tile.rows; ++i)
                {
                  for (int j = 0; j < tile.cols; ++j)
                  {
                    if (tile.actualCoors[i][j]->valid)
                    {
                      localStats[n].stdDev += std::pow(tile.data[i][j] - localStats[n].mean, 2);
                    }
                  }
                }
                localStats[n].stdDev = std::sqrt(localStats[n].stdDev / tile.numValidPoints); });
    std::cout << "Thread " << n << " started" << std::endl;
    n++;
  }

  // Wait for all threads to complete
  for (auto &thread : threads)
  {
    thread.join();
  }

  // Get mean and standard deviation for global
  globalStats.mean = globalSum.load() / totalValidPoints.load();
  globalStats.gradMean = globalGradSum.load() / totalValidPoints.load();
  globalStats.stdDev = 0.0f;
  globalStats.gradStdDev = 0.0f;
  for (int i = 0; i < fullRows; ++i)
  {
    for (int j = 0; j < fullCols; ++j)
    {
      if (actualCoordinates[i][j].valid)
      {
        globalStats.stdDev += std::pow(heights[i][j] - globalStats.mean, 2);
        globalStats.gradStdDev += std::pow(result[i][j] - globalStats.gradMean, 2);
      }
    }
  }
  globalStats.stdDev = std::sqrt(globalStats.stdDev / totalValidPoints.load());

  // 2nd pass to identify outliers - need to refactor this to be more optimal
  for (int i = 0; i < fullRows; ++i)
  {
    for (int j = 0; j < fullCols; ++j)
    {
      if (actualCoordinates[i][j].valid)
      {
        float localMean = localStats[i / tileRows].mean;
        float localStdDev = localStats[i / tileRows].stdDev;
        bool isLocalOutlier = std::abs(heights[i][j] - localMean) > 1.5 * localStdDev;
        bool hasHighLocalVariance = localStdDev > 1.5 * globalStats.stdDev;
        bool isGlobalOutlier = std::abs(heights[i][j] - globalStats.mean) > globalStats.stdDev;
        bool isGradientOutlier = std::abs(result[i][j] - globalStats.gradMean) > 1.5 * globalStats.gradStdDev;
        if (((isLocalOutlier || hasHighLocalVariance) && isGlobalOutlier) || isGradientOutlier)
        {
          obstacleVertices.push_back(Vertex(actualCoordinates[i][j].x, actualCoordinates[i][j].y, heights[i][j]));
        }
      }
    }
  }

  return result;
}

Tile ParallelGradientCalculator::createTileWithOverlap(
    const std::vector<std::vector<float>> &heights,
    const std::vector<std::vector<Coordinate>> &actualCoordinates,
    int startRow, int startCol,
    int rows, int cols)
{
  // Calculate tile dimensions with overlap
  int overlapStartRow = std::max(0, startRow - OVERLAP);
  int overlapStartCol = std::max(0, startCol - OVERLAP);
  int overlapEndRow = std::min((int)heights.size(), startRow + rows + OVERLAP);
  int overlapEndCol = std::min((int)heights[0].size(), startCol + cols + OVERLAP);

  Tile tile(startRow, startCol,
            overlapEndRow - overlapStartRow,
            overlapEndCol - overlapStartCol);

  // Copy data including overlap
  for (int i = overlapStartRow; i < overlapEndRow; ++i)
  {
    for (int j = overlapStartCol; j < overlapEndCol; ++j)
    {
      tile.data[i - overlapStartRow][j - overlapStartCol] = heights[i][j];
      tile.actualCoors[i - overlapStartRow][j - overlapStartCol] = std::make_shared<Coordinate>(actualCoordinates[i][j]);
      if (actualCoordinates[i][j].valid)
      {
        tile.numValidPoints++;
      }
      tile.tileSum += heights[i][j];
    }
  }

  return tile;
}

std::vector<std::vector<float>> ParallelGradientCalculator::calculateTileGradients(
    Tile &tile)
{
  std::vector<std::vector<float>> gradients(
      tile.rows, std::vector<float>(tile.cols));

  for (int i = 0; i < tile.rows; ++i)
  {
    for (int j = 0; j < tile.cols; ++j)
    {
      double dzdx = calculatePartialX(tile.data, tile.actualCoors, i, j, tile.cols);
      double dzdy = calculatePartialY(tile.data, tile.actualCoors, i, j, tile.rows);
      gradients[i][j] = magnitude(dzdx, dzdy);
      tile.gradientSum += gradients[i][j];
    }
  }

  return gradients;
}

void ParallelGradientCalculator::copyTileResults(
    std::vector<std::vector<float>> &result,
    const std::vector<std::vector<float>> &tileGradients,
    const Tile &tile)
{
  // Copy excluding overlap regions
  for (int i = OVERLAP; i < tile.rows - OVERLAP; ++i)
  {
    for (int j = OVERLAP; j < tile.cols - OVERLAP; ++j)
    {
      int globalRow = tile.sRow + (i - OVERLAP);
      int globalCol = tile.sCol + (j - OVERLAP);
      if (globalRow < result.size() && globalCol < result[0].size())
      {
        result[globalRow][globalCol] = tileGradients[i][j];
      }
    }
  }
}

float ParallelGradientCalculator::calculatePartialX(
    const std::vector<std::vector<float>> &heights,
    const std::vector<std::vector<std::shared_ptr<Coordinate>>> &actualCoordinates,
    int i, int j, int cols)
{
  if (actualCoordinates[i][j]->valid == false)
  {
    return 0.0f;
  }
  if (j == 0)
  {
    int nextValidJ = j + 1;
    while (nextValidJ < cols && !actualCoordinates[i][nextValidJ]->valid)
    {
      nextValidJ++;
    }
    if (nextValidJ < cols)
    {
      return (heights[i][nextValidJ] - heights[i][j]) / (actualCoordinates[i][nextValidJ]->x - actualCoordinates[i][j]->x);
    }
    else
    {
      return 0.0f; // No valid next point found
    }
  }
  else if (j == cols - 1)
  {
    // return (heights[i][j] - heights[i][j - 1]) / (actualCoordinates[i][j]->x - actualCoordinates[i][j - 1]->x);
    int nextValidJ = j - 1;
    while (nextValidJ > 0 && !actualCoordinates[i][nextValidJ]->valid)
    {
      nextValidJ--;
    }
    if (nextValidJ < cols)
    {
      return (heights[i][nextValidJ] - heights[i][j]) / (actualCoordinates[i][nextValidJ]->x - actualCoordinates[i][j]->x);
    }
    else
    {
      return 0.0f; // No valid next point found
    }
  }
  else
  {
    int nextValidJPlus = j + 1;
    while (nextValidJPlus < cols && !actualCoordinates[i][nextValidJPlus]->valid)
    {
      nextValidJPlus++;
    }

    int nextValidJMinus = j - 1;
    while (nextValidJMinus >= 0 && !actualCoordinates[i][nextValidJMinus]->valid)
    {
      nextValidJMinus--;
    }

    if (nextValidJPlus < cols && nextValidJMinus >= 0)
    {
      return (heights[i][nextValidJPlus] - heights[i][nextValidJMinus]) / (actualCoordinates[i][nextValidJPlus]->x - actualCoordinates[i][nextValidJMinus]->x);
    }
    else if (nextValidJPlus < cols)
    {
      return (heights[i][nextValidJPlus] - heights[i][j]) / (actualCoordinates[i][nextValidJPlus]->x - actualCoordinates[i][j]->x);
    }
    else if (nextValidJMinus >= 0)
    {
      return (heights[i][nextValidJMinus] - heights[i][j]) / (actualCoordinates[i][nextValidJMinus]->x - actualCoordinates[i][j]->x);
    }
    else
    {
      return 0.0f; // No valid points found
    }
  }
}

float ParallelGradientCalculator::calculatePartialY(
    const std::vector<std::vector<float>> &heights,
    const std::vector<std::vector<std::shared_ptr<Coordinate>>> &actualCoordinates,
    int i, int j, int rows)
{
  if (actualCoordinates[i][j]->valid == false)
  {
    return 0.0f;
  }
  if (i == 0)
  {
    int nextValidI = i + 1;
    while (nextValidI < rows && !actualCoordinates[nextValidI][j]->valid)
    {
      nextValidI++;
    }
    if (nextValidI < rows)
    {
      return (heights[nextValidI][j] - heights[i][j]) / (actualCoordinates[nextValidI][j]->y - actualCoordinates[i][j]->y);
    }
    else
    {
      return 0.0f; // No valid next point found
    }
  }
  else if (i == rows - 1)
  {
    int nextValidI = i - 1;
    while (nextValidI > 0 && !actualCoordinates[nextValidI][j]->valid)
    {
      nextValidI--;
    }
    if (nextValidI < rows)
    {
      return (heights[nextValidI][j] - heights[i][j]) / (actualCoordinates[nextValidI][j]->y - actualCoordinates[i][j]->y);
    }
    else
    {
      return 0.0f; // No valid next point found
    }
  }
  else
  {
    int nextValidIPlus = i + 1;
    while (nextValidIPlus < rows && !actualCoordinates[nextValidIPlus][j]->valid)
    {
      nextValidIPlus++;
    }

    int nextValidIMinus = i - 1;
    while (nextValidIMinus >= 0 && !actualCoordinates[nextValidIMinus][j]->valid)
    {
      nextValidIMinus--;
    }

    if (nextValidIPlus < rows && nextValidIMinus >= 0)
    {
      return (heights[nextValidIPlus][j] - heights[nextValidIMinus][j]) / (actualCoordinates[nextValidIPlus][j]->y - actualCoordinates[nextValidIMinus][j]->y);
    }
    else if (nextValidIPlus < rows)
    {
      return (heights[nextValidIPlus][j] - heights[i][j]) / (actualCoordinates[nextValidIPlus][j]->y - actualCoordinates[i][j]->y);
    }
    else if (nextValidIMinus >= 0)
    {
      return (heights[nextValidIMinus][j] - heights[i][j]) / (actualCoordinates[nextValidIMinus][j]->y - actualCoordinates[i][j]->y);
    }
    else
    {
      return 0.0f; // No valid points found
    }
  }
}

// static float calculatePartialY(
//     const std::vector<std::vector<float>> &heights,
//     const std::vector<std::vector<std::shared_ptr<Coordinate>>> &actualCoordinates,
//     int i, int j, int rows)
// {
//   if (i == 0)
//   {
//     return (heights[i + 1][j] - heights[i][j]) / (actualCoordinates[i + 1][j]->y - actualCoordinates[i][j]->y);
//   }
//   else if (i == rows - 1)
//   {
//     return (heights[i][j] - heights[i - 1][j]) / (actualCoordinates[i][j]->y - actualCoordinates[i - 1][j]->y);
//   }
//   else
//   {
//     return (heights[i + 1][j] - heights[i - 1][j]) / (actualCoordinates[i + 1][j]->y - actualCoordinates[i - 1][j]->y);
//   }
// }
