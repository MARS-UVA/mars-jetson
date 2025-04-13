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
#include <numeric>
#include <omp.h>
#include <chrono>

std::mutex sumMutex;
std::mutex copyMutex;

float magnitude(float x, float y)
{
  return std::sqrt((x * x) + (y * y));
}

/* Depending on Autonomous travel decision, can use parallel processes or Jetson CUDA cores to create gradient map */
// heights is basically the depths in the depth matix but because he pointcloud has bn rotated about the X-axis, the depths are now the heights
std::vector<std::vector<float>> ParallelGradientCalculator::calculateGradientsParallel(
    const std::vector<std::vector<float>> &heights,
    const std::vector<std::vector<Coordinate>> &actualCoordinates,
    int numThreads, std::vector<Vertex> &obstacleVertices,
    ObstacleClusteringTree &obstacleTree)
{
  Stats globalStats;
  std::vector<Stats> localStats(numThreads);
  int fullRows = (int)heights.size();
  int fullCols = (int)heights[0].size();

  std::vector<std::vector<float>> result(fullRows, std::vector<float>(fullCols));

  int tileRows = (fullRows + numThreads - 1) / numThreads;
  std::vector<std::thread> threads;
  std::mutex resultMutex;

  // std::atomic<float> globalSum(0.0f);
  // std::atomic<float> globalSquareSum(0.0f);
  // std::atomic<float> globalGradSum(0.0f);
  // std::atomic<float> globalSquareGradSum(0.0f);
  // std::atomic<int> totalValidPoints(0);
  std::vector<float> localMeans(numThreads, 0.0f);
  std::vector<float> localSquareSums(numThreads, 0.0f);
  std::vector<float> localStdevs(numThreads, 0.0f);
  std::vector<float> localGradMeans(numThreads, 0.0f);
  std::vector<float> localGradSquareSums(numThreads, 0.0f);
  std::vector<int> localValidPoints(numThreads, 0);
  auto start = std::chrono::high_resolution_clock::now();
  for (int tileStart = 0; tileStart < fullRows; tileStart += tileRows)
  {
    threads.emplace_back([&, tileStart, n = tileStart / tileRows]()
                         {
                           int actualTileRows = std::min(tileRows, fullRows - tileStart);

                           Tile tile = createTileWithOverlap(
                               heights, actualCoordinates, tileStart, 0,
                               actualTileRows, fullCols);

                           auto tileGradients = calculateTileGradients(tile);

                           localMeans[n] = tile.tileMean;
                           localStdevs[n] = tile.tileStdDev;
                           localSquareSums[n] = tile.tileSquareSum;
                           localGradSquareSums[n] = tile.gradientSquareSum;
                           localValidPoints[n] = tile.numValidPoints;
                           localGradMeans[n] = tile.gradientSum / tile.numValidPoints;
                           copyTileResults(result, tileGradients, tile); });
  }

  for (auto &thread : threads)
  {
    thread.join();
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "Time taken to calculate gradients: " << elapsed.count() << "s" << std::endl;

  // -------- trying to find a way to optimize/take this piece out
  // Get mean and standard deviation for global
  globalStats.mean = std::accumulate(localMeans.begin(), localMeans.end(), 0.0f) / numThreads;
  globalStats.gradMean = std::accumulate(localGradMeans.begin(), localGradMeans.end(), 0.0f) / numThreads;
  int totalValidPoints = std::accumulate(localValidPoints.begin(), localValidPoints.end(), 0);
  globalStats.stdDev = static_cast<float>(std::sqrt((std::accumulate(localSquareSums.begin(), localSquareSums.end(), 0.0f) / totalValidPoints) - std::pow(globalStats.mean, 2)));
  globalStats.gradStdDev = static_cast<float>(std::sqrt((std::accumulate(localGradSquareSums.begin(), localGradSquareSums.end(), 0.0f) / totalValidPoints) - std::pow(globalStats.gradMean, 2)));

  // 2nd pass to identify outliers - need to refactor this to be more optimal
  std::cout << "Global mean: " << globalStats.mean << std::endl;
  std::cout << "Global std dev: " << globalStats.stdDev << std::endl;
  std::cout << "Global gradient mean: " << globalStats.gradMean << std::endl;
  std::cout << "Global gradient std dev: " << globalStats.gradStdDev << std::endl;

  std::pair<int, int> leftIndices(fullCols / 2, fullRows / 2);
  std::pair<int, int> rightIndices(leftIndices.first + 1, fullRows / 2);
// float localMean = 0.0f, localStdDev = 0.0f, isLocalOutlier = false, hasHighLocalVariance = false, isGlobalOutlier = false, isGradientOutlier = false;
#pragma omp parallel for collapse(2)
  while (leftIndices.second > 0 || rightIndices.second < fullRows)
  {
    while (leftIndices.first > 0 || rightIndices.first < fullCols)
    {
      if (leftIndices.second > 0 && leftIndices.first > 0 && actualCoordinates[leftIndices.second][leftIndices.first].valid)
      {
        float localMean = localStats[leftIndices.second / tileRows].mean;
        float localStdDev = localStats[leftIndices.second / tileRows].stdDev;
        bool isLocalOutlier = std::abs(heights[leftIndices.second][leftIndices.first] - localMean) > 2.5 * localStdDev;
        bool hasHighLocalVariance = localStdDev > globalStats.stdDev;
        bool isGlobalOutlier = std::abs(heights[leftIndices.second][leftIndices.first] - globalStats.mean) > 2.5 * globalStats.stdDev;
        bool isGradientOutlier = std::abs(result[leftIndices.second][leftIndices.first] - globalStats.gradMean) > 20 * globalStats.gradStdDev;
        if (isGlobalOutlier || isGradientOutlier)
        {
          Vertex partOfObstacle(actualCoordinates[leftIndices.second][leftIndices.first].x, actualCoordinates[leftIndices.second][leftIndices.first].y, heights[leftIndices.second][leftIndices.first]);
          obstacleVertices.push_back(partOfObstacle);
          obstacleTree.add(partOfObstacle);
        }
      }

      if (rightIndices.second < fullRows && rightIndices.first < fullCols && actualCoordinates[rightIndices.second][rightIndices.first].valid)
      {
        float localMean = localStats[rightIndices.second / tileRows].mean;
        float localStdDev = localStats[rightIndices.second / tileRows].stdDev;
        bool isLocalOutlier = std::abs(heights[rightIndices.second][rightIndices.first] - localMean) > 2.5 * localStdDev;
        bool hasHighLocalVariance = localStdDev > globalStats.stdDev;
        bool isGlobalOutlier = std::abs(heights[rightIndices.second][rightIndices.first] - globalStats.mean) > 2.5 * globalStats.stdDev;
        bool isGradientOutlier = std::abs(result[rightIndices.second][rightIndices.first] - globalStats.gradMean) > 20 * globalStats.gradStdDev;
        if (isGlobalOutlier || isGradientOutlier)
        {
          Vertex partOfObstacle(actualCoordinates[rightIndices.second][rightIndices.first].x, actualCoordinates[rightIndices.second][rightIndices.first].y, heights[rightIndices.second][rightIndices.first]);
          obstacleVertices.push_back(partOfObstacle);
          obstacleTree.add(partOfObstacle);
        }
      }

      leftIndices.first--;
      rightIndices.first++;
    }
    leftIndices.second--;
    rightIndices.second++;
    leftIndices.first = fullCols - 1;
    rightIndices.first = 0;
  }

  // #pragma omp parallel for collapse(2)
  //   for (int i = 0; i < fullRows; ++i)
  //   {
  //     for (int j = 0; j < fullCols; ++j)
  //     {
  //       if (actualCoordinates[i][j].valid)
  //       {
  //         float localMean = localStats[i / tileRows].mean;
  //         float localStdDev = localStats[i / tileRows].stdDev;
  //         bool isLocalOutlier = std::abs(heights[i][j] - localMean) > 2.5 * localStdDev;
  //         bool hasHighLocalVariance = localStdDev > globalStats.stdDev;
  //         bool isGlobalOutlier = std::abs(heights[i][j] - globalStats.mean) > 2.5 * globalStats.stdDev;
  //         bool isGradientOutlier = std::abs(result[i][j] - globalStats.gradMean) > 20 * globalStats.gradStdDev;
  //         if (isGlobalOutlier || isGradientOutlier)
  //         {
  //           Vertex partOfObstacle(actualCoordinates[i][j].x, actualCoordinates[i][j].y, heights[i][j]);
  //           obstacleVertices.push_back(partOfObstacle);
  //           obstacleTree.add(partOfObstacle);
  //         }
  //       }
  //     }
  //   }

  return result;
}

Tile ParallelGradientCalculator::createTileWithOverlap(
    const std::vector<std::vector<float>> &heights,
    const std::vector<std::vector<Coordinate>> &actualCoordinates,
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
      tile.actualCoors[i - overlapStartRow][j - overlapStartCol] = std::make_shared<Coordinate>(actualCoordinates[i][j]);
      if (actualCoordinates[i][j].valid)
      {
        tile.numValidPoints++;
      }
      tile.tileSquareSum += std::pow(heights[i][j], 2);
      tile.tileSum += heights[i][j];
    }
  }
  tile.tileMean = tile.tileSum / tile.numValidPoints;
  tile.tileStdDev = std::sqrt((tile.tileSquareSum / tile.numValidPoints) - std::pow(tile.tileMean, 2));
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
      tile.gradientSquareSum += std::pow(gradients[i][j], 2);
    }
  }
  // tile.gradientMean = tile.gradientSum / tile.numValidPoints;

  return gradients;
}

void ParallelGradientCalculator::copyTileResults(
    std::vector<std::vector<float>> &result,
    const std::vector<std::vector<float>> &tileGradients,
    const Tile &tile)
{
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
      return 0.0f;
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
      return 0.0f;
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
      return 0.0f;
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
      return 0.0f;
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
      return 0.0f;
    }
  }
}
