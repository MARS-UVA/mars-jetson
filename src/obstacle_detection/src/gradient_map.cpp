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
#include <random>

std::mutex sumMutex;
std::mutex copyMutex;

std::random_device rd;

float magnitude(float x, float y)
{
  return std::sqrt((x * x) + (y * y));
}

std::vector<float> fit_plane(const Vertex &v1, const Vertex &v2, const Vertex &v3)
{
  float x1 = v1.x, y1 = v1.y, z1 = v1.z;
  float x2 = v2.x, y2 = v2.y, z2 = v2.z;
  float x3 = v3.x, y3 = v3.y, z3 = v3.z;

  // Normal vector components <a, b, c>
  float a = (y2 - y1) * (z3 - z1) - (y3 - y1) * (z2 - z1);
  float b = (x3 - x1) * (z2 - z1) - (x2 - x1) * (z3 - z1);
  float c = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);

  // Normalize the normal vector using cross product of normal vector with any point
  float d = -(a * x1 + b * y1 + c * z1);

  return {a, b, c, d};
}

float height_from_plane(const std::vector<float> &plane, const Vertex &v)
{
  // Calculates distance from plane, the expected behavior is that the plan fits points in the x,y plane and the height of points is subject to the condition of being an outlier
  float a = plane[0], b = plane[1], c = plane[2], d = plane[3];
  return std::abs(a * v.x + b * v.y + c * v.z + d) / std::sqrt(a * a + b * b + c * c);
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
  std::vector<std::vector<bool>> inliers(fullRows, std::vector<bool>(fullCols, false));

  // Create a mapping of valid points for more efficient random selection
  std::vector<std::pair<int, int>> validPoints;
  validPoints.reserve(actualCoordinates.size() * actualCoordinates[0].size());
#pragma omp parallel for collapse(2)
  for (int r = 0; r < fullRows; r++)
  {
    for (int c = 0; c < fullCols; c++)
    {
      if (c < actualCoordinates[r].size() && actualCoordinates[r][c].valid)
      {
#pragma omp critical
        {
          validPoints.push_back({r, c});
        }
      }
    }
  }

#pragma omp parallel for
  for (int i = 0; i < RANSAC_ITERATIONS; i++)
  {
    // Get a secure random seed for this thread
    unsigned int seed = rd() ^ omp_get_thread_num();
    std::mt19937 local_gen(seed);
    std::uniform_int_distribution<> local_dist(0, validPoints.size() - 1);

    // Randomly select 3 different valid points
    int idx1 = local_dist(local_gen);
    int idx2 = local_dist(local_gen);
    int idx3 = local_dist(local_gen);

    // Make sure we have 3 different points
    while (idx1 == idx2 || idx1 == idx3 || idx2 == idx3)
    {
      if (idx1 == idx2)
        idx2 = local_dist(local_gen);
      if (idx1 == idx3 || idx2 == idx3)
        idx3 = local_dist(local_gen);
    }

    auto [r1, c1] = validPoints[idx1];
    auto [r2, c2] = validPoints[idx2];
    auto [r3, c3] = validPoints[idx3];

    // Fit plane to these 3 points
    std::vector<float> plane = fit_plane(
        Vertex(actualCoordinates[r1][c1].x, actualCoordinates[r1][c1].y, heights[r1][c1]),
        Vertex(actualCoordinates[r2][c2].x, actualCoordinates[r2][c2].y, heights[r2][c2]),
        Vertex(actualCoordinates[r3][c3].x, actualCoordinates[r3][c3].y, heights[r3][c3]));

// Check all points against this plane
#pragma omp parallel for collapse(2)
    for (int row = 0; row < fullRows; row++)
    {
      for (int col = 0; col < fullCols; col++)
      {
        if (col < actualCoordinates[row].size() && actualCoordinates[row][col].valid)
        {
          float height = height_from_plane(plane,
                                           Vertex(actualCoordinates[row][col].x, actualCoordinates[row][col].y, heights[row][col]));

          if (height < RANSAC_THRESHOLD)
          {
#pragma omp critical
            {
              inliers[row][col] = true;
            }
          }
        }
      }
    }
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
        if (isGlobalOutlier || isGradientOutlier || !inliers[leftIndices.second][leftIndices.first])
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
        if (isGlobalOutlier || isGradientOutlier || !inliers[rightIndices.second][rightIndices.first])
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
