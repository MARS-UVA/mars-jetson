#include "local_path_planner_graph.h"
#include <utility>
#include <vector>
#include <queue>
#include <map>

float AStarPathPlanner::hValue(float x, float y)
{
    return sqrt(pow(x - goal.x, 2) + pow(y - goal.y, 2));
}

struct CompareCoordinate
{
    bool operator()(std::pair<int, float> const &p1, std::pair<int, float> const &p2)
    {
        return p1.second > p2.second;
    }
};

std::vector<Vertex> AStarPathPlanner::retracePath(PathPoint *current)
{
    std::vector<Vertex> path;
    while (current != nullptr)
    {
        path.push_back(Vertex(current->coordinate->x, current->coordinate->y, 0));
        current = current->parent;
    }
    return path;
}

bool AStarPathPlanner::isDestination(PathPoint *current)
{
    return fabs(current->coordinate->x - goal.x) <= 0.1 && fabs(current->coordinate->y - goal.y) <= 0.1;
}

std::vector<Vertex> AStarPathPlanner::planPath(std::vector<std::vector<Coordinate>> &actualCoordinates,
                                               Vertex &start,
                                               std::pair<int, int> &startIndices)
{
    std::cout << "\n=== Debug Information ===\n";
    std::cout << "Matrix dimensions: " << actualCoordinates.size() << "x" << actualCoordinates[0].size() << std::endl;
    std::cout << "Start indices: (" << startIndices.first << "," << startIndices.second << ")" << std::endl;
    std::cout << "Start coordinates: (" << start.x << "," << start.y << ")" << std::endl;
    std::cout << "Goal coordinates: (" << goal.x << "," << goal.y << ")" << std::endl;

    constexpr float INVALID_POINT_PENALTY = 5.0f;       // Penalty for invalid points
    constexpr float OBSTACLE_PROXIMITY_PENALTY = 10.0f; // Penalty for being near obstacles

    costArray.clear();
    costArray.resize(actualCoordinates.size() * actualCoordinates[0].size(), nullptr);

    std::priority_queue<std::pair<int, float>,
                        std::vector<std::pair<int, float>>,
                        CompareCoordinate>
        openSet;

    int startIndex = startIndices.first * actualCoordinates[0].size() + startIndices.second;

    if (startIndices.first >= actualCoordinates.size() ||
        startIndices.second >= actualCoordinates[0].size())
    {
        std::cout << "Error: Start indices out of bounds!" << std::endl;
        return std::vector<Vertex>();
    }

    Coordinate *startCoord = &actualCoordinates[startIndices.first][startIndices.second];
    std::cout << "Start coordinate valid: " << (startCoord->valid ? "true" : "false") << std::endl;
    std::cout << "Start coordinate actual position: (" << startCoord->x << "," << startCoord->y << ")" << std::endl;

    float startCost = startCoord->valid ? 0.0f : INVALID_POINT_PENALTY;
    PathPoint *startNode = new PathPoint(nullptr,
                                         startCoord,
                                         startIndices.first,
                                         startIndices.second,
                                         startCost,
                                         startCost);

    costArray[startIndex] = startNode;
    openSet.push({startIndex, startCost});
    std::cout << "Added start node to openSet. OpenSet size: " << openSet.size() << std::endl;

    std::unordered_map<int, bool> closedSet;

    int iterations = 0;
    while (!openSet.empty())
    {
        iterations++;

        int currentIndex = openSet.top().first;
        openSet.pop();

        PathPoint *current = costArray[currentIndex];
        if (!current)
        {
            std::cout << "Error: Null current point at index " << currentIndex << std::endl;
            continue;
        }

        // std::cout << "\nIteration " << iterations << std::endl;
        // std::cout << "Current position: (" << current->coordinate->x << ","
        //           << current->coordinate->y << ")" << std::endl;

        if (isDestination(current))
        {
            // Clean up dynamically allocated memory
            std::cout << "Path found after " << iterations << " iterations!" << std::endl;
            std::vector<Vertex> path = retracePath(current);
            for (auto &point : costArray)
            {
                delete point;
            }
            std::cout << "Final OpenSet size: " << openSet.size() << std::endl;
            std::cout << "path size: " << path.size() << std::endl;
            return path;
        }

        if (closedSet.find(currentIndex) != closedSet.end())
        {
            // std::cout << "Node already in closedSet, skipping" << std::endl;
            continue;
        }

        closedSet[currentIndex] = true;

        int neighborsFound = 0;
// Generate neighbors (8-connected grid)
#pragma omp parallel for collapse(2)
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dx = -1; dx <= 1; dx++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                int newRow = current->indices.first + dy;
                int newCol = current->indices.second + dx;

                // Check bounds
                if (newRow < 0 || newRow >= actualCoordinates.size() ||
                    newCol < 0 || newCol >= actualCoordinates[0].size())
                {
                    continue;
                }

                Coordinate *nextCoord = &actualCoordinates[newRow][newCol];
                float moveCost = 0.0; //  = (abs(dx) + abs(dy) == 2) ? 1.414f : 1.0f

                // Add penalty for invalid points instead of skipping
                if (!nextCoord->valid)
                {
                    moveCost += INVALID_POINT_PENALTY;
                    // std::cout << "Invalid point found at (" << nextCoord->x << ","
                    //   << nextCoord->y << "), adding penalty" << std::endl;
                }

                // Add penalty for obstacle proximity instead of skipping
                auto obstacle = obstacleTree.findNearestObstacle(
                    Vertex(nextCoord->x, nextCoord->y, 0));
                // std::cout << "Obstacle found: " << (obstacle ? "true" : "false") << std::endl;
                if (obstacle)
                {
                    float obstacle_dist = sqrt(
                        pow(obstacle->getVertex().x - nextCoord->x, 2) +
                        pow(obstacle->getVertex().y - nextCoord->y, 2));
                    if (obstacle_dist < 0.1)
                    {
                        // moveCost += OBSTACLE_PROXIMITY_PENALTY * (0.2f - obstacle_dist);
                        continue;
                    }
                    // std::cout << "Obstacle proximity penalty: " << moveCost << std::endl;
                }

                float localCost = current->costs.first;
                if (nextCoord->valid)
                {
                    localCost += sqrt(pow(nextCoord->x - current->coordinate->x, 2) + pow(nextCoord->y - current->coordinate->y, 2));
                }
                else
                {
                    localCost += 1.0f;
                }
                float newCost = localCost + moveCost;
                int neighborIndex = newRow * actualCoordinates[0].size() + newCol;
                PathPoint *neighbor = costArray[neighborIndex];
                // std::cout << "Neighbor index: " << neighborIndex << std::endl;
                if (!neighbor)
                {
                    neighborsFound++;
                    // std::cout << "Neighbor local cost updated to " << localCost << std::endl;
                    // std::cout << "Neighbor coordinate valid" << std::endl;
                    if (nextCoord->valid)
                    {
                        newCost += hValue(nextCoord->x, nextCoord->y);
                        // std::cout << "Neighbor cost: " << newCost << std::endl;
                    }
                    neighbor = new PathPoint(current, nextCoord, newRow, newCol,
                                             localCost, newCost);
                    costArray[neighborIndex] = neighbor;
                    openSet.push({neighborIndex, newCost});
                    // std::cout << "Added new neighbor at (" << nextCoord->x << ","
                    //           << nextCoord->y << ") with cost " << newCost << std::endl;
                }
                else if (newCost < neighbor->costs.first)
                {
                    neighborsFound++;
                    neighbor->parent = current;
                    neighbor->costs.first = localCost;
                    // std::cout << "Neighbor local cost updated to " << localCost << std::endl;
                    if (neighbor->coordinate->valid)
                    {
                        // std::cout << "Neighbor coordinate valid" << std::endl;
                        newCost += hValue(nextCoord->x, nextCoord->y);
                    }
                    // std::cout << "Neighbor cost: " << newCost << std::endl;
                    neighbor->costs.second = newCost;
                    openSet.push({neighborIndex, newCost});
                    // std::cout << "Updated neighbor at (" << nextCoord->x << ","
                    //           << nextCoord->y << ") with new cost " << newCost << std::endl;
                }
            }
        }

        // std::cout << "Found " << neighborsFound << " neighbors" << std::endl;
        // std::cout << "OpenSet size: " << openSet.size() << std::endl;
    }

    std::cout << "No path found after " << iterations << " iterations" << std::endl;
    std::cout << "Final OpenSet size: " << openSet.size() << std::endl;
    // Clean up dynamically allocated memory
    for (auto &point : costArray)
    {
        delete point;
    }
    return std::vector<Vertex>();
}

// std::vector<Vertex> AStarPathPlanner::planPath(std::vector<std::vector<Coordinate>> &actualCoordinates, Vertex &start, std::pair<int, int> &startIndices)
// {
//     costArray.clear();
//     costArray.resize(actualCoordinates.size() * actualCoordinates[0].size(), nullptr);

//     std::priority_queue<std::pair<int, float>,
//                         std::vector<std::pair<int, float>>,
//                         CompareCoordinate>
//         openSet;

//     int startIndex = startIndices.first * actualCoordinates[0].size() + startIndices.second;
//     float startHeuristic = hValue(start.x, start.y);
//     PathPoint *startNode = new PathPoint(nullptr,
//                                          &actualCoordinates[startIndices.first][startIndices.second],
//                                          startIndices.first,
//                                          startIndices.second,
//                                          0,
//                                          startHeuristic);

//     std::cout << "Start node: (" << startNode->coordinate->x << ", " << startNode->coordinate->y << ")" << std::endl;

//     costArray[startIndex] = startNode;
//     openSet.push({startIndex, startHeuristic});

//     std::unordered_map<int, bool> closedSet;

//     int iterations = 0;
//     const int MAX_ITERATIONS = 10000; // Prevent infinite loops

//     while (openSet.size() > 0)
//     {
//         iterations++;
//         int costArrayIndex = openSet.top().first;
//         openSet.pop();
//         PathPoint *parentPoint = costArray[costArrayIndex];
//         if (!parentPoint)
//         {
//             std::cout << "Error: Null point in costArray at index " << costArrayIndex << std::endl;
//             continue;
//         }
//         if (iterations % 1000 == 0)
//         {
//             std::cout << "Iteration " << iterations << ", current position: ("
//                       << parentPoint->coordinate->x << ", " << parentPoint->coordinate->y << ")" << std::endl;
//         }
//         if (parentPoint != nullptr && isDestination(parentPoint))
//         {
//             std::cout << "Path found" << std::endl;
//             return retracePath(parentPoint);
//             break;
//         }
//         // std::cout << "Past destination check" << std::endl;
//         std::vector<std::pair<int, int>> nextIndicesList;
//         std::pair<int, int> currentIndices = parentPoint->indices;
//         nextIndicesList.push_back({currentIndices.first - 1, currentIndices.second});     // Up
//         nextIndicesList.push_back({currentIndices.first + 1, currentIndices.second});     // Down
//         nextIndicesList.push_back({currentIndices.first, currentIndices.second - 1});     // Left
//         nextIndicesList.push_back({currentIndices.first, currentIndices.second + 1});     // Right
//         nextIndicesList.push_back({currentIndices.first - 1, currentIndices.second - 1}); // Up-Left
//         nextIndicesList.push_back({currentIndices.first - 1, currentIndices.second + 1}); // Up-Right
//         nextIndicesList.push_back({currentIndices.first + 1, currentIndices.second - 1}); // Down-Left
//         nextIndicesList.push_back({currentIndices.first + 1, currentIndices.second + 1}); // Down-Right
//         // std::cout << "Next indices list size: " << nextIndicesList.size() << std::endl;
//         for (auto index : nextIndicesList)
//         {
//             if (index.first >= 0 && index.first < actualCoordinates.size() && index.second >= 0 && index.second < actualCoordinates[0].size())
//             {
//                 Coordinate *nextCoordinate = &actualCoordinates[index.first][index.second];

//                 // Check if the next coordinate is an obstacle, if so, skip
//                 auto obstacle = obstacleTree.findNearestObstacle(Vertex(nextCoordinate->x, nextCoordinate->y, 0));
//                 if (obstacle != nullptr)
//                 {
//                     float obstacle_dist = sqrt(pow(obstacle->getVertex().x - nextCoordinate->x, 2) +
//                                                pow(obstacle->getVertex().y - nextCoordinate->y, 2));
//                     // Use both safety margin and minimum distance
//                     if (obstacle_dist < 0.5)
//                         continue;
//                 }
//                 std::cout << "Past obstacle check" << std::endl;

//                 // Check if the next coordinate is invalid, if so, skip
//                 // if (nextCoordinate->valid == false)
//                 // {
//                 //     std::cout << "Invalid coordinate at " << index.first << " " << index.second << std::endl;
//                 //     continue;
//                 // }
//                 // std::cout << "Past valid check" << std::endl;

//                 int indexForNewPoint = index.first * actualCoordinates[0].size() + index.second;
//                 // Check if the next coordinate is in the closed set, if so, skip --> this is to avoid rechecking the same point
//                 if (closedSet.find(indexForNewPoint) != closedSet.end())
//                 {
//                     continue;
//                 }
//                 // std::cout << "Past closed set check" << std::endl;
//                 if (costArray[indexForNewPoint] == nullptr)
//                 {
//                     std::cout << "Creating new point" << std::endl;
//                     float localCost = parentPoint->costs.first + 1;
//                     float globalCost;
//                     if (nextCoordinate->valid == false)
//                     {
//                         globalCost = std::numeric_limits<float>::max();
//                     }
//                     else
//                     {
//                         globalCost = hValue(nextCoordinate->x - parentPoint->coordinate->x, nextCoordinate->y - parentPoint->coordinate->y) + hValue(nextCoordinate->x - goal.x, nextCoordinate->y - goal.y);
//                     }
//                     PathPoint *nextPoint = new PathPoint(parentPoint, nextCoordinate, index.first, index.second, localCost, globalCost);
//                     costArray[indexForNewPoint] = nextPoint;
//                     openSet.push({indexForNewPoint, localCost});
//                 }
//                 if (costArray[indexForNewPoint]->costs.first > parentPoint->costs.first + 1)
//                 {
//                     std::cout << "Updating existing point" << std::endl;
//                     costArray[indexForNewPoint]->costs.first = parentPoint->costs.first + 1;
//                     costArray[indexForNewPoint]->parent = parentPoint;
//                     float globalCost;
//                     if (nextCoordinate->valid == false)
//                     {
//                         globalCost = std::numeric_limits<float>::max();
//                     }
//                     else
//                     {
//                         globalCost = hValue(nextCoordinate->x - parentPoint->coordinate->x, nextCoordinate->y - parentPoint->coordinate->y) + costArray[indexForNewPoint]->costs.second;
//                     }
//                     // Update the priority of the existing element in the priority queue
//                     openSet.push({indexForNewPoint, costArray[indexForNewPoint]->costs.first});
//                     std::cout << "Updated point" << std::endl;
//                 }
//             }
//             else
//             {
//                 std::cout << "Invalid index" << std::endl;
//             }
//         }
//         std::cout << "Past for loop" << std::endl;
//         closedSet.insert({costArrayIndex, true});
//     }
//     std::cout << iterations << " iterations" << std::endl;
//     std::cout << "No path found" << std::endl;
// }