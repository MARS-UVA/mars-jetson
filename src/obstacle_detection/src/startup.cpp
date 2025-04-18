#include "realsense_capture.h"
#include <ctime>
#include <string>
#include "models/obstacle_clustering_tree.h"
#include "gradient_map.h"
#include "local_path_planner_graph.h"
#include <chrono>

#define DECIMATION_KERNEL_SIZE 4

// bool reachedGoal(const RobotState &state, float goal_x, float goal_y)
// {
//     float distance = sqrt(pow(state.x - goal_x, 2) + pow(state.y - goal_y, 2));
//     return distance < 0.3f; // Or whatever threshold you consider "reached"
// }

/* arguments: 1 - capture depth matrix and use gradient matrix method
                2 - capture depth matrix and save matrices to txt file
                3 - capture depth matrix and save vertices to ply file
                4 - capture depth matrix and save matrices to txt file and vertices to ply file
                5 - save vertices, gradients and obstacles to ply files
                6 - save vertices, gradients and obstacles to ply files and save matrices to txt file
                0 - capture pointcloud tree
*/
int main(int argc, char *argv[])
{
    int capture_format = 0;
    if (argc > 1)
    {
        capture_format = std::stoi(argv[1]);
    }
    std::optional<std::vector<Vertex> *> vertices;
    vertices = new std::vector<Vertex>();

    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);

    pipe.start(cfg);

    for (int i = 0; i < 30; i++) {
        pipe.wait_for_frames();
    }

    if (capture_format == 1)
    {
        capture_depth_matrix(vertices, DECIMATION_KERNEL_SIZE, pipe);
    }
    else if (capture_format == 2)
    {
        std::shared_ptr<Matrices> retMatrices = capture_depth_matrix(vertices, DECIMATION_KERNEL_SIZE, pipe);
        time_t now = time(0);
        tm *localtm = localtime(&now);
        char buffer[80];
        strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", localtm);
        std::string filename = "matrix_benchmarks_" + std::string(buffer) + ".txt";
        save_matrices_to_txt(retMatrices->heights, retMatrices->actualCoordinates, filename);
    }
    else if (capture_format == 3)
    {
        capture_depth_matrix(vertices, DECIMATION_KERNEL_SIZE, pipe);
        time_t now = time(0);
        tm *localtm = localtime(&now);
        char buffer[80];
        strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", localtm);
        std::string filename = "vertices_" + std::string(buffer) + ".ply";
        if (vertices.has_value() && vertices.value() != nullptr)
        {
            save_to_ply(*(vertices.value()), filename);
        }
        else
        {
            std::cerr << "Error: vertices is either empty or null." << std::endl;
        }
    }
    else if (capture_format == 4)
    {
        std::shared_ptr<Matrices> retMatrices = capture_depth_matrix(vertices, DECIMATION_KERNEL_SIZE, pipe);
        time_t now = time(0);
        tm *localtm = localtime(&now);
        char buffer[80];
        strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", localtm);
        std::string filename_mat = "matrix_benchmark_" + std::string(buffer) + ".txt";
        save_matrices_to_txt(retMatrices->heights, retMatrices->actualCoordinates, filename_mat);
        std::string filename_vert = "vertices_" + std::string(buffer) + ".ply";
        if (vertices.has_value() && vertices.value() != nullptr)
        {
            save_to_ply(*(vertices.value()), filename_vert);
        }
        else
        {
            std::cerr << "Error: vertices is either empty or null." << std::endl;
        }
    }
    else if (capture_format == 5 || capture_format == 6)
    {
        std::shared_ptr<Matrices> retMatrices = capture_depth_matrix(vertices, DECIMATION_KERNEL_SIZE, pipe);
        std::vector<Vertex> obstacleVertices;
        ObstacleClusteringTree obstacleTree(2);
        std::vector<std::vector<float>> gradients = ParallelGradientCalculator::calculateGradientsParallel(retMatrices->heights, retMatrices->actualCoordinates, 3, obstacleVertices, obstacleTree);
        std::vector<Vertex> gradientVertices;
        for (size_t i = 0; i < gradients.size(); i++)
        {
            for (size_t j = 0; j < gradients[i].size(); j++)
            {
                if (retMatrices->actualCoordinates[i][j].valid)
                {
                    float x = retMatrices->actualCoordinates[i][j].x;
                    float y = retMatrices->actualCoordinates[i][j].y;
                    gradientVertices.push_back(Vertex(x, y, gradients[i][j]));
                }
            }
        }

        time_t now = time(0);
        tm *localtm = localtime(&now);
        char buffer[80];
        strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", localtm);
        std::string filename;
        filename = "vertices_" + std::string(buffer) + ".ply";
        if (vertices.has_value() && vertices.value() != nullptr)
        {
            save_to_ply(*(vertices.value()), filename);
        }
        else
        {
            std::cerr << "Error: vertices is either empty or null." << std::endl;
        }
        filename = "gradients_" + std::string(buffer) + ".ply";
        save_to_ply(gradientVertices, filename);
        filename = "obstacles_" + std::string(buffer) + ".ply";
        save_to_ply(obstacleVertices, filename);
        filename = "matrix_benchmark_" + std::string(buffer) + ".txt";
        save_matrices_to_txt(retMatrices->heights, retMatrices->actualCoordinates, filename);
        if (capture_format == 6)
        {
            std::cout << "Saving matrices to txt file..." << std::endl;
            save_matrices_to_txt(retMatrices->heights, retMatrices->actualCoordinates, filename);
        }
    }
    else if (capture_format == 7)
    {
        auto start = std::chrono::high_resolution_clock::now();
        std::shared_ptr<Matrices> retMatrices = capture_depth_matrix(vertices, DECIMATION_KERNEL_SIZE, pipe);
        std::vector<Vertex> obstacleVertices;
        std::string filename;
        filename = "vertices_python.ply";
        if (vertices.has_value() && vertices.value() != nullptr)
        {
            save_to_ply(*(vertices.value()), filename);
        }
        else
        {
            std::cerr << "Error: vertices is either empty or null." << std::endl;
        }
        ObstacleClusteringTree obstacleTree(2);
        std::vector<std::vector<float>> gradients = ParallelGradientCalculator::calculateGradientsParallel(retMatrices->heights, retMatrices->actualCoordinates, 3, obstacleVertices, obstacleTree);

        filename = "obstacles_python.ply";
        save_to_ply(obstacleVertices, filename);

        std::vector<Vertex> pointsOnPath;
        AStarPathPlanner navigator(obstacleTree, Vertex(-0.5, 1.5, 0));
        // Get some starting index in the graph and then get the vertex associated with it:
        std::pair<int, int> startIndices(1, 1);
        bool found = false;
        std::pair actualPoint = {0.0, 0.2};
        for (size_t i = 0; i < retMatrices->actualCoordinates.size(); ++i)
        {
            for (size_t j = 0; j < retMatrices->actualCoordinates[i].size(); ++j)
            {
                if (std::abs(retMatrices->actualCoordinates[i][j].x - actualPoint.first) < 0.01 && std::abs(retMatrices->actualCoordinates[i][j].y - actualPoint.second) < 0.01)
                {
                    startIndices = std::make_pair(i, j);
                    found = true;
                    break;
                }
            }
            if (found)
            {
                break;
            }
        }
        Vertex startVertex = Vertex(retMatrices->actualCoordinates[startIndices.first][startIndices.second].x, retMatrices->actualCoordinates[startIndices.first][startIndices.second].y, 0);
        pointsOnPath = navigator.planPath(retMatrices->actualCoordinates, startVertex, startIndices);
        std::cout << "Path length: " << pointsOnPath.size() << std::endl;
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - start);
        std::cout << "Time for robot to traverse a single frame: " << elapsed.count() << "s" << std::endl;

        filename = "path_python.ply";
        save_to_ply(pointsOnPath, filename);
    }
    if (vertices.has_value() && vertices.value() != nullptr)
    {
        delete vertices.value();
        vertices.reset();
    }
    pipe.stop();
}