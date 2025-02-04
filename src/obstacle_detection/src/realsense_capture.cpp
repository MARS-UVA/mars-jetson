#include <iostream>
#include "realsense_capture.h"
#include <filesystem>
namespace fs = std::filesystem;

void save_to_ply(const std::vector<Vertex> &vertices, const std::string &filename)
{
    try
    {
        fs::path filepath = fs::path("../../../test/assets") / filename;
        std::ofstream out(filepath);
        // just creating the ply file according to it format: https://fileinfo.com/extension/ply
        out << "ply\n";
        out << "format ascii 1.0\n";
        out << "element vertex " << vertices.size() << "\n";
        out << "property float x\n";
        out << "property float y\n";
        out << "property float z\n";
        out << "end_header\n";

        for (const auto &vertex : vertices)
        {
            out << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
        }
        out.close();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

void save_matrices_to_txt(const std::vector<std::vector<float>> &heights,
                          const std::vector<std::vector<Coordinate>> &actualCoordinates,
                          const std::string &filename)
{
    fs::path filepath = fs::path("../../../test/assets/matrix_benchmarks") / filename;
    std::ofstream out(filepath);

    out << heights.size() << " " << heights[0].size() << "\n";

    for (size_t i = 0; i < heights.size(); i++)
    {
        for (size_t j = 0; j < heights[i].size(); j++)
        {
            float height = heights[i][j];
            const Coordinate &coord = actualCoordinates[i][j];

            if (coord.valid)
            {
                out << height << " " << coord.x << " " << coord.y << "\n";
            }
            else
            {
                out << height << " inf inf\n";
            }
        }
    }
    out.close();
}

std::shared_ptr<Matrices> load_matrices_from_txt(const std::string &filename)
{
    std::ifstream in(filename);
    if (!in.is_open())
    {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return nullptr;
    }

    size_t rows, cols;
    in >> rows >> cols;

    auto matrices = std::make_shared<Matrices>();
    matrices->heights = std::vector<std::vector<float>>(rows, std::vector<float>(cols));
    matrices->actualCoordinates = std::vector<std::vector<Coordinate>>(rows, std::vector<Coordinate>(cols));

    float height, x, y;
    for (size_t i = 0; i < rows; i++)
    {
        for (size_t j = 0; j < cols; j++)
        {
            in >> height >> x >> y;
            matrices->heights[i][j] = height;

            if (x == std::numeric_limits<float>::infinity() &&
                y == std::numeric_limits<float>::infinity())
            {
                matrices->actualCoordinates[i][j] = Coordinate();
            }
            else
            {
                matrices->actualCoordinates[i][j] = Coordinate(x, y);
            }
        }
    }

    in.close();
    return matrices;
}

std::shared_ptr<Matrices> capture_depth_matrix(std::optional<std::vector<Vertex> *> &vertices, int decimationKernelSize)
{
    // std::vector<Vertex> verticesWithCommonCoordinates;
    rs2::pipeline pipe;
    rs2::config cfg;

    std::vector<std::vector<float>> heights;
    std::vector<std::vector<Coordinate>> actualCoordinates;

    try
    {
        pipe.start();

        std::cout << "Waiting for frames...\n";
        for (int i = 0; i < 30; i++)
        {
            pipe.wait_for_frames(); // Warmup
        }

        rs2::decimation_filter decimation;
        int decimation_magnitude = decimationKernelSize;
        decimation.set_option(RS2_OPTION_FILTER_MAGNITUDE, decimation_magnitude);
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth = decimation.process(frames.get_depth_frame());
        // rs2::depth_frame decimated_depth = decimation.process(depth);
        rs2::frame color = frames.get_color_frame();

        auto stream = depth.get_profile().as<rs2::video_stream_profile>();
        auto intrinsics = stream.get_intrinsics();

        // std::vector<Vertex> vertices;
        // std::vector<std::vector<float>> heights = std::vector<std::vector<float>>(depth.get_height(), std::vector<float>(depth.get_width(), 0.0));

        // float REALSENSE_CAM_ANGLE = 2 * M_PI + ((-45.0 * M_PI) / 180.0); // -45 degree angle with respect to the horizontal plane
        // float ANGLE_ROTATION = REALSENSE_CAM_ANGLE - M_PI / 2;
        // std::cout << "Angle of rotation: " << ANGLE_ROTATION << std::endl;
        int num_vertices = 0;
        heights = std::vector<std::vector<float>>(depth.get_height(), std::vector<float>(depth.get_width(), 0.0));
        actualCoordinates = std::vector<std::vector<Coordinate>>(depth.get_height(), std::vector<Coordinate>(depth.get_width(), Coordinate()));
        std::vector<Vertex> *verticesRef = vertices.value_or(nullptr);

        for (int y = 0; y < depth.get_height(); y++)
        {
            for (int x = 0; x < depth.get_width(); x++)
            {
                float pixel_depth = depth.get_distance(x, y);

                if (pixel_depth > 0)
                {
                    float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
                    float point[3];

                    // Deproject from pixel to 3D points using the depth value
                    rs2_deproject_pixel_to_point(point, &intrinsics, pixel, pixel_depth);
                    // float z = point[2] * sin(REALSENSE_CAM_ANGLE * M_PI / 180.0);
                    float angle = ((-180.0f + REALSENSE_ANGLE_FROM_HORIZONTAL) * M_PI) / 180.0f;
                    float y_rotated = point[1] * cos(angle) - point[2] * sin(angle);
                    float z_rotated = point[1] * sin(angle) + point[2] * cos(angle);

                    Vertex vertex(point[0], y_rotated, z_rotated);
                    Vertex vertexCommonCoor(x, y, z_rotated);

                    // verticesWithCommonCoordinates.push_back(vertexCommonCoor);
                    num_vertices++;
                    heights[y][x] = z_rotated;
                    actualCoordinates[y][x] = Coordinate(point[0], y_rotated);

                    if (verticesRef)
                    {
                        verticesRef->push_back(vertex);
                    }
                }
                else
                {
                    heights[y][x] = 0.0;
                    actualCoordinates[y][x] = Coordinate();
                }
            }
        }

        // save_to_ply(verticesWithCommonCoordinates, "out_common.ply");

        std::cout << "Number of vertices: " << num_vertices << std::endl;

        pipe.stop();
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n"
                  << e.what() << std::endl;
        return nullptr;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return nullptr;
    }

    std::shared_ptr<Matrices> matrices = std::make_shared<Matrices>();
    matrices->heights = heights;
    matrices->actualCoordinates = actualCoordinates;
    return matrices;
}