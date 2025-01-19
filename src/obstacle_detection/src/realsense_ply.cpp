#include <librealsense2/rs.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
// #include <opencv2/opencv.hpp>
#define M_PI 3.14159265358979323846
#define REALSENSE_ANGLE_FROM_HORIZONTAL 40.0f

struct Vertex
{
    float x, y, z;
};

void save_to_ply(const std::vector<Vertex> &vertices, const std::string &filename)
{
    std::ofstream out(filename);
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

int main()
{
    rs2::pipeline pipe;
    rs2::config cfg;

    try
    {
        pipe.start();

        std::cout << "Waiting for frames...\n";
        for (int i = 0; i < 30; i++)
        {
            pipe.wait_for_frames(); // Warmup
        }
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::frame color = frames.get_color_frame();

        auto stream = depth.get_profile().as<rs2::video_stream_profile>();
        auto intrinsics = stream.get_intrinsics();

        std::vector<Vertex> vertices;

        // float REALSENSE_CAM_ANGLE = 2 * M_PI + ((-45.0 * M_PI) / 180.0); // -45 degree angle with respect to the horizontal plane
        // float ANGLE_ROTATION = REALSENSE_CAM_ANGLE - M_PI / 2;
        // std::cout << "Angle of rotation: " << ANGLE_ROTATION << std::endl;
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

                    vertices.push_back({point[0], y_rotated, z_rotated});
                }
            }
        }

        save_to_ply(vertices, "out.ply");
        std::cout << "PLY file saved successfullyy!" << std::endl;
        std::cout << "Done!" << std::endl;

        pipe.stop();
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n"
                  << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}