#include <iostream>
#include "realsense_capture.h"
#include <filesystem>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <opencv2/opencv.hpp>
#include "../../../server/main.hpp"
#include "../../../server/client.hpp"
namespace fs = std::filesystem;

static uint8_t *monoBuffer = nullptr;
static size_t bufferSize = 0;
// ImageWriter imgWriter;

// void sendPic(int width, int height) {
//     cv::Mat image(height, width, CV_8UC1, (void*)monoBuffer);
//     std::vector<uchar> compressed_buf;
//     std::vector<int> compression_params = {cv::IMWRITE_PNG_COMPRESSION, 3}; // 0 = fast, 9 = small

//     cv::imencode(".png", image, compressed_buf, compression_params);
//     std::cout << "Compressed size: " << compressed_buf.size() << " bytes" << std::endl;

//     // int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
//     ftruncate(shm_fd, sizeof(SharedImage));
//     void* shm_ptr = mmap(0, sizeof(SharedImage), PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd, 0);
//     auto* shared_img = static_cast<SharedImage*>(shm_ptr);
//     shared_img->data_size = compressed_buf.size();
//     std::memcpy(shared_img->data, compressed_buf.data(), compressed_buf.size());
//     shared_img->frame_id.fetch_add(1, std::memory_order_release);
// }

void save_pic(int width, int height)
{
    fs::path filename = fs::path("../../test/assets/pic.png");
    cv::Mat image(height, width, CV_8UC1, (void *)monoBuffer);
    cv::imwrite(filename.c_str(), image);
}

void save_to_ply(const std::vector<Vertex> &vertices, const std::string &filename)
{
    try
    {
        fs::path filepath = fs::path("../../test/assets") / filename;
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

void processColorFrame(rs2::frame &color)
{
    rs2::video_frame videoFrame = color.as<rs2::video_frame>();
    int width = videoFrame.get_width();
    std::cout << "RGB width: " << width << std::endl;
    int height = videoFrame.get_height();
    std::cout << "RGB height: " << height << std::endl;
    int bytes_per_pixel = videoFrame.get_bytes_per_pixel();
    std::cout << "Bytes per pixel: " << bytes_per_pixel << std::endl;
    size_t requiredSize = width * height;
    std::cout << "Buffer size: " << requiredSize << std::endl;

    if (!monoBuffer || bufferSize < requiredSize)
    {
        if (monoBuffer)
        {
            delete[] monoBuffer;
        }
        monoBuffer = new uint8_t[requiredSize];
        bufferSize = requiredSize;
    }

    const uint8_t *colorData = static_cast<const uint8_t *>(videoFrame.get_data());

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            int pixelIndex = i * width + j;
            int colorIndex = pixelIndex * bytes_per_pixel;

            uint8_t r = colorData[colorIndex];
            uint8_t g = colorData[colorIndex + 1];
            uint8_t b = colorData[colorIndex + 2];

            // 0.299 * r + 0.587 * g + 0.114 * b - grayscale formula
            monoBuffer[pixelIndex] = static_cast<uint8_t>(0.299 * r + 0.587 * g + 0.114 * b);
        }
    }
    // sendPic(width, height);
    // imgWriter.processImage(width, height, monoBuffer);
    cv::Mat image(height, width, CV_8UC1, (void *)monoBuffer);
    std::vector<uchar> compressed_buf;
    std::vector<int> compression_params = {cv::IMWRITE_PNG_COMPRESSION, 3};
    cv::imencode(".jpeg", image, compressed_buf, compression_params);
    cv::Mat compressedImg = cv::imdecode(compressed_buf, cv::IMREAD_GRAYSCALE);
    client_send(compressedImg, IMAGE_PORT);
}

std::shared_ptr<Matrices> capture_depth_matrix(std::optional<std::vector<Vertex> *> &vertices, int decimationKernelSize, rs2::pipeline &pipe)
{
    // rs2::pipeline pipe;
    // rs2::config cfg;

    std::vector<std::vector<float>> heights;
    std::vector<std::vector<Coordinate>> actualCoordinates;

    try
    {
        // pipe.start();

        // for (int i = 0; i < 30; i++)
        // {
        //     pipe.wait_for_frames();
        // }

        rs2::decimation_filter decimation;
        int decimation_magnitude = decimationKernelSize;
        decimation.set_option(RS2_OPTION_FILTER_MAGNITUDE, decimation_magnitude);
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth = decimation.process(frames.get_depth_frame());
        rs2::frame color = frames.get_color_frame();
        processColorFrame(color);

        auto stream = depth.get_profile().as<rs2::video_stream_profile>();
        auto intrinsics = stream.get_intrinsics();

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
                    rs2_deproject_pixel_to_point(point, &intrinsics, pixel, pixel_depth);
                    // float z = point[2] * sin(REALSENSE_CAM_ANGLE * M_PI / 180.0);
                    float angle = ((-180.0f + REALSENSE_ANGLE_FROM_HORIZONTAL) * M_PI) / 180.0f;
                    float y_rotated = point[1] * cos(angle) - point[2] * sin(angle);
                    float z_rotated = point[1] * sin(angle) + point[2] * cos(angle);

                    Vertex vertex(point[0], y_rotated, z_rotated);
                    Vertex vertexCommonCoor(x, y, z_rotated);

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

        // pipe.stop();
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