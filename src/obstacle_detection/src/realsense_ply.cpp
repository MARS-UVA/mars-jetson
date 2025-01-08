#include <iostream>
#include <librealsense2/rs.hpp>
#include <iomanip>
#include <chrono> // Add this
#include <thread> // Add this

int main()
{
    // Create RealSense pipeline and config
    rs2::pipeline pipe;
    rs2::config cfg;

    try
    {
        // Check if camera is connected
        rs2::context ctx;
        auto devices = ctx.query_devices();
        if (devices.size() == 0)
        {
            std::cout << "No RealSense device connected!" << std::endl;
            return -1;
        }

        std::cout << "RealSense device found." << std::endl;

        // Configure and start the pipeline
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        pipe.start(cfg);

        // Print depths for a few frames
        for (int frame_number = 0; frame_number < 3; frame_number++)
        {
            // Get frameset
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::depth_frame depth = frames.get_depth_frame();

            // Get dimensions
            const int width = depth.get_width();
            const int height = depth.get_height();

            std::cout << "\nFrame " << frame_number + 1 << " Depth Matrix (showing 5x5 section):" << std::endl;

            // Print a 5x5 section from the middle of the depth image
            int start_x = width / 2 - 2;
            int start_y = height / 2 - 2;

            for (int y = start_y; y < start_y + 5; y++)
            {
                for (int x = start_x; x < start_x + 5; x++)
                {
                    float depth_value = depth.get_distance(x, y);
                    std::cout << std::fixed << std::setprecision(3) << depth_value << "\t";
                }
                std::cout << std::endl;
            }

            // Wait a bit between frames
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // Stop the pipeline
        pipe.stop();
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return -1;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "\nPress Enter to exit..." << std::endl;
    std::cin.get();
    return 0;
}