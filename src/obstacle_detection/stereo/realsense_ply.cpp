#include <iostream>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

int main()
{
    rs2::pipeline pipe;
    rs2::config cfg;

    try
    {
        rs2::context ctx;
        auto dev_list = ctx.query_devices();
        if (dev_list.size() == 0)
            throw std::runtime_error("No device detected. Is it plugged in?");
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }

    cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8, 30);
    pipe.start();

    bool first = true;
    while (true)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame color = frames.get_color_frame();
        rs2::depth_frame depth = frames.get_depth_frame();
        if (first)
        {
            first = false;
            cout << "Depth frame: " << depth.get_width() << "x" << depth.get_height() << endl;
        }
    }
}