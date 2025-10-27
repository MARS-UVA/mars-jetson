#include "realsense_capture.h"
#include <ctime>
#include <string>
#include "models/obstacle_clustering_tree.h"
#include "gradient_map.h"
#include "local_path_planner_graph.h"
#include <chrono>
#include <filesystem>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include "../../server/client.hpp"
#include "../../server/main.hpp"

#define DECIMATION_KERNEL_SIZE 4

volatile std::sig_atomic_t halt = 0;

void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        halt = 1;
    }
}

int main(int argc, char *argv[])
{
	const char* control_station_ip;
	if(argc == 2){
		control_station_ip = argv[1];
		setenv("CONTROL_STATION_IP", control_station_ip, 1);
	}


    std::signal(SIGINT, signal_handler);
    std::optional<std::vector<Vertex> *> vertices;
    rs2::pipeline pipe;
    rs2::config cfg;
    std::cout << "instantiated pipe" << std::endl;

    // cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
    cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    pipe.start(cfg);
    std::cout << "started pipe" << std::endl;

    for (int i = 0; i < 30; i++)
    {
	//std::cout << "Requesting frame" << std::endl;
        pipe.wait_for_frames();
	//std::cout << "Got a frame" << std::endl;
    }
    while (!halt)
    {
        vertices = new std::vector<Vertex>();
	rs2::frameset frames = pipe.wait_for_frames();
	//std::cout << "Got an actual frame" << std::endl;
	rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
	//std::cout << "Extracted gyro frame" << std::endl;
        //std::shared_ptr<Matrices> retMatrices = 
	//capture_depth_matrix(vertices, DECIMATION_KERNEL_SIZE, pipe, frames);
	rs2::frame color_frame = frames.get_color_frame();
	processColorFrame(color_frame);
	//std::cout << "Got done with capture_depth_matrix" << std::endl;
	if(gyro_frame){
		rs2_vector gyro_data = gyro_frame.get_motion_data();
		std::cout << "Gyro X: " << gyro_data.x << ", Y: " << gyro_data.y << ", Z: " << gyro_data.z << std::endl;
		size_t buffer_size = 12;
		unsigned char *buffer = new unsigned char[buffer_size];
		std::memcpy(&buffer[0], &gyro_data.x, 4);
		std::memcpy(&buffer[4], &gyro_data.y, 4);
		std::memcpy(&buffer[8], &gyro_data.z, 4);
		client_send(buffer, buffer_size, 2027);
	}	
        delete *vertices;
        vertices.reset();
    }
    // shm_unlink(SHM_NAME);
    pipe.stop();
    return 0;
}
