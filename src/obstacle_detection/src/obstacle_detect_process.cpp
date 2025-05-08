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

    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 90);
    cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 30);

    pipe.start(cfg);

    for (int i = 0; i < 30; i++)
    {
        pipe.wait_for_frames();
    }
    while (!halt)
    {
        vertices = new std::vector<Vertex>();
        std::shared_ptr<Matrices> retMatrices = capture_depth_matrix(vertices, DECIMATION_KERNEL_SIZE, pipe);
        delete *vertices;
        vertices.reset();
    }
    // shm_unlink(SHM_NAME);
    pipe.stop();
    return 0;
}
