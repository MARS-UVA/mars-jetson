#include <string.h>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "../../platform_compat.h"

#define CONTROL_STATION_IP "192.168.0.200"
#define IMAGE_PORT 2000
#define CURRENT_FEEDBACK_PORT 2001
#define ROBOT_POSE_PORT 2003
#define OBSTACLE_POS_PORT 2008
#define PATH_PORT 2025
#define CHUNK_SIZE 1400
