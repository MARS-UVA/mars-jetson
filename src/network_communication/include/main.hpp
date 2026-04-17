#pragma once
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// #define CONTROL_STATION_IP "192.168.0.200"
#define IMAGE_PORT 2000
#define WEBCAM_PORT 2026
#define CURRENT_FEEDBACK_PORT 2001
#define ROBOT_POSE_PORT 2003
#define OBSTACLE_POS_PORT 2008
#define PATH_PORT 2025
#define GYRO_PORT 5
//#define ACTUATOR_HEIGHT_PORT 2026
#define CHUNK_SIZE 1400
// insert good documentation
enum FeedbackByteIndices {
    FRONT_LEFT_WHEEL_CURRENT = 0,
    BACK_LEFT_WHEEL_CURRENT = 4,
    FRONT_RIGHT_WHEEL_CURRENT = 8,
    BACK_RIGHT_WHEEL_CURRENT = 12,
    FRONT_DRUM_CURRENT = 16,
    BACK_DRUM_CURRENT = 20,
    FRONT_ACTUATOR_CURRENT = 24,
    BACK_ACTUATOR_CURRENT = 28,
    MAIN_BATTERY_VOLTAGE = 32,
    AUX_BATTERY_VOLTAGE = 36,
    FRONT_LEFT_WHEEL_TEMPERATURE = 40,
    BACK_LEFT_WHEEL_TEMPERATURE = 44,
    FRONT_RIGHT_WHEEL_TEMPERATURE = 48,
    BACK_RIGHT_WHEEL_TEMPERATURE = 52,
    FRONT_DRUM_TEMPERATURE = 56,
    BACK_DRUM_TEMPERATURE = 60,
    FRONT_ACTUATOR_POSITION = 64,
    BACK_ACTUATOR_POSITION = 68,
    ROBOT_STATE = 72,
    PURSUIT_STATE = 76
};