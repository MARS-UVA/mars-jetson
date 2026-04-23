#pragma once

#include "main.hpp"

#include <serial_msgs/msg/current_bus_voltage.hpp>
#include <serial_msgs/msg/position.hpp>
#include <serial_msgs/msg/temperature.hpp>
#include <teleop_msgs/msg/arm_control.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <vector>

#define FEEDBACK_PORT 2001
#define FEEDBACK_PACKET_LENGTH 88

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
    FRONT_ARM_CONTROL = 76,
    BACK_ARM_CONTROL = 80,
};

struct ConnectionHeaders
{
    int client_socket_fd;
    struct sockaddr_in control_station_addr;
};