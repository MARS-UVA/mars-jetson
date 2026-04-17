#pragma once

#include "main.hpp"

#include <teleop_msgs/msg/stick_position.hpp>
#include "teleop_msgs/msg/human_input_state.hpp"
#include <autonomy_msgs/action/autonomous_actions.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <thread>
#include <cstring>
#include <atomic>
#include <mutex>
#include <iostream>
#include <cerrno>

// PORT FOR RECEIVING
#define PORT 8080

enum class RecvPacketTypes {
    HumanInput = 0,
    Action = 1,
    Config = 2
};

struct __attribute__((packed)) GamepadPacket {
    bool x;
    bool y;
    bool a;
    bool b;
    float lt;
    float rt;
    bool lb;
    bool rb;
    bool dd;
    bool du;
    bool dl;
    bool dr;
    bool l3;
    bool r3;
    bool back;
    bool start;
    float left_stick_x;
    float left_stick_y;
    float right_stick_x;
    float right_stick_y;
};

// TODO: Convert these to enum
#define DIG_AUTO 1
#define DUMP_AUTO 2
#define ESTOP 3