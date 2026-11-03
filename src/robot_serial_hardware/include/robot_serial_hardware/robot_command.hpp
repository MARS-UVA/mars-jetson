#pragma once

#include <cstdint>

constexpr uint8_t START_BYTE = 0xFF;

constexpr uint8_t TELEOP_HEADER = 0x00;
constexpr uint8_t CURRENT_FEEDBACK_HEADER = 0x01;
constexpr uint8_t TEMPERATURE_FEEDBACK_HEADER = 0x02;
constexpr uint8_t POSITION_FEEDBACK_HEADER = 0x03;

typedef struct Command
{
    double front_left;
    double front_right;
    double rear_left;
    double rear_right;
    double front_drum;
    double front_arm;
    double rear_drum;
    double rear_arm;
} Command;

#pragma pack(push, 1)

typedef struct CurrentBusVoltageFeedback
{
    float front_left_wheel_current;
    float rear_left_wheel_current;
    float front_right_wheel_current;
    float rear_right_wheel_current;
    float front_drum_current;
    float rear_drum_current;
    float front_actuator_current;
    float rear_actuator_current;
    float main_battery_voltage;
    float aux_battery_voltage;
} CurrentBusVoltageFeedback;

typedef struct TemperatureFeedback
{
    float front_left_wheel_temperature;
    float rear_left_wheel_temperature;
    float front_right_wheel_temperature;
    float rear_right_wheel_temperature;
    float front_drum_temperature;
    float rear_drum_temperature;
} TemperatureFeedback;

typedef struct PositionFeedback
{
    float front_actuator_position;
    float rear_actuator_position;
} PositionFeedback;

#pragma pack(pop)

typedef struct Feedback
{
    uint64_t timestamp;
    uint8_t header;
    union {
        CurrentBusVoltageFeedback current_bus_voltage;
        TemperatureFeedback temperature;
        PositionFeedback position;
    };
} Feedback;
