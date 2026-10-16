#pragma once

#include <cstdint>

constexpr uint8_t START_BYTE = 0xFF;

constexpr uint8_t TELEOP_HEADER = 0x00;
constexpr uint8_t CURRENT_FEEDBACK_HEADER = 0x01;
constexpr uint8_t TEMPERATURE_FEEDBACK_HEADER = 0x02;
constexpr uint8_t POSITION_FEEDBACK_HEADER = 0x03;

typedef struct Command
{
    double left;
    double right;
} Command;

#pragma pack(push, 1)

typedef struct CurrentBusVoltageFeedback
{
    double front_left_wheel_current;
    double back_left_wheel_current;
    double front_right_wheel_current;
    double back_right_wheel_current;
    double front_drum_current;
    double back_drum_current;
    double front_actuator_current;
    double back_actuator_current;
    double main_battery_voltage;
    double aux_battery_voltage;
} CurrentBusVoltageFeedback;

typedef struct TemperatureFeedback
{
    double front_left_wheel_temperature;
    double back_left_wheel_temperature;
    double front_right_wheel_temperature;
    double back_right_wheel_temperature;
    double front_drum_temperature;
    double back_drum_temperature;
} TemperatureFeedback;

typedef struct PositionFeedback
{
    double front_actuator_position;
    double back_actuator_position;
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
