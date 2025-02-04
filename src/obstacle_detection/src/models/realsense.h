#pragma once
#include <librealsense2/rs.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <vector>

#define M_PI 3.14159265358979323846

struct Vertex
{
    float x, y, z;
    Vertex() : x(-1), y(-1), z(-1) {}
    Vertex(float x, float y, float z) : x(x), y(y), z(z) {}

    bool operator==(const Vertex &other) const
    {
        return std::fabs(x - other.x) <= 0.5 && std::fabs(y - other.y) <= 0.5 && std::fabs(z - other.z) <= 0.5;
    }
};