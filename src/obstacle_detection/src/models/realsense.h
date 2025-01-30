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
};