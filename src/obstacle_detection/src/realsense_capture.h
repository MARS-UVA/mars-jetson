#include <librealsense2/rs.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include "models/pc_adacency_tree.h"
// #include <opencv2/opencv.hpp>

#ifndef REALSENSECAPTURE_H
#define REALSENSECAPTURE_H

#define M_PI 3.14159265358979323846
#define REALSENSE_ANGLE_FROM_HORIZONTAL 40.0f

struct Coordinate
{
    float x, y;
    bool valid = false;
    Coordinate(float x, float y) : x(x), y(y) { valid = true; }
    Coordinate() {}
    Coordinate &operator=(const Coordinate &other)
    {
        if (this == &other)
            return *this;
        x = other.x;
        y = other.y;
        valid = other.valid;
        return *this;
    }
};

struct Matrices
{
    std::vector<std::vector<float>> heights;
    std::vector<std::vector<Coordinate>> actualCoordinates;
};

void save_to_ply(const std::vector<Vertex> &vertices, const std::string &filename);
std::shared_ptr<Matrices> capture_depth_matrix(PointcloudTree *tree, std::vector<Vertex> &vertices);

#endif