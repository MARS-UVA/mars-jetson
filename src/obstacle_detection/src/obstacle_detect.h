#ifndef OBSTACLEDETECT_H
#define OBSTACLEDETECT_H

#include <vector>
#include <cmath>
#include "realsense_capture.h"
std::vector<std::vector<float>> processGradients(const std::vector<std::vector<float>> &heights,
                                                 const std::vector<std::vector<Coordinate>> &actualCoordinates);
#endif