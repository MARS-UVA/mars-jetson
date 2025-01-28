#ifndef EXTRACTCAPTURE_H
#define EXTRACTCAPTURE_H

#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include "models/realsense.h"
#include "models/pc_adacency_tree.h"
#include "realsense_capture.h"

std::shared_ptr<Matrices> runMatrixCollector(std::vector<Vertex> &vertices);
void runPcTreeCollector(PointcloudTree *tree, std::vector<Vertex> &vertices);

#endif
