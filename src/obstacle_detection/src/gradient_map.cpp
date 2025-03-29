#include <librealsense2/rs.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include "realsense.h"

struct Tile
{
    int sRow, sCol;
    int rows, cols;
    std::vector<std::vector<double>> data;

    Tile(int sRow, int sCol, int r, int c)
        : sRow(sRow), sCol(sCol), rows(r), cols(c),
          data(r, std::vector<double>(c)) {}
};

/* Depending on Autonomous travel decision, can use parallel processes or Jetson CUDA cores to create gradient map */