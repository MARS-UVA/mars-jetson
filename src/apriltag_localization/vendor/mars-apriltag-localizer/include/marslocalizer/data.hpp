//
// Created by Ivan on 1/12/26.
//

#ifndef MARSLOCALIZER_DATA_HPP
#define MARSLOCALIZER_DATA_HPP

#include <Eigen/Geometry>

namespace apriltag {

struct Affine3dWithError {
    Eigen::Affine3d pose;
    double reprojection_error;
};

}

#endif //MARSLOCALIZER_DATA_HPP