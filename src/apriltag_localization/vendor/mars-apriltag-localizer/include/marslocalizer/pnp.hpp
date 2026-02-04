#ifndef MARSLOCALIZER_PNP_HPP
#define MARSLOCALIZER_PNP_HPP


#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "camera.hpp"
#include "data.hpp"

namespace apriltag {

/**
 * Methods for solving the Perspective-N-Point problem.
 */
enum class PnPMethod : int {

    /**
     * Method which iteratively resolves a pose using the non-linear Levenberg-Marquet minimization scheme.
     *
     * @see cv::SOLVEPNP_ITERATIVE
     */
    ITERATIVE = cv::SOLVEPNP_ITERATIVE,

    /**
     * Method which algebraically resolves a pose from three points.
     *
     * @see cv::SOLVEPNP_AP3P
     */
    AP3P = cv::SOLVEPNP_AP3P,

    /**
     * Method which resolves a pose from coplanar points using the Infinitesimal Plane-Based Pose Estimation algorithm.
     *
     * @see cv::SOLVEPNP_IPPE
     */
    IPPE_COPLANAR = cv::SOLVEPNP_IPPE,

    /**
     * Method which resolves a pose from points in a square using the Infinitesimal Plane-Based Pose Estimation
     * algorithm.
     *
     * @see cv::SOLVEPNP_IPPE_SQUARE
     */
    IPPE_SQUARE = cv::SOLVEPNP_IPPE_SQUARE,

    /**
     * Method which resolves a pose from points using the SQPnP algorithm.
     */
    SQPNP = cv::SOLVEPNP_SQPNP

};

std::vector<Affine3dWithError> solve_pnp(
    const cv::Mat& object_points,
    const cv::Mat& image_points,
    cv::InputArray camera_intrinsics,
    cv::InputArray distortion_vector,
    PnPMethod method = PnPMethod::ITERATIVE);

}

#endif