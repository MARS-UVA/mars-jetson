#include "pnp.hpp"

#include <algorithm>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


std::vector<apriltag::Affine3dWithError> apriltag::solve_pnp(const cv::Mat& object_points,
                                                             const cv::Mat& image_points,
                                                             cv::InputArray camera_intrinsics,
                                                             cv::InputArray distortion_vector,
                                                             PnPMethod method) {
    cv::Mat processed_object_points(object_points.size(), object_points.type());
    cv::Mat processed_image_points(image_points.size(), image_points.type());
    std::vector<cv::Mat> rvecs_cv;
    std::vector<cv::Mat> tvecs_cv;
    std::vector<double> errors_cv;

    if (method == PnPMethod::IPPE_SQUARE) {
        if (object_points.rows != 4) {
            throw std::invalid_argument("PnPMethod::IPPE_SQUARE requires exactly 4 object points");
        }
        if (image_points.rows != 4) {
            throw std::invalid_argument("PnPMethod::IPPE_SQUARE requires exactly 4 image points");
        }

        object_points.row(0).copyTo(processed_object_points.row(1));
        object_points.row(1).copyTo(processed_object_points.row(0));
        object_points.row(2).copyTo(processed_object_points.row(3));
        object_points.row(3).copyTo(processed_object_points.row(2));

        image_points.row(0).copyTo(processed_image_points.row(1));
        image_points.row(1).copyTo(processed_image_points.row(0));
        image_points.row(2).copyTo(processed_image_points.row(3));
        image_points.row(3).copyTo(processed_image_points.row(2));
    } else {
        processed_object_points = object_points;
        processed_image_points = image_points;
    }

    const int num_solutions = cv::solvePnPGeneric(
            processed_object_points,
            processed_image_points,
            camera_intrinsics,
            distortion_vector,
            rvecs_cv,
            tvecs_cv,
            false,
            static_cast<cv::SolvePnPMethod>(method),
            cv::noArray(),
            cv::noArray(),
            errors_cv);

    std::vector<Affine3dWithError> results(num_solutions);
    cv::Mat rotation_matrix_cv;

    for (int i = 0; i < num_solutions; i += 1) {
        const cv::Affine3d transform_cv { rvecs_cv.at(i), tvecs_cv.at(i) };
        Affine3dWithError result;
        cv::cv2eigen(transform_cv.matrix, result.pose.matrix());
        result.reprojection_error = errors_cv.at(i);

        results.at(i) = (std::move(result));
    }

    return results;
}
