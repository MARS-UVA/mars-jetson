//
// Created by Ivan on 1/12/26.
//

#include "localizer.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "pnp.hpp"

namespace {

cv::Mat get_corner_points(const double tag_size) {
    cv::Mat_<double> corners(4, 3);
    corners << +1, +1, 0,
               -1, +1, 0,
               -1, -1, 0,
               +1, -1, 0;
    corners *= tag_size / 2;
    return corners;
}

}

const apriltag::AprilTagDetector& apriltag::CameraLocalizer::detector() const {
    return _detector;
}

apriltag::AprilTagDetector& apriltag::CameraLocalizer::detector() {
    return _detector;
}

std::shared_ptr<const apriltag::AprilTagField> apriltag::CameraLocalizer::field() const {
    return _field;
}

std::optional<apriltag::CameraLocalizationResult> apriltag::CameraLocalizer::localize(const cv::Mat& image, const CameraInfo& camera_info) const {
    cv::Mat camera_matrix;
    cv::Mat distortion_vector;
    cv::eigen2cv(camera_info.matrix(), camera_matrix);
    cv::eigen2cv(camera_info.distortion_vector(), distortion_vector);
    return localize(image, camera_matrix, distortion_vector);
}

std::optional<apriltag::CameraLocalizationResult> apriltag::CameraLocalizer::localize(const cv::Mat& image,
    cv::InputArray camera_matrix, cv::InputArray distortion_vector) const {
    image_u8_t cimage = cv2cimage(image);
    zarray_t* result_zarr = apriltag_detector_detect(_detector.raw(), &cimage);
    if (result_zarr == nullptr) {
        return std::nullopt;
    }
    const auto detection_result = detail::wrapping_ptr<zarray_t, zarray_destroy>(result_zarr);
    const int detected_tags = zarray_size(detection_result.get());

    apriltag_detection_t* detection = nullptr; // NOLINT(*-const-correctness)
    CameraLocalizationResult result;
    result.detections.reserve(detected_tags);
    cv::Mat object_points(4 * detected_tags, 3, CV_64F);
    cv::Mat image_points(4 * detected_tags, 2, CV_64F);
    int extraneous = 0;
    for (int i = 0; i < detected_tags; i += 1) {
        zarray_get(detection_result.get(), i, &detection);  // this performs a copy NOLINT(*-multi-level-implicit-pointer-conversion)
        if (const AprilTagInfo* info = _field->tag(detection->id); info != nullptr) {
            result.detections.emplace_back(detection, AprilTagFamily::get_existing(detection->family));
            cv::Mat roi;
            roi = object_points.rowRange(4 * (i - extraneous), 4 * (i - extraneous + 1));
            info->_corners_cv.copyTo(roi);
            roi = image_points.rowRange(4 * (i - extraneous), 4 * (i - extraneous + 1));
            cv::Mat(4, 2, CV_64F, detection->p).copyTo(roi);
        } else {
            extraneous += 1;
        }
    }
    const int used_tags = detected_tags - extraneous;
    if (extraneous > 0) {
        object_points.resize(4 * used_tags);
        image_points.resize(4 * used_tags);
    }
    if (used_tags <= 0) {
        return std::nullopt;
    }

    std::vector<Affine3dWithError> candidates;
    if (used_tags == 1) {
        const AprilTagInfo* tag = _field->tag(result.detections.at(0).id());
        if (tag == nullptr) {
            throw std::runtime_error("Tag did not actually exist???");
        }
        object_points = get_corner_points(tag->_size);
        candidates = solve_pnp(object_points, image_points, camera_matrix, distortion_vector, PnPMethod::IPPE_SQUARE);
        if (candidates.empty()) {
            return std::nullopt;
        }
        result.estimate.pose = candidates.at(0).pose * tag->_pose.inverse();
        result.estimate.reprojection_error = candidates.at(0).reprojection_error;
    } else {
        candidates = solve_pnp(object_points, image_points, camera_matrix, distortion_vector, _method);
        if (candidates.empty()) {
            return std::nullopt;
        }
        result.estimate = *std::min_element(
            candidates.cbegin(),
            candidates.cend(),
            [](const Affine3dWithError& lhs, const Affine3dWithError& rhs) -> bool {
                return lhs.reprojection_error < rhs.reprojection_error;
            }
        );
    }

    result.estimate.pose = result.estimate.pose.inverse();
    return result;
}
