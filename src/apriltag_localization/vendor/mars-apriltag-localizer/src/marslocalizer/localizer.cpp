//
// Created by Ivan on 1/12/26.
//

#include "localizer.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "common.hpp"
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

apriltag::CameraLocalizer::CameraLocalizer(const std::shared_ptr<const AprilTagField>& field,
                                           const PnPMethod many_tags_method, const PnPMethod one_tag_method)
    : _field(field), _many_tags_method(many_tags_method), _one_tag_method(one_tag_method) {
    _detector.add_family(AprilTagFamily::get(field->tag_family()));
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

std::optional<apriltag::CameraLocalizationResult> apriltag::CameraLocalizer::localize(
    const cv::Mat& image, const CameraInfo& camera_info) const {
    cv::Mat camera_matrix;
    cv::Mat distortion_vector;
    cv::eigen2cv(camera_info.matrix(), camera_matrix);
    cv::eigen2cv(camera_info.distortion_vector(), distortion_vector);
    return localize(image, camera_matrix, distortion_vector);
}

std::optional<apriltag::CameraLocalizationResult> apriltag::CameraLocalizer::localize(const cv::Mat& image,
    cv::InputArray camera_matrix, cv::InputArray distortion_vector) const {
    CameraLocalizationResult result;
    result.detections = _detector.detect(image);
    auto remove_begin = std::remove_if(
        result.detections.begin(),
        result.detections.end(),
        [this](const AprilTagDetection& detection) -> bool {
            return _field->tag(detection.id()) == nullptr;
        });
    result.detections.erase(remove_begin, result.detections.end());
    if (result.detections.empty()) {
        return std::nullopt;
    }
    std::size_t used_tags = result.detections.size();

    cv::Mat object_points(4 * used_tags, 3, CV_64F);
    cv::Mat image_points(4 * used_tags, 2, CV_64F);
    for (std::size_t i = 0; i < used_tags; i += 1) {
        cv::Mat roi;
        roi = object_points.rowRange(4 * i, 4 * (i + 1));
        _field->tag(result.detections.at(i).id())->_corners_cv.copyTo(roi);
        roi = image_points.rowRange(4 * i, 4 * (i + 1));
        result.detections.at(i).corners_view().copyTo(roi);
    }

    std::vector<Affine3dWithError> candidates;
    PnPMethod method = _many_tags_method;
    if (used_tags == 1) {
        if (_one_tag_method == PnPMethod::IPPE_SQUARE) {
            const AprilTagInfo* tag = _field->tag(result.detections.at(0).id());
            if (tag == nullptr) {
                throw std::runtime_error("Tag did not actually exist???");
            }
            object_points = get_corner_points(tag->_size);
            candidates = solve_pnp(
                object_points,
                image_points,
                camera_matrix,
                distortion_vector,
                _one_tag_method
            );
            if (candidates.empty()) {
                return std::nullopt;
            }
            result.estimate.pose = tag->_pose * candidates.at(0).pose.inverse();
            result.estimate.reprojection_error = candidates.at(0).reprojection_error;
            return result;
        }
        method = _one_tag_method;
    }
    candidates = solve_pnp(
        object_points,
        image_points,
        camera_matrix,
        distortion_vector,
        method
    );
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

    result.estimate.pose = result.estimate.pose.inverse();
    return result;
}
