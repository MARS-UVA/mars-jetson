//
// Created by Ivan on 1/12/26.
//

#ifndef MARSLOCALIZER_LOCALIZER_HPP
#define MARSLOCALIZER_LOCALIZER_HPP

#include <memory>
#include <optional>
#include <vector>
#include <opencv2/opencv.hpp>

#include "camera.hpp"
#include "common.hpp"
#include "data.hpp"
#include "detector.hpp"
#include "field.hpp"
#include "pnp.hpp"

namespace apriltag {

struct CameraLocalizationResult {
    Affine3dWithError estimate;
    std::vector<AprilTagDetection> detections;
};

class CameraLocalizer {

    AprilTagDetector _detector;
    std::shared_ptr<const AprilTagField> _field;
    PnPMethod _many_tags_method;
    PnPMethod _one_tag_method;

public:
    explicit CameraLocalizer(const std::shared_ptr<const AprilTagField>& field,
                             PnPMethod many_tags_method = PnPMethod::ITERATIVE,
                             PnPMethod one_tag_method = PnPMethod::IPPE_SQUARE);

    [[nodiscard]] const AprilTagDetector& detector() const;

    [[nodiscard]] AprilTagDetector& detector();

    [[nodiscard]] std::shared_ptr<const AprilTagField> field() const;

    [[nodiscard]] std::optional<CameraLocalizationResult> localize(const cv::Mat& image,
                                                                   const CameraInfo& camera_info) const;

    [[nodiscard]] std::optional<CameraLocalizationResult> localize(const cv::Mat& image,
                                                                   cv::InputArray camera_matrix,
                                                                   cv::InputArray distortion_vector) const;

};

}

#endif //MARSLOCALIZER_LOCALIZER_HPP