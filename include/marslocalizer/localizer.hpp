//
// Created by Ivan on 1/12/26.
//

#ifndef MARSLOCALIZER_LOCALIZER_HPP
#define MARSLOCALIZER_LOCALIZER_HPP

#include "camera.hpp"
#include "data.hpp"
#include "field.hpp"
#include "pnp.hpp"
#include "wrapper.hpp"

namespace apriltag {

struct CameraLocalizationResult {
    Affine3dWithError estimate;
    std::vector<AprilTagDetection> detections;
};

class CameraLocalizer {

    AprilTagDetector _detector;
    std::shared_ptr<const AprilTagField> _field;
    PnPMethod _method;

public:

    CameraLocalizer(const std::shared_ptr<const AprilTagField>& field,
                    const PnPMethod method = PnPMethod::ITERATIVE)
        : _field(field), _method(method) {
    }

    [[nodiscard]] const AprilTagDetector& detector() const;

    [[nodiscard]] AprilTagDetector& detector();

    [[nodiscard]] std::shared_ptr<const AprilTagField> field() const;

    [[nodiscard]] std::optional<CameraLocalizationResult> localize(const cv::Mat& image, const CameraInfo& camera_info) const;

    [[nodiscard]] std::optional<CameraLocalizationResult> localize(const cv::Mat& image,
                                                                   cv::InputArray camera_matrix,
                                                                   cv::InputArray distortion_vector) const;

};

}

#endif //MARSLOCALIZER_LOCALIZER_HPP