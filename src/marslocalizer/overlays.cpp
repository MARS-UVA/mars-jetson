#include <opencv2/imgproc.hpp>

#include "overlays.hpp"

namespace {

cv::Point2i vector2d_to_point2i(const Eigen::Vector2d& vector) {
    return cv::Point2i{static_cast<int>(vector.x()), static_cast<int>(vector.y())};
}

}


const auto MAGENTA = cv::Scalar{255, 0, 255, 255}; // NOLINT(*-throwing-static-initialization)

void apriltag::overlay_squares(cv::Mat image, const std::vector<AprilTagDetection> &detections) {
    for (const auto& detection : detections) {
        const std::array<Eigen::Vector2d, 4> corners = detection.corners();
        for (auto it = corners.cbegin(), end = corners.cend() - 1; it != end; ++it) {
            cv::line(image, vector2d_to_point2i(*it), vector2d_to_point2i(*(it + 1)), MAGENTA, 2);
        }
        cv::line(image, vector2d_to_point2i(*corners.crbegin()), vector2d_to_point2i(*corners.cbegin()), MAGENTA, 2);
    }
}

void apriltag::overlay_labels(cv::Mat image, const std::vector<AprilTagDetection> &detections) {
    for (const auto& detection : detections) {
        const Eigen::Vector2d center = detection.center();
        cv::putText(image,
            std::to_string(detection.id()),
            vector2d_to_point2i(center),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            MAGENTA,
            2,
            cv::LINE_AA);
    }
}
