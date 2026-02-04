#include "wrapper.hpp"

int apriltag::AprilTagDetection::id() const {
    return _detection->id;
}

Eigen::Vector2d apriltag::AprilTagDetection::center() const {
    return { _detection->c[0], _detection->c[1] };
}

std::array<Eigen::Vector2d, 4> apriltag::AprilTagDetection::corners() const {
    return {
        Eigen::Vector2d { _detection->p[0][0], _detection->p[0][1] },
        Eigen::Vector2d { _detection->p[1][0], _detection->p[1][1] },
        Eigen::Vector2d { _detection->p[2][0], _detection->p[2][1] },
        Eigen::Vector2d { _detection->p[3][0], _detection->p[3][1] },
    };
}

cv::Mat apriltag::AprilTagDetection::corners_view() const {
    return {4, 2, CV_64F, static_cast<void*>(_detection->p)};
}

std::shared_ptr<apriltag::AprilTagFamily> apriltag::AprilTagDetection::family() const {
    return _family;
}

float apriltag::AprilTagDetection::detection_margin() const {
    return _detection->decision_margin;
}

int apriltag::AprilTagDetection::hamming() const {
    return _detection->hamming;
}

apriltag_detection_t* apriltag::AprilTagDetection::raw() const {
    return _detection.get();
}
