#ifndef MARSLOCALIZER_DISPLAY_HPP
#define MARSLOCALIZER_DISPLAY_HPP

// ReSharper disable once CppUnusedIncludeDirective
#include <opencv2/core/mat.hpp>

#include "wrapper.hpp"

namespace apriltag {

/**
 * Overlays magenta-colored squares over each detected AprilTag in the provided image.
 *
 * @param image Image array on which to overlay the squares.
 * @param detections A std::vector of AprilTagDetection objects.
 */
void overlay_squares(cv::Mat image, const std::vector<AprilTagDetection>& detections);

/**
 * Overlays magenta-colored labels over each detected AprilTag in the provided image.
 *
 * @param image Image array on which to overlay the labels.
 * @param detections A std::vector of AprilTagDetection objects.
 */
void overlay_labels(cv::Mat image, const std::vector<AprilTagDetection>& detections);

}

#endif //MARSLOCALIZER_DISPLAY_HPP