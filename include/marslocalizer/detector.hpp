//
// Created by Ivan on 3/8/26.
//

#ifndef MARSLOCALIZER_DETECTOR_HPP
#define MARSLOCALIZER_DETECTOR_HPP

#include <apriltag.h>
#include <memory>
#include <vector>

#include "common.hpp"

namespace apriltag {

/**
 * A detector for AprilTags.
 *
 * This wraps the @code apriltag_detector_t@endcode struct from the original AprilTag library.
 */
class AprilTagDetector {
public:
    /**
     * Creates a new AprilTagDetector object.
     */
    AprilTagDetector()
        : _detector(apriltag_detector_create()) {
    }

    /**
     * Const accessor to the number of threads the detector may use.
     *
     * @return The number of threads which the detector may use.
     */
    [[nodiscard]] int nthreads() const;

    /**
     * Non-const accessor to the number of threads the detector may use.
     *
     * @return The number of threads which the detector may use.
     */
    [[nodiscard]] int& nthreads();

    /**
     * Const accessor to the amount by which the detector decimates the image.
     *
     * @return The amount by which to decimate the image before performing quad detection.
     */
    [[nodiscard]] float quad_decimate() const;

    /**
     * Non-const accessor to the amount by which the detector decimates the image.
     *
     * @return The amount by which to decimate the image before performing quad detection.
     */
    [[nodiscard]] float& quad_decimate();

    /**
     * Const accessor to the amount of Gaussian blur to apply to the image before performing quad detection.
     *
     * @return The amount of Gaussian blur to apply to the image before performing quad detection.
     */
    [[nodiscard]] float quad_sigma() const;

    /**
     * Non-const accessor to the amount of Gaussian blur to apply to the image before performing quad detection.
     *
     * @return The amount of Gaussian blur to apply to the image before performing quad detection.
     */
    [[nodiscard]] float& quad_sigma();

    /**
     * Const accessor to whether to refine quad edges.
     *
     * @return Whether the detector will adjust quad edges to snap to areas of higher luminance gradients.
     */
    [[nodiscard]] bool refine_edges() const;

    /**
     * Non-const accessor to whether to refine quad edges.
     *
     * @return Whether the detector will adjust quad edges to snap to areas of higher luminance gradients.
     */
    [[nodiscard]] bool& refine_edges();

    /**
     * Const accessor to the amount of sharpening to apply.
     *
     * @return The amount of sharpening to apply to decoded images before quad detection is performed.
     */
    [[nodiscard]] double decode_sharpening() const;

    /**
     * Non-const accessor to the amount of sharpening to apply.
     *
     * @return The amount of sharpening to apply to decoded images before quad detection is performed.
     */
    [[nodiscard]] double& decode_sharpening();

    /**
     * Const accessor to whether to output additional debug information.
     *
     * @return Whether to output additional debug information.
     */
    [[nodiscard]] bool debug() const;

    /**
     * Non-const accessor to whether to output additional debug information.
     *
     * @return Whether to output additional debug information.
     */
    [[nodiscard]] bool& debug();

    /**
     * Adds an AprilTag family to the set of families which this detector will detect.
     *
     * @param family Shared pointer to the AprilTag family to add.
     */
    void add_family(const std::shared_ptr<AprilTagFamily>& family);

    /**
     * Removes an AprilTag family from the set of families which this detector will detect.
     *
     * @param family Shared pointer to the AprilTag family to remove.
     */
    void remove_family(const std::shared_ptr<AprilTagFamily>& family);

    /**
     * Removes all AprilTag families from the set of families which this detector will detect.
     */
    void clear_families();

    /**
     * Detects AprilTags in the provided image.
     *
     * @param image Image in which AprilTags will be detected.
     * @return A std::vector of AprilTags that were detected. May be empty.
     */
    [[nodiscard]] std::vector<AprilTagDetection> detect(const cv::Mat& image) const;

    /**
     * Returns a raw pointer to the wrapped C value.
     *
     * @return A raw pointer to the underlying @code apriltag_detector_t@endcode value.
     */
    [[nodiscard]] apriltag_detector_t* raw() const;

    AprilTagDetector(const AprilTagDetector&) = delete;
    AprilTagDetector(AprilTagDetector&&) = default;

    AprilTagDetector& operator=(const AprilTagDetector&) = delete;
    AprilTagDetector& operator=(AprilTagDetector&&) = default;

private:

    detail::wrapping_ptr<apriltag_detector_t, apriltag_detector_destroy> _detector;
    std::vector<std::shared_ptr<AprilTagFamily>> _families;

};

}

#endif //MARSLOCALIZER_DETECTOR_HPP