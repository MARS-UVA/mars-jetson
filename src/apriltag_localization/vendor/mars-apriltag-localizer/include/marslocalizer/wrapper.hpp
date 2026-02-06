#ifndef MARSLOCALIZER_WRAPPER_HPP
#define MARSLOCALIZER_WRAPPER_HPP

#include <apriltag.h>
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <Eigen/Dense>
// ReSharper disable once CppUnusedIncludeDirective
#include <opencv2/opencv.hpp>

/** @file */

namespace apriltag {

namespace detail {

    /**
     * A functor that wraps a function that deletes an object.
     *
     * @tparam fn The function being wrapped.
     */
    template <auto fn>
    struct deleter_from_fn {
        template <typename T>
        constexpr void operator()(T* arg) const {
            fn(arg);
        }
    };

    /**
     * A unique pointer which wraps a C struct that performs deletion via some sort of deletion function.
     *
     * @tparam T Type of the pointee.
     * @tparam fn The deleter function for the type.
     */
    template <typename T, auto fn>
    using wrapping_ptr = std::unique_ptr<T, deleter_from_fn<fn>>;

}

/**
 * Represents an AprilTag family.
 *
 * Every tag belongs to a tag family. An AprilTagDetector will use information from this class to detect tags from that
 * family.
 *
 * This wraps the @code apriltag_family_t@endcode struct from the original AprilTag library.
 *
 * @see AprilTagDetector
 */
class AprilTagFamily {
protected:

    struct ConstructorKey;

public:
    /**
     * Type for AprilTag family deleter function pointers.
     */
    using Deleter = void(apriltag_family_t*);

    /**
     * Obtains a corresponding AprilTagFamily object for the given family data, or creates a new one if there isn't
     * one yet.
     *
     * @param family A raw pointer to the family which the AprilTagFamily object represent.
     * @param deleter A deleter function pointer for the family.
     * @return If an AprilTagFamily object already exists for the family data, then return a shared pointer to that
     *         object. Otherwise, create a new AprilTagFamily object for the data and return a shared pointer to it.
     */
    static std::shared_ptr<AprilTagFamily> get(apriltag_family_t* family, Deleter& deleter);

    // ReSharper disable once CppParameterMayBeConstPtrOrRef
    /**
     * Obtains a corresponding AprilTagFamily object for the given family data.
     *
     * @param family A raw pointer to the family which the AprilTagFamily object will represent.
     * @return A shared pointer to the AprilTagFamily object which represents the given AprilTag family data.
     * @throw std::invalid_argument There is no corresponding AprilTagFamily object for the given data.
     */
    static std::shared_ptr<AprilTagFamily> get_existing(apriltag_family_t* family);

    /**
     * Creates a new AprilTagFamily. This cannot be called directly.
     *
     * @param constructor_key A key for calling this constructor.
     * @param family Original apriltag_family_t object. Will probably be the result of a function with name tag*_create.
     * @param deleter Deleter function for this AprilTag family.
     */
    AprilTagFamily([[maybe_unused]] const ConstructorKey& constructor_key, apriltag_family_t* family, Deleter& deleter)
    : _family(family), _deleter(deleter) {
    }

    /**
     * Deletes an AprilTagFamily.
     *
     * This will also destroy the underlying @code apriltag_family_t@endcode object with the deleter provided in
     * the constructor.
     */
    ~AprilTagFamily() {
        _deleter(_family);
    }

    /**
     * Returns the family's name.
     *
     * @return A human-readable name for the AprilTag family.
     */
    [[nodiscard]] std::string_view name() const;

    /**
     * Returns the width of the border square for AprilTags in this family.
     *
     * @return The distance, in pixels, between opposite edges on the border of an AprilTag in this family. The four
     *         corners of the border are what the AprilTagDetector detects.
     */
    [[nodiscard]] int width_at_border() const;

    /**
     * Returns the total width of AprilTags in this family.
     *
     * @return The total width of an AprilTag in this family, in pixels.
     */
    [[nodiscard]] int total_width() const;

    /**
     * Returns the minimum hamming distance between AprilTags in this family.
     *
     * @return The smallest hamming distance between any two AprilTags in the family.
     */
    [[nodiscard]] std::uint32_t minimum_hamming_distance() const;

    /**
     * Returns a raw pointer to the wrapped C value.
     *
     * @return A raw pointer to the underlying @code apriltag_family_t@endcode value.
     */
    [[nodiscard]] apriltag_family_t *raw() const;

    AprilTagFamily(const AprilTagFamily&) = delete;
    AprilTagFamily(AprilTagFamily&&) = delete;

    AprilTagFamily& operator=(const AprilTagFamily&) = delete;
    AprilTagFamily& operator=(AprilTagFamily&&) = delete;

protected:

    struct ConstructorKey {
        explicit ConstructorKey() = default;
    };

private:

    static std::map<const apriltag_family_t*, std::shared_ptr<AprilTagFamily>> _instances;

    apriltag_family_t *_family;
    Deleter& _deleter;

};

class AprilTagDetector;

/**
 * Represents a detected AprilTag.
 *
 * This wraps the @code apriltag_detection_t@endcode struct from the original AprilTag library.
 */
class AprilTagDetection {
public:

    /**
     * Creates a new AprilTagDetection to wrap the given @code apriltag_detection_t@endcode.
     *
     * @param detection Raw pointer to the @code apriltag_detection_t@endcode to wrap.
     * @param family A const shared pointer to the AprilTagFamily of which the detected AprilTag is a member.
     */
    AprilTagDetection(apriltag_detection_t *detection, const std::shared_ptr<AprilTagFamily>& family)
    : _detection(detection), _family(family) {
    }

    /**
     * Returns the ID for the tag.
     *
     * @return The ID for the detected AprilTag within its family.
     */
    [[nodiscard]] int id() const;

    /**
     * Returns the AprilTag family of which the detected tag is a member.
     *
     * @return A const shared pointer to the AprilTagFamily of which the detected AprilTag is a member.
     */
    [[nodiscard]] std::shared_ptr<AprilTagFamily> family() const;

    /**
     * Returns the average difference between the intensity of a data bit versus the detection threshold.
     *
     * Higher numbers roughly indicate better AprilTag decodes. This could be used as a measure of detection accuracy
     * for very small AprilTags.
     *
     * @return The average difference between the intensity of a data bit versus the detection threshold.
     */
    [[nodiscard]] float detection_margin() const;

    /**
     * Returns the number of error bits which were corrected.
     *
     * @return The number of error bits which were corrected.
     */
    [[nodiscard]] int hamming() const;

    /**
     * Returns the center point of the tag.
     *
     * @return An Eigen::Vector2d representing the center point of the AprilTag in pixel coordinates for the image
     *         (the origin is at the top left of the image, the first coordinate increases in the rightward direction,
     *         and the second coordinate increases in the downward direction).
     */
    [[nodiscard]] Eigen::Vector2d center() const;

    /**
     * Returns the corner points of the tag.
     *
     * @return A 4-array of the detected AprilTag's corner points in pixel coordinates for the image (the origin is at
     *         the top left of the image, the first coordinate increases in the rightward direction, and the second
     *         coordinate increases in the downward direction). Always specified in counterclockwise order.
     */
    [[nodiscard]] std::array<Eigen::Vector2d, 4> corners() const;

    /**
     * Returns a view of the tag's corner points. Callers should be careful to not persist the matrix instance after
     * the AprilTagDetection object is deallocated, since the data will be freed.
     *
     * @return An OpenCV matrix containing the AprilTag's corner points in pixel coordinates for the image (the origin
     *         is at the top left of the image, the first coordinate increases in the rightward direction, and the
     *         second coordinate increases in the downward direction). Always specified in counterclockwise order.
     */
    [[nodiscard]] cv::Mat corners_view() const;

    /**
     * Returns a raw pointer to the wrapped C value.
     *
     * @return A raw pointer to the underlying @code apriltag_detection_t@endcode value.
     */
    [[nodiscard]] apriltag_detection_t *raw() const;

    AprilTagDetection(const AprilTagDetection&) = delete;
    AprilTagDetection(AprilTagDetection&&) = default;

    AprilTagDetection& operator=(const AprilTagDetection&) = delete;
    AprilTagDetection& operator=(AprilTagDetection&&) = default;

private:

    detail::wrapping_ptr<apriltag_detection_t, apriltag_detection_destroy> _detection;
    std::shared_ptr<AprilTagFamily> _family;

};

struct AprilTagDetectorConfiguration {
    int nthreads;
    float quad_decimate;
    float quad_sigma;
    bool refine_edges;
    double decode_sharpening;
    bool debug;
};

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

image_u8_t cv2cimage(const cv::Mat& image);

}

#endif //MARSLOCALIZER_WRAPPER_HPP
