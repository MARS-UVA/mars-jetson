#include "wrapper.hpp"

#include <apriltag.h>
#include <cstdint>
#include <exception>
#include <memory>
#include <vector>
#include <opencv2/core/mat.hpp>

// Accessors

int apriltag::AprilTagDetector::nthreads() const {
    return _detector->nthreads;
}

int& apriltag::AprilTagDetector::nthreads() {
    return _detector->nthreads;
}

float apriltag::AprilTagDetector::quad_decimate() const {
    return _detector->quad_decimate;
}

float& apriltag::AprilTagDetector::quad_decimate() {
    return _detector->quad_decimate;
}

float apriltag::AprilTagDetector::quad_sigma() const {
    return _detector->quad_sigma;
}

float& apriltag::AprilTagDetector::quad_sigma() {
    return _detector->quad_sigma;
}

bool apriltag::AprilTagDetector::refine_edges() const {
    return _detector->refine_edges;
}

bool& apriltag::AprilTagDetector::refine_edges() {
    return _detector->refine_edges;
}

double apriltag::AprilTagDetector::decode_sharpening() const {
    return _detector->decode_sharpening;
}

double& apriltag::AprilTagDetector::decode_sharpening() {
    return _detector->decode_sharpening;
}

bool apriltag::AprilTagDetector::debug() const {
    return _detector->debug;
}

bool& apriltag::AprilTagDetector::debug() {
    return _detector->debug;
}

void apriltag::AprilTagDetector::add_family(const std::shared_ptr<AprilTagFamily>& family) {
    apriltag_detector_add_family(raw(), family->raw());
    _families.push_back(family);
}

void apriltag::AprilTagDetector::remove_family(const std::shared_ptr<AprilTagFamily>& family) {
    apriltag_detector_remove_family(raw(), family->raw());
    _families.erase(std::remove(_families.begin(), _families.end(), family), _families.end());
}

void apriltag::AprilTagDetector::clear_families() {
    apriltag_detector_clear_families(raw());
    _families.clear();
}

std::vector<apriltag::AprilTagDetection> apriltag::AprilTagDetector::detect(const cv::Mat& image) const {
    image_u8_t cimage = cv2cimage(image);
    zarray_t* result_zarr = apriltag_detector_detect(raw(), &cimage);
    if (result_zarr == nullptr) {
        return std::vector<AprilTagDetection>{};
    }
    const auto result = detail::wrapping_ptr<zarray_t, zarray_destroy>(result_zarr);

    apriltag_detection_t* detection = nullptr; // NOLINT(*-const-correctness)
    std::vector<AprilTagDetection> detections;
    for (int i = 0; i < zarray_size(result.get()); i += 1) {
        zarray_get(result.get(), i, &detection);  // this performs a copy NOLINT(*-multi-level-implicit-pointer-conversion)
        detections.emplace_back(detection, AprilTagFamily::get_existing(detection->family));
    }
    return detections;
}

apriltag_detector_t* apriltag::AprilTagDetector::raw() const {
    return _detector.get();
}

image_u8_t apriltag::cv2cimage(const cv::Mat& image) {
    if (image.dims != 2) {
        throw std::invalid_argument("Invalid image dimensions");
    }
    if (image.type() != CV_8U) {
        throw std::invalid_argument("Invalid image type");
    }
    return {
        image.cols,  // width
        image.rows,  // height
        static_cast<std::int32_t>(image.step[0]), // step NOLINT(*-pro-bounds-avoid-unchecked-container-access)
        image.data  // buffer
    };
}
