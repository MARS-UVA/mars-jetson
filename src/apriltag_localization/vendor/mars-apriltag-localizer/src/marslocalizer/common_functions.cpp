//
// Created by Ivan on 3/8/26.
//

#include "common.hpp"

#include <apriltag.h>
#include <cstdint>
#include <exception>
#include <opencv2/core/mat.hpp>

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
