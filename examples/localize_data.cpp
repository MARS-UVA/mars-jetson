//
// Created by Ivan on 2/1/26.
//

#include <fstream>
#include <iostream>

#include "camera.hpp"
#include "field.hpp"
#include "localizer.hpp"
#include "overlays.hpp"
#include "tagStandard41h12.h"

constexpr apriltag::CameraInfo info {
    apriltag::CameraIntrinsics {
        921.4459062879504,
        920.5545512136466,
        640.4116071984264,
        389.86971089068913
    },
    apriltag::RadialDistortionCoefficients {
        0.07166453170075777,
        -0.11227733992947839,
        0.04562948613626775
    },
    apriltag::TangentialDistortionCoefficients {
        -0.0014616393622597851,
        -0.0014945237103234227
    }
};

int main(const int argc, const char* argv[]) {
    if (argc < 2) {
        std::cerr << "need to specify a field file" << std::endl;
        return 1;
    }

    if (argc < 3) {
        std::cerr << "need to specify a video file" << std::endl;
        return 1;
    }

    std::ifstream field_file { argv[1], field_file.in };

    std::shared_ptr<apriltag::AprilTagField> field = apriltag::AprilTagField::parse(field_file);

    apriltag::CameraLocalizer localizer { field, apriltag::PnPMethod::SQPNP };

    const auto family = apriltag::AprilTagFamily::get(tagStandard41h12_create(), tagStandard41h12_destroy);
    localizer.detector().nthreads() = 8;
    localizer.detector().add_family(family);

    cv::VideoCapture capture { argv[2] };
    if (!capture.isOpened()) {
        std::cerr << "Could not open video " << argv[2] << std::endl;
        return 1;
    }

    cv::Mat frame;
    cv::Mat gray_frame;
    std::uint64_t frame_index = 0;
    std::cout << "frame,x,y,z,zxz_1,zxz_2,zxz_3,error\n";
    while (capture.read(frame)) {
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        if (std::optional<apriltag::CameraLocalizationResult> result = localizer.localize(gray_frame, info); result.has_value()) {
            Eigen::Vector3d translation = result->estimate.pose.translation();
            Eigen::Vector3d rotation = result->estimate.pose.linear().canonicalEulerAngles(2, 0, 2);
            std::cout << frame_index << "," <<
                translation.x() << "," <<
                translation.y() << "," <<
                translation.z() << "," <<
                rotation.x() << "," <<
                rotation.y() << "," <<
                rotation.z() << "," <<
                result->estimate.reprojection_error << "\n";
        }
        frame_index += 1;
    }
    cv::destroyAllWindows();
    return 0;
}
