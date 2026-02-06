//
// Created by Ivan on 1/30/26.
//

#include <iostream>
#include <tagStandard41h12.h>
#include <opencv2/opencv.hpp>

#include "localizer.hpp"
#include "overlays.hpp"
#include "wrapper.hpp"

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

// constexpr apriltag::CameraInfo info = {
//     apriltag::CameraIntrinsics {
//         917.7882863753648,
//         919.1565057085057,
//         646.5877410071772,
//         352.13798352528124
//     },
//     apriltag::RadialDistortionCoefficients {
//         0.10532188820909076,
//         -0.2239874813023289,
//         0.08242129571825572
//     },
//     apriltag::TangentialDistortionCoefficients {
//         -0.0002431858202170748,
//         0.0024169899536584096
//     }
// };

int main(const int argc, const char* argv[]) {
    if (argc < 2) {
        std::cerr << "need to specify a field file" << std::endl;
        return 1;
    }

    int camera_index;
    if (argc < 3) {
        std::cerr << "Camera index not specified; defaulting to 0" << std::endl;
        camera_index = 0;
    } else {
        camera_index = std::stoi(argv[2]);
    }

    std::ifstream field_file { argv[1], field_file.in };

    std::shared_ptr<apriltag::AprilTagField> field = apriltag::AprilTagField::parse(field_file);

    apriltag::CameraLocalizer localizer { field };

    const auto family = apriltag::AprilTagFamily::get(tagStandard41h12_create(), tagStandard41h12_destroy);
    localizer.detector().nthreads() = 8;
    localizer.detector().add_family(family);

    cv::VideoCapture capture {camera_index};
    if (!capture.isOpened()) {
        std::cerr << "Could not open camera at index " << camera_index << std::endl;
        return 1;
    }

    cv::Mat frame;
    cv::namedWindow("camera", cv::WINDOW_KEEPRATIO);
    cv::Mat gray_frame;
    while (true) {
        if (capture.read(frame)) {
            cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
            if (std::optional<apriltag::CameraLocalizationResult> result = localizer.localize(gray_frame, info); result.has_value()) {
                apriltag::overlay_squares(frame, result.value().detections);
                apriltag::overlay_labels(frame, result.value().detections);
                std::cout << "pose:\ntranslation:\n"
                          << result.value().estimate.pose.translation()
                          << "\nrotation:\n"
                          << result.value().estimate.pose.rotation().canonicalEulerAngles(2, 1, 0)
                          << "\n\nreprojection error: "
                          << result.value().estimate.reprojection_error
                          << "\n-----\n";
            }

            cv::imshow("camera", frame);
        }
        if (const int key = cv::waitKey(1); key == 'q') {
            break;
        }
    }
    cv::destroyAllWindows();
    return 0;
}
