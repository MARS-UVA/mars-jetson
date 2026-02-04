//
// Created by Ivan on 10/28/25.
//

#include <iostream>
#include <tagStandard41h12.h>
#include <opencv2/opencv.hpp>

#include "overlays.hpp"
#include "wrapper.hpp"

int main(const int argc, const char* argv[]) {
    int camera_index;
    if (argc < 2) {
        std::cerr << "Camera index not specified; defaulting to 0" << std::endl;
        camera_index = 0;
    } else {
        camera_index = std::stoi(argv[1]);
    }

    const auto family = apriltag::AprilTagFamily::get(tagStandard41h12_create(), tagStandard41h12_destroy);
    apriltag::AprilTagDetector detector;
    detector.nthreads() = 8;
    detector.add_family(family);

    cv::VideoCapture capture { "/Users/ivan/Downloads/apriltag_tests/H 145cm A 0°/1.mp4" };
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
            if (std::vector<apriltag::AprilTagDetection> detections = detector.detect(gray_frame);
                    !detections.empty()) {
                apriltag::overlay_squares(frame, detections);
                apriltag::overlay_labels(frame, detections);
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
