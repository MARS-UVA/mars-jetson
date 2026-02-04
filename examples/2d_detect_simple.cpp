//
// Created by Ivan on 11/23/25.
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

    cv::VideoCapture capture {camera_index};
    if (!capture.isOpened()) {
        std::cerr << "Could not open camera at index " << camera_index << std::endl;
        return 1;
    }

    cv::Mat frame;
    cv::Mat gray_frame;
    while (true) {
        if (capture.read(frame)) {
            cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
            if (std::vector<apriltag::AprilTagDetection> detections = detector.detect(gray_frame);
                    !detections.empty()) {
                std::cout << "Detected " << detections.size() << " AprilTags:\n";

                for (const auto& detection : detections) {
                    std::cout
                        << "\tID "
                        << detection.id()
                        << ": ("
                        << detection.center().x()
                        << ", "
                        << detection.center().y()
                        << ")\n";
                }
            }
        }
    }
    cv::destroyAllWindows();
    return 0;
}
