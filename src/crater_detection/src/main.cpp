#include "rclcpp/rclcpp.hpp"

#include "crater_detection/crater_detector.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<CraterDetector> detector { std::make_shared<CraterDetector>() };
    rclcpp::spin(detector);

    rclcpp::shutdown();

    return 0;
}
