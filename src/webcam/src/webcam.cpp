#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <dirent.h>
#include <cstring>
#include <iostream>
#include <vector>
#include "../../server/main.hpp"
#include "../../server/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <functional>

class Webcam : public rclcpp::Node
{
public:
    Webcam(cv::VideoCapture &cap)
        : Node("Webcam"), vc_(cap), count_(0)
    {
        timer_ = this->create_wall_timer(10ms, std::bind(&Webcam::timer_callback, this));
        if (!cap.isOpened())
        {
            std::cerr << "Failed to open cam \n"
                      << std::endl;
            rclcpp::shutdown();
        }
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        vc_ >> frame;
        if (frame.empty())
            return;
        if (cv::waitKey(30) >= 0)
            return;

        client_send(frame, WEBCAM_PORT);
        RCLCPP_INFO(this->get_logger(), "Sent webcam feed");
    }
    cv::VideoCapture vc_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

bool isRealSenseCamera(const std::string &deviceName)
{
    return deviceName.find("RealSense") != std::string::npos ||
           deviceName.find("Intel") != std::string::npos;
}

std::string findWebcam()
{
    std::vector<std::string> allVideoDevices;
    DIR *devPath = opendir("/dev");
    if (devPath == nullptr)
        return "";

    struct dirent *entry;
    while ((entry = readdir(devPath)) != nullptr)
    {
        if (strncmp(entry->d_name, "video", 5) == 0)
        {
            allVideoDevices.emplace_back("/dev/" + std::string(entry->d_name));
        }
    }
    closedir(devPath);
    for (const auto &devicePath : allVideoDevices)
    {
        int fd = open(devicePath.c_str(), O_RDWR);
        if (fd == -1)
        {
            std::cerr << "Could not open the devices path I just found " << devicePath << "\n";
            continue;
        }

        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)
        {
            std::cerr << "Could not get tehj configs for " << devicePath << "\n";
            close(fd);
            continue;
        }

        std::string devVideoCard(reinterpret_cast<char *>(cap.card));
        std::cout << "Yo I just found the webcam, can you believe it, BOOM! : " << devicePath << " [" << devVideoCard << "]\n";

        if (!isRealSenseCamera(devVideoCard))
        {
            close(fd);
            return devicePath;
        }

        close(fd);
    }
    return "";
}

int main(int argc, char *argv[])
{
    std::string devicePath = findWebcam();
    if (devicePath.empty())
    {
        std::cerr << "No non-RealSense camera found.\n";
        return -1;
    }
    cv::VideoCapture cap(devicePath);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Webcam>(cap);
    rclcpp::spin(node);
    rclcpp::shutdown();
    cap.release();
    cv::destroyAllWindows();
    return 0;
}