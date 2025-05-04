// #include <iostream>
// #include <string>
// #include <opencv2/opencv.hpp>
// #include <fcntl.h>
// #include <sys/ioctl.h>
// #include <linux/videodev2.h>
// #include <dirent.h>
// #include <cstring>
// #include <iostream>
// #include <vector>
// #include "../../server/main.hpp"
// #include "../../server/client.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float32_multi_array.hpp"
// #include <chrono>
// #include <functional>

// using namespace std::chrono_literals;
// class Webcam : public rclcpp::Node
// {
// public:
//     Webcam(cv::VideoCapture &cap)
//         : Node("Webcam"), vc_(cap), count_(0)
//     {
//         timer_ = this->create_wall_timer(10ms, std::bind(&Webcam::timer_callback, this));
//         if (!vc_.isOpened())
//         {
//             std::cerr << "Failed to open cam" << std::endl;
//             rclcpp::shutdown();
//         }
//     }

// private:
//     void timer_callback()
//     {
//         cv::Mat frame;
//         vc_ >> frame;
//         if (frame.empty())
//             return;
//         if (cv::waitKey(30) >= 0)
//             return;

//         client_send(frame, WEBCAM_PORT);
//         RCLCPP_INFO(this->get_logger(), "Sent webcam feed");
//     }
//     cv::VideoCapture vc_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     size_t count_;
// };

// bool isRealSenseCamera(const std::string &deviceName)
// {
//     return deviceName.find("RealSense") != std::string::npos ||
//            deviceName.find("Intel") != std::string::npos;
// }

// std::string findWebcam()
// {
//     std::vector<std::string> allVideoDevices;
//     DIR *devPath = opendir("/dev");
//     if (devPath == nullptr)
//         return "";

//     struct dirent *entry;
//     while ((entry = readdir(devPath)) != nullptr)
//     {
//         if (strncmp(entry->d_name, "video", 5) == 0)
//         {
//             allVideoDevices.emplace_back("/dev/" + std::string(entry->d_name));
//         }
//     }
//     closedir(devPath);
//     for (const auto &devicePath : allVideoDevices)
//     {
//         int fd = open(devicePath.c_str(), O_RDWR);
//         if (fd == -1)
//         {
//             std::cerr << "Could not open the devices path I just found " << devicePath << "\n";
//             continue;
//         }

//         struct v4l2_capability cap;
//         if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)
//         {
//             std::cerr << "Could not get tehj configs for " << devicePath << "\n";
//             close(fd);
//             continue;
//         }

//         std::string devVideoCard(reinterpret_cast<char *>(cap.card));
//         std::cout << "Yo I just found the webcam, can you believe it, BOOM! : " << devicePath << " [" << devVideoCard << "]\n";

//         if (!isRealSenseCamera(devVideoCard))
//         {
//             std::cout << "Found a non-RealSense camera" << std::endl;
//             close(fd);
//             return devicePath;
//         }
//         else
//         {
//             std::cout << "Found a RealSense camera" << std::endl;
//         }

//         close(fd);
//     }
//     return "";
// }

// int main(int argc, char *argv[])
// {
//     std::string devicePath = findWebcam();
//     if (devicePath.empty())
//     {
//         std::cerr << "No non-RealSense camera found.\n";
//         return -1;
//     }
//     cv::VideoCapture cap(devicePath, cv2::CAP_V4L2);
//     if (!cap.isOpened())
//     {
//         std::cerr << "Failed to open the cam before init of ros node" << std::endl;
//     }
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<Webcam>(cap);
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     cap.release();
//     cv::destroyAllWindows();
//     return 0;
// }

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <dirent.h>
#include <cstring>
#include <vector>
#include "../../server/main.hpp"
#include "../../server/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

class Webcam : public rclcpp::Node
{
public:
    Webcam(const std::string &devicePath)
        : Node("Webcam"), count_(0)
    {
        cap_ = cv::VideoCapture(devicePath, cv::CAP_V4L2);

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera at %s", devicePath.c_str());
            int deviceIndex = std::stoi(devicePath.substr(10));
            cap_.release();
            cap_ = cv::VideoCapture(deviceIndex, cv::CAP_V4L2);

            if (!cap_.isOpened())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera by index %d", deviceIndex);
                rclcpp::shutdown();
                return;
            }
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        RCLCPP_INFO(this->get_logger(), "Camera opened successfully");
        timer_ = this->create_wall_timer(10ms, std::bind(&Webcam::timer_callback, this));
    }

    ~Webcam()
    {
        if (cap_.isOpened())
        {
            cap_.release();
        }
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Camera disconnected");
            return;
        }

        cap_ >> frame;
        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty frame");
            return;
        }
        client_send(frame, WEBCAM_PORT);
        RCLCPP_INFO(this->get_logger(), "Sent webcam feed");
    }

    cv::VideoCapture cap_;
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
    {
        std::cerr << "Failed to open /dev directory" << std::endl;
        return "";
    }

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
            std::cerr << "Could not open device path: " << devicePath << std::endl;
            continue;
        }

        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)
        {
            std::cerr << "Could not get capabilities for: " << devicePath << std::endl;
            close(fd);
            continue;
        }

        std::string devVideoCard(reinterpret_cast<char *>(cap.card));
        std::cout << "Found device: " << devicePath << " [" << devVideoCard << "]" << std::endl;

        if (!isRealSenseCamera(devVideoCard))
        {
            std::cout << "Found a non-RealSense camera" << std::endl;
            close(fd);
            return devicePath;
        }
        else
        {
            std::cout << "Found a RealSense camera" << std::endl;
        }

        close(fd);
    }

    std::cerr << "No suitable camera found" << std::endl;
    return "";
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::string devicePath = findWebcam();
    if (devicePath.empty())
    {
        std::cerr << "No non-RealSense camera found." << std::endl;
        rclcpp::shutdown();
        return -1;
    }

    std::cout << "Device path: " << devicePath << std::endl;

    {
        cv::VideoCapture testCap(devicePath, cv::CAP_V4L2);
        if (!testCap.isOpened())
        {
            std::cerr << "Initial test: Failed to open camera with OpenCV" << std::endl;
            testCap.release();
            testCap = cv::VideoCapture(devicePath, cv::CAP_ANY);

            if (!testCap.isOpened())
            {
                std::cerr << "Failed to open camera with any backend, trying index method..." << std::endl;
                int deviceIndex = std::stoi(devicePath.substr(10));
                testCap.release();
                testCap = cv::VideoCapture(deviceIndex);

                if (!testCap.isOpened())
                {
                    std::cerr << "All camera opening methods failed." << std::endl;
                }
            }
        }
        else
        {
            std::cout << "Test camera open successful!" << std::endl;
        }
        testCap.release();
    }
    auto node = std::make_shared<Webcam>(devicePath);
    rclcpp::spin(node);

    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}