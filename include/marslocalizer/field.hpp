//
// Created by Ivan on 1/12/26.
//

#ifndef MARSLOCALIZER_FIELD_HPP
#define MARSLOCALIZER_FIELD_HPP

#include <cstdint>
#include <fstream>
#include <string>
#include <unordered_map>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace apriltag {

class AprilTagInfo {
    friend class AprilTagField;
    friend class CameraLocalizer;

    std::uint32_t _id;
    double _size;
    Eigen::Affine3d _pose;
    Eigen::Matrix<double, 4, 3> _corners;
    cv::Mat _corners_cv;

public:

    AprilTagInfo();

    std::uint32_t id() const { return _id; }

    double size() const { return _size; }

    const Eigen::Affine3d& pose() const { return _pose; }

private:

    std::uint32_t& id() { return _id; }

    double& size() { return _size; }

    void set_pose(const Eigen::Affine3d& pose);

};

class AprilTagField {

    std::string _name = "Unnamed AprilTag field";
    std::string _tag_family;
    std::unordered_map<std::uint32_t, AprilTagInfo> _fiducials;

public:

    static std::shared_ptr<AprilTagField> parse(std::ifstream& input);

    const AprilTagInfo* tag(const std::uint32_t tag_id) const {
        if (const auto entry = _fiducials.find(tag_id); entry != _fiducials.end()) {
            return &entry->second;
        }
        return nullptr;
    }

    const std::string& name() const {
        return _name;
    }

    const std::string& tag_family() const {
        return _tag_family;
    }

    const std::unordered_map<std::uint32_t, AprilTagInfo>& fiducials() const {
        return _fiducials;
    }

};

}

#endif //MARSLOCALIZER_FIELD_HPP