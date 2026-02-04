//
// Created by Ivan on 1/12/26.
//

#include "field.hpp"

#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace {

Eigen::Matrix<double, 3, 4> get_corner_points(const double tag_size) {
    Eigen::Matrix<double, 3, 4> corners;
    corners << +1, -1, -1, +1,
               +1, +1, -1, -1,
                0,  0,  0,  0;
    corners *= tag_size / 2;
    return corners;
}

constexpr double deg2rad(const double degrees) {
    return degrees * EIGEN_PI / 180; // NOLINT(*-avoid-magic-numbers)
}

}

apriltag::AprilTagInfo::AprilTagInfo()
: _id(0), _size(0) {
}

void apriltag::AprilTagInfo::set_pose(const Eigen::Affine3d& pose) {
    _pose = pose;
    _corners = (pose * get_corner_points(_size)).transpose().eval();
    cv::eigen2cv(_corners, _corners_cv);
}

std::shared_ptr<apriltag::AprilTagField> apriltag::AprilTagField::parse(std::ifstream& input) {
    const json json_data = json::parse(input);

    if (!json_data.contains("fiducials")) {
        throw std::invalid_argument("Bad AprilTag field JSON: no fiducials provided");
    }

    std::shared_ptr<AprilTagField> field = std::make_shared<AprilTagField>();

    if (json_data.contains("name")) {
        field->_name = json_data.at("name").get<std::string>();
    };

    if (!json_data.contains("tag_family")) {
        throw std::invalid_argument("Bad AprilTag field JSON: tag_family not provided");
    }
    field->_tag_family = json_data.at("tag_family").get<std::string>();

    const json fiducials = json_data.at("fiducials").get<json>();
    std::for_each(fiducials.cbegin(), fiducials.cend(), [&](const json& fiducial) {

        AprilTagInfo info;

        if (!fiducial.contains("id")) {
            throw std::invalid_argument("Bad AprilTag field JSON: ID was not provided for an Apriltag");
        }
        info._id = fiducial.at("id").get<std::uint32_t>();
        if (auto search = field->_fiducials.find(info._id); search != field->_fiducials.end()) {
            std::ostringstream oss;
            oss << "Bad AprilTag field JSON: more than one AprilTag with ID " << info._id;
        }

        if (!fiducial.contains("size")) {
            std::ostringstream oss;
            oss << "Bad AprilTag field JSON: size not provided for AprilTag with ID " << info._id;
            throw std::invalid_argument(oss.str());
        }
        info._size = fiducial.at("size").get<double>();

        Eigen::Affine3d pose = Eigen::Affine3d::Identity();

        if (!fiducial.contains("translation")) {
            std::ostringstream oss;
            oss << "Bad AprilTag field JSON: translation not provided for AprilTag with ID " << info._id;
            throw std::invalid_argument(oss.str());
        }
        const json translation_array = fiducial.at("translation").get<json>();
        pose.translation() <<
            translation_array.at(0).get<double>(),
            translation_array.at(1).get<double>(),
            translation_array.at(2).get<double>();

        if (!fiducial.contains("rotation")) {
            std::ostringstream oss;
            oss << "Bad AprilTag field JSON: rotation not provided for AprilTag with ID " << info._id;
            throw std::invalid_argument(oss.str());
        }
        const json angles = fiducial.at("rotation").get<json>();
        pose.linear() = (Eigen::AngleAxisd(deg2rad(angles.at(2)), Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(deg2rad(angles.at(1)), Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(deg2rad(angles.at(0)), Eigen::Vector3d::UnitX())).toRotationMatrix();

        info.set_pose(pose);

        field->_fiducials[info._id] = info;
    });

    return field;
}
