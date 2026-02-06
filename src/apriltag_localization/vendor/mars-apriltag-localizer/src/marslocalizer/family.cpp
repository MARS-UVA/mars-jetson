#include "wrapper.hpp"

#include <apriltag.h>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

std::map<const apriltag_family_t*, std::shared_ptr<apriltag::AprilTagFamily>> apriltag::AprilTagFamily::_instances {};

std::shared_ptr<apriltag::AprilTagFamily> apriltag::AprilTagFamily::get(apriltag_family_t* family, Deleter& deleter) {
    if (const auto entry = _instances.find(family); entry != _instances.end()) {
        return entry->second;
    }
    auto new_obj = std::make_shared<AprilTagFamily>(ConstructorKey{}, family, deleter);
    _instances.emplace(family, new_obj);
    return new_obj;
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
std::shared_ptr<apriltag::AprilTagFamily> apriltag::AprilTagFamily::get_existing(apriltag_family_t* family) {
    if (const auto entry = _instances.find(family); entry != _instances.end()) {
        return entry->second;
    }
    throw std::invalid_argument("AprilTag family doesn't already exist");
}

std::string_view apriltag::AprilTagFamily::name() const {
    return _family->name;
}

int apriltag::AprilTagFamily::width_at_border() const {
    return _family->width_at_border;
}

int apriltag::AprilTagFamily::total_width() const {
    return _family->total_width;
}

std::uint32_t apriltag::AprilTagFamily::minimum_hamming_distance() const {
    return _family->h;
}

apriltag_family_t* apriltag::AprilTagFamily::raw() const {
    return _family;
}
