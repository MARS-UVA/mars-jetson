#include "common.hpp"

#include <apriltag.h>
#include <cstdint>
#include <exception>
#include <map>
#include <memory>
#include <string>

#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h10.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>

std::map<const apriltag_family_t*, std::shared_ptr<apriltag::AprilTagFamily>> apriltag::AprilTagFamily::_instances {};

std::shared_ptr<apriltag::AprilTagFamily> apriltag::AprilTagFamily::get(const std::string_view& name) {
    if (name == "tag16h5") {
        return get(tag16h5_create(), tag16h5_destroy);
    }
    if (name == "tag25h9") {
        return get(tag25h9_create(), tag25h9_destroy);
    }
    if (name == "tag36h10") {
        return get(tag36h10_create(), tag36h10_destroy);
    }
    if (name == "tag36h11") {
        return get(tag36h11_create(), tag36h11_destroy);
    }
    if (name == "tagCircle21h7") {
        return get(tagCircle21h7_create(), tagCircle21h7_destroy);
    }
    if (name == "tagCircle49h12") {
        return get(tagCircle49h12_create(), tagCircle49h12_destroy);
    }
    if (name == "tagCustom48h12") {
        return get(tagCustom48h12_create(), tagCustom48h12_destroy);
    }
    if (name == "tagStandard41h12") {
        return get(tagStandard41h12_create(), tagStandard41h12_destroy);
    }
    if (name == "tagStandard52h13") {
        return get(tagStandard52h13_create(), tagStandard52h13_destroy);
    }
    throw std::invalid_argument("Family not recognized");
}

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
