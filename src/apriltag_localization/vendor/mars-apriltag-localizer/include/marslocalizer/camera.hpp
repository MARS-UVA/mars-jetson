#ifndef MARSLOCALIZER_CAMERA_HPP
#define MARSLOCALIZER_CAMERA_HPP

#include <Eigen/Dense>

namespace apriltag {

/**
 * Data describing intrinsic information about the camera.
 */
struct CameraIntrinsics {

    /**
     * Focal length in the x direction.
     */
    double fx;

    /**
     * Focal length in the y direction.
     */
    double fy;

    /**
     * Optical center of the camera in the x direction.
     */
    double cx;

    /**
     * Optical center of the camera in the y direction.
     */
    double cy;

    constexpr CameraIntrinsics(const double fx, const double fy, const double cx, const double cy)
        : fx{fx}, fy{fy}, cx{cx}, cy{cy} {
    }

    explicit CameraIntrinsics(const Eigen::Matrix3d& matrix)
        : fx{matrix(0, 0)}, fy{matrix(1, 1)}, cx{matrix(0, 2)}, cy{matrix(1, 2)} {
    }

    operator Eigen::Matrix3d() const;

};

/**
 * Data describing radial distortion due to the camera's lens.
 *
 * In real camera lenses, the coefficients will probably be monotonic (either increasing or decreasing). Users should
 * reexamine any camera calibrations that produce non-monotonic radial distortion coefficients.
 */
struct RadialDistortionCoefficients {
    double k1, k2, k3, k4, k5, k6;

    constexpr RadialDistortionCoefficients()
        : k1{0}, k2{0}, k3{0}, k4{0}, k5{0}, k6{0} {
    }

    constexpr RadialDistortionCoefficients(const double k1, const double k2)
        : k1{k1}, k2{k2}, k3{0}, k4{0}, k5{0}, k6{0} {
    }

    constexpr RadialDistortionCoefficients(const double k1, const double k2, const double k3)
        : k1{k1}, k2{k2}, k3{k3}, k4{0}, k5{0}, k6{0} {
    }

    constexpr RadialDistortionCoefficients(
        const double k1, const double k2, const double k3, const double k4, const double k5, const double k6)
        : k1{k1}, k2{k2}, k3{k3}, k4{k4}, k5{k5}, k6{k6} {
    }

    constexpr bool is_zero() const;

};

struct TangentialDistortionCoefficients {
    double p1, p2;

    constexpr TangentialDistortionCoefficients()
        : p1{0}, p2{0} {
    }

    constexpr TangentialDistortionCoefficients(const double p1, const double p2)
        : p1{p1}, p2{p2} {
    }

    constexpr bool is_zero() const;

};

struct ThinPrismDistortionCoefficients {
    double s1, s2, s3, s4;

    constexpr ThinPrismDistortionCoefficients()
        : s1{0}, s2{0}, s3{0}, s4{0} {
    }

    constexpr ThinPrismDistortionCoefficients(const double s1, const double s2, const double s3, const double s4)
        : s1{s1}, s2{s2}, s3{s3}, s4{s4} {
    }

    constexpr bool is_zero() const;

};

struct CameraInfo {
    CameraIntrinsics camera_intrinsics;
    RadialDistortionCoefficients radial_distortion_coefficients;
    TangentialDistortionCoefficients tangential_distortion_coefficients;
    ThinPrismDistortionCoefficients thin_prism_distortion_coefficients;

    explicit CameraInfo(const CameraIntrinsics& camera_intrinsics)
        : camera_intrinsics{camera_intrinsics} {
    }

    constexpr CameraInfo(
        const CameraIntrinsics& camera_intrinsics,
        const RadialDistortionCoefficients& radial_distortion_coefficients,
        const TangentialDistortionCoefficients& tangential_distortion_coefficients)
        : camera_intrinsics{camera_intrinsics}, radial_distortion_coefficients{radial_distortion_coefficients},
          tangential_distortion_coefficients{tangential_distortion_coefficients} {
    }

    constexpr CameraInfo(
        const CameraIntrinsics& camera_intrinsics,
        const RadialDistortionCoefficients& radial_distortion_coefficients,
        const TangentialDistortionCoefficients& tangential_distortion_coefficients,
        const ThinPrismDistortionCoefficients& thin_prism_distortion_coefficients)
        : camera_intrinsics{camera_intrinsics}, radial_distortion_coefficients{radial_distortion_coefficients},
          tangential_distortion_coefficients{tangential_distortion_coefficients},
          thin_prism_distortion_coefficients{thin_prism_distortion_coefficients} {
    }

    explicit CameraInfo(const Eigen::Matrix3d& camera_intrinsics)
        : camera_intrinsics{camera_intrinsics(0,0) , camera_intrinsics(1,1),
            camera_intrinsics(0,2), camera_intrinsics(1,2)} {
    }

    CameraInfo(const Eigen::Matrix3d& camera_intrinsics, [[maybe_unused]] const Eigen::Vector<double, 0>& _)
        : camera_intrinsics{camera_intrinsics(0,0) , camera_intrinsics(1,1),
            camera_intrinsics(0,2), camera_intrinsics(1,2)} {
    }

    CameraInfo(
        const Eigen::Matrix3d& camera_intrinsics, const Eigen::Vector4d& distortion_coefficients)
        : camera_intrinsics{camera_intrinsics(0,0) , camera_intrinsics(1,1),
            camera_intrinsics(0,2), camera_intrinsics(1,2)},
          radial_distortion_coefficients{distortion_coefficients(0), distortion_coefficients(1)},
          tangential_distortion_coefficients{distortion_coefficients(2), distortion_coefficients(3)} {
    }

    CameraInfo(
        const Eigen::Matrix3d& camera_intrinsics, const Eigen::Vector<double, 5>& distortion_coefficients)
        : camera_intrinsics{camera_intrinsics(0,0) , camera_intrinsics(1,1),
            camera_intrinsics(0,2), camera_intrinsics(1,2)},
          radial_distortion_coefficients{distortion_coefficients(0), distortion_coefficients(1),
              distortion_coefficients(4)},
          tangential_distortion_coefficients{distortion_coefficients(2), distortion_coefficients(3)} {
    }

    CameraInfo(
        const Eigen::Matrix3d& camera_intrinsics, const Eigen::Vector<double, 8>& distortion_coefficients)
        : camera_intrinsics{camera_intrinsics(0,0) , camera_intrinsics(1,1),
            camera_intrinsics(0,2), camera_intrinsics(1,2)},
          radial_distortion_coefficients{distortion_coefficients(0), distortion_coefficients(1),
              distortion_coefficients(4), distortion_coefficients(5), distortion_coefficients(6),
              distortion_coefficients(7)},
          tangential_distortion_coefficients{distortion_coefficients(2), distortion_coefficients(3)} {
    }

    CameraInfo(
        const Eigen::Matrix3d& camera_intrinsics, const Eigen::Vector<double, 12>& distortion_coefficients)
        : camera_intrinsics{camera_intrinsics(0,0) , camera_intrinsics(1,1),
            camera_intrinsics(0,2), camera_intrinsics(1,2)},
          radial_distortion_coefficients{distortion_coefficients(0), distortion_coefficients(1),
              distortion_coefficients(4), distortion_coefficients(5), distortion_coefficients(6),
              distortion_coefficients(7)},
          tangential_distortion_coefficients{distortion_coefficients(2), distortion_coefficients(3)},
          thin_prism_distortion_coefficients{distortion_coefficients(8), distortion_coefficients(9),
              distortion_coefficients(10), distortion_coefficients(11)} {
    }

    [[nodiscard]] Eigen::Matrix3d matrix() const;

    [[nodiscard]] Eigen::Vector<double, 12> distortion_vector() const;
     
};

}

#endif // MARSLOCALIZER_CAMERA_HPP
