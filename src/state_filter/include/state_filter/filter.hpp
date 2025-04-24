#ifndef state_filter_filter_hpp
#define state_filter_filter_hpp

#include <cmath>
#include <exception>
#include <functional>
#include <Eigen/Dense>
#include <rclcpp/logger.hpp>

#include "util.hpp"

namespace filter {

/// Number of dimensions to the state vector (x, x', x'', y, y', y'', theta, theta', theta'').
constexpr int STATE_VARS = 9;
/// Index for the x variable in the state vector.
constexpr int STATE_X = 0;
/// Index for the x' variable in the state vector.
constexpr int STATE_DX = 1;
/// Index for the x'' variable in the state vector.
constexpr int STATE_DDX = 2;
/// Index for the y variable in the state vector.
constexpr int STATE_Y = 3;
/// Index for the y' variable in the state vector.
constexpr int STATE_DY = 4;
/// Index for the y'' variable in the state vector.
constexpr int STATE_DDY = 5;
/// Index for the theta variable in the state vector.
constexpr int STATE_THETA = 6;
/// Index for the theta' variable in the state vector.
constexpr int STATE_DTHETA = 7;
/// Index for the theta'' variable in the state vector.
constexpr int STATE_DDTHETA = 8;
/// Number of dimensions to AprilTag observation vectors (x, y, theta).
constexpr int APRILTAG_VARS = 3;
/// Index for the x variable in the AprilTag observation vector.
constexpr int APRILTAG_X = 0;
/// Index for the y variable in the AprilTag observation vector.
constexpr int APRILTAG_Y = 1;
/// Index for the theta variable in the AprilTag observation vector.
constexpr int APRILTAG_THETA = 2;
/// Number of dimensions to gyro observation vectors (theta, theta', x'', y'')
constexpr int GYRO_VARS = 4;
/// Index for the theta variable in the gyro observation vector.
constexpr int GYRO_THETA = 0;
/// Index for the theta' variable in the gyro observation vector.
constexpr int GYRO_DTHETA = 1;
/// Index for the x'' variable in the gyro observation vector.
constexpr int GYRO_DDX = 2;
/// Index for the y'' variable in the gyro observation vector.
constexpr int GYRO_DDY = 3;
/// Number of sigma points (2 \* STATE\_VARS + 1).
constexpr int K = 19;

/// Matrix type for the state vector.
using State = Eigen::Matrix<double, STATE_VARS, 1>;
/// Matrix type for the state covariance matrix.
using StateCovariance = Eigen::Matrix<double, STATE_VARS, STATE_VARS>;

/// Matrix type for the AprilTag observation vector.
using AprilTagObservation = Eigen::Matrix<double, APRILTAG_VARS, 1>;
/// Matrix type for the AprilTag observation covariance matrix.
using AprilTagObservationCovariance = Eigen::Matrix<double, APRILTAG_VARS, APRILTAG_VARS>;
/// Matrix type for the gyro observation vector.
using GyroObservation = Eigen::Matrix<double, GYRO_VARS, 1>;
/// Matrix type for the gyro observation covariance matrix.
using GyroObservationCovariance = Eigen::Matrix<double, GYRO_VARS, GYRO_VARS>;

using AprilTagKalmanGain = Eigen::Matrix<double, STATE_VARS, APRILTAG_VARS>;
using GyroKalmanGain = Eigen::Matrix<double, STATE_VARS, GYRO_VARS>;

using SigmaPoints = Eigen::Matrix<double, STATE_VARS, K>;
using Weights = Eigen::Matrix<double, K, 1>;
using AprilTagObservationSigmaPoints = Eigen::Matrix<double, APRILTAG_VARS, K>;
using GyroObservationSigmaPoints = Eigen::Matrix<double, GYRO_VARS, K>;

using ProcessModel = std::function<void(Eigen::Ref<State>, const double&)>;
using AprilTagObservationModel = std::function<AprilTagObservation(State)>;
using GyroObservationModel = std::function<GyroObservation(State)>;

/// An unscented Kalman filter for our robot.
class RobotModelFilter {

    State _state_estimate;
    Cholesky<StateCovariance> _state_covariance_estimate_cholesky;

    SigmaPoints _sigma_points;
    AprilTagObservationSigmaPoints _apriltag_observation_sigma_points;
    GyroObservationSigmaPoints _gyro_observation_sigma_points;
    Weights _mean_weights;
    Weights _covariance_weights;

    ProcessModel _process_model;
    AprilTagObservationModel _apriltag_observation_model;
    GyroObservationModel _gyro_observation_model;

    AprilTagKalmanGain _apriltag_kalman_gain;
    GyroKalmanGain _gyro_kalman_gain;

    SigmaPoints _prediction_residual;

    double _alpha;
    double _beta;
    double _kappa;
    double _lambda;
    double _eta;

    rclcpp::Logger _logger;

public:

    RobotModelFilter(State initial_state,
                     StateCovariance initial_state_covariance,
                     ProcessModel process_model,
                     AprilTagObservationModel apriltag_observation_model,
                     GyroObservationModel gyro_observation_model,
                     double alpha,
                     double beta,
                     double kappa,
                     rclcpp::Logger logger)
    : _state_estimate(initial_state), _process_model(process_model), _apriltag_observation_model(apriltag_observation_model), _gyro_observation_model(gyro_observation_model), _alpha(alpha), _beta(beta), _kappa(kappa), _logger(logger) {
        _lambda = alpha * alpha * (STATE_VARS + kappa) - STATE_VARS;
        _eta = std::sqrt(_lambda + STATE_VARS);

        _mean_weights(0) = _lambda / (_lambda + STATE_VARS);
        _mean_weights(Eigen::seq(1, Eigen::last)) = Eigen::Matrix<double, K - 1, 1>::Constant(0.5 / (_lambda + STATE_VARS));
        _covariance_weights(0) = _mean_weights(0) + 1 - (_alpha * _alpha) + _beta;
        _covariance_weights(Eigen::seq(1, Eigen::last)) = _mean_weights(Eigen::seq(1, Eigen::last), 0);

        _state_covariance_estimate_cholesky = Cholesky(initial_state_covariance);
        if (_state_covariance_estimate_cholesky.info() == Eigen::NumericalIssue) {
            throw std::invalid_argument("Initial state covariance is not positive definite");
        }
    }

    /// Gets the current estimate of the robot's state.
    const State& state_estimate() const { return _state_estimate; }
    /// Gets the current covariance of the state estimate.
    StateCovariance state_covariance_estimate() const { return _state_covariance_estimate_cholesky.reconstructedMatrix(); }

    /// Predicts the next state of the robot.
    /// - Parameter dt: The time, in seconds, since the last prediction was made.
    void predict(const double& dt, const StateCovariance& process_model_covariance_sqrt);

    /// Updates the state of the model from an observation from the AprilTag pose estimator.
    /// - Parameters:
    ///   - observation: Observation from the AprilTag pose estimator.
    ///   - covariance: Covariance of the observation.
    void update_from_apriltag(const AprilTagObservation& observation, const AprilTagObservationCovariance& covariance_sqrt);

    /// Updates the state of the model from an observation from the gyro.
    /// - Parameters:
    ///   - observation: Observation from the gyro.
    ///   - covariance: Covariance of the observation.
    void update_from_gyro(const GyroObservation& observation, const GyroObservationCovariance& covariance_sqrt);

};

template <typename T>
void normalize_angle(T& theta);

/// Normalizes the angle parameter in the provided vector or matrix of column vectors to be in the range (-pi/2, pi/2].
/// - Parameter states: Vectors which have angles which may need to be normalized.
template <typename T, int vars, int cols, int theta_index>
void normalize_angles(Eigen::Ref<Eigen::Matrix<T, vars, cols>> states);

} /* namespace filter */

#endif /* state_filter_filter_hpp */
