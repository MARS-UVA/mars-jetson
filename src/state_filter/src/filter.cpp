#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <rclcpp/rclcpp.hpp>

#include "state_filter/filter.hpp"
#include "state_filter/util.hpp"

namespace filter {

    /// Computes the "mathematical" modulus of *value* in *limit*. The result of the function is always between 0 and *limit*.
    template <typename T>
    T mod(const T& value, const T& limit) {
        return std::fmod(limit + std::fmod(value, limit), limit);
    }

    template <typename T>
    void normalize_angle(T& theta) {
        if (theta > M_PI || theta <= -M_PI) {
            theta = mod(theta + M_PI, 2 * M_PI) - M_PI;
        }
    }

    template <typename T, int vars, int cols, int theta_index>
    void normalize_angles(Eigen::Ref<Eigen::Matrix<T, vars, cols>> states) {
        static_assert(theta_index >= 0 && theta_index < vars, "Invalid value for theta_index");
        for (Eigen::Index i{0}; i < cols; i++) {
            normalize_angle(states(theta_index, i));
        }
    }

    void compute_sigma_points(const State& last_estimate /* x_hat_prev */, const Cholesky<StateCovariance>& last_estimate_covariance_cholesky /* S_prev */, const double& eta, SigmaPoints& sigma_points /* ? -> X_prev */) {
        StateCovariance shifts = last_estimate_covariance_cholesky.matrixL();
        shifts *= eta;
        sigma_points.leftCols<1>() = last_estimate;
        sigma_points.middleCols<STATE_VARS>(1) = shifts.colwise() + last_estimate;
        sigma_points.rightCols<STATE_VARS>() = (-shifts).colwise() + last_estimate;
        normalize_angles<double, STATE_VARS, K, STATE_THETA>(sigma_points);
    }

    template <typename T, int vars, int theta_index>
    void unscented_transform(const Eigen::Matrix<T, vars, K>& sigma_points /* X */, const Eigen::Matrix<T, K, 1>& mean_weights /* W(m) */, const Eigen::Matrix<T, K, 1>& covariance_weights /* W(c) */, const Eigen::Matrix<T, vars, vars>& additional_covariance_sqrt /* Q or R */, Eigen::Matrix<T, vars, 1>& mean /* ? -> x_hat */, Cholesky<Eigen::Matrix<T, vars, vars>>& covariance_cholesky /* ? -> S */, Eigen::Matrix<T, vars, K>& distance /* ? -> X_i - x_hat */, const rclcpp::Logger& logger) {
        mean = sigma_points * mean_weights;  // x_hat = X * W(m)
        if constexpr (theta_index >= 0) {
            normalize_angles<T, vars, 1, theta_index>(mean);
        }

        distance = sigma_points.colwise() - mean;  // X - x_hat
        if constexpr (theta_index >= 0) {
            normalize_angles<T, vars, K, theta_index>(distance);
        }

        Eigen::Matrix<double, K - 1 + vars, vars> combined;
        combined.template topRows<K - 1>() = std::sqrt(covariance_weights(1)) * distance.template rightCols<K - 1>().transpose();
        combined.template bottomRows<vars>() = additional_covariance_sqrt;
        covariance_cholesky.set_upper(combined.householderQr().matrixQR().template topRightCorner<vars, vars>());
        covariance_cholesky.rankUpdate(distance.template leftCols<1>(), covariance_weights(0));
        if (covariance_cholesky.info() == Eigen::NumericalIssue) {
            RCLCPP_WARN(logger, "A covariance estimate has diverged");
        }
    }

    void RobotModelFilter::predict(const double& dt, const StateCovariance& process_model_covariance_sqrt) {
        compute_sigma_points(_state_estimate, _state_covariance_estimate_cholesky, _eta, _sigma_points);

        SigmaPoints transformed_sigma_points = _sigma_points;  // X*
        auto transformed_sigma_points_colwise = transformed_sigma_points.colwise();
        for (auto it = transformed_sigma_points_colwise.begin(); it < transformed_sigma_points_colwise.end(); it++) {
            _process_model(*it, dt);
        }

        // Predict the next state
        unscented_transform<double, STATE_VARS, STATE_THETA>(transformed_sigma_points, _mean_weights, _covariance_weights, process_model_covariance_sqrt, _state_estimate, _state_covariance_estimate_cholesky, _prediction_residual, _logger);
    }

    template <int vars, int theta_index>
    void update_from_observation(const Eigen::Matrix<double, vars, 1>& observation /* y_hat_curr */, const Eigen::Matrix<double, vars, vars>& covariance_sqrt /* R_curr */, const std::function<Eigen::Matrix<double, vars, 1>(State)>& observation_model /* h_curr */, const SigmaPoints& sigma_points /* X_prev */, const Weights& mean_weights /* W(m) */, const Weights& covariance_weights /* W(c) */, State& state_estimate /* x_hat_prev -> x_hat_curr */, Cholesky<StateCovariance>& state_estimate_covariance_cholesky /* P_prev -> P_curr */, const rclcpp::Logger& logger) {
        using Observation = Eigen::Matrix<double, vars, 1>;
        using ObservationCovariance = Eigen::Matrix<double, vars, vars>;
        using ObservationSigmaPoints = Eigen::Matrix<double, vars, K>;
        using StateReconstruction = Eigen::Matrix<double, STATE_VARS, vars>;

        ObservationSigmaPoints observation_sigma_points;  // Y_curr
        auto sigma_points_colwise = sigma_points.colwise();
        std::transform(sigma_points_colwise.begin(), sigma_points_colwise.end(), observation_sigma_points.colwise().begin(), observation_model);

        Observation observation_prediction;  // y_hat_curr
        Cholesky<ObservationCovariance> observation_prediction_covariance_cholesky;  // S_y_curr
        ObservationSigmaPoints observation_distance;  // (Y_curr)_i - y_hat_curr
        unscented_transform<double, vars, theta_index>(observation_sigma_points, mean_weights, covariance_weights, covariance_sqrt, observation_prediction, observation_prediction_covariance_cholesky, observation_distance, logger);

        SigmaPoints last_distance = sigma_points.colwise() - state_estimate;  // (X_prev)_i - x_hat_prev
        if constexpr (theta_index >= 0) {
            normalize_angles<double, STATE_VARS, K, STATE_THETA>(last_distance);
        }
        // TODO: Double check this step
        StateReconstruction cross_covariance = last_distance * covariance_weights.asDiagonal() * observation_distance.transpose();  // P_xy_curr = [ (X_prev)_i - x_hat_prev ] * W(c) * [ (Y_curr)_i - y_hat_curr ]T
        StateReconstruction kalman_gain = observation_prediction_covariance_cholesky.solve(cross_covariance.transpose()).transpose();  // (S_y_curr * (S_y_curr)T) * (K_curr)T = (P_xy_curr)T

        Observation residual = observation - observation_prediction;  // y_curr - y_hat_curr
        if constexpr (theta_index >= 0) {
            normalize_angles<double, vars, 1, theta_index>(residual);
        }
        state_estimate += kalman_gain * residual;  // x_hat_curr = x_hat_prev + K_curr * (y_curr - y_hat_curr)
        normalize_angles<double, STATE_VARS, 1, STATE_THETA>(state_estimate);
        StateReconstruction update = kalman_gain * observation_prediction_covariance_cholesky.matrixU();  // U_curr = K_curr * (S_y_curr)T
        auto update_colwise = update.colwise();
        // S_curr * (S_curr)T = S_prev * (S_prev)T - sum( (U_curr)_i * ((U_curr)_i)T for i in observation variable indices )
        for (auto it = update_colwise.begin(); it < update_colwise.end(); it++) {
            state_estimate_covariance_cholesky.rankUpdate(*it, -1);
        }
    }

    void RobotModelFilter::update_from_apriltag(const AprilTagObservation &observation, const AprilTagObservationCovariance& covariance_sqrt) {
        compute_sigma_points(_state_estimate, _state_covariance_estimate_cholesky, _eta, _sigma_points);

        update_from_observation<APRILTAG_VARS, APRILTAG_THETA>(observation, covariance_sqrt, _apriltag_observation_model, _sigma_points, _mean_weights, _covariance_weights, _state_estimate, _state_covariance_estimate_cholesky, _logger);
    }

    void RobotModelFilter::update_from_gyro(const GyroObservation &observation, const GyroObservationCovariance& covariance_sqrt) {
        compute_sigma_points(_state_estimate, _state_covariance_estimate_cholesky, _eta, _sigma_points);

        update_from_observation<GYRO_VARS, GYRO_THETA>(observation, covariance_sqrt, _gyro_observation_model, _sigma_points, _mean_weights, _covariance_weights, _state_estimate, _state_covariance_estimate_cholesky, _logger);
    }

} /* namespace filter */