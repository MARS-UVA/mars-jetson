#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

#include <vector>
#include <cmath>
#include "models/realsense.h"
#include "models/obstacle_clustering_tree.h"

struct RobotState
{
    float x, y;
    float theta;
    float v;
    float omega;

    RobotState(float x, float y, float theta, float v, float omega)
        : x(x), y(y), theta(theta), v(v), omega(omega) {}
};

struct DWAParams
{
    // Robot physical constraints - max and min linear, angular velocities and accelerations --> change once we determine the actual values
    float max_velocity = 1.0f;
    float min_velocity = -1.0f;
    float max_omega = M_PI / 2;
    float min_omega = -M_PI / 2;
    float max_accel = 0.5f;
    float max_angular_accel = M_PI / 4;

    float dt = 0.1f;
    float predict_time = 4.0f;

    float heading_weight = 0.6f;
    float velocity_weight = 0.2f;
    float gradient_weight = 0.8f;
    float clearance_weight = 1.0f;
};

class DynamicWindowNavigator
{
private:
    DWAParams params;
    ObstacleClusteringTree &obstacleTree;
    float gridResolution;

public:
    DynamicWindowNavigator(ObstacleClusteringTree &tree, float resolution)
        : obstacleTree(tree), gridResolution(resolution) {}

    std::pair<float, float> computeVelocities(
        RobotState &current_state,
        const std::vector<std::vector<float>> &gradients,
        float goal_x, float goal_y);

private:
    RobotState predictState(const RobotState &start, float v, float omega);

    bool trajectoryCollides(const RobotState &start, const RobotState &end);

    float calculateGradientCost(const RobotState &state,
                                const std::vector<std::vector<float>> &gradients);

    float calculateHeadingCost(const RobotState &state, float goal_x, float goal_y);

    float calculateVelocityCost(float v);

    float calculateClearanceCost(const RobotState &state);

    float normalizeAngle(float angle);
};

#endif