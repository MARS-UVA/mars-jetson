#include "local_obstacle_avoidance.h"

std::pair<float, float> DynamicWindowNavigator::computeVelocities(
    RobotState &current_state,
    const std::vector<std::vector<float>> &gradients,
    float goal_x, float goal_y)
{
    // Calculate dynamic window based on current state
    float v_min = std::max(
        params.min_velocity,
        current_state.v - params.max_accel * params.dt);
    float v_max = std::min(
        params.max_velocity,
        current_state.v + params.max_accel * params.dt);
    float omega_min = std::max(
        params.min_omega,
        current_state.omega - params.max_angular_accel * params.dt);
    float omega_max = std::min(
        params.max_omega,
        current_state.omega + params.max_angular_accel * params.dt);

    float best_v = current_state.v;
    float best_omega = current_state.omega;
    float best_cost = std::numeric_limits<float>::max();

    // Discretize velocity space and evaluate trajectories
    const int v_samples = 20;
    const int omega_samples = 30;

    for (int i = 0; i < v_samples; i++)
    {
        float v = v_min + (v_max - v_min) * i / (v_samples - 1);

        for (int j = 0; j < omega_samples; j++)
        {
            float omega = omega_min + (omega_max - omega_min) * j / (omega_samples - 1);
            RobotState final_state = predictState(current_state, v, omega);
            // std::cout << "state: " << current_state.x << ", " << current_state.y << ", " << current_state.theta << std::endl;

            if (trajectoryCollides(current_state, final_state))
            {
                continue;
            }

            float heading_cost = calculateHeadingCost(final_state, goal_x, goal_y);
            float velocity_cost = calculateVelocityCost(v);
            float gradient_cost = calculateGradientCost(final_state, gradients);
            float clearance_cost = calculateClearanceCost(final_state);

            float total_cost =
                params.heading_weight * heading_cost +
                params.velocity_weight * velocity_cost +
                params.gradient_weight * gradient_cost +
                params.clearance_weight * clearance_cost;

            if (total_cost < best_cost)
            {
                best_cost = total_cost;
                best_v = v;
                best_omega = omega;
            }
        }
    }

    current_state.v = best_v;
    current_state.omega = best_omega;
    current_state.x += best_v * cos(current_state.theta) * params.dt;
    current_state.y += best_v * sin(current_state.theta) * params.dt;
    current_state.theta += best_omega * params.dt;

    std::cout << "Position: " << current_state.x << ", " << current_state.y << ", " << current_state.theta << std::endl;

    return {best_v, best_omega};
}

RobotState DynamicWindowNavigator::predictState(const RobotState &start, float v, float omega)
{
    RobotState predicted = start;
    float total_time = 0;

    while (total_time < params.predict_time)
    {
        predicted.x += v * cos(predicted.theta) * params.dt;
        predicted.y += v * sin(predicted.theta) * params.dt;
        predicted.theta += omega * params.dt;
        predicted.v = v;
        predicted.omega = omega;

        total_time += params.dt;
    }

    return predicted;
}

bool DynamicWindowNavigator::trajectoryCollides(const RobotState &start, const RobotState &end)
{
    // Add safety margin to robot radius
    const float SAFETY_MARGIN = 0.05;      // Adjust based on robot size
    const float MIN_OBSTACLE_DIST = 0.05; // Current threshold

    float dx = end.x - start.x;
    float dy = end.y - start.y;
    float dist = sqrt(dx * dx + dy * dy);

    int steps = static_cast<int>(dist / gridResolution) + 1; // Add +1 to ensure enough checks

    for (int i = 0; i <= steps; i++)
    {
        float ratio = static_cast<float>(i) / steps;
        float x = start.x + dx * ratio;
        float y = start.y + dy * ratio;

        auto obstacle = obstacleTree.findNearestObstacle(Vertex(x, y, 0));

        if (obstacle != nullptr)
        {
            float obstacle_dist = sqrt(pow(obstacle->getVertex().x - x, 2) +
                                       pow(obstacle->getVertex().y - y, 2));
            // Use both safety margin and minimum distance
            if (obstacle_dist < (MIN_OBSTACLE_DIST + SAFETY_MARGIN))
                return true;
        }
    }
    return false;
}

float DynamicWindowNavigator::calculateGradientCost(const RobotState &state,
                                                    const std::vector<std::vector<float>> &gradients)
{
    int i = static_cast<int>(state.y / gridResolution);
    int j = static_cast<int>(state.x / gridResolution);

    if (i >= 0 && i < gradients.size() && j >= 0 && j < gradients[0].size())
    {
        return gradients[i][j];
    }
    return std::numeric_limits<float>::max();
}

float DynamicWindowNavigator::calculateHeadingCost(const RobotState &state, float goal_x, float goal_y)
{
    float goal_heading = atan2(goal_y - state.y, goal_x - state.x);
    float heading_diff = std::abs(normalizeAngle(goal_heading - state.theta));

    // Add penalty for being too close to obstacles while turning
    auto nearest_obstacle = obstacleTree.findNearestObstacle(Vertex(state.x, state.y, 0));
    float obstacle_penalty = 0.0;

    if (nearest_obstacle != nullptr)
    {
        float dist = sqrt(pow(nearest_obstacle->getVertex().x - state.x, 2) +
                          pow(nearest_obstacle->getVertex().y - state.y, 2));
        if (dist < 200.0)
        { // Adjust this threshold
            obstacle_penalty = (200.0 - dist) / 200.0;
        }
    }

    return heading_diff + obstacle_penalty;
}

float DynamicWindowNavigator::calculateVelocityCost(float v)
{
    return (params.max_velocity - v) / params.max_velocity;
}

float DynamicWindowNavigator::calculateClearanceCost(const RobotState &state)
{
    float min_dist = std::numeric_limits<float>::max();
    float search_radius = 1.0f; // Change this once we get robot size!!!

    for (float angle = 0; angle < 2 * M_PI; angle += M_PI / 8)
    {
        float check_x = state.x + search_radius * cos(angle);
        float check_y = state.y + search_radius * sin(angle);

        auto obstacle = obstacleTree.findNearestObstacle(Vertex(check_x, check_y, 0));

        if (obstacle != nullptr)
        {
            float dist = sqrt(pow(check_x - state.x, 2) + pow(check_y - state.y, 2));
            min_dist = std::min(min_dist, dist);
        }
    }

    return 1.0f / (min_dist + 0.1f); // Division by 0 earlier so added by 0.1 in case of no distance
}

float DynamicWindowNavigator::normalizeAngle(float angle)
{
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}