#include "nav2_lazy_theta_star_planner/lazy_theta_star_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_lazy_theta_star_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();
constexpr std::array<int, 8> kNeighborX{{-1, 0, 1, -1, 1, -1, 0, 1}};
constexpr std::array<int, 8> kNeighborY{{-1, -1, -1, 0, 0, 1, 1, 1}};

struct Cell
{
  unsigned int x;
  unsigned int y;
};

struct Point
{
  double x;
  double y;
};

struct QueueEntry
{
  double estimate;
  double cost;
  std::size_t index;
};

struct QueueEntryGreater
{
  bool operator()(const QueueEntry & lhs, const QueueEntry & rhs) const
  {
    return lhs.estimate > rhs.estimate;
  }
};

double distance(const Point & a, const Point & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::size_t cellIndex(const Cell & cell, unsigned int width)
{
  return static_cast<std::size_t>(cell.y) * width + cell.x;
}

Cell indexCell(std::size_t index, unsigned int width)
{
  return Cell{
    static_cast<unsigned int>(index % width),
    static_cast<unsigned int>(index / width)};
}

Point cellCenter(const nav2_costmap_2d::Costmap2D & costmap, const Cell & cell)
{
  Point point{};
  costmap.mapToWorld(cell.x, cell.y, point.x, point.y);
  return point;
}

Point pointAt(const Point & start, const Point & end, double ratio)
{
  return Point{
    start.x + (end.x - start.x) * ratio,
    start.y + (end.y - start.y) * ratio};
}

double cross(const Point & a, const Point & b)
{
  return a.x * b.y - a.y * b.x;
}

}  // namespace

void LazyThetaStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  name_ = std::move(name);

  auto node = parent_.lock();
  if (!node) {
    throw nav2_core::InvalidPlanner("Planner lifecycle node expired during configuration");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".turn_penalty_weight", rclcpp::ParameterValue(turn_penalty_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".interpolation_step", rclcpp::ParameterValue(interpolation_step_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".smoothing_radius", rclcpp::ParameterValue(smoothing_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(allow_unknown_));

  node->get_parameter(name_ + ".turn_penalty_weight", turn_penalty_weight_);
  node->get_parameter(name_ + ".interpolation_step", interpolation_step_);
  node->get_parameter(name_ + ".smoothing_radius", smoothing_radius_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);

  if (!std::isfinite(turn_penalty_weight_) || turn_penalty_weight_ < 0.0 ||
    !std::isfinite(interpolation_step_) || interpolation_step_ <= 0.0 ||
    !std::isfinite(smoothing_radius_) || smoothing_radius_ <= 0.0)
  {
    throw nav2_core::InvalidPlanner("Planner weights and distances must be finite and non-negative");
  }
  if (!costmap_ros_ || costmap_ros_->getCostmap() == nullptr) {
    throw nav2_core::InvalidPlanner("A valid costmap is required");
  }
}

void LazyThetaStarPlanner::cleanup()
{
  costmap_ros_.reset();
  tf_.reset();
  parent_.reset();
}

void LazyThetaStarPlanner::activate()
{
}

void LazyThetaStarPlanner::deactivate()
{
}

nav_msgs::msg::Path LazyThetaStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  auto node = parent_.lock();
  if (!node || !costmap_ros_) {
    throw nav2_core::PlannerException("Planner is not configured");
  }

  auto * costmap = costmap_ros_->getCostmap();
  if (costmap == nullptr) {
    throw nav2_core::PlannerException("Costmap is unavailable");
  }
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap->getMutex());

  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
    throw nav2_core::StartOutsideMapBounds("Start pose is outside the costmap");
  }
  if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    throw nav2_core::GoalOutsideMapBounds("Goal pose is outside the costmap");
  }

  const unsigned int width = costmap->getSizeInCellsX();
  const unsigned int height = costmap->getSizeInCellsY();
  const Cell start_cell{start_x, start_y};
  const Cell goal_cell{goal_x, goal_y};
  const auto traversable = [costmap, this](const Cell & cell) {
      const auto cost = costmap->getCost(cell.x, cell.y);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        return allow_unknown_;
      }
      return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    };

  if (!traversable(start_cell)) {
    throw nav2_core::StartOccupied("Start pose is on an occupied costmap cell");
  }
  if (!traversable(goal_cell)) {
    throw nav2_core::GoalOccupied("Goal pose is on an occupied costmap cell");
  }

  const auto lineOfSight = [&traversable](Cell from, Cell to) {
      int x = static_cast<int>(from.x);
      int y = static_cast<int>(from.y);
      const int end_x = static_cast<int>(to.x);
      const int end_y = static_cast<int>(to.y);
      const int dx = std::abs(end_x - x);
      const int sx = x < end_x ? 1 : -1;
      const int dy = -std::abs(end_y - y);
      const int sy = y < end_y ? 1 : -1;
      int error = dx + dy;

      while (true) {
        if (!traversable(Cell{static_cast<unsigned int>(x), static_cast<unsigned int>(y)})) {
          return false;
        }
        if (x == end_x && y == end_y) {
          return true;
        }

        const int doubled_error = 2 * error;
        const bool move_x = doubled_error >= dy;
        const bool move_y = doubled_error <= dx;
        const int old_x = x;
        const int old_y = y;
        if (move_x) {
          error += dy;
          x += sx;
        }
        if (move_y) {
          error += dx;
          y += sy;
        }
        if (move_x && move_y &&
          (!traversable(Cell{static_cast<unsigned int>(x), static_cast<unsigned int>(old_y)}) ||
          !traversable(Cell{static_cast<unsigned int>(old_x), static_cast<unsigned int>(y)})))
        {
          return false;
        }
      }
    };

  const std::size_t start_index = cellIndex(start_cell, width);
  const std::size_t goal_index = cellIndex(goal_cell, width);
  const std::size_t cell_count = static_cast<std::size_t>(width) * height;
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> costs(cell_count, infinity);
  std::vector<std::size_t> parents(cell_count, kNoParent);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> open;
  const double resolution = costmap->getResolution();

  const auto edgeDistance = [costmap](std::size_t from, std::size_t to) {
      const Point a = cellCenter(*costmap, indexCell(from, costmap->getSizeInCellsX()));
      const Point b = cellCenter(*costmap, indexCell(to, costmap->getSizeInCellsX()));
      return distance(a, b);
    };
  const auto transitionCost = [&costs, &parents, &edgeDistance, costmap, width, this](
    std::size_t from, std::size_t to)
    {
      const double length = edgeDistance(from, to);
      const std::size_t previous = parents[from];
      if (previous == kNoParent || previous == from) {
        return costs[from] + length;
      }
      const Point before = cellCenter(*costmap, indexCell(previous, width));
      const Point pivot = cellCenter(*costmap, indexCell(from, width));
      const Point after = cellCenter(*costmap, indexCell(to, width));
      const double incoming = std::atan2(pivot.y - before.y, pivot.x - before.x);
      const double outgoing = std::atan2(after.y - pivot.y, after.x - pivot.x);
      const double turn = std::abs(normalizeAngle(outgoing - incoming));
      return costs[from] + length + turn_penalty_weight_ * turn;
    };

  costs[start_index] = 0.0;
  parents[start_index] = start_index;
  const Point goal_center = cellCenter(*costmap, goal_cell);
  open.push(QueueEntry{
    distance(cellCenter(*costmap, start_cell), goal_center),
    0.0,
    start_index});

  bool reached_goal = false;
  while (!open.empty()) {
    if (cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Lazy Theta* planning was cancelled");
    }

    const QueueEntry entry = open.top();
    open.pop();
    if (closed[entry.index] || entry.cost > costs[entry.index]) {
      continue;
    }

    const Cell current = indexCell(entry.index, width);
    const std::size_t parent_index = parents[entry.index];
    if (parent_index != entry.index &&
      !lineOfSight(indexCell(parent_index, width), current))
    {
      double repaired_cost = infinity;
      std::size_t repaired_parent = kNoParent;
      for (std::size_t direction = 0; direction < kNeighborX.size(); ++direction) {
        const int neighbor_x = static_cast<int>(current.x) + kNeighborX[direction];
        const int neighbor_y = static_cast<int>(current.y) + kNeighborY[direction];
        if (neighbor_x < 0 || neighbor_y < 0 ||
          neighbor_x >= static_cast<int>(width) || neighbor_y >= static_cast<int>(height))
        {
          continue;
        }
        const Cell neighbor{
          static_cast<unsigned int>(neighbor_x), static_cast<unsigned int>(neighbor_y)};
        const std::size_t neighbor_index = cellIndex(neighbor, width);
        if (!closed[neighbor_index] || !lineOfSight(neighbor, current)) {
          continue;
        }
        const double candidate = transitionCost(neighbor_index, entry.index);
        if (candidate < repaired_cost) {
          repaired_cost = candidate;
          repaired_parent = neighbor_index;
        }
      }

      if (repaired_parent == kNoParent) {
        continue;
      }
      parents[entry.index] = repaired_parent;
      costs[entry.index] = repaired_cost;
      const Point current_center = cellCenter(*costmap, current);
      open.push(QueueEntry{
        repaired_cost + distance(current_center, goal_center),
        repaired_cost,
        entry.index});
      continue;
    }

    closed[entry.index] = true;
    if (entry.index == goal_index) {
      reached_goal = true;
      break;
    }

    const std::size_t shortcut_parent = parents[entry.index];
    for (std::size_t direction = 0; direction < kNeighborX.size(); ++direction) {
      const int neighbor_x = static_cast<int>(current.x) + kNeighborX[direction];
      const int neighbor_y = static_cast<int>(current.y) + kNeighborY[direction];
      if (neighbor_x < 0 || neighbor_y < 0 ||
        neighbor_x >= static_cast<int>(width) || neighbor_y >= static_cast<int>(height))
      {
        continue;
      }
      const Cell neighbor{
        static_cast<unsigned int>(neighbor_x), static_cast<unsigned int>(neighbor_y)};
      if (!traversable(neighbor)) {
        continue;
      }
      const std::size_t neighbor_index = cellIndex(neighbor, width);
      if (closed[neighbor_index]) {
        continue;
      }

      const double candidate = transitionCost(shortcut_parent, neighbor_index);
      if (candidate >= costs[neighbor_index]) {
        continue;
      }
      costs[neighbor_index] = candidate;
      parents[neighbor_index] = shortcut_parent;
      const Point neighbor_center = cellCenter(*costmap, neighbor);
      open.push(QueueEntry{
        candidate + distance(neighbor_center, goal_center),
        candidate,
        neighbor_index});
    }
  }

  if (!reached_goal) {
    throw nav2_core::NoValidPathCouldBeFound("Lazy Theta* could not find a collision-free path");
  }

  std::vector<Point> waypoints;
  for (std::size_t current = goal_index;; current = parents[current]) {
    waypoints.push_back(cellCenter(*costmap, indexCell(current, width)));
    if (current == start_index) {
      break;
    }
    if (parents[current] == kNoParent || waypoints.size() > cell_count) {
      throw nav2_core::NoValidPathCouldBeFound("Planner produced an invalid parent chain");
    }
  }
  std::reverse(waypoints.begin(), waypoints.end());
  const Point start_point{start.pose.position.x, start.pose.position.y};
  const Point goal_point{goal.pose.position.x, goal.pose.position.y};
  waypoints.front() = start_point;
  if (waypoints.size() == 1) {
    if (distance(start_point, goal_point) > 1e-6) {
      waypoints.push_back(goal_point);
    }
  } else {
    waypoints.back() = goal_point;
  }

  std::vector<Point> corners;
  corners.reserve(waypoints.size());
  for (const Point & waypoint : waypoints) {
    if (corners.empty() || distance(corners.back(), waypoint) > 1e-6) {
      corners.push_back(waypoint);
    }
  }
  for (std::size_t i = 1; i + 1 < corners.size();) {
    const Point incoming{corners[i].x - corners[i - 1].x, corners[i].y - corners[i - 1].y};
    const Point outgoing{corners[i + 1].x - corners[i].x, corners[i + 1].y - corners[i].y};
    const double incoming_length = std::hypot(incoming.x, incoming.y);
    const double outgoing_length = std::hypot(outgoing.x, outgoing.y);
    if (incoming_length < 1e-6 || outgoing_length < 1e-6) {
      corners.erase(corners.begin() + static_cast<std::ptrdiff_t>(i));
      continue;
    }
    const double turn = std::acos(std::clamp(
        (incoming.x * outgoing.x + incoming.y * outgoing.y) /
        (incoming_length * outgoing_length), -1.0, 1.0));
    if (turn < 1e-3) {
      ++i;
      continue;
    }
    if (std::abs(kPi - turn) < 1e-3) {
      throw nav2_core::NoValidPathCouldBeFound(
              "Cannot create a collision-free fillet for a reversing path corner");
    }
    const double tangent_factor = std::tan(turn * 0.5);
    const double max_radius =
      0.45 * std::min(incoming_length, outgoing_length) / tangent_factor;
    double radius = std::min(smoothing_radius_, max_radius);
    const double direction = cross(incoming, outgoing) >= 0.0 ? 1.0 : -1.0;
    std::vector<Point> arc;
    Point tangent_start{};
    Point tangent_end{};
    bool found_safe_arc = false;
    for (int attempt = 0; attempt < 12 && radius > 1e-3; ++attempt) {
      const double tangent_distance = radius * tangent_factor;
      tangent_start = Point{
        corners[i].x - incoming.x / incoming_length * tangent_distance,
        corners[i].y - incoming.y / incoming_length * tangent_distance};
      tangent_end = Point{
        corners[i].x + outgoing.x / outgoing_length * tangent_distance,
        corners[i].y + outgoing.y / outgoing_length * tangent_distance};
      const Point center{
        tangent_start.x - direction * incoming.y / incoming_length * radius,
        tangent_start.y + direction * incoming.x / incoming_length * radius};
      const double start_angle = std::atan2(
        tangent_start.y - center.y, tangent_start.x - center.x);
      const double sweep = direction * turn;
      const double arc_length = radius * turn;
      const double sample_step = std::min(interpolation_step_, resolution * 0.5);
      const std::size_t count = std::max<std::size_t>(
        2, static_cast<std::size_t>(std::ceil(arc_length / sample_step)));
      arc.clear();
      arc.reserve(count + 1);
      for (std::size_t sample = 0; sample <= count; ++sample) {
        const double angle = start_angle + sweep *
          static_cast<double>(sample) / static_cast<double>(count);
        arc.push_back(Point{
          center.x + radius * std::cos(angle),
          center.y + radius * std::sin(angle)});
      }

      bool safe = true;
      for (std::size_t sample = 1; sample < arc.size(); ++sample) {
        unsigned int from_x = 0;
        unsigned int from_y = 0;
        unsigned int to_x = 0;
        unsigned int to_y = 0;
        if (!costmap->worldToMap(arc[sample - 1].x, arc[sample - 1].y, from_x, from_y) ||
          !costmap->worldToMap(arc[sample].x, arc[sample].y, to_x, to_y) ||
          !lineOfSight(Cell{from_x, from_y}, Cell{to_x, to_y}))
        {
          safe = false;
          break;
        }
      }
      if (safe) {
        found_safe_arc = true;
        break;
      }
      radius *= 0.5;
    }

    if (found_safe_arc) {
      corners[i] = tangent_start;
      corners.insert(
        corners.begin() + static_cast<std::ptrdiff_t>(i + 1),
        arc.begin() + 1, arc.end() - 1);
      corners.insert(
        corners.begin() + static_cast<std::ptrdiff_t>(i + arc.size() - 1),
        tangent_end);
      i += arc.size();
    } else {
      throw nav2_core::NoValidPathCouldBeFound(
              "Could not create a collision-free fillet for a path corner");
    }
  }

  std::vector<Point> sampled;
  sampled.push_back(corners.front());
  for (std::size_t i = 1; i < corners.size(); ++i) {
    const double segment_length = distance(corners[i - 1], corners[i]);
    const std::size_t segments = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(segment_length / interpolation_step_)));
    for (std::size_t step = 1; step <= segments; ++step) {
      sampled.push_back(pointAt(
        corners[i - 1], corners[i],
        static_cast<double>(step) / static_cast<double>(segments)));
    }
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();
  path.header.stamp = node->now();
  path.poses.reserve(sampled.size());
  for (std::size_t i = 0; i < sampled.size(); ++i) {
    double yaw = 0.0;
    if (sampled.size() > 1) {
      const std::size_t before = i == 0 ? 0 : i - 1;
      const std::size_t after = i + 1 < sampled.size() ? i + 1 : i;
      yaw = std::atan2(
        sampled[after].y - sampled[before].y,
        sampled[after].x - sampled[before].x);
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = sampled[i].x;
    pose.pose.position.y = sampled[i].y;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    path.poses.push_back(std::move(pose));
  }

  if (!path.poses.empty()) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  }
  return path;
}

}  // namespace nav2_lazy_theta_star_planner

PLUGINLIB_EXPORT_CLASS(
  nav2_lazy_theta_star_planner::LazyThetaStarPlanner,
  nav2_core::GlobalPlanner)
