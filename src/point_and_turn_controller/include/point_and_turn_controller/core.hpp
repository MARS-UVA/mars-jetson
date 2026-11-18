// ROS-free core of the point-and-turn controller: path simplification and the
// ALIGN / DRIVE / FINAL_ALIGN state machine. Plain C++17 so it can be unit
// tested and closed-loop simulated without a ROS graph. The plugin wrapper
// (point_and_turn_controller.hpp) owns parameters, TF, costmap checks and
// debug topics, and calls into this class once per control cycle.
#ifndef POINT_AND_TURN_CONTROLLER__CORE_HPP_
#define POINT_AND_TURN_CONTROLLER__CORE_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace point_and_turn_controller
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Command
{
  double linear{0.0};   // m/s along the robot's x axis; negative when reversing
  double angular{0.0};  // rad/s about z
};

enum class State { IDLE, ALIGN, DRIVE, FINAL_ALIGN, DONE };

const char * toString(State state);

// Wraps an angle to (-pi, pi].
double normalizeAngle(double angle);

struct SimplifierParams
{
  // Points within this perpendicular distance of the corridor centre line
  // belong to the current segment.
  double corridor_tolerance{0.05};
  // The corridor direction is taken from the first point at least this far
  // from the anchor, and a corner closer than this to the goal is merged
  // into the goal.
  double min_segment_length{0.10};
};

// Reduces a dense planner path to its corner vertices (Reumann-Witkam
// corridor pass). Keeps the first and last points. See core.cpp for details.
std::vector<Point2D> simplifyPath(
  const std::vector<Point2D> & path, const SimplifierParams & params);

struct Params
{
  double max_linear_speed{0.40};
  double min_linear_speed{0.05};
  double linear_decel{0.40};
  double max_angular_speed{0.80};
  double min_angular_speed{0.15};
  double angular_gain{1.5};
  double align_tolerance{0.05};
  double realign_tolerance{0.20};
  double max_cross_track_error{0.15};
  double min_bearing_distance{0.15};
  double segment_arrival_tolerance{0.05};
  double goal_arrival_tolerance{0.10};
  double final_align_tolerance{0.10};
  double replan_match_tolerance{0.25};
  // 0 keeps DRIVE purely linear. See README "Heading correction".
  double drive_heading_gain{0.0};
  double max_drive_angular_speed{0.20};
  bool allow_reversing{true};
  SimplifierParams simplifier;
};

// Returns an empty string when the parameters are usable, otherwise a
// description of the first problem found.
std::string validate(const Params & params);

class PointAndTurnCore
{
public:
  explicit PointAndTurnCore(const Params & params);

  // Stores the plan; it is applied by the next update(), which has the robot
  // pose that matching and nearest-segment selection need.
  void setPlan(const std::vector<Pose2D> & plan);

  // One control cycle. The robot pose must be in the plan's frame.
  Command update(const Pose2D & robot);

  void reset();

  // Scales max_linear_speed; clamped to [0, 1].
  void setSpeedScale(double scale);

  State state() const {return state_;}
  bool hasPlan() const {return has_pending_ || !vertices_.empty();}
  const std::vector<Point2D> & vertices() const {return vertices_;}
  // Incremented whenever vertices() changes, so the wrapper publishes the
  // debug path only when there is something new.
  std::size_t verticesRevision() const {return vertices_revision_;}
  // Signed rotation still to go in ALIGN / FINAL_ALIGN after the last
  // update(); 0 in other states. Used for the swept-footprint check.
  double remainingRotation() const {return remaining_rotation_;}
  const Params & params() const {return params_;}

private:
  void applyPlan(const Pose2D & robot);
  bool matchTarget(const std::vector<Point2D> & vertices, std::size_t & segment) const;
  void startAtNearestSegment(const Pose2D & robot);
  void enterSegment(std::size_t segment, const Pose2D & robot);
  void enterDrive(const Pose2D & robot);
  bool isLastSegment() const {return segment_ + 2 == vertices_.size();}
  const Point2D & target() const {return vertices_[segment_ + 1];}
  double targetHeading(const Pose2D & robot) const;
  double headingError(const Pose2D & robot) const;
  Command rotate(double error) const;
  bool step(const Pose2D & robot, Command & command);

  Params params_;
  double speed_scale_{1.0};

  std::vector<Pose2D> pending_;
  bool has_pending_{false};

  std::vector<Point2D> vertices_;
  std::size_t vertices_revision_{0};
  double goal_yaw_{0.0};
  State state_{State::IDLE};
  std::size_t segment_{0};
  // Per-segment travel direction, decided once on entry: -1 drives backward.
  int direction_{1};
  // The line DRIVE measures progress and cross-track against.
  Point2D drive_origin_;
  double drive_heading_{0.0};
  double remaining_rotation_{0.0};
};

}  // namespace point_and_turn_controller

#endif  // POINT_AND_TURN_CONTROLLER__CORE_HPP_
