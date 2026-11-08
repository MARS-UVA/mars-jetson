#include "point_and_turn_controller/core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace point_and_turn_controller
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
// Consecutive points closer than this are treated as one point.
constexpr double kDuplicateDistance = 1e-6;
// A transition re-runs the new state in the same cycle (see update()). Real
// chains are at most ALIGN -> next segment -> DRIVE; this only bounds a bug.
constexpr int kMaxStepsPerCycle = 8;

Point2D position(const Pose2D & pose)
{
  return {pose.x, pose.y};
}

double distance(const Point2D & a, const Point2D & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

double distanceToSegment(const Point2D & p, const Point2D & a, const Point2D & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_sq = dx * dx + dy * dy;
  double t = 0.0;
  if (length_sq > 0.0) {
    t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length_sq, 0.0, 1.0);
  }
  return distance(p, {a.x + t * dx, a.y + t * dy});
}

}  // namespace

const char * toString(State state)
{
  switch (state) {
    case State::IDLE: return "IDLE";
    case State::ALIGN: return "ALIGN";
    case State::DRIVE: return "DRIVE";
    case State::FINAL_ALIGN: return "FINAL_ALIGN";
    case State::DONE: return "DONE";
  }
  return "UNKNOWN";
}

double normalizeAngle(double angle)
{
  // fmod keeps this O(1) for any input, unlike a while loop adding 2*pi.
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle <= 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

// Reumann-Witkam: lay a corridor of half-width corridor_tolerance from the
// current anchor and absorb points until one leaves it; that segment's end
// becomes the next anchor. Each point is examined a bounded number of times
// (only the short tail after a double-back vertex is scanned twice), so the
// pass is O(n), unlike Douglas-Peucker's O(n log n) average / O(n^2) worst.
std::vector<Point2D> simplifyPath(
  const std::vector<Point2D> & path, const SimplifierParams & params)
{
  std::vector<Point2D> vertices;
  if (path.empty()) {
    return vertices;
  }
  vertices.push_back(path.front());
  const std::size_t last = path.size() - 1;
  std::size_t anchor = 0;
  while (anchor < last) {
    const Point2D & a = path[anchor];
    // Planner noise right next to the anchor would give a meaningless
    // direction, so aim the corridor at the first point a segment away.
    std::size_t probe = anchor + 1;
    while (probe <= last && distance(a, path[probe]) < params.min_segment_length) {
      ++probe;
    }
    if (probe > last) {
      break;  // The rest is shorter than a segment; the goal is appended below.
    }
    const double length = distance(a, path[probe]);
    const double ux = (path[probe].x - a.x) / length;
    const double uy = (path[probe].y - a.y) / length;

    // Points before the probe lie within min_segment_length of the anchor and
    // are absorbed. probe > anchor, so every pass makes progress.
    //
    // Either way the corridor ends, the vertex is the point of maximum
    // progress along it. A Reeds-Shepp cusp reverses along the same line and
    // never leaves the corridor sideways, so it is caught by progress falling
    // back past the tolerance; the last point before that would cut the cusp
    // short. When the path leaves sideways while still advancing, maximum
    // progress is the last point inside (classic Reumann-Witkam). At a 90+
    // degree corner it is the corner itself rather than a point up to
    // corridor_tolerance along the next leg, which would put the vertex off
    // the robot's line and trip the bearing check just before arrival.
    std::size_t vertex = last;
    double max_progress = -std::numeric_limits<double>::infinity();
    std::size_t max_progress_index = probe;
    for (std::size_t i = probe; i <= last; ++i) {
      const double dx = path[i].x - a.x;
      const double dy = path[i].y - a.y;
      const double along = dx * ux + dy * uy;
      const double perpendicular = std::abs(dx * uy - dy * ux);
      if (perpendicular > params.corridor_tolerance ||
        along < max_progress - params.corridor_tolerance)
      {
        vertex = max_progress_index;
        break;
      }
      if (along > max_progress) {
        max_progress = along;
        max_progress_index = i;
      }
    }
    if (distance(vertices.back(), path[vertex]) > kDuplicateDistance) {
      vertices.push_back(path[vertex]);
    }
    anchor = vertex;
  }
  if (distance(vertices.back(), path[last]) > kDuplicateDistance) {
    vertices.push_back(path[last]);
  }
  // A corner a few centimetres before the goal would cost a pivot for almost
  // no travel. Merge it into the goal, but never drop the first point.
  while (vertices.size() >= 3 &&
    distance(vertices[vertices.size() - 2], vertices.back()) < params.min_segment_length)
  {
    vertices.erase(vertices.end() - 2);
  }
  return vertices;
}

std::string validate(const Params & p)
{
  // Written as !(x > 0) so NaN fails too: every comparison with NaN is false.
  if (!(p.max_linear_speed > 0.0)) {return "max_linear_speed must be > 0";}
  if (!(p.min_linear_speed >= 0.0 && p.min_linear_speed <= p.max_linear_speed)) {
    return "min_linear_speed must be in [0, max_linear_speed]";
  }
  if (!(p.linear_decel > 0.0)) {return "linear_decel must be > 0";}
  if (!(p.max_angular_speed > 0.0)) {return "max_angular_speed must be > 0";}
  if (!(p.min_angular_speed >= 0.0 && p.min_angular_speed <= p.max_angular_speed)) {
    return "min_angular_speed must be in [0, max_angular_speed]";
  }
  if (!(p.angular_gain > 0.0)) {return "angular_gain must be > 0";}
  if (!(p.align_tolerance > 0.0)) {return "align_tolerance must be > 0";}
  if (!(p.align_tolerance < p.realign_tolerance)) {
    return "align_tolerance must be smaller than realign_tolerance; without the "
           "gap ALIGN and DRIVE hand the robot back and forth on sensor noise";
  }
  if (!(p.max_cross_track_error > 0.0)) {return "max_cross_track_error must be > 0";}
  if (!(p.min_bearing_distance >= 0.0)) {return "min_bearing_distance must be >= 0";}
  if (!(p.segment_arrival_tolerance > 0.0)) {return "segment_arrival_tolerance must be > 0";}
  if (!(p.goal_arrival_tolerance > 0.0)) {return "goal_arrival_tolerance must be > 0";}
  if (!(p.final_align_tolerance > 0.0)) {return "final_align_tolerance must be > 0";}
  if (!(p.replan_match_tolerance >= 0.0)) {return "replan_match_tolerance must be >= 0";}
  if (!(p.drive_heading_gain >= 0.0)) {return "drive_heading_gain must be >= 0";}
  if (!(p.max_drive_angular_speed >= 0.0)) {return "max_drive_angular_speed must be >= 0";}
  if (!(p.simplifier.corridor_tolerance > 0.0)) {return "corridor_tolerance must be > 0";}
  if (!(p.simplifier.min_segment_length > 0.0)) {return "min_segment_length must be > 0";}
  return {};
}

PointAndTurnCore::PointAndTurnCore(const Params & params)
: params_(params)
{
}

void PointAndTurnCore::setPlan(const std::vector<Pose2D> & plan)
{
  if (plan.empty()) {
    reset();
    return;
  }
  pending_ = plan;
  has_pending_ = true;
}

void PointAndTurnCore::reset()
{
  // The speed scale survives: it comes from a speed-limit zone, not the plan.
  pending_.clear();
  has_pending_ = false;
  vertices_.clear();
  ++vertices_revision_;
  state_ = State::IDLE;
  segment_ = 0;
  direction_ = 1;
  remaining_rotation_ = 0.0;
}

void PointAndTurnCore::setSpeedScale(double scale)
{
  speed_scale_ = std::isfinite(scale) ? std::clamp(scale, 0.0, 1.0) : 1.0;
}

Command PointAndTurnCore::update(const Pose2D & robot)
{
  if (has_pending_) {
    applyPlan(robot);
  }
  Command command;
  remaining_rotation_ = 0.0;
  if (vertices_.empty()) {
    return command;
  }
  // When a state hands over, run the new state now rather than publishing a
  // zero command for one cycle; a stop-start every transition would cost
  // time and look like a stall to the progress checker.
  for (int i = 0; i < kMaxStepsPerCycle && step(robot, command); ++i) {
  }
  return command;
}

void PointAndTurnCore::applyPlan(const Pose2D & robot)
{
  has_pending_ = false;
  std::vector<Point2D> points;
  points.reserve(pending_.size());
  for (const Pose2D & pose : pending_) {
    points.push_back(position(pose));
  }
  const double goal_yaw = pending_.back().yaw;
  pending_.clear();

  std::vector<Point2D> vertices = simplifyPath(points, params_.simplifier);
  // A plan that collapses to one point still needs a segment to drive.
  if (vertices.size() == 1 &&
    distance(position(robot), vertices.front()) >= params_.goal_arrival_tolerance)
  {
    vertices.insert(vertices.begin(), position(robot));
  }

  // The behavior tree replans about once a second. Restarting every time
  // would re-align the robot each second, so keep the current state when the
  // new plan still goes where the robot is heading.
  const bool aligning_to_goal = state_ == State::FINAL_ALIGN || state_ == State::DONE;
  const bool same_goal = aligning_to_goal && !vertices_.empty() &&
    distance(vertices.back(), vertices_.back()) <= params_.goal_arrival_tolerance;
  std::size_t matched_segment = 0;
  const bool same_target = (state_ == State::ALIGN || state_ == State::DRIVE) &&
    matchTarget(vertices, matched_segment);

  vertices_ = std::move(vertices);
  ++vertices_revision_;
  goal_yaw_ = goal_yaw;

  if (same_goal) {
    // Arrived already: never re-drive. Only a changed goal yaw matters.
    segment_ = vertices_.size() >= 2 ? vertices_.size() - 2 : 0;
    if (state_ == State::DONE &&
      std::abs(normalizeAngle(goal_yaw_ - robot.yaw)) > params_.final_align_tolerance)
    {
      state_ = State::FINAL_ALIGN;
    }
    return;
  }
  if (same_target) {
    // Keep the state, direction and drive origin; only the index moves.
    segment_ = matched_segment;
    return;
  }
  if (vertices_.size() == 1) {
    // One point and the robot is already on it.
    segment_ = 0;
    state_ = State::FINAL_ALIGN;
    return;
  }
  startAtNearestSegment(robot);
}

bool PointAndTurnCore::matchTarget(
  const std::vector<Point2D> & vertices, std::size_t & segment) const
{
  // Index 0 is the new plan's start, which is near the robot, not a target.
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < vertices.size(); ++i) {
    const double d = distance(vertices[i], target());
    if (d < best) {
      best = d;
      segment = i - 1;
    }
  }
  return best <= params_.replan_match_tolerance;
}

void PointAndTurnCore::startAtNearestSegment(const Pose2D & robot)
{
  // Strict < gives ties to the earliest segment. At a cusp the outbound and
  // return segments overlap, and the robot has not done the return leg yet.
  std::size_t best_segment = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < vertices_.size(); ++i) {
    const double d = distanceToSegment(position(robot), vertices_[i], vertices_[i + 1]);
    if (d < best) {
      best = d;
      best_segment = i;
    }
  }
  enterSegment(best_segment, robot);
}

void PointAndTurnCore::enterSegment(std::size_t segment, const Pose2D & robot)
{
  segment_ = segment;
  state_ = State::ALIGN;
  direction_ = 1;
  if (params_.allow_reversing) {
    // Decided once here, not every cycle: re-deciding near +/-90 degrees
    // would flip the robot between forward and reverse on noise.
    const double forward_error = normalizeAngle(targetHeading(robot) - robot.yaw);
    if (std::abs(forward_error) > kPi / 2.0) {
      direction_ = -1;
    }
  }
}

void PointAndTurnCore::enterDrive(const Pose2D & robot)
{
  // Re-anchored on every entry, so cross-track measures drift since the
  // robot last aligned, not distance from a path it has already left.
  drive_origin_ = position(robot);
  drive_heading_ = targetHeading(robot);
  state_ = State::DRIVE;
}

double PointAndTurnCore::targetHeading(const Pose2D & robot) const
{
  // Aim at the vertex itself. After slip, the segment's heading would send
  // the robot along a line parallel to the path that never reaches it.
  const Point2D & t = target();
  const double d = distance(position(robot), t);
  // Close to a vertex the bearing swings wildly with small position errors,
  // so fall back to the segment heading. On the last segment the bearing is
  // still needed down to the goal tolerance, or a robot beside the goal
  // could never close the gap.
  const bool use_bearing = d >= params_.min_bearing_distance ||
    (isLastSegment() && d >= params_.goal_arrival_tolerance);
  if (use_bearing) {
    return std::atan2(t.y - robot.y, t.x - robot.x);
  }
  const Point2D & s = vertices_[segment_];
  return std::atan2(t.y - s.y, t.x - s.x);
}

double PointAndTurnCore::headingError(const Pose2D & robot) const
{
  // Reversing, the robot's back is its direction of travel.
  const double travel_heading = robot.yaw + (direction_ < 0 ? kPi : 0.0);
  return normalizeAngle(targetHeading(robot) - travel_heading);
}

Command PointAndTurnCore::rotate(double error) const
{
  // Proportional, with a floor so the last few hundredths of a radian don't
  // take forever (and real drivetrains stall below some speed). The floor is
  // safe as long as one cycle's rotation stays inside the tolerance band:
  // 0.15 rad/s at 20 Hz is 0.0075 rad, well inside align_tolerance.
  const double speed = std::clamp(
    std::abs(params_.angular_gain * error), params_.min_angular_speed,
    params_.max_angular_speed);
  return {0.0, std::copysign(speed, error)};
}

bool PointAndTurnCore::step(const Pose2D & robot, Command & command)
{
  command = Command{};
  remaining_rotation_ = 0.0;
  switch (state_) {
    case State::IDLE:
    case State::DONE:
      return false;

    case State::FINAL_ALIGN: {
        const double error = normalizeAngle(goal_yaw_ - robot.yaw);
        if (std::abs(error) <= params_.final_align_tolerance) {
          state_ = State::DONE;
          return false;
        }
        remaining_rotation_ = error;
        command = rotate(error);
        return false;
      }

    case State::ALIGN: {
        const double d = distance(position(robot), target());
        if (isLastSegment()) {
          if (d < params_.goal_arrival_tolerance) {
            state_ = State::FINAL_ALIGN;
            return true;
          }
        } else if (d < params_.segment_arrival_tolerance) {
          // Already on the vertex, e.g. after a restart: don't pivot to a
          // segment that is already done.
          enterSegment(segment_ + 1, robot);
          return true;
        }
        const double error = headingError(robot);
        if (std::abs(error) <= params_.align_tolerance) {
          enterDrive(robot);
          return true;
        }
        remaining_rotation_ = error;
        command = rotate(error);
        return false;
      }

    case State::DRIVE: {
        const Point2D & t = target();
        const double ux = std::cos(drive_heading_);
        const double uy = std::sin(drive_heading_);
        // Progress is measured along the line the robot aligned to, so
        // passing beside a vertex still counts as arriving.
        const double remaining = (t.x - robot.x) * ux + (t.y - robot.y) * uy;
        if (isLastSegment()) {
          // The goal checker measures Euclidean distance, so arrival does too.
          if (distance(position(robot), t) < params_.goal_arrival_tolerance) {
            state_ = State::FINAL_ALIGN;
            return true;
          }
          if (remaining < 0.0) {
            // Passed the goal outside the tolerance: aim again, and let the
            // direction be re-decided since the goal may now be behind.
            enterSegment(segment_, robot);
            return true;
          }
        } else if (remaining < params_.segment_arrival_tolerance) {
          enterSegment(segment_ + 1, robot);
          return true;
        }

        double angular = 0.0;
        // Near the vertex the bearing is noise; checking it there would
        // trigger pointless re-alignments right before arrival.
        if (remaining >= params_.min_bearing_distance) {
          const double error = headingError(robot);
          const double ox = robot.x - drive_origin_.x;
          const double oy = robot.y - drive_origin_.y;
          const double cross_track = ox * uy - oy * ux;
          // Hysteresis: leaving ALIGN takes |error| <= align_tolerance, coming
          // back takes more than realign_tolerance.
          if (std::abs(error) > params_.realign_tolerance ||
            std::abs(cross_track) > params_.max_cross_track_error)
          {
            state_ = State::ALIGN;
            return true;
          }
          if (params_.drive_heading_gain > 0.0) {
            angular = std::clamp(
              params_.drive_heading_gain * error, -params_.max_drive_angular_speed,
              params_.max_drive_angular_speed);
          }
        }
        // Brake so the robot could stop exactly at the vertex: v^2 = 2 a d.
        const double cap = params_.max_linear_speed * speed_scale_;
        double speed = std::min(
          cap, std::sqrt(2.0 * params_.linear_decel * std::max(remaining, 0.0)));
        // The floor keeps the last centimetres from crawling, but never
        // exceeds a speed limit.
        speed = std::max(speed, std::min(params_.min_linear_speed, cap));
        command = {direction_ * speed, angular};
        return false;
      }
  }
  return false;
}

}  // namespace point_and_turn_controller
