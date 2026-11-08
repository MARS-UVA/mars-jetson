#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "point_and_turn_controller/core.hpp"

using point_and_turn_controller::Command;
using point_and_turn_controller::normalizeAngle;
using point_and_turn_controller::Params;
using point_and_turn_controller::Point2D;
using point_and_turn_controller::PointAndTurnCore;
using point_and_turn_controller::Pose2D;
using point_and_turn_controller::State;
using point_and_turn_controller::validate;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.05;  // controller_frequency: 20 Hz

// A planner-like path: a pose every 5 cm along each leg, heading along the
// leg, and the goal yaw on the last pose.
std::vector<Pose2D> densePath(const std::vector<Point2D> & corners, double goal_yaw)
{
  std::vector<Pose2D> path;
  for (std::size_t i = 0; i + 1 < corners.size(); ++i) {
    const Point2D & a = corners[i];
    const Point2D & b = corners[i + 1];
    const double heading = std::atan2(b.y - a.y, b.x - a.x);
    const int steps = static_cast<int>(std::round(std::hypot(b.x - a.x, b.y - a.y) / 0.05));
    for (int k = 0; k < steps; ++k) {
      const double t = static_cast<double>(k) / steps;
      path.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, heading});
    }
  }
  path.push_back({corners.back().x, corners.back().y, goal_yaw});
  return path;
}

bool combined(const Command & c)
{
  return std::abs(c.linear) > 1e-9 && std::abs(c.angular) > 1e-9;
}

struct SimResult
{
  Pose2D pose;
  int ticks{0};
  int combined_ticks{0};
  int pivot_ticks{0};  // rotating in place, before FINAL_ALIGN
  double total_rotation{0.0};
  bool done{false};
};

// Integrates unicycle kinematics. slip adds a sideways body velocity of
// slip * |v|, i.e. slip metres of lateral drift per metre driven.
SimResult simulate(PointAndTurnCore & core, Pose2D pose, double slip = 0.0, int max_ticks = 4000)
{
  SimResult run;
  for (; run.ticks < max_ticks; ++run.ticks) {
    const Command c = core.update(pose);
    if (core.state() == State::DONE) {
      run.done = true;
      break;
    }
    if (combined(c)) {
      ++run.combined_ticks;
    }
    if (core.state() == State::ALIGN && c.angular != 0.0) {
      ++run.pivot_ticks;
    }
    const double lateral = slip * std::abs(c.linear);
    pose.x += (c.linear * std::cos(pose.yaw) - lateral * std::sin(pose.yaw)) * kDt;
    pose.y += (c.linear * std::sin(pose.yaw) + lateral * std::cos(pose.yaw)) * kDt;
    pose.yaw = normalizeAngle(pose.yaw + c.angular * kDt);
    run.total_rotation += std::abs(c.angular * kDt);
  }
  run.pose = pose;
  return run;
}

double distanceTo(const Pose2D & pose, Point2D p)
{
  return std::hypot(pose.x - p.x, pose.y - p.y);
}

// Steps the robot along until pred holds, for setting up mid-drive states.
template<typename Pred>
Pose2D advanceUntil(PointAndTurnCore & core, Pose2D pose, Pred pred)
{
  for (int i = 0; i < 4000 && !pred(core, pose); ++i) {
    const Command c = core.update(pose);
    pose.x += c.linear * std::cos(pose.yaw) * kDt;
    pose.y += c.linear * std::sin(pose.yaw) * kDt;
    pose.yaw = normalizeAngle(pose.yaw + c.angular * kDt);
  }
  return pose;
}

const std::vector<Point2D> kLCorner{{0, 0}, {2, 0}, {2, 2}};

}  // namespace

TEST(Validate, DefaultsAreValid)
{
  EXPECT_EQ(validate(Params{}), "");
}

TEST(Validate, RejectsAlignToleranceNotBelowRealign)
{
  Params p;
  p.align_tolerance = p.realign_tolerance;
  EXPECT_NE(validate(p), "");
  p.align_tolerance = p.realign_tolerance + 0.1;
  EXPECT_NE(validate(p), "");
}

TEST(Validate, RejectsNaN)
{
  Params p;
  p.max_linear_speed = std::nan("");
  EXPECT_NE(validate(p), "");
}

TEST(Core, EmptyPlanHasNoPlanAndCommandsZero)
{
  PointAndTurnCore core{Params{}};
  EXPECT_FALSE(core.hasPlan());
  core.setPlan({});
  EXPECT_FALSE(core.hasPlan());
  const Command c = core.update({0, 0, 0});
  EXPECT_EQ(c.linear, 0.0);
  EXPECT_EQ(c.angular, 0.0);
  EXPECT_EQ(core.state(), State::IDLE);
}

TEST(Core, SinglePointPlanDrivesToIt)
{
  PointAndTurnCore core{Params{}};
  core.setPlan({{1.0, 0.0, 0.0}});
  const SimResult run = simulate(core, {0, 0, 0});
  ASSERT_TRUE(run.done);
  EXPECT_LT(distanceTo(run.pose, {1, 0}), 0.10);
}

TEST(Core, SinglePointPlanAtRobotOnlyAligns)
{
  PointAndTurnCore core{Params{}};
  core.setPlan({{0.02, 0.0, kPi / 2}});
  const Command c = core.update({0, 0, 0});
  EXPECT_EQ(core.state(), State::FINAL_ALIGN);
  EXPECT_EQ(c.linear, 0.0);
  EXPECT_GT(c.angular, 0.0);
}

TEST(Core, StraightLineHasNoCombinedMotion)
{
  PointAndTurnCore core{Params{}};
  core.setPlan(densePath({{0, 0}, {5, 0}}, 0.0));
  const SimResult run = simulate(core, {0, 0, 0});
  ASSERT_TRUE(run.done);
  EXPECT_EQ(run.combined_ticks, 0);
  EXPECT_LT(distanceTo(run.pose, {5, 0}), 0.10);
}

TEST(Core, LCornerHasNoCombinedMotion)
{
  PointAndTurnCore core{Params{}};
  core.setPlan(densePath(kLCorner, kPi / 2));
  const SimResult run = simulate(core, {0, 0, 0});
  ASSERT_TRUE(run.done);
  EXPECT_EQ(run.combined_ticks, 0);
  EXPECT_LT(distanceTo(run.pose, {2, 2}), 0.10);
  EXPECT_LT(std::abs(normalizeAngle(run.pose.yaw - kPi / 2)), 0.10);
  // One pivot at the corner: about 90 degrees in total.
  EXPECT_NEAR(run.total_rotation, kPi / 2, 0.15);
}

TEST(Core, FinalAlignRotatesToGoalYaw)
{
  PointAndTurnCore core{Params{}};
  core.setPlan(densePath({{0, 0}, {2, 0}}, -kPi / 2));
  const SimResult run = simulate(core, {0, 0, 0});
  ASSERT_TRUE(run.done);
  EXPECT_LT(std::abs(normalizeAngle(run.pose.yaw + kPi / 2)), 0.10);
  // Strictly inside the shared goal checker (0.20 m / 0.20 rad), so the
  // controller never stops short of what the checker needs.
  EXPECT_LT(distanceTo(run.pose, {2, 0}), 0.20);
}

TEST(Core, LateralSlipStillReachesGoal)
{
  PointAndTurnCore core{Params{}};
  core.setPlan(densePath({{0, 0}, {5, 0}}, 0.0));
  const SimResult run = simulate(core, {0, 0, 0}, 0.15);
  ASSERT_TRUE(run.done);
  EXPECT_LT(distanceTo(run.pose, {5, 0}), 0.10);
  EXPECT_EQ(run.combined_ticks, 0);
  // Slip pushed it off line and it pivoted to re-aim rather than curving.
  EXPECT_GT(run.pivot_ticks, 0);
}

TEST(Core, HeadingGainTradesPurityForFewerPivots)
{
  Params p;
  p.drive_heading_gain = 1.0;
  PointAndTurnCore with_gain{p};
  with_gain.setPlan(densePath({{0, 0}, {5, 0}}, 0.0));
  const SimResult corrected = simulate(with_gain, {0, 0, 0}, 0.15);

  PointAndTurnCore pure{Params{}};
  pure.setPlan(densePath({{0, 0}, {5, 0}}, 0.0));
  const SimResult uncorrected = simulate(pure, {0, 0, 0}, 0.15);

  ASSERT_TRUE(corrected.done);
  EXPECT_LT(distanceTo(corrected.pose, {5, 0}), 0.10);
  EXPECT_GT(corrected.combined_ticks, 0);
  EXPECT_LT(corrected.pivot_ticks, uncorrected.pivot_ticks);
}

TEST(Core, HysteresisBandBetweenAlignAndRealign)
{
  PointAndTurnCore core{Params{}};
  core.setPlan(densePath({{0, 0}, {5, 0}}, 0.0));
  // Error 0.10: outside align_tolerance, so ALIGN rotates.
  Command c = core.update({0.5, 0, 0.10});
  EXPECT_EQ(core.state(), State::ALIGN);
  EXPECT_LT(c.angular, 0.0);
  // Inside align_tolerance: DRIVE.
  c = core.update({0.5, 0, 0.03});
  EXPECT_EQ(core.state(), State::DRIVE);
  EXPECT_GT(c.linear, 0.0);
  // 0.15 is inside the band: DRIVE keeps going, purely linear.
  c = core.update({0.5, 0, 0.15});
  EXPECT_EQ(core.state(), State::DRIVE);
  EXPECT_EQ(c.angular, 0.0);
  // Past realign_tolerance: back to ALIGN, which rotates without driving.
  c = core.update({0.5, 0, 0.25});
  EXPECT_EQ(core.state(), State::ALIGN);
  EXPECT_EQ(c.linear, 0.0);
  // Back inside the band: ALIGN still has to reach align_tolerance.
  c = core.update({0.5, 0, 0.10});
  EXPECT_EQ(core.state(), State::ALIGN);
}

TEST(Core, CrossTrackErrorTriggersRealign)
{
  PointAndTurnCore core{Params{}};
  core.setPlan(densePath({{0, 0}, {5, 0}}, 0.0));
  core.update({0.5, 0, 0});
  ASSERT_EQ(core.state(), State::DRIVE);
  // Slid 0.16 m sideways while keeping the heading. The bearing error here
  // is 0.158 rad, inside realign_tolerance, so cross-track is what trips.
  const Command c = core.update({4.0, 0.16, 0});
  EXPECT_EQ(core.state(), State::ALIGN);
  EXPECT_EQ(c.linear, 0.0);
}

TEST(Core, ReplanThroughSameVertexStaysInDrive)
{
  PointAndTurnCore core{Params{}};
  core.setPlan(densePath(kLCorner, kPi / 2));
  const Pose2D pose = advanceUntil(
    core, {0, 0, 0}, [](const PointAndTurnCore & c, const Pose2D & p) {
      return c.state() == State::DRIVE && p.x > 1.0;
    });
  ASSERT_EQ(core.state(), State::DRIVE);
  // The replan's corner moved 10 cm: still the same target.
  core.setPlan(densePath({{pose.x, pose.y}, {2.1, 0}, {2.1, 2}}, kPi / 2));
  const Command c = core.update(pose);
  EXPECT_EQ(core.state(), State::DRIVE);
  EXPECT_GT(c.linear, 0.0);
  EXPECT_NEAR(core.vertices()[1].x, 2.1, 1e-9);
}

TEST(Core, ReplanToDifferentRouteRestartsAlign)
{
  PointAndTurnCore core{Params{}};
  core.setPlan(densePath(kLCorner, kPi / 2));
  const Pose2D pose = advanceUntil(
    core, {0, 0, 0}, [](const PointAndTurnCore & c, const Pose2D & p) {
      return c.state() == State::DRIVE && p.x > 1.0;
    });
  core.setPlan(densePath({{pose.x, pose.y}, {pose.x, 3}}, kPi / 2));
  const Command c = core.update(pose);
  EXPECT_EQ(core.state(), State::ALIGN);
  EXPECT_EQ(c.linear, 0.0);
  EXPECT_GT(c.angular, 0.0);
}

TEST(Core, ReplanAtGoalDoesNotRedrive)
{
  PointAndTurnCore core{Params{}};
  const auto plan = densePath({{0, 0}, {2, 0}}, 0.0);
  core.setPlan(plan);
  const SimResult run = simulate(core, {0, 0, 0});
  ASSERT_TRUE(run.done);
  // The behavior tree replans to the same goal from where the robot stopped.
  core.setPlan(densePath({{run.pose.x, run.pose.y}, {2, 0}}, 0.0));
  const Command c = core.update(run.pose);
  EXPECT_EQ(core.state(), State::DONE);
  EXPECT_EQ(c.linear, 0.0);
  EXPECT_EQ(c.angular, 0.0);
}

TEST(Core, ReversingAvoidsHalfTurnPivot)
{
  PointAndTurnCore core{Params{}};  // allow_reversing defaults to true
  core.setPlan(densePath({{0, 0}, {-2, 0}}, 0.0));
  const Command first = core.update({0, 0, 0});
  EXPECT_LT(first.linear, 0.0);
  const SimResult run = simulate(core, {0, 0, 0});
  ASSERT_TRUE(run.done);
  EXPECT_LT(distanceTo(run.pose, {-2, 0}), 0.10);
  EXPECT_LT(run.total_rotation, 0.1);
  EXPECT_EQ(run.combined_ticks, 0);
}

TEST(Core, WithoutReversingGoalBehindPivotsFirst)
{
  Params p;
  p.allow_reversing = false;
  PointAndTurnCore core{p};
  core.setPlan(densePath({{0, 0}, {-2, 0}}, 0.0));
  const Command first = core.update({0, 0, 0});
  EXPECT_EQ(core.state(), State::ALIGN);
  EXPECT_EQ(first.linear, 0.0);
  EXPECT_DOUBLE_EQ(std::abs(first.angular), p.max_angular_speed);
  const SimResult run = simulate(core, {0, 0, 0});
  ASSERT_TRUE(run.done);
  // Half a turn out, and half a turn back to the goal yaw.
  EXPECT_GT(run.total_rotation, 1.9 * kPi);
}

TEST(Core, RotationCommandFloorAndCeiling)
{
  Params p;
  p.allow_reversing = false;  // keep the sign of the error obvious
  PointAndTurnCore core{p};
  core.setPlan(densePath({{0, 0}, {5, 0}}, 0.0));
  // Error -0.06 rad: gain * error = -0.09, raised to the floor.
  Command c = core.update({0, 0, 0.06});
  EXPECT_DOUBLE_EQ(c.angular, -p.min_angular_speed);
  EXPECT_NEAR(core.remainingRotation(), -0.06, 1e-12);
  // Error +2.0 rad: gain * error = 3.0, capped at the ceiling.
  c = core.update({0, 0, -2.0});
  EXPECT_DOUBLE_EQ(c.angular, p.max_angular_speed);
}

TEST(Core, SpeedScaleCapsDriveSpeed)
{
  PointAndTurnCore core{Params{}};
  core.setPlan(densePath({{0, 0}, {5, 0}}, 0.0));
  EXPECT_DOUBLE_EQ(core.update({0, 0, 0}).linear, 0.40);
  core.setSpeedScale(0.5);
  EXPECT_DOUBLE_EQ(core.update({0, 0, 0}).linear, 0.20);
  // A limit below min_linear_speed wins over the floor.
  core.setSpeedScale(0.1);
  EXPECT_DOUBLE_EQ(core.update({0, 0, 0}).linear, 0.04);
}

TEST(Core, DriveBrakesTowardVertex)
{
  PointAndTurnCore core{Params{}};
  core.setPlan(densePath({{0, 0}, {5, 0}}, 0.0));
  // 0.15 m to go: sqrt(2 * 0.40 * 0.15) = 0.346 m/s.
  EXPECT_NEAR(core.update({4.85, 0, 0}).linear, std::sqrt(0.12), 1e-9);
}

TEST(Core, ResetClearsPlan)
{
  PointAndTurnCore core{Params{}};
  core.setPlan(densePath({{0, 0}, {5, 0}}, 0.0));
  core.update({0, 0, 0});
  core.reset();
  EXPECT_FALSE(core.hasPlan());
  EXPECT_EQ(core.state(), State::IDLE);
  EXPECT_EQ(core.update({0, 0, 0}).linear, 0.0);
}
