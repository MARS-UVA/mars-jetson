#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "point_and_turn_controller/core.hpp"

using point_and_turn_controller::Point2D;
using point_and_turn_controller::SimplifierParams;
using point_and_turn_controller::simplifyPath;

namespace
{

// Planner-like samples every 5 cm. Dividing an integer keeps them exact
// multiples, so vertex positions can be compared exactly.
std::vector<Point2D> line(Point2D from, Point2D to, int steps)
{
  std::vector<Point2D> points;
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    points.push_back({from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t});
  }
  return points;
}

std::vector<Point2D> join(std::vector<Point2D> a, const std::vector<Point2D> & b)
{
  // b starts where a ends; skip the shared point.
  a.insert(a.end(), b.begin() + 1, b.end());
  return a;
}

}  // namespace

TEST(Simplifier, EmptyAndSinglePoint)
{
  EXPECT_TRUE(simplifyPath({}, SimplifierParams{}).empty());
  const auto single = simplifyPath({{1.0, 2.0}}, SimplifierParams{});
  ASSERT_EQ(single.size(), 1u);
  EXPECT_DOUBLE_EQ(single[0].x, 1.0);
}

TEST(Simplifier, StraightLineKeepsOnlyEndpoints)
{
  const auto vertices = simplifyPath(line({0, 0}, {5, 0}, 100), SimplifierParams{});
  ASSERT_EQ(vertices.size(), 2u);
  EXPECT_DOUBLE_EQ(vertices[0].x, 0.0);
  EXPECT_DOUBLE_EQ(vertices[1].x, 5.0);
}

TEST(Simplifier, LCornerGivesOneCornerVertex)
{
  const auto vertices = simplifyPath(
    join(line({0, 0}, {2, 0}, 40), line({2, 0}, {2, 2}, 40)), SimplifierParams{});
  ASSERT_EQ(vertices.size(), 3u);
  // Exactly the corner, not (2, 0.05): the last point inside the corridor.
  EXPECT_DOUBLE_EQ(vertices[1].x, 2.0);
  EXPECT_DOUBLE_EQ(vertices[1].y, 0.0);
  EXPECT_DOUBLE_EQ(vertices[2].y, 2.0);
}

TEST(Simplifier, GentleCurveVertexIsLastPointInsideCorridor)
{
  // Bends 20 degrees: progress still increases when the path leaves the
  // corridor sideways, so the vertex is the classic last point inside.
  const double c = std::cos(0.35);
  const double s = std::sin(0.35);
  const auto vertices = simplifyPath(
    join(line({0, 0}, {1, 0}, 20), line({1, 0}, {1 + c, s}, 20)), SimplifierParams{});
  ASSERT_GE(vertices.size(), 3u);
  EXPECT_GT(vertices[1].x, 1.0);
  EXPECT_LE(vertices[1].y, 0.05);
}

TEST(Simplifier, NoiseInsideCorridorIsAbsorbed)
{
  // Off-line blips up to 4 cm, either side. The probe point (index 2) that
  // sets the corridor direction is on the line: a tilted probe tilts the
  // whole corridor, which is the documented limit of this rule (README).
  auto points = line({0, 0}, {5, 0}, 100);
  for (std::size_t i = 3; i + 1 < points.size(); i += 4) {
    points[i].y = (i % 8 == 3) ? 0.04 : -0.04;
  }
  EXPECT_EQ(simplifyPath(points, SimplifierParams{}).size(), 2u);
}

TEST(Simplifier, ReverseCuspVertexIsExactlyAtTurnaround)
{
  // Drive out to x = 1, then reverse along the same line to x = 0.3. The
  // cusp never leaves the corridor sideways.
  const auto vertices = simplifyPath(
    join(line({0, 0}, {1, 0}, 20), line({1, 0}, {0.3, 0}, 14)), SimplifierParams{});
  ASSERT_EQ(vertices.size(), 3u);
  // Not 0.95, the last point within the tolerance behind max progress.
  EXPECT_DOUBLE_EQ(vertices[1].x, 1.0);
  EXPECT_DOUBLE_EQ(vertices[1].y, 0.0);
  EXPECT_DOUBLE_EQ(vertices[2].x, 0.3);
}

TEST(Simplifier, ConsecutiveDuplicatesAreDropped)
{
  std::vector<Point2D> doubled;
  for (const Point2D & p : join(line({0, 0}, {2, 0}, 40), line({2, 0}, {2, 2}, 40))) {
    doubled.push_back(p);
    doubled.push_back(p);
  }
  const auto vertices = simplifyPath(doubled, SimplifierParams{});
  ASSERT_EQ(vertices.size(), 3u);
  for (std::size_t i = 1; i < vertices.size(); ++i) {
    EXPECT_GT(std::hypot(vertices[i].x - vertices[i - 1].x, vertices[i].y - vertices[i - 1].y), 0.1);
  }
}

TEST(Simplifier, CornerNextToGoalIsMergedIntoGoal)
{
  auto points = line({0, 0}, {2, 0}, 40);
  points.push_back({2.0, 0.08});  // Goal 8 cm to the side of the corner.
  const auto vertices = simplifyPath(points, SimplifierParams{});
  ASSERT_EQ(vertices.size(), 2u);
  EXPECT_DOUBLE_EQ(vertices[1].x, 2.0);
  EXPECT_DOUBLE_EQ(vertices[1].y, 0.08);
}

TEST(Simplifier, ShortPathKeepsFirstPoint)
{
  // Shorter than min_segment_length: the merge must not drop the start.
  const auto vertices = simplifyPath({{0, 0}, {0.03, 0}, {0.06, 0}}, SimplifierParams{});
  ASSERT_EQ(vertices.size(), 2u);
  EXPECT_DOUBLE_EQ(vertices[0].x, 0.0);
  EXPECT_DOUBLE_EQ(vertices[1].x, 0.06);
}
