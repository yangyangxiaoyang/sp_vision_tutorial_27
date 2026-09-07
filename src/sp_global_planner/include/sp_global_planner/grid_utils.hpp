#pragma once
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>
#include <cstdint>
#include <optional>

namespace sp_global_planner {

inline double yaw_to_quat_z(double yaw) { return std::sin(yaw * 0.5); }
inline double yaw_to_quat_w(double yaw) { return std::cos(yaw * 0.5); }

// NOTE: We assume map origin yaw == 0 for simplicity (common for static maps).
// If your map origin yaw is non-zero, add a transform here.
struct GridIndex { int x; int y; };

inline bool worldToGrid(
  const nav_msgs::msg::OccupancyGrid& map,
  double wx, double wy,
  GridIndex& out)
{
  const auto& info = map.info;
  const double res = info.resolution;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;

  const double mx = (wx - ox) / res;
  const double my = (wy - oy) / res;

  int gx = static_cast<int>(std::floor(mx));
  int gy = static_cast<int>(std::floor(my));

  if (gx < 0 || gy < 0 || gx >= static_cast<int>(info.width) || gy >= static_cast<int>(info.height)) {
    return false;
  }
  out = {gx, gy};
  return true;
}

inline void gridToWorld(
  const nav_msgs::msg::OccupancyGrid& map,
  int gx, int gy,
  double& wx, double& wy)
{
  const auto& info = map.info;
  const double res = info.resolution;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;

  // center of cell
  wx = ox + (gx + 0.5) * res;
  wy = oy + (gy + 0.5) * res;
}

inline int toIndex(const nav_msgs::msg::OccupancyGrid& map, int x, int y) {
  return y * static_cast<int>(map.info.width) + x;
}

inline bool inBounds(const nav_msgs::msg::OccupancyGrid& map, int x, int y) {
  return x >= 0 && y >= 0 &&
         x < static_cast<int>(map.info.width) &&
         y < static_cast<int>(map.info.height);
}

} // namespace sp_global_planner
