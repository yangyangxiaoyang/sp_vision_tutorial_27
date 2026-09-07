#include "astar_planner.hpp"
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>

namespace sp_global_planner {

void AStarPlanner::configure(const rclcpp::Node::SharedPtr& node, const std::string& plugin_name)
{
  logger_ = node->get_logger();

  // parameters are namespaced under plugin_name.*
  lethal_cost_  = node->declare_parameter<int>(plugin_name + ".lethal_cost", 100);
  cost_weight_  = node->declare_parameter<double>(plugin_name + ".cost_weight", 2.0);

  RCLCPP_INFO(logger_, "AStarPlanner configured: lethal_cost=%d cost_weight=%.3f",
              lethal_cost_, cost_weight_);
}

void AStarPlanner::setMap(const nav_msgs::msg::OccupancyGrid& costmap)
{
  map_ = costmap;
}

bool AStarPlanner::isBlocked(int8_t c) const
{
  // OccupancyGrid data range for our costmap: 0..100
  // treat unknown (-1) as blocked too, unless you want otherwise
  if (c < 0) return true;
  return c >= lethal_cost_;
}

double AStarPlanner::cellCostFactor(int8_t c) const
{
  // scale traversal cost by (1 + cost_weight * cost/100)
  // so higher cost cells are still passable (if not lethal) but discouraged
  double cc = std::max<int>(0, c);
  return 1.0 + cost_weight_ * (cc / 100.0);
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal)
{
  nav_msgs::msg::Path path;
  if (!map_) {
    RCLCPP_ERROR(logger_, "No map received yet.");
    return path;
  }
  const auto& map = *map_;
  path.header = map.header;

  if (start.header.frame_id != map.header.frame_id || goal.header.frame_id != map.header.frame_id) {
    RCLCPP_WARN(logger_, "Frame mismatch: start=%s goal=%s map=%s",
      start.header.frame_id.c_str(), goal.header.frame_id.c_str(), map.header.frame_id.c_str());
  }

  GridIndex s, g;
  if (!worldToGrid(map, start.pose.position.x, start.pose.position.y, s)) {
    RCLCPP_ERROR(logger_, "Start out of map bounds.");
    return path;
  }
  if (!worldToGrid(map, goal.pose.position.x, goal.pose.position.y, g)) {
    RCLCPP_ERROR(logger_, "Goal out of map bounds.");
    return path;
  }

  const int W = static_cast<int>(map.info.width);
  const int H = static_cast<int>(map.info.height);
  const int N = W * H;

  auto idx = [&](int x, int y){ return y * W + x; };

  // Validate start/goal not blocked
  if (isBlocked(map.data[idx(s.x, s.y)])) {
    RCLCPP_ERROR(logger_, "Start is in blocked cell (cost=%d).", (int)map.data[idx(s.x, s.y)]);
    return path;
  }
  if (isBlocked(map.data[idx(g.x, g.y)])) {
    RCLCPP_ERROR(logger_, "Goal is in blocked cell (cost=%d).", (int)map.data[idx(g.x, g.y)]);
    return path;
  }

  struct Node {
    int i;
    double f;
    double g;
  };
  struct Cmp { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };

  std::priority_queue<Node, std::vector<Node>, Cmp> open;
  std::vector<double> gscore(N, std::numeric_limits<double>::infinity());
  std::vector<int> parent(N, -1);
  std::vector<uint8_t> closed(N, 0);

  auto h = [&](int x, int y) {
    double dx = (x - g.x);
    double dy = (y - g.y);
    return std::sqrt(dx*dx + dy*dy);
  };

  int s_i = idx(s.x, s.y);
  int g_i = idx(g.x, g.y);

  gscore[s_i] = 0.0;
  open.push({s_i, h(s.x, s.y), 0.0});

  // 8-connected neighbors
  const int dxs[8] = {1,-1,0,0, 1,1,-1,-1};
  const int dys[8] = {0,0,1,-1, 1,-1,1,-1};

  bool found = false;

  while (!open.empty()) {
    Node cur = open.top();
    open.pop();

    if (closed[cur.i]) continue;
    closed[cur.i] = 1;

    if (cur.i == g_i) {
      found = true;
      break;
    }

    int cy = cur.i / W;
    int cx = cur.i - cy * W;

    for (int k = 0; k < 8; ++k) {
      int nx = cx + dxs[k];
      int ny = cy + dys[k];
      if (!inBounds(map, nx, ny)) continue;

      int ni = idx(nx, ny);
      if (closed[ni]) continue;

      int8_t c = map.data[ni];
      if (isBlocked(c)) continue;

      // step length: 1 for cardinal, sqrt(2) for diagonal
      double step = (k < 4) ? 1.0 : std::sqrt(2.0);

      // incorporate cell cost as multiplier
      double factor = cellCostFactor(c);
      double tentative = gscore[cur.i] + step * factor;

      if (tentative < gscore[ni]) {
        gscore[ni] = tentative;
        parent[ni] = cur.i;
        double f = tentative + h(nx, ny);
        open.push({ni, f, tentative});
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* failed to find a path.");
    return path;
  }

  // reconstruct
  std::vector<int> cells;
  int cur = g_i;
  while (cur != -1) {
    cells.push_back(cur);
    if (cur == s_i) break;
    cur = parent[cur];
  }
  if (cells.back() != s_i) {
    RCLCPP_WARN(logger_, "Path reconstruction failed.");
    return path;
  }
  std::reverse(cells.begin(), cells.end());

  // convert to Path
  path.poses.reserve(cells.size());
  for (int ci : cells) {
    int y = ci / W;
    int x = ci - y * W;

    double wx, wy;
    gridToWorld(map, x, y, wx, wy);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = wx;
    ps.pose.position.y = wy;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0; // no orientation for now
    path.poses.push_back(ps);
  }

  return path;
}

} // namespace sp_global_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sp_global_planner::AStarPlanner, sp_global_planner::GlobalPlannerPlugin)
