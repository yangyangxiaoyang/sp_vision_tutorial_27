#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <pluginlib/class_loader.hpp>
#include "sp_global_planner/global_planner_plugin.hpp"
#include "sp_global_planner/grid_utils.hpp"

#include <memory>
#include <mutex>
#include <optional>
#include <chrono>
namespace sp_global_planner
{

    class PlannerServer : public rclcpp::Node
    {
    public:
        PlannerServer();
        ~PlannerServer() override = default;
        void init();

    private:
        // 全局地图回调
        void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void onLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        // 路径规划服务回调
        void onPlanRequest(
            const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
            std::shared_ptr<nav_msgs::srv::GetPlan::Response> res);
        // 加载路径规划插件
        void loadPlugin();
        void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg);
        void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void tryPlanFromPair();
        void publishStartGoalMarkers(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal,
            const std::string &frame_id);

        void updateCombinedMapLocked();
        void fuseLocalIntoGlobal(
            nav_msgs::msg::OccupancyGrid &global_map,
            const nav_msgs::msg::OccupancyGrid &local_map);

    private:
        // Subscriptions for RViz clicking
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

        // Cache latest start/goal
        std::optional<geometry_msgs::msg::PoseStamped> pending_start_;
        std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;
        std::mutex pair_mutex_;

    std::mutex map_mutex_;
    std::optional<nav_msgs::msg::OccupancyGrid> map_;             // fused map used for planning
    std::optional<nav_msgs::msg::OccupancyGrid> global_map_raw_;   // latest global/static map
    std::optional<nav_msgs::msg::OccupancyGrid> local_map_;        // latest local map window

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr plan_srv_;

        std::string costmap_topic_;
    std::string local_costmap_topic_;
        std::string path_topic_;

        std::string plugin_name_;
        std::string plugin_type_;

        pluginlib::ClassLoader<sp_global_planner::GlobalPlannerPlugin> loader_;
        std::shared_ptr<sp_global_planner::GlobalPlannerPlugin> planner_;
    };

} // namespace sp_global_planner
