#include "sp_global_planner/planner_server.hpp"

#include <algorithm>

namespace sp_global_planner
{

    PlannerServer::PlannerServer()
        : Node("planner_server"),
          loader_("sp_global_planner", "sp_global_planner::GlobalPlannerPlugin")
    {
    costmap_topic_ = this->declare_parameter<std::string>("costmap_topic", "/global_costmap");
    local_costmap_topic_ = this->declare_parameter<std::string>("local_costmap_topic", "/local_costmap/costmap");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/global_path");

        plugin_name_ = this->declare_parameter<std::string>("plugin_name", "AStar");
        plugin_type_ = this->declare_parameter<std::string>(
            "plugin_type", "sp_global_planner::AStarPlanner");

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 1);

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
            std::bind(&PlannerServer::onMap, this, std::placeholders::_1));

        local_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            local_costmap_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
            std::bind(&PlannerServer::onLocalMap, this, std::placeholders::_1));

        plan_srv_ = this->create_service<nav_msgs::srv::GetPlan>(
            "make_plan",
            std::bind(&PlannerServer::onPlanRequest, this, std::placeholders::_1, std::placeholders::_2));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/plan_markers", 10);
        // Subscribe RViz tools
        clicked_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&PlannerServer::onClickedPoint, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&PlannerServer::onGoalPose, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
            "PlannerServer ready. costmap_topic=%s local_costmap_topic=%s path_topic=%s service=make_plan plugin=%s (%s)",
            costmap_topic_.c_str(), local_costmap_topic_.c_str(), path_topic_.c_str(),
            plugin_name_.c_str(), plugin_type_.c_str());
    }
    void PlannerServer::init()
    {
        loadPlugin();

    RCLCPP_INFO(this->get_logger(),
            "PlannerServer ready. costmap_topic=%s local_costmap_topic=%s path_topic=%s service=make_plan plugin=%s (%s)",
            costmap_topic_.c_str(), local_costmap_topic_.c_str(), path_topic_.c_str(),
            plugin_name_.c_str(), plugin_type_.c_str());
    }
    void PlannerServer::loadPlugin()
    {
        try
        {
            planner_ = loader_.createSharedInstance(plugin_type_);
            // 不要用 shared_from_this()，直接构造一个非 owning shared_ptr 指向 this
            rclcpp::Node::SharedPtr node_ptr(this, [](rclcpp::Node *) {});
            planner_->configure(node_ptr, plugin_name_);
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                if (map_)
                {
                    planner_->setMap(*map_);
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to load plugin: %s", e.what());
            throw;
        }
    }

    void PlannerServer::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        global_map_raw_ = *msg;
        updateCombinedMapLocked();
    }

    void PlannerServer::onLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        local_map_ = *msg;
        updateCombinedMapLocked();
    }

    void PlannerServer::onPlanRequest(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response> res)
    {
        // Ensure we have a map
        nav_msgs::msg::OccupancyGrid map_copy;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (!map_)
            {
                RCLCPP_WARN(this->get_logger(), "No costmap received yet; cannot plan.");
                res->plan = nav_msgs::msg::Path();
                return;
            }
            map_copy = *map_;
        }

        if (!planner_)
        {
            RCLCPP_ERROR(this->get_logger(), "Planner plugin not loaded.");
            res->plan = nav_msgs::msg::Path();
            return;
        }

        // Force plan header to map frame
        auto start = req->start;
        auto goal = req->goal;
        start.header.frame_id = map_copy.header.frame_id;
        goal.header.frame_id = map_copy.header.frame_id;

        publishStartGoalMarkers(start, goal, map_copy.header.frame_id);
        
        auto plan_start_time = std::chrono::steady_clock::now();
        nav_msgs::msg::Path path = planner_->createPlan(start, goal);
        auto plan_end_time = std::chrono::steady_clock::now();
        auto plan_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(plan_end_time - plan_start_time).count();
        RCLCPP_INFO(this->get_logger(), "Planning took %ld ms", plan_duration_ms);
        // publish and respond
        // path.header.stamp = this->now();
        path_pub_->publish(path);
        res->plan = path;

        RCLCPP_INFO(this->get_logger(), "Plan computed: %zu poses", path.poses.size());
    }
    void PlannerServer::onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped start;
        start.header = msg->header;
        start.pose.position = msg->point;
        start.pose.position.z = 0.0;
        start.pose.orientation.w = 1.0; // no yaw info from clicked point

        {
            std::lock_guard<std::mutex> lk(pair_mutex_);
            pending_start_ = start;
        }

        RCLCPP_INFO(this->get_logger(), "Received start from /clicked_point: (%.3f, %.3f) frame=%s",
                    start.pose.position.x, start.pose.position.y, start.header.frame_id.c_str());

        tryPlanFromPair();
    }

    void PlannerServer::onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped goal = *msg;

        {
            std::lock_guard<std::mutex> lk(pair_mutex_);
            pending_goal_ = goal;
        }

        RCLCPP_INFO(this->get_logger(), "Received goal from /goal_pose: (%.3f, %.3f) frame=%s",
                    goal.pose.position.x, goal.pose.position.y, goal.header.frame_id.c_str());

        tryPlanFromPair();
    }
    void PlannerServer::tryPlanFromPair()
    {
        // Make sure we have both start and goal
        geometry_msgs::msg::PoseStamped start, goal;
        {
            std::lock_guard<std::mutex> lk(pair_mutex_);
            if (!pending_start_.has_value() || !pending_goal_.has_value())
            {
                return;
            }
            start = pending_start_.value();
            goal = pending_goal_.value();

            // 可选：规划一次后清空，避免重复触发
            pending_start_.reset();
            pending_goal_.reset();
        }

        // Ensure we have a map
        nav_msgs::msg::OccupancyGrid map_copy;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (!map_)
            {
                RCLCPP_WARN(this->get_logger(), "No costmap received yet; cannot plan.");
                return;
            }
            map_copy = *map_;
        }

        if (!planner_)
        {
            RCLCPP_ERROR(this->get_logger(), "Planner plugin not loaded.");
            return;
        }

        // Force frame to costmap frame
        const std::string frame_id = map_copy.header.frame_id;
        start.header.frame_id = frame_id;
        goal.header.frame_id = frame_id;

        // Publish markers
        publishStartGoalMarkers(start, goal, frame_id);

        // Plan
        auto t0 = std::chrono::steady_clock::now();
        nav_msgs::msg::Path path = planner_->createPlan(start, goal);
        auto t1 = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        RCLCPP_INFO(this->get_logger(), "Auto planning took %ld ms", ms);

        path.header.stamp = this->now();
        path.header.frame_id = frame_id;

        path_pub_->publish(path);

        RCLCPP_INFO(this->get_logger(), "Auto plan computed: %zu poses", path.poses.size());
    }
    void PlannerServer::publishStartGoalMarkers(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        const std::string &frame_id)
    {
        if (!marker_pub_)
            return;

        const auto stamp = this->now();

        visualization_msgs::msg::Marker start_m;
        start_m.header.frame_id = frame_id;
        start_m.header.stamp = stamp;
        start_m.ns = "plan_points";
        start_m.id = 0;
        start_m.type = visualization_msgs::msg::Marker::SPHERE; // start: sphere
        start_m.action = visualization_msgs::msg::Marker::ADD;
        start_m.pose = start.pose;
        start_m.scale.x = 0.25;
        start_m.scale.y = 0.25;
        start_m.scale.z = 0.25;
        start_m.color.r = 0.0f;
        start_m.color.g = 1.0f;
        start_m.color.b = 0.0f;
        start_m.color.a = 1.0f;

        visualization_msgs::msg::Marker goal_m;
        goal_m.header.frame_id = frame_id;
        goal_m.header.stamp = stamp;
        goal_m.ns = "plan_points";
        goal_m.id = 1;
        goal_m.type = visualization_msgs::msg::Marker::SPHERE; // goal: cube
        goal_m.action = visualization_msgs::msg::Marker::ADD;
        goal_m.pose = goal.pose;
        goal_m.scale.x = 0.25;
        goal_m.scale.y = 0.25;
        goal_m.scale.z = 0.25;
        goal_m.color.r = 1.0f;
        goal_m.color.g = 0.0f;
        goal_m.color.b = 0.0f;
        goal_m.color.a = 1.0f;

        marker_pub_->publish(start_m);
        marker_pub_->publish(goal_m);
    }

    void PlannerServer::updateCombinedMapLocked()
    {
        if (!global_map_raw_)
        {
            return;
        }

        nav_msgs::msg::OccupancyGrid fused = *global_map_raw_;
        if (local_map_)
        {
            fuseLocalIntoGlobal(fused, *local_map_);
        }

        map_ = fused;
        if (planner_)
        {
            planner_->setMap(*map_);
        }
    }

    void PlannerServer::fuseLocalIntoGlobal(
        nav_msgs::msg::OccupancyGrid &global_map,
        const nav_msgs::msg::OccupancyGrid &local_map)
    {
        const int local_w = static_cast<int>(local_map.info.width);
        const int local_h = static_cast<int>(local_map.info.height);

        for (int y = 0; y < local_h; ++y)
        {
            for (int x = 0; x < local_w; ++x)
            {
                const int local_idx = y * local_w + x;
                int8_t local_cost = local_map.data[local_idx];
                if (local_cost < 0)
                {
                    continue; // skip unknown cells
                }

                double wx, wy;
                gridToWorld(local_map, x, y, wx, wy);
                GridIndex gidx;
                if (!worldToGrid(global_map, wx, wy, gidx))
                {
                    continue;
                }

                const int global_idx = toIndex(global_map, gidx.x, gidx.y);
                int8_t &target = global_map.data[global_idx];
                if (target < 0)
                {
                    target = local_cost;
                }
                else
                {
                    target = static_cast<int8_t>(std::max<int>(target, local_cost));
                }
            }
        }
    }

} // namespace sp_global_planner
