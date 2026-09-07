#ifndef SP_DECISION_BLACKBOARD_BRIDGE_HPP_
#define SP_DECISION_BLACKBOARD_BRIDGE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <chrono>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_msg/msg/team_robot_hp_msg.hpp"
#include "robot_msg/msg/enemy_robot_hp_msg.hpp"
#include "robot_msg/msg/enemy_robot_status_msg.hpp"
#include "robot_msg/msg/enemy_robot_buff_msg.hpp"
#include "robot_msg/msg/enemy_robot_position_msg.hpp"
#include "robot_msg/msg/referee_info_msg.hpp"
#include "robot_msg/msg/area_status_msg.hpp"
#include "robot_msg/msg/posture_feedback_msg.hpp"
#include "robot_msg/msg/buffer_state_msg.hpp"
#include "sp_msgs/msg/enemy_status_msg.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sp_decision
{
  class BlackboardBridge : public rclcpp::Node
  {
  public:
    explicit BlackboardBridge(
        BT::Blackboard::Ptr blackboard,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~BlackboardBridge() override = default;

    /** @brief Bind a different blackboard (e.g. after a tree switch). */
    void set_blackboard(BT::Blackboard::Ptr blackboard);

  private:
    void init_all_variables();
    // ── Subscriber callbacks ────────────────────────────────────────────────
    void on_referee_info(const robot_msg::msg::RefereeInfoMsg::SharedPtr msg);
    void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void on_team_hp(const robot_msg::msg::TeamRobotHpMsg::SharedPtr msg);
    void on_enemy_hp(const robot_msg::msg::EnemyRobotHpMsg::SharedPtr msg);
    void on_enemy_status(const robot_msg::msg::EnemyRobotStatusMsg::SharedPtr msg);
    void on_enemy_buff(const robot_msg::msg::EnemyRobotBuffMsg::SharedPtr msg);
    void on_enemy_position(const robot_msg::msg::EnemyRobotPositionMsg::SharedPtr msg);
    void on_area_status(const robot_msg::msg::AreaStatusMsg::SharedPtr msg);
    void on_posture_feedback(const robot_msg::msg::PostureFeedbackMsg::SharedPtr msg);
    void on_buffer_state(const robot_msg::msg::BufferStateMsg::SharedPtr msg);
    void on_auto_aim_target_pos(const std_msgs::msg::String::SharedPtr msg);
    void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ── Helpers ─────────────────────────────────────────────────────────────
    template <typename T>
    void write(const std::string &key, const T &value)
    {
      std::lock_guard<std::mutex> lk(bb_mutex_);
      if (blackboard_)
      {
        blackboard_->set<T>(key, value);
      }
    }

    // ── State ────────────────────────────────────────────────────────────────
    BT::Blackboard::Ptr blackboard_;
    std::mutex bb_mutex_;
    int add_blood_stage_{0};
    int add_bullet_stage_{0};  // 补弹状态机: 0=正常, 1=需要补弹(回补弹点)
    // 第二套阈值：补弹进 allowance<200 & can_get>20，退 can_get<=20 或 allowance>90；补血 hp<320 / >=380
    int add_blood_stage2_{0};
    int add_bullet_stage2_{0};
    // 第三套阈值：补弹进 allowance<50 & can_get>20，退 can_get<=20 或 allowance>=90；补血 hp<120 / >=380
    int add_blood_stage3_{0};
    int add_bullet_stage3_{0};
    int back_count_{0};        // 回家补血次数（add_blood_stage 0→1 的次数）

    // ── 补弹量追踪 ────────────────────────────────────────────────────────────
    // projectile_allowance_can_get_ = 100*minutes_elapsed_ - total_taken_（夹到≥0）
    // 规则：每隔1分钟放出100发；弹量上升按 100/200/300… 对齐记账（不受开火干扰）
    int projectile_allowance_can_get_{0};
    int minutes_elapsed_{0};               // 开赛后跨过的整分钟数
    int total_taken_{0};                   // 累计领走量（按 100 对齐）
    int last_projectile_allowance_{300};  // 上次收到的 projectile_allowance，初始300
    int last_stage_remain_time_{-1};      // 上次时间（用于检测分钟跨越）
    int last_game_progress_{0};           // 0=未开始, 1=比赛中, 2=已结束

    // 未被击打计时：记录上一次 be_attacked==true 的时刻
    bool last_be_attacked_{false};
    bool safe_timing_active_{false};  // 仅在出现下降沿后、下一次上升沿前为 true

    // ── enemy_detected 保持去抖：收到1立即输出true并启动保持计时器 ──────────
    // 计时器期间收到1刷新计时，收到0忽略；计时器到期后收到0才输出false
    bool   enemy_detected_hold_active_{false};  // 保持计时器是否激活
    std::chrono::steady_clock::time_point enemy_detected_last_true_time_{};
    static constexpr double kEnemyDetectedHoldSeconds = 2.0;  // 保持时长（秒）

    // ── 堡垒状态延时去抖写入黑板 ────────────────────────────────────────────
    int    bastion_pending_{0};
    int    bastion_stable_{0};
    bool   bastion_debounce_active_{false};
    std::chrono::steady_clock::time_point bastion_change_time_{};
    int    enemy_bastion_pending_{0};
    int    enemy_bastion_stable_{0};
    bool   enemy_bastion_debounce_active_{false};
    std::chrono::steady_clock::time_point enemy_bastion_change_time_{};
    static constexpr double kBastionDebounceSeconds = 1.0;

    // ── 堡垒连续占领计时（2/3 视为占领方；未锁定时 0/1 立即清零重计）──────
    // 满 19.9s 后锁定为 20.0，之后不受 0/1 影响（开堡后状态可能是 0 或 1）
    // 敌方堡垒被我方占领：enemy_bastion == 2 || 3
    bool enemy_bastion_our_occupy_active_{false};
    bool enemy_bastion_our_occupy_locked_{false};
    std::chrono::steady_clock::time_point enemy_bastion_our_occupy_start_{};
    // 我方堡垒被敌方占领：team_bastion == 2 || 3
    bool team_bastion_enemy_occupy_active_{false};
    bool team_bastion_enemy_occupy_locked_{false};
    std::chrono::steady_clock::time_point team_bastion_enemy_occupy_start_{};

    // ── 飞镖命中次数统计：last_hit_time 变化时按 last_hit_target 累加 ───────
    int last_dart_hit_time_{-1};  // -1 表示尚未收到过
    int dart_hit_outpost_count_{0};                 // target=1
    int dart_hit_base_fixed_count_{0};              // target=2
    int dart_hit_base_random_fixed_count_{0};       // target=3
    int dart_hit_base_random_moving_count_{0};      // target=4
    int dart_hit_base_end_moving_count_{0};          // target=5 基地末端移动

    std::chrono::steady_clock::time_point last_attacked_time_{std::chrono::steady_clock::now()};
    rclcpp::TimerBase::SharedPtr attack_timer_;  // 定期刷新 time_since_attacked 到黑板
    rclcpp::TimerBase::SharedPtr zone_timer_;    // 定期检查机器人是否在 zone_a / zone_b

    // ── TF ───────────────────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ── 区域多边形（从 YAML 加载，射线法判断）───────────────────────────────
    using Point2d = std::pair<double, double>;
    std::vector<Point2d> zone_a_polygon_;
    std::vector<Point2d> zone_b_polygon_;
    std::vector<Point2d> zone_c_polygon_;  // 用于障碍物检测（代价地图）
    std::vector<Point2d> zone_d_polygon_;  // 用于障碍物检测（代价地图）
    std::vector<Point2d> zone_e_polygon_;
    std::vector<Point2d> zone_f_polygon_;
    std::vector<Point2d> zone_g_polygon_;
    std::vector<Point2d> zone_h_polygon_;
    std::vector<Point2d> zone_i_polygon_;
    std::vector<Point2d> zone_j_polygon_;

    /** 从 check_in_area_config.yaml 加载指定名称区域的多边形顶点 */
    std::vector<Point2d> load_polygon(const std::string & zone_name);
    void load_zone_a_polygon();
    /** 射线法：点 (px,py) 是否在多边形内 */
    static bool point_in_polygon(double px, double py,
                                 const std::vector<Point2d> & poly);
    /**
     * 通用代价地图-多边形检测：
     * 遍历多边形包围盒内的所有格子，若中心点在多边形内且代价 >= threshold 则返回 true。
     */
    static bool has_local_cost_in_polygon(
        const nav_msgs::msg::OccupancyGrid & costmap,
        const std::vector<Point2d> & polygon,
        int threshold = 100);

    /** 根据缓存的敌方状态/增益，向 /enemy_status 发布无敌机器人列表 */
    void publish_enemy_invincible_status();

    /** 统计 zone_h 内且 status==0（存活）的敌方英雄/步兵3/步兵4/哨兵数量 */
    void update_zone_h_alive_enemy_count();

    /** 刷新堡垒连续占领时长到黑板 */
    void refresh_bastion_occupy_times();

    // ── Subscribers ──────────────────────────────────────────────────────────
    rclcpp::Subscription<robot_msg::msg::RefereeInfoMsg>::SharedPtr sub_referee_info_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_hp_;
    rclcpp::Subscription<robot_msg::msg::TeamRobotHpMsg>::SharedPtr sub_team_hp_;
    rclcpp::Subscription<robot_msg::msg::EnemyRobotHpMsg>::SharedPtr sub_enemy_hp_;
    rclcpp::Subscription<robot_msg::msg::EnemyRobotStatusMsg>::SharedPtr sub_enemy_status_;
    rclcpp::Subscription<robot_msg::msg::EnemyRobotBuffMsg>::SharedPtr sub_enemy_buff_;
    rclcpp::Subscription<robot_msg::msg::EnemyRobotPositionMsg>::SharedPtr sub_enemy_position_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
    rclcpp::Subscription<robot_msg::msg::AreaStatusMsg>::SharedPtr sub_area_status_;
    rclcpp::Subscription<robot_msg::msg::PostureFeedbackMsg>::SharedPtr sub_be_attacked_;
    rclcpp::Subscription<robot_msg::msg::BufferStateMsg>::SharedPtr sub_buffer_state_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_auto_aim_target_pos_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_local_costmap_;

    // ── Publishers ───────────────────────────────────────────────────────────
    rclcpp::Publisher<sp_msgs::msg::EnemyStatusMsg>::SharedPtr enemy_status_pub_;

    // ── 敌方状态/增益/位置缓存 ───────────────────────────────────────────────
    robot_msg::msg::EnemyRobotStatusMsg latest_enemy_status_{};
    robot_msg::msg::EnemyRobotBuffMsg latest_enemy_buff_{};
    robot_msg::msg::EnemyRobotPositionMsg latest_enemy_position_{};
    bool has_enemy_status_{false};
    bool has_enemy_buff_{false};
    bool has_enemy_position_{false};

    // ── 代价地图缓存（zone_timer 中读取）────────────────────────────────────
    mutable std::mutex costmap_mutex_;
    std::optional<nav_msgs::msg::OccupancyGrid> latest_local_costmap_;
  };

} // namespace sp_decision

#endif // SP_DECISION_BLACKBOARD_BRIDGE_HPP_
