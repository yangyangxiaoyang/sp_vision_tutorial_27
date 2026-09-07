#include "sp_decision/blackboard_bridge.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace sp_decision
{

  BlackboardBridge::BlackboardBridge(
      BT::Blackboard::Ptr blackboard,
      const rclcpp::NodeOptions &options)
      : Node("blackboard_bridge", options),
        blackboard_(blackboard)
  {
    using namespace std::placeholders;

    auto qos_reliable = rclcpp::QoS(1).reliable();

    // ── Referee info: includes game_progress/stage info ────────────────
    sub_referee_info_ = this->create_subscription<robot_msg::msg::RefereeInfoMsg>(
        "/referee_info", qos_reliable,
        std::bind(&BlackboardBridge::on_referee_info, this, _1));

    // ── Full team hp (includes sentry_hp) ───────────────────────────────
    sub_team_hp_ = this->create_subscription<robot_msg::msg::TeamRobotHpMsg>(
        "/team_robot_hp", qos_reliable,
        std::bind(&BlackboardBridge::on_team_hp, this, _1));

    // ── Enemy outpost / base hp ─────────────────────────────────────────
    sub_enemy_hp_ = this->create_subscription<robot_msg::msg::EnemyRobotHpMsg>(
        "/enemy_robot_hp", qos_reliable,
        std::bind(&BlackboardBridge::on_enemy_hp, this, _1));

    // ── 敌方主要状态 / 增益 → 合成无敌列表发布到 /enemy_status ──────────
    enemy_status_pub_ = this->create_publisher<sp_msgs::msg::EnemyStatusMsg>(
        "/enemy_status", qos_reliable);
    sub_enemy_status_ = this->create_subscription<robot_msg::msg::EnemyRobotStatusMsg>(
        "/enemy_robot_status", qos_reliable,
        std::bind(&BlackboardBridge::on_enemy_status, this, _1));
    sub_enemy_buff_ = this->create_subscription<robot_msg::msg::EnemyRobotBuffMsg>(
        "/enemy_robot_buff", qos_reliable,
        std::bind(&BlackboardBridge::on_enemy_buff, this, _1));
    sub_enemy_position_ = this->create_subscription<robot_msg::msg::EnemyRobotPositionMsg>(
        "/enemy_robot_position", qos_reliable,
        std::bind(&BlackboardBridge::on_enemy_position, this, _1));

    // ── Manually clicked / published goal pose ────────────────────────────
    sub_goal_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", qos_reliable,
        std::bind(&BlackboardBridge::on_goal_pose, this, _1));

    // ── Field area status (center boost / supply zone) ────────────────────
    sub_area_status_ = this->create_subscription<robot_msg::msg::AreaStatusMsg>(
        "/area_status", qos_reliable,
        std::bind(&BlackboardBridge::on_area_status, this, _1));

    // ── 哨兵被攻击反馈 ──
    sub_be_attacked_ = this->create_subscription<robot_msg::msg::PostureFeedbackMsg>(
        "/sentry/posture_feedback", qos_reliable,
        std::bind(&BlackboardBridge::on_posture_feedback, this, _1));

    // ── 能量机关状态 (0=关闭, 1=小符正在激活中, 2=小符已激活, 3=大符正在激活中, 4=大符已激活) ────────────────────────────
    sub_buffer_state_ = this->create_subscription<robot_msg::msg::BufferStateMsg>(
        "/buffer_state", qos_reliable,
        std::bind(&BlackboardBridge::on_buffer_state, this, _1));

    // ── 自瞄目标位置（70Hz）：解析 "x,y,detected,ArmorName" 格式字符串 ──
    // detected==1 时写 enemy_detected=true，否则写 false
    sub_auto_aim_target_pos_ = this->create_subscription<std_msgs::msg::String>(
        "/auto_aim_target_pos", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&BlackboardBridge::on_auto_aim_target_pos, this, _1));

    // ── 里程计速度（500Hz）：写入 odom_vx / odom_vy（里程计坐标系，m/s）──
    sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&BlackboardBridge::on_odometry, this, _1));

    // ── 局部代价地图缓存（用于 zone 内障碍检测）──────────────────────────────
    sub_local_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap/costmap",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(costmap_mutex_);
          latest_local_costmap_ = *msg;
        });

    // ── 计时器：每 100ms 刷新未被击打时长 / 堡垒连续占领时长 ──
    attack_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
          if (safe_timing_active_) {
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_attacked_time_).count();
            write<double>("time_since_attacked", elapsed);
          }
          refresh_bastion_occupy_times();
        });

    // ── TF：初始化 buffer 和 listener ──────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ── 从 YAML 加载多边形 ────────────────────────────────
    load_zone_a_polygon();
    zone_b_polygon_ = load_polygon("zone_b");
    RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] zone_b polygon loaded: %zu vertices.", zone_b_polygon_.size());
    zone_c_polygon_ = load_polygon("zone_c");
    RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] zone_c polygon loaded: %zu vertices.", zone_c_polygon_.size());
    zone_d_polygon_ = load_polygon("zone_d");
    RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] zone_d polygon loaded: %zu vertices.", zone_d_polygon_.size());
    zone_e_polygon_ = load_polygon("zone_e");
    RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] zone_e polygon loaded: %zu vertices.", zone_e_polygon_.size());
    zone_f_polygon_ = load_polygon("zone_f");
    RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] zone_f polygon loaded: %zu vertices.", zone_f_polygon_.size());
    zone_g_polygon_ = load_polygon("zone_g");
    RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] zone_g polygon loaded: %zu vertices.", zone_g_polygon_.size());
    zone_h_polygon_ = load_polygon("zone_h");
    RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] zone_h polygon loaded: %zu vertices.", zone_h_polygon_.size());
    zone_i_polygon_ = load_polygon("zone_i");
    RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] zone_i polygon loaded: %zu vertices.", zone_i_polygon_.size());
    zone_j_polygon_ = load_polygon("zone_j");
    RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] zone_j polygon loaded: %zu vertices.", zone_j_polygon_.size());

    // ── zone 检查定时器：每 200ms 查一次 TF ──────────────────────────────────
    zone_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() {
          // ── 机器人位置判断（需要 TF）────────────────────────────────────────
          if (zone_a_polygon_.size() >= 3 || zone_b_polygon_.size() >= 3 ||
              zone_e_polygon_.size() >= 3 || zone_f_polygon_.size() >= 3 ||
              zone_g_polygon_.size() >= 3 || zone_h_polygon_.size() >= 3 ||
              zone_i_polygon_.size() >= 3 || zone_j_polygon_.size() >= 3) {
            try {
              auto tf = tf_buffer_->lookupTransform(
                  "map", "base_link",
                  tf2::TimePointZero,
                  tf2::durationFromSec(0.05));
              const double x = tf.transform.translation.x;
              const double y = tf.transform.translation.y;
              if (zone_a_polygon_.size() >= 3)
                write<bool>("in_zone_a", point_in_polygon(x, y, zone_a_polygon_));
              if (zone_b_polygon_.size() >= 3)
                write<bool>("in_zone_b", point_in_polygon(x, y, zone_b_polygon_));
              if (zone_e_polygon_.size() >= 3)
                write<bool>("in_zone_e", point_in_polygon(x, y, zone_e_polygon_));
              if (zone_f_polygon_.size() >= 3)
                write<bool>("in_zone_f", point_in_polygon(x, y, zone_f_polygon_));
              if (zone_g_polygon_.size() >= 3)
                write<bool>("in_zone_g", point_in_polygon(x, y, zone_g_polygon_));
              if (zone_h_polygon_.size() >= 3)
                write<bool>("in_zone_h", point_in_polygon(x, y, zone_h_polygon_));
              if (zone_i_polygon_.size() >= 3)
                write<bool>("in_zone_i", point_in_polygon(x, y, zone_i_polygon_));
              if (zone_j_polygon_.size() >= 3)
                write<bool>("in_zone_j", point_in_polygon(x, y, zone_j_polygon_));
            } catch (const tf2::TransformException &) {
              // TF 尚未就绪时静默忽略，黑板保持上次值
            }
          }

          // ── zone_c / zone_d 代价地图障碍检测（独立于机器人位置）────────────
          std::optional<nav_msgs::msg::OccupancyGrid> costmap_copy;
          {
            std::lock_guard<std::mutex> lk(costmap_mutex_);
            costmap_copy = latest_local_costmap_;
          }
          if (costmap_copy.has_value()) {
            if (zone_c_polygon_.size() >= 3)
              write<bool>("zone_c_has_local_cost",
                  has_local_cost_in_polygon(*costmap_copy, zone_c_polygon_));
            if (zone_d_polygon_.size() >= 3)
              write<bool>("zone_d_has_local_cost",
                  has_local_cost_in_polygon(*costmap_copy, zone_d_polygon_));
          }
        });

    RCLCPP_INFO(this->get_logger(), "[BlackboardBridge] Initialized.");
  }

  // ─────────────────────────────────────────────────────────────────────────────

  void BlackboardBridge::set_blackboard(BT::Blackboard::Ptr blackboard)
  {
    std::lock_guard<std::mutex> lk(bb_mutex_);
    blackboard_ = blackboard;
    init_all_variables();
  }

  void BlackboardBridge::init_all_variables()
  {
    blackboard_->set<double>("team_sentry_hp", 400.0);
    blackboard_->set<double>("add_blood_stage", 0.0);
    blackboard_->set<double>("add_blood_stage2", 0.0);
    blackboard_->set<double>("add_blood_stage3", 0.0);
    blackboard_->set<int>("game_progress", 1);
    blackboard_->set<int>("stage_remain_time", 180);
    blackboard_->set<int>("key", 0);
    blackboard_->set<double>("clicked_point_update", 0.0);
    blackboard_->set<geometry_msgs::msg::PoseStamped>("clicked_point", geometry_msgs::msg::PoseStamped());
    blackboard_->set<int>("centrum", 0);
    blackboard_->set<int>("team_bastion", 0);
    blackboard_->set<int>("enemy_bastion", 0);
    blackboard_->set<double>("enemy_bastion_our_occupy_time", 0.0);
    blackboard_->set<double>("team_bastion_enemy_occupy_time", 0.0);
    bastion_pending_ = 0;
    bastion_stable_ = 0;
    bastion_debounce_active_ = false;
    enemy_bastion_pending_ = 0;
    enemy_bastion_stable_ = 0;
    enemy_bastion_debounce_active_ = false;
    enemy_bastion_our_occupy_active_ = false;
    enemy_bastion_our_occupy_locked_ = false;
    team_bastion_enemy_occupy_active_ = false;
    team_bastion_enemy_occupy_locked_ = false;
    blackboard_->set<int>("dart_hit_outpost_count", 0);
    blackboard_->set<int>("dart_hit_base_fixed_count", 0);
    blackboard_->set<int>("dart_hit_base_random_fixed_count", 0);
    blackboard_->set<int>("dart_hit_base_random_moving_count", 0);
    blackboard_->set<int>("dart_hit_base_end_moving_count", 0);
    last_dart_hit_time_ = -1;
    dart_hit_outpost_count_ = 0;
    dart_hit_base_fixed_count_ = 0;
    dart_hit_base_random_fixed_count_ = 0;
    dart_hit_base_random_moving_count_ = 0;
    dart_hit_base_end_moving_count_ = 0;
    blackboard_->set<bool>("be_attacked", 0);
    blackboard_->set<double>("time_since_attacked", 0.0);
    blackboard_->set<int>("buffer_state", 0);
    blackboard_->set<int>("projectile_allowance", 300);
    blackboard_->set<int>("projectile_allowance_can_get", 0);
    blackboard_->set<int>("projectile_allowance_bastion", 0);
    blackboard_->set<double>("add_bullet_stage", 0.0);
    blackboard_->set<double>("add_bullet_stage2", 0.0);
    blackboard_->set<double>("add_bullet_stage3", 0.0);
    blackboard_->set<int>("current_posture", 0);
    blackboard_->set<int>("barrel_heat", 0);
    blackboard_->set<bool>("in_zone_a", false);
    blackboard_->set<bool>("in_zone_b", false);
    blackboard_->set<bool>("in_zone_e", false);
    blackboard_->set<bool>("in_zone_f", false);
    blackboard_->set<int>("zone_h_alive_enemy_count", 0);
    blackboard_->set<bool>("in_zone_g", false);
    blackboard_->set<bool>("in_zone_h", false);
    blackboard_->set<bool>("in_zone_i", false);
    blackboard_->set<bool>("in_zone_j", false);
    blackboard_->set<bool>("zone_c_has_local_cost", false);
    blackboard_->set<bool>("zone_d_has_local_cost", false);
    blackboard_->set<int>("nav_success", 1);
    blackboard_->set<bool>("enemy_detected", false);
    blackboard_->set<double>("odom_speed", 0.0);
    blackboard_->set<int>("back_count", 0);
    blackboard_->set<int>("remaining_energy", 100);
    blackboard_->set<int>("team_outpost_hp", 1500);
    blackboard_->set<int>("team_base_hp", 5000);
    blackboard_->set<int>("enemy_outpost_hp", 1500);
    blackboard_->set<int>("enemy_base_hp", 5000);
    blackboard_->set<double>("back_G", 0.0);
    blackboard_->set<double>("back_H", 0.0);
    blackboard_->set<double>("back_J", 0.0);
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Callbacks

  void BlackboardBridge::on_referee_info(const robot_msg::msg::RefereeInfoMsg::SharedPtr msg)
  {
    const int game_progress = static_cast<int>(msg->game_progress);
    const int stage_remain_time = static_cast<int>(msg->stage_remain_time);
    const int projectile_allowance = static_cast<int>(msg->projectile_allowance);

    write<int>("game_progress", game_progress);
    write<int>("stage_remain_time", stage_remain_time);
    write<int>("projectile_allowance", projectile_allowance);
    write<int>("projectile_allowance_bastion", static_cast<int>(msg->projectile_allowance_bastion));
    write<int>("remaining_energy", static_cast<int>(msg->remaining_energy));
    write<int>("key", static_cast<int>(msg->key));

    const int can_get_before = projectile_allowance_can_get_;

    // ── 补弹量追踪状态机 ─────────────────────────────────────────────────────
    // last_game_progress_: 0=等待开赛, 1=比赛中, 2=已结束
    if (last_game_progress_ == 0)
    {
      if (game_progress == 4)
      {
        last_game_progress_ = 1;
        last_stage_remain_time_ = stage_remain_time;
        last_projectile_allowance_ = projectile_allowance;
      }
    }
    else if (last_game_progress_ == 1)
    {
      // 规则：7 分钟赛（remain 从 420 倒数），落到 360/300/…/60 时各放出 100。
      // 必须用 (420-remain)/60，不能用 remain/60 桶变（后者要到 180→179 才跳，晚 1 秒）。
      // 先更新放出量，再记领弹，这样 remain=180 领弹能领到当秒刚放出的 100。
      minutes_elapsed_ = std::min(6, std::max(0, (420 - stage_remain_time) / 60));
      last_stage_remain_time_ = stage_remain_time;

      // 领弹：弹量上升即视为一次领完（规则上是 100/200/300…）。
      // 不直接用实测增量（开火会干扰），而是对齐到最近的 100 倍数。
      if (projectile_allowance > last_projectile_allowance_)
      {
        const int delta = projectile_allowance - last_projectile_allowance_;
        int packs = (delta + 50) / 100;  // 四舍五入到 100 的倍数
        if (packs < 1) packs = 1;        // 只要上升，至少记 1 包（覆盖 0→86 这类开火干扰）
        total_taken_ += packs * 100;
      }
      last_projectile_allowance_ = projectile_allowance;

      // 可领量 = 累计放出 − 累计领走
      projectile_allowance_can_get_ = 100 * minutes_elapsed_ - total_taken_;
      if (projectile_allowance_can_get_ < 0) projectile_allowance_can_get_ = 0;

      // 比赛结束检测
      if (game_progress != 4)
      {
        last_game_progress_ = 2;
      }
    }

    write<int>("projectile_allowance_can_get", projectile_allowance_can_get_);

    if (projectile_allowance_can_get_ != can_get_before)
    {
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] can_get %d -> %d (allowance %d, remain=%ds)",
        can_get_before, projectile_allowance_can_get_,
        projectile_allowance, stage_remain_time);
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[BlackboardBridge] allowance=%d  can_get=%d  game_progress=%d  remain=%ds",
      projectile_allowance, projectile_allowance_can_get_, game_progress, stage_remain_time);

    // ── 补弹状态机 ──────────────────────────────────────────────────────────
    // 0: 弹量充足或无弹可领, 1: 弹量不足且补给区有弹可领 → 去补弹点
    const bool low_ammo   = (projectile_allowance < 50);
    const bool has_supply = (projectile_allowance_can_get_ > 20);

    // 进入补弹：弹量低且补给区有弹
    if (low_ammo && has_supply && add_bullet_stage_ == 0)
    {
      add_bullet_stage_ = 1;
      write<double>("add_bullet_stage", static_cast<double>(add_bullet_stage_));
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] add_bullet_stage -> 1 (allowance=%d can_get=%d)",
        projectile_allowance, projectile_allowance_can_get_);
    }

    // 退出补弹：处于补弹阶段，且无弹可领或弹量>90
    if (add_bullet_stage_ == 1 && (!has_supply || projectile_allowance > 90))
    {
      add_bullet_stage_ = 0;
      write<double>("add_bullet_stage", static_cast<double>(add_bullet_stage_));
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] add_bullet_stage -> 0 (allowance=%d can_get=%d)",
        projectile_allowance, projectile_allowance_can_get_);
    }

    // ── 补弹状态机 2 ──────────────────────────────────────────────────────────
    const bool low_ammo2 = (projectile_allowance < 200);
    if (low_ammo2 && has_supply && add_bullet_stage2_ == 0)
    {
      add_bullet_stage2_ = 1;
      write<double>("add_bullet_stage2", static_cast<double>(add_bullet_stage2_));
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] add_bullet_stage2 -> 1 (allowance=%d can_get=%d)",
        projectile_allowance, projectile_allowance_can_get_);
    }
    if (add_bullet_stage2_ == 1 && (!has_supply || projectile_allowance > 90))
    {
      add_bullet_stage2_ = 0;
      write<double>("add_bullet_stage2", static_cast<double>(add_bullet_stage2_));
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] add_bullet_stage2 -> 0 (allowance=%d can_get=%d)",
        projectile_allowance, projectile_allowance_can_get_);
    }

    // ── 补弹状态机 3 ──────────────────────────────────────────────────────────
    if (low_ammo && has_supply && add_bullet_stage3_ == 0)
    {
      add_bullet_stage3_ = 1;
      write<double>("add_bullet_stage3", static_cast<double>(add_bullet_stage3_));
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] add_bullet_stage3 -> 1 (allowance=%d can_get=%d)",
        projectile_allowance, projectile_allowance_can_get_);
    }
    if (add_bullet_stage3_ == 1 && (!has_supply || projectile_allowance > 90))
    {
      add_bullet_stage3_ = 0;
      write<double>("add_bullet_stage3", static_cast<double>(add_bullet_stage3_));
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] add_bullet_stage3 -> 0 (allowance=%d can_get=%d)",
        projectile_allowance, projectile_allowance_can_get_);
    }
  }

  void BlackboardBridge::on_team_hp(const robot_msg::msg::TeamRobotHpMsg::SharedPtr msg)
  {
    write<double>("team_sentry_hp", static_cast<double>(msg->sentry_hp));
    write<int>("team_outpost_hp", static_cast<int>(msg->outpost_hp));
    write<int>("team_base_hp", static_cast<int>(msg->base_hp));

    // ── 哨兵加血状态机 ──
    // 0: 初始/正常/补血完成，1: 回家补血
    if (msg->sentry_hp < 220 && add_blood_stage_ == 0)
    {
      add_blood_stage_ = 1;
      write<double>("add_blood_stage", static_cast<double>(add_blood_stage_));
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] add_blood_stage -> 1 (sentry_hp=%u)", msg->sentry_hp);
    }
    else if (msg->sentry_hp >= 380 && add_blood_stage_ == 1)
    {
      add_blood_stage_ = 0;
      back_count_++;
      write<int>("back_count", back_count_);
      write<double>("add_blood_stage", static_cast<double>(add_blood_stage_));
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] add_blood_stage -> 0 (sentry_hp=%u, back_count=%d)",
        msg->sentry_hp, back_count_);
    }

    // ── 哨兵加血状态机 2（阈值：<320 进入，>=380 退出）──────────────────────
    if (msg->sentry_hp < 320 && add_blood_stage2_ == 0)
    {
      add_blood_stage2_ = 1;
      write<double>("add_blood_stage2", static_cast<double>(add_blood_stage2_));
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] add_blood_stage2 -> 1 (sentry_hp=%u)", msg->sentry_hp);
    }
    else if (msg->sentry_hp >= 380 && add_blood_stage2_ == 1)
    {
      add_blood_stage2_ = 0;
      write<double>("add_blood_stage2", static_cast<double>(add_blood_stage2_));
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] add_blood_stage2 -> 0 (sentry_hp=%u)", msg->sentry_hp);
    }

    // ── 哨兵加血状态机 3（阈值：<120 进入，>=240 退出）──────────────────────
    if (msg->sentry_hp < 120 && add_blood_stage3_ == 0)
    {
      add_blood_stage3_ = 1;
      write<double>("add_blood_stage3", static_cast<double>(add_blood_stage3_));
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] add_blood_stage3 -> 1 (sentry_hp=%u)", msg->sentry_hp);
    }
    else if (msg->sentry_hp >= 240 && add_blood_stage3_ == 1)
    {
      add_blood_stage3_ = 0;
      write<double>("add_blood_stage3", static_cast<double>(add_blood_stage3_));
      RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] add_blood_stage3 -> 0 (sentry_hp=%u)", msg->sentry_hp);
    }
  }

  void BlackboardBridge::on_enemy_hp(const robot_msg::msg::EnemyRobotHpMsg::SharedPtr msg)
  {
    write<int>("enemy_outpost_hp", static_cast<int>(msg->outpost_hp));
    write<int>("enemy_base_hp", static_cast<int>(msg->base_hp));
  }

  void BlackboardBridge::on_enemy_status(const robot_msg::msg::EnemyRobotStatusMsg::SharedPtr msg)
  {
    latest_enemy_status_ = *msg;
    has_enemy_status_ = true;
    publish_enemy_invincible_status();
    update_zone_h_alive_enemy_count();
  }

  void BlackboardBridge::on_enemy_buff(const robot_msg::msg::EnemyRobotBuffMsg::SharedPtr msg)
  {
    latest_enemy_buff_ = *msg;
    has_enemy_buff_ = true;
    publish_enemy_invincible_status();
  }

  void BlackboardBridge::on_enemy_position(const robot_msg::msg::EnemyRobotPositionMsg::SharedPtr msg)
  {
    latest_enemy_position_ = *msg;
    has_enemy_position_ = true;
    update_zone_h_alive_enemy_count();
  }

  void BlackboardBridge::update_zone_h_alive_enemy_count()
  {
    if (!has_enemy_status_ || !has_enemy_position_ || zone_h_polygon_.size() < 3) {
      return;
    }

    // 雷达位置为 cm 原值，zone 多边形为 map 坐标系米
    auto alive_in_zone_h = [this](uint8_t status, int16_t x_cm, int16_t y_cm) {
      if (status != 0) {
        return false;
      }
      const double x = static_cast<double>(x_cm) * 0.01;
      const double y = static_cast<double>(y_cm) * 0.01;
      return point_in_polygon(x, y, zone_h_polygon_);
    };

    int count = 0;
    if (alive_in_zone_h(latest_enemy_status_.hero_status,
                        latest_enemy_position_.hero_x,
                        latest_enemy_position_.hero_y)) {
      ++count;
    }
    if (alive_in_zone_h(latest_enemy_status_.infantry_3_status,
                        latest_enemy_position_.infantry_3_x,
                        latest_enemy_position_.infantry_3_y)) {
      ++count;
    }
    if (alive_in_zone_h(latest_enemy_status_.infantry_4_status,
                        latest_enemy_position_.infantry_4_x,
                        latest_enemy_position_.infantry_4_y)) {
      ++count;
    }
    if (alive_in_zone_h(latest_enemy_status_.sentry_status,
                        latest_enemy_position_.sentry_x,
                        latest_enemy_position_.sentry_y)) {
      ++count;
    }

    write<int>("zone_h_alive_enemy_count", count);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[BlackboardBridge] zone_h_alive_enemy_count=%d", count);
  }

  void BlackboardBridge::publish_enemy_invincible_status()
  {
    if (!has_enemy_status_ && !has_enemy_buff_) {
      return;
    }

    auto is_invincible_status = [](uint8_t status) {
      return status == 2 || status == 3;
    };

    sp_msgs::msg::EnemyStatusMsg out;
    out.timestamp = this->now();

    // 编号: 1英雄 2工程 3号步兵 4号步兵 6哨兵（无5）
    if (has_enemy_status_) {
      if (is_invincible_status(latest_enemy_status_.hero_status)) {
        out.invincible_enemy_ids.push_back(1);
      }
      if (is_invincible_status(latest_enemy_status_.engineer_status)) {
        out.invincible_enemy_ids.push_back(2);
      }
      if (is_invincible_status(latest_enemy_status_.infantry_3_status)) {
        out.invincible_enemy_ids.push_back(3);
      }
      if (is_invincible_status(latest_enemy_status_.infantry_4_status)) {
        out.invincible_enemy_ids.push_back(4);
      }
    }

    bool sentry_invincible = false;
    if (has_enemy_status_ &&
        is_invincible_status(latest_enemy_status_.sentry_status)) {
      sentry_invincible = true;
    }
    // 哨兵防御增益 > 90 也视为无敌
    if (has_enemy_buff_ && latest_enemy_buff_.sentry_def > 90) {
      sentry_invincible = true;
    }
    if (sentry_invincible) {
      out.invincible_enemy_ids.push_back(6);
    }

    enemy_status_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[BlackboardBridge] /enemy_status invincible count=%zu sentry_def=%u",
        out.invincible_enemy_ids.size(),
        has_enemy_buff_ ? static_cast<unsigned>(latest_enemy_buff_.sentry_def) : 0u);
  }

  void BlackboardBridge::on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    write<geometry_msgs::msg::PoseStamped>("clicked_point", *msg);
    write<double>("clicked_point_update", 1.0);
  }

  void BlackboardBridge::on_area_status(const robot_msg::msg::AreaStatusMsg::SharedPtr msg)
  {
    write<int>("centrum", static_cast<int>(msg->centrum));

    const int team_bastion = static_cast<int>(msg->team_bastion);
    const int enemy_bastion = static_cast<int>(msg->enemy_bastion);

    const auto now = std::chrono::steady_clock::now();

    // ── team_bastion / enemy_bastion：连续保持相同值超过阈值才写黑板 ──────
    if (team_bastion == bastion_pending_) {
      if (bastion_debounce_active_) {
        const double elapsed = std::chrono::duration<double>(
            now - bastion_change_time_).count();
        if (elapsed >= kBastionDebounceSeconds) {
          bastion_stable_ = team_bastion;
          bastion_debounce_active_ = false;
          write<int>("team_bastion", bastion_stable_);
          RCLCPP_INFO(this->get_logger(),
              "[BlackboardBridge] team_bastion confirmed -> %d (after %.1fs)",
              bastion_stable_, elapsed);
        }
      }
    } else {
      bastion_pending_ = team_bastion;
      bastion_change_time_ = now;
      bastion_debounce_active_ = (team_bastion != bastion_stable_);
    }

    if (enemy_bastion == enemy_bastion_pending_) {
      if (enemy_bastion_debounce_active_) {
        const double elapsed = std::chrono::duration<double>(
            now - enemy_bastion_change_time_).count();
        if (elapsed >= kBastionDebounceSeconds) {
          enemy_bastion_stable_ = enemy_bastion;
          enemy_bastion_debounce_active_ = false;
          write<int>("enemy_bastion", enemy_bastion_stable_);
          RCLCPP_INFO(this->get_logger(),
              "[BlackboardBridge] enemy_bastion confirmed -> %d (after %.1fs)",
              enemy_bastion_stable_, elapsed);
        }
      }
    } else {
      enemy_bastion_pending_ = enemy_bastion;
      enemy_bastion_change_time_ = now;
      enemy_bastion_debounce_active_ = (enemy_bastion != enemy_bastion_stable_);
    }

    // 敌方堡垒被我方连续占领：enemy_bastion 为 2/3；未锁定时 0/1 立即重置
    const bool our_hold_enemy = (enemy_bastion == 2 || enemy_bastion == 3);
    if (!enemy_bastion_our_occupy_locked_) {
      if (our_hold_enemy) {
        if (!enemy_bastion_our_occupy_active_) {
          enemy_bastion_our_occupy_active_ = true;
          enemy_bastion_our_occupy_start_ = now;
          write<double>("enemy_bastion_our_occupy_time", 0.0);
          RCLCPP_INFO(this->get_logger(),
              "[BlackboardBridge] enemy_bastion our occupy start (value=%d)", enemy_bastion);
        }
      } else if (enemy_bastion_our_occupy_active_) {
        enemy_bastion_our_occupy_active_ = false;
        write<double>("enemy_bastion_our_occupy_time", 0.0);
        RCLCPP_INFO(this->get_logger(),
            "[BlackboardBridge] enemy_bastion our occupy reset (value=%d)", enemy_bastion);
      }
    }

    // 我方堡垒被敌方连续占领：team_bastion 为 2/3；未锁定时 0/1 立即重置
    const bool enemy_hold_team = (team_bastion == 2 || team_bastion == 3);
    if (!team_bastion_enemy_occupy_locked_) {
      if (enemy_hold_team) {
        if (!team_bastion_enemy_occupy_active_) {
          team_bastion_enemy_occupy_active_ = true;
          team_bastion_enemy_occupy_start_ = now;
          write<double>("team_bastion_enemy_occupy_time", 0.0);
          RCLCPP_INFO(this->get_logger(),
              "[BlackboardBridge] team_bastion enemy occupy start (value=%d)", team_bastion);
        }
      } else if (team_bastion_enemy_occupy_active_) {
        team_bastion_enemy_occupy_active_ = false;
        write<double>("team_bastion_enemy_occupy_time", 0.0);
        RCLCPP_INFO(this->get_logger(),
            "[BlackboardBridge] team_bastion enemy occupy reset (value=%d)", team_bastion);
      }
    }

    refresh_bastion_occupy_times();

    // ── 飞镖命中：last_hit_time 刷新时按 target 累加次数 ───────────────────
    const int hit_time = static_cast<int>(msg->last_hit_time);
    const int hit_target = static_cast<int>(msg->last_hit_target);
    if (hit_time != last_dart_hit_time_) {
      const int prev_time = last_dart_hit_time_;
      last_dart_hit_time_ = hit_time;
      // 跳过初值同步（prev=-1）以及 time==0 的默认态；仅在时间刷新为非 0 时计数
      if (prev_time >= 0 && hit_time != 0 && hit_target >= 1 && hit_target <= 5) {
        switch (hit_target) {
          case 1:
            ++dart_hit_outpost_count_;
            write<int>("dart_hit_outpost_count", dart_hit_outpost_count_);
            break;
          case 2:
            ++dart_hit_base_fixed_count_;
            write<int>("dart_hit_base_fixed_count", dart_hit_base_fixed_count_);
            break;
          case 3:
            ++dart_hit_base_random_fixed_count_;
            write<int>("dart_hit_base_random_fixed_count", dart_hit_base_random_fixed_count_);
            break;
          case 4:
            ++dart_hit_base_random_moving_count_;
            write<int>("dart_hit_base_random_moving_count", dart_hit_base_random_moving_count_);
            break;
          case 5:
            ++dart_hit_base_end_moving_count_;
            write<int>("dart_hit_base_end_moving_count", dart_hit_base_end_moving_count_);
            break;
          default:
            break;
        }
        RCLCPP_INFO(this->get_logger(),
            "[BlackboardBridge] dart hit: time %d->%d target=%d "
            "(outpost=%d fixed=%d rnd_fixed=%d rnd_move=%d end_move=%d)",
            prev_time, hit_time, hit_target,
            dart_hit_outpost_count_,
            dart_hit_base_fixed_count_,
            dart_hit_base_random_fixed_count_,
            dart_hit_base_random_moving_count_,
            dart_hit_base_end_moving_count_);
      }
    }
  }

  void BlackboardBridge::refresh_bastion_occupy_times()
  {
    const auto now = std::chrono::steady_clock::now();
    // 连续占领满 19.9s 后锁定为 20s；锁定后开堡状态 0/1 不再清零
    constexpr double kOccupyLockThreshold = 19.9;
    constexpr double kOccupyLockedValue = 20.0;

    if (enemy_bastion_our_occupy_locked_) {
      write<double>("enemy_bastion_our_occupy_time", kOccupyLockedValue);
    } else if (enemy_bastion_our_occupy_active_) {
      const double elapsed = std::chrono::duration<double>(
          now - enemy_bastion_our_occupy_start_).count();
      if (elapsed >= kOccupyLockThreshold) {
        enemy_bastion_our_occupy_locked_ = true;
        enemy_bastion_our_occupy_active_ = false;
        write<double>("enemy_bastion_our_occupy_time", kOccupyLockedValue);
        RCLCPP_INFO(this->get_logger(),
            "[BlackboardBridge] enemy_bastion_our_occupy_time locked -> 20.0");
      } else {
        write<double>("enemy_bastion_our_occupy_time", elapsed);
      }
    }

    if (team_bastion_enemy_occupy_locked_) {
      write<double>("team_bastion_enemy_occupy_time", kOccupyLockedValue);
    } else if (team_bastion_enemy_occupy_active_) {
      const double elapsed = std::chrono::duration<double>(
          now - team_bastion_enemy_occupy_start_).count();
      if (elapsed >= kOccupyLockThreshold) {
        team_bastion_enemy_occupy_locked_ = true;
        team_bastion_enemy_occupy_active_ = false;
        write<double>("team_bastion_enemy_occupy_time", kOccupyLockedValue);
        RCLCPP_INFO(this->get_logger(),
            "[BlackboardBridge] team_bastion_enemy_occupy_time locked -> 20.0");
      } else {
        write<double>("team_bastion_enemy_occupy_time", elapsed);
      }
    }
  }

  void BlackboardBridge::on_buffer_state(const robot_msg::msg::BufferStateMsg::SharedPtr msg)
  {
    write<int>("buffer_state", static_cast<int>(msg->state));
  }

  void BlackboardBridge::on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 里程计坐标系（通常为 lidar_odom）下的线速度
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    const double vz = msg->twist.twist.linear.z;
    write<double>("odom_speed", std::hypot(std::hypot(vx, vy), vz));
  }

  void BlackboardBridge::on_auto_aim_target_pos(const std_msgs::msg::String::SharedPtr msg)
  {
    // 消息格式: "x_in_gimbal,y_in_gimbal,detected,ArmorName"
    // 例如识别到: "0.12,-0.05,1,1"   未识别到: "0,0,0,0"
    // 只关心第3个字段（detected，0或1）
    const std::string & data = msg->data;
    bool enemy_detected = false;

    // 找到第2个逗号（跳过x, y）
    size_t p1 = data.find(',');
    if (p1 != std::string::npos) {
      size_t p2 = data.find(',', p1 + 1);
      if (p2 != std::string::npos) {
        size_t p3 = data.find(',', p2 + 1);
        const std::string detected_str = data.substr(p2 + 1,
            p3 == std::string::npos ? std::string::npos : p3 - p2 - 1);
        try {
          enemy_detected = (std::stoi(detected_str) != 0);
        } catch (...) {
          // 解析失败保持 false
        }
      }
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[BlackboardBridge] /auto_aim_target_pos received: \"%s\"  detected=%d",
        data.c_str(), static_cast<int>(enemy_detected));

    // ── enemy_detected 保持去抖逻辑 ──────────────────────────────────────────
    // 收到1：立即写 true，刷新保持计时器
    // 计时器期间收到0：忽略
    // 计时器到期后收到0：写 false
    const auto now = std::chrono::steady_clock::now();
    if (enemy_detected) {
      // 刷新计时器
      enemy_detected_last_true_time_ = now;
      if (!enemy_detected_hold_active_) {
        enemy_detected_hold_active_ = true;
        write<bool>("enemy_detected", true);
        RCLCPP_INFO(this->get_logger(),
            "[BlackboardBridge] enemy_detected -> true");
      }
    } else {
      if (enemy_detected_hold_active_) {
        const double elapsed = std::chrono::duration<double>(
            now - enemy_detected_last_true_time_).count();
        if (elapsed >= kEnemyDetectedHoldSeconds) {
          enemy_detected_hold_active_ = false;
          write<bool>("enemy_detected", false);
          RCLCPP_INFO(this->get_logger(),
              "[BlackboardBridge] enemy_detected -> false (hold expired %.2fs)", elapsed);
        }
        // 未到期：忽略此次 0
      }
    }
  }

  void BlackboardBridge::on_posture_feedback(const robot_msg::msg::PostureFeedbackMsg::SharedPtr msg)
  {
    // 当前姿态 & 热量 → 直接写入黑板
    write<int>("current_posture", static_cast<int>(msg->current_posture));
    write<int>("barrel_heat", static_cast<int>(msg->shooter_17mm_1_barrel_heat));

    const bool attacked_now = msg->be_attacked;    

    // 下降沿：被攻击结束 → 开始计时
    if (!attacked_now && last_be_attacked_)
    {
      last_attacked_time_ = std::chrono::steady_clock::now();
      safe_timing_active_ = true;
    }
    // 上升沿：再次被攻击 → 停止计时并清零
    if (attacked_now && !last_be_attacked_)
    {
      safe_timing_active_ = false;
      write<double>("time_since_attacked", 0.0);
    }

    last_be_attacked_ = attacked_now;
    write<bool>("be_attacked", attacked_now);
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // zone_a 多边形加载 & 射线法

  void BlackboardBridge::load_zone_a_polygon()
  {
    zone_a_polygon_ = load_polygon("zone_a");
    RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] zone_a polygon loaded: %zu vertices.",
        zone_a_polygon_.size());
  }

  std::vector<BlackboardBridge::Point2d> BlackboardBridge::load_polygon(const std::string & zone_name)
  {
    const std::string config_path =
        "/home/rm/Desktop/sp_nav_26/src/sp_decision/config/zone_config.yaml";

    std::ifstream fin(config_path);
    if (!fin.is_open()) {
      RCLCPP_WARN(this->get_logger(),
          "[BlackboardBridge] Cannot open %s, %s will always be false.",
          config_path.c_str(), zone_name.c_str());
      return {};
    }

    // ── 第一遍：读取 team_color ────────────────────────────────────────────
    std::string team_color = "red";  // 默认红方
    {
      std::string line;
      while (std::getline(fin, line)) {
        auto start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        std::string trimmed = line.substr(start, line.find_last_not_of(" \t\r\n") - start + 1);
        if (trimmed.empty() || trimmed[0] == '#') continue;
        if (trimmed.find("team_color:") == 0) {
          std::string val = trimmed.substr(11);
          auto vs = val.find_first_not_of(" \t\"");
          auto ve = val.find_last_not_of(" \t\"");
          if (vs != std::string::npos) team_color = val.substr(vs, ve - vs + 1);
          break;
        }
      }
    }
    // 构造目标 section 名，如 "red_areas:" 或 "blue_areas:"
    const std::string target_section = team_color + "_areas:";
    RCLCPP_INFO(this->get_logger(),
        "[BlackboardBridge] Loading zone '%s' from section '%s'",
        zone_name.c_str(), target_section.c_str());

    // ── 第二遍：从头重新扫描，找到正确的 section 再解析 ─────────────────────
    fin.clear();
    fin.seekg(0);

    bool in_areas  = false;
    bool in_target = false;
    std::vector<Point2d> result;

    std::string line;
    while (std::getline(fin, line)) {
      auto start = line.find_first_not_of(" \t\r\n");
      if (start == std::string::npos) continue;
      std::string trimmed = line.substr(start,
          line.find_last_not_of(" \t\r\n") - start + 1);
      if (trimmed.empty() || trimmed[0] == '#') continue;

      // 进入目标 section（如 red_areas:）
      if (trimmed.find(target_section) == 0) { in_areas = true; in_target = false; continue; }
      // 遇到其他顶级 key（不缩进，含冒号）→ 退出 section
      if (in_areas && trimmed[0] != '-' && trimmed.find(':') != std::string::npos) {
        const auto indent = line.find_first_not_of(" \t");
        if (indent == 0) { in_areas = false; in_target = false; continue; }
      }
      if (!in_areas) continue;

      if (trimmed[0] == '-') {
        auto content = trimmed.substr(1);
        auto cs = content.find_first_not_of(" \t");
        if (cs != std::string::npos) trimmed = content.substr(cs);
        else continue;
      }

      auto colon = trimmed.find(':');
      if (colon == std::string::npos) continue;
      std::string key = trimmed.substr(0, colon);
      auto vs = key.find_last_not_of(" \t");
      if (vs != std::string::npos) key = key.substr(0, vs + 1);

      std::string val = trimmed.substr(colon + 1);
      auto vs2 = val.find_first_not_of(" \t");
      if (vs2 != std::string::npos) val = val.substr(vs2);
      if (val.size() >= 2 && val.front() == '"' && val.back() == '"')
        val = val.substr(1, val.size() - 2);

      if (key == "name") {
        in_target = (val == zone_name);
        continue;
      }
      if (in_target && key == "vertices") {
        std::istringstream iss(val);
        std::string token;
        while (std::getline(iss, token, ';')) {
          auto s = token.find_first_not_of(" \t(");
          auto e = token.find_last_not_of(" \t)");
          if (s == std::string::npos) continue;
          token = token.substr(s, e - s + 1);
          auto comma = token.find(',');
          if (comma == std::string::npos) continue;
          try {
            double x = std::stod(token.substr(0, comma));
            double y = std::stod(token.substr(comma + 1));
            result.emplace_back(x, y);
          } catch (...) {}
        }
        break;
      }
    }
    return result;
  }

  bool BlackboardBridge::point_in_polygon(double px, double py, const std::vector<Point2d> & poly)
  {
    bool inside = false;
    const size_t n = poly.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
      const double xi = poly[i].first,  yi = poly[i].second;
      const double xj = poly[j].first,  yj = poly[j].second;
      if (((yi > py) != (yj > py)) &&
          (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
        inside = !inside;
    }
    return inside;
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 通用代价地图 × 多边形检测
  // 遍历多边形包围盒内的每个格子，若格子中心落在多边形内且代价 >= threshold，返回 true。
  // threshold 默认 100（lethal），可传入更低值检测 inflated 障碍。
  bool BlackboardBridge::has_local_cost_in_polygon(
      const nav_msgs::msg::OccupancyGrid & costmap,
      const std::vector<Point2d> & polygon,
      int threshold)
  {
    if (costmap.data.empty() || costmap.info.resolution <= 0.0 || polygon.size() < 3)
      return false;

    // 求包围盒
    double xmin = polygon[0].first,  xmax = polygon[0].first;
    double ymin = polygon[0].second, ymax = polygon[0].second;
    for (const auto & p : polygon) {
      xmin = std::min(xmin, p.first);  xmax = std::max(xmax, p.first);
      ymin = std::min(ymin, p.second); ymax = std::max(ymax, p.second);
    }

    const double origin_x  = costmap.info.origin.position.x;
    const double origin_y  = costmap.info.origin.position.y;
    const double resolution = costmap.info.resolution;
    const int width  = static_cast<int>(costmap.info.width);
    const int height = static_cast<int>(costmap.info.height);

    int ix_min = std::clamp(static_cast<int>(std::floor((xmin - origin_x) / resolution)), 0, width  - 1);
    int ix_max = std::clamp(static_cast<int>(std::floor((xmax - origin_x) / resolution)), 0, width  - 1);
    int iy_min = std::clamp(static_cast<int>(std::floor((ymin - origin_y) / resolution)), 0, height - 1);
    int iy_max = std::clamp(static_cast<int>(std::floor((ymax - origin_y) / resolution)), 0, height - 1);

    for (int iy = iy_min; iy <= iy_max; ++iy) {
      for (int ix = ix_min; ix <= ix_max; ++ix) {
        // 格子中心世界坐标
        const double cx = origin_x + (ix + 0.5) * resolution;
        const double cy = origin_y + (iy + 0.5) * resolution;
        if (!point_in_polygon(cx, cy, polygon)) continue;

        const int idx = iy * width + ix;
        if (idx < 0 || idx >= static_cast<int>(costmap.data.size())) continue;
        if (static_cast<int>(costmap.data[static_cast<size_t>(idx)]) >= threshold)
          return true;
      }
    }
    return false;
  }

} // namespace sp_decision
