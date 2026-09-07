#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <cmath>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

static std::string join_path_if_relative(const std::string& base_dir, const std::string& maybe_rel) {
  if (maybe_rel.empty()) return maybe_rel;
  fs::path p(maybe_rel);
  if (p.is_absolute()) return maybe_rel;
  return (fs::path(base_dir) / p).string();
}

static cv::Mat load_pgm_grayscale_u8(const std::string& image_path) {
  cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (img.empty()) {
    throw std::runtime_error("Failed to read image: " + image_path);
  }
  if (img.type() != CV_8UC1) {
    img.convertTo(img, CV_8UC1);
  }
  return img;
}

static cv::Mat distance_transform_meters_from_binary_u8(const cv::Mat& binary_u8, float resolution) {
  // binary_u8: non-zero = foreground, zero = background
  // distanceTransform returns distance of each foreground pixel to the nearest background pixel.
  cv::Mat dist;
  cv::distanceTransform(binary_u8, dist, cv::DIST_L2, 5); // CV_32FC1, in pixels
  dist *= resolution;
  return dist;
}

class EsdfMapPublisher : public rclcpp::Node {
public:
  EsdfMapPublisher() : Node("esdf_map_publisher") {
    this->declare_parameter<std::string>("map_yaml", "");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("publish_rate_hz", 1.0);

    this->declare_parameter<bool>("unknown_as_obstacle", true);

    // Cost mapping params
    this->declare_parameter<double>("robot_radius", 0.25);
    this->declare_parameter<double>("margin", 0.10);
    this->declare_parameter<double>("d_safe", 0.55);
    this->declare_parameter<double>("w", 10.0);
    this->declare_parameter<int>("unknown_cost", 100);
    this->declare_parameter<bool>("visualize_esdf", false);

    map_yaml_path_ = this->get_parameter("map_yaml").as_string();
    if (map_yaml_path_.empty()) {
      throw std::runtime_error("Parameter 'map_yaml' is empty. Provide path to map.yaml");
    }
    frame_id_ = this->get_parameter("frame_id").as_string();

    esdf_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/esdf", 1);
    global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_costmap", 1);
    esdf_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/esdf_costmap", 1);

    load_and_compute();

    double rate = this->get_parameter("publish_rate_hz").as_double();
    double period = 1.0 / std::max(rate, 0.1);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&EsdfMapPublisher::on_timer, this)
    );
  }

private:
  void load_and_compute() {
    YAML::Node cfg = YAML::LoadFile(map_yaml_path_);

    if (!cfg["image"] || !cfg["resolution"] || !cfg["origin"]) {
      throw std::runtime_error("map yaml missing required keys: image/resolution/origin");
    }

    std::string image_rel = cfg["image"].as<std::string>();
    double resolution = cfg["resolution"].as<double>();
    YAML::Node origin = cfg["origin"]; // [x,y,yaw]

    int negate = cfg["negate"] ? cfg["negate"].as<int>() : 0;
    double occ_thresh = cfg["occupied_thresh"] ? cfg["occupied_thresh"].as<double>() : 0.65;
    double free_thresh = cfg["free_thresh"] ? cfg["free_thresh"].as<double>() : 0.196;

    std::string yaml_dir = fs::path(map_yaml_path_).parent_path().string();
    std::string image_path = join_path_if_relative(yaml_dir, image_rel);

    cv::Mat pgm = load_pgm_grayscale_u8(image_path);
    const int H = pgm.rows;
    const int W = pgm.cols;

    // Normalize to [0,1], where 0=black, 1=white
    cv::Mat pgm_f;
    pgm.convertTo(pgm_f, CV_32FC1, 1.0 / 255.0);

    // ROS map_server convention:
    // if negate==0: occ_prob = 1 - pixel
    // if negate==1: occ_prob = pixel
    cv::Mat occ_prob = (negate == 0) ? (1.0f - pgm_f) : pgm_f;

    // occupancy grid (image order): -1 unknown, 0 free, 100 occupied
    cv::Mat occ_img(H, W, CV_16SC1, cv::Scalar(-1));
    for (int y = 0; y < H; ++y) {
      const float* row = occ_prob.ptr<float>(y);
      int16_t* out = occ_img.ptr<int16_t>(y);
      for (int x = 0; x < W; ++x) {
        float p = row[x];
        if (p >= occ_thresh) out[x] = 100;
        else if (p <= free_thresh) out[x] = 0;
        else out[x] = -1;
      }
    }

    bool unknown_as_obstacle = this->get_parameter("unknown_as_obstacle").as_bool();

    // Build masks in IMAGE coordinates:
    // obstacle_img: 255 where obstacle (and unknown if treated as obstacle), else 0
    // free_img:     255 where free, else 0
    cv::Mat obstacle_img(H, W, CV_8UC1, cv::Scalar(0));
    cv::Mat free_img(H, W, CV_8UC1, cv::Scalar(0));

    for (int y = 0; y < H; ++y) {
      const int16_t* row = occ_img.ptr<int16_t>(y);
      uint8_t* ob = obstacle_img.ptr<uint8_t>(y);
      uint8_t* fr = free_img.ptr<uint8_t>(y);
      for (int x = 0; x < W; ++x) {
        bool is_occ = (row[x] == 100);
        bool is_unk = (row[x] == -1);
        bool is_obs = is_occ || (unknown_as_obstacle && is_unk);
        bool is_free = (row[x] == 0);

        ob[x] = is_obs ? 255 : 0;
        fr[x] = (!is_obs && is_free) ? 255 : 0; // unknown not free
      }
    }

    // Distance in free space to nearest obstacle:
    // distanceTransform computes distance for foreground pixels to nearest background.
    // So: free=255 (foreground), obstacle=0 (background)
    cv::Mat dist_free_to_obs = distance_transform_meters_from_binary_u8(free_img, static_cast<float>(resolution));

    // Distance inside obstacle/unknown to nearest free:
    // obstacle_space=255 (foreground), free=0 (background)
    cv::Mat obstacle_space(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y) {
      const uint8_t* fr = free_img.ptr<uint8_t>(y);
      uint8_t* os = obstacle_space.ptr<uint8_t>(y);
      for (int x = 0; x < W; ++x) {
        os[x] = (fr[x] == 0) ? 255 : 0; // obstacle/unknown are foreground
      }
    }
    cv::Mat dist_obs_to_free = distance_transform_meters_from_binary_u8(obstacle_space, static_cast<float>(resolution));

    // Store ESDF in GRID order (origin at bottom-left):
    // grid_index = (H-1-y_img)*W + x
    esdf_.assign(H * W, 0.0f);
    for (int y = 0; y < H; ++y) {
      const uint8_t* ob = obstacle_img.ptr<uint8_t>(y);
      const float* dfo = dist_free_to_obs.ptr<float>(y);
      const float* dof = dist_obs_to_free.ptr<float>(y);
      int gy = (H - 1 - y);
      for (int x = 0; x < W; ++x) {
        int idx = gy * W + x;
        esdf_[idx] = (ob[x] != 0) ? -dof[x] : dfo[x];
      }
    }

    // Build /esdf message (Float32MultiArray), data is in GRID order
    std_msgs::msg::Float32MultiArray esdf_msg;
    esdf_msg.layout.dim.resize(2);
    esdf_msg.layout.dim[0].label = "height";
    esdf_msg.layout.dim[0].size = H;
    esdf_msg.layout.dim[0].stride = H * W;
    esdf_msg.layout.dim[1].label = "width";
    esdf_msg.layout.dim[1].size = W;
    esdf_msg.layout.dim[1].stride = W;
    esdf_msg.data.assign(esdf_.begin(), esdf_.end());
    esdf_msg_ = esdf_msg;

    // Build /esdf_costmap as OccupancyGrid
    double robot_radius = this->get_parameter("robot_radius").as_double();
    double margin = this->get_parameter("margin").as_double();
    double d_safe = this->get_parameter("d_safe").as_double();
    double w_pen = this->get_parameter("w").as_double();
    int unknown_cost = this->get_parameter("unknown_cost").as_int();

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = frame_id_;
    grid.info.resolution = static_cast<float>(resolution);
    grid.info.width = W;
    grid.info.height = H;

    grid.info.origin.position.x = origin[0].as<double>();
    grid.info.origin.position.y = origin[1].as<double>();
    double yaw = origin[2].as<double>();
    grid.info.origin.orientation.z = std::sin(yaw / 2.0);
    grid.info.origin.orientation.w = std::cos(yaw / 2.0);

    grid.data.resize(H * W);

    for (int i = 0; i < H * W; ++i) {
      float d = esdf_[i];

      // Original costmap logic (for /global_costmap)
      float clearance = d - static_cast<float>(robot_radius + margin);
      if (clearance < 0.0f) {
        grid.data[i] = 100;  // lethal
      } else {
        if (d >= static_cast<float>(d_safe)) {
          grid.data[i] = 0;
        } else {
          float diff = static_cast<float>(d_safe) - d;
          float penalty = static_cast<float>(w_pen) * diff * diff;
          float scaled = 100.0f * (1.0f - std::exp(-penalty));
          int c = static_cast<int>(std::round(std::min(99.0f, std::max(0.0f, scaled))));
          grid.data[i] = static_cast<int8_t>(c);
        }
      }
    }
    costmap_msg_ = grid;

    // Build /esdf_costmap (distance-based for local planner)
    nav_msgs::msg::OccupancyGrid esdf_grid = grid; 
    for (int i = 0; i < H * W; ++i) {
      float d = esdf_[i];
      // Mapping: 1.0m = 100 in data. 
      int8_t c = static_cast<int8_t>(std::min(127.0f, std::max(-128.0f, d * 100.0f)));
      esdf_grid.data[i] = c;
    }
    esdf_costmap_msg_ = esdf_grid;

    // Visualization with OpenCV
    if (this->get_parameter("visualize_esdf").as_bool()) {
      cv::Mat esdf_vis(H, W, CV_8UC3);
      for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
          // esdf_ is in GRID order (bottom-up), OpenCV image is top-down
          float d = esdf_[(H - 1 - y) * W + x];
          if (d <= 0) {
            esdf_vis.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // Black for occupied
          } else {
            // Mapping distance to 0-255 for visualization
            // d=0 -> 0, d=2.0m -> 255
            uint8_t val = static_cast<uint8_t>(std::min(255.0f, d * 127.0f));
            esdf_vis.at<cv::Vec3b>(y, x) = cv::Vec3b(val, val, val);
          }
        }
      }
      cv::Mat color_map;
      cv::applyColorMap(esdf_vis, color_map, cv::COLORMAP_JET);
      
      // Overlay obstacles in black for clarity
      for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
          if (esdf_[(H - 1 - y) * W + x] <= 0) {
            color_map.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
          }
        }
      }

      cv::imshow("ESDF Visualization", color_map);
      cv::waitKey(1);
    }

    RCLCPP_INFO(this->get_logger(),
      "Loaded map: %s  size=%dx%d  res=%.3f m/cell  origin=[%.3f, %.3f, %.3f]  negate=%d  unknown_as_obstacle=%s",
      image_path.c_str(), W, H, resolution,
      origin[0].as<double>(), origin[1].as<double>(), origin[2].as<double>(),
      negate, unknown_as_obstacle ? "true" : "false"
    );
  }

  void on_timer() {
    // /esdf: Float32MultiArray (no header)
    esdf_pub_->publish(esdf_msg_);

    // /global_costmap: OccupancyGrid
    costmap_msg_.header.stamp = this->now();
    global_costmap_pub_->publish(costmap_msg_);

    // /esdf_costmap: OccupancyGrid (Raw distances)
    esdf_costmap_msg_.header.stamp = this->now();
    esdf_costmap_pub_->publish(esdf_costmap_msg_);
  }

private:
  std::string map_yaml_path_;
  std::string frame_id_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr esdf_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_costmap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<float> esdf_;
  std_msgs::msg::Float32MultiArray esdf_msg_;
  nav_msgs::msg::OccupancyGrid costmap_msg_;
  nav_msgs::msg::OccupancyGrid esdf_costmap_msg_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<EsdfMapPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
