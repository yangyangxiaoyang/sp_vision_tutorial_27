#include <atomic>
#include <ctime>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QFont>
#include <QFontDatabase>

#include "rclcpp/rclcpp.hpp"
#include "sp_decision/blackboard_bridge.hpp"
#include "sp_decision/decision_engine.hpp"
#include "sp_decision/decision_gui.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Small node that owns the DecisionEngine and its ROS parameters.

class DecisionNode : public rclcpp::Node
{
public:
    DecisionNode()
        : rclcpp::Node("sp_decision")
    {
        // Declare parameters
        this->declare_parameter<std::vector<std::string>>(
            "tree_files", std::vector<std::string>{});

        this->declare_parameter<std::vector<std::string>>(
            "bt_plugins", std::vector<std::string>{});

        this->declare_parameter<double>("tick_rate_hz", 20.0);

        this->declare_parameter<bool>("cout_logger", true);
        this->declare_parameter<bool>("file_logger", false);
        this->declare_parameter<bool>("zmq_publisher", false);
        this->declare_parameter<std::string>("log_file", "bt_trace.fbl");
    }
};

// ─────────────────────────────────────────────────────────────────────────────

static void setup_chinese_font(QApplication & app)
{
  static const char * candidates[] = {
    "Noto Sans CJK SC",
    "Noto Sans SC",
    "WenQuanYi Micro Hei",
    "WenQuanYi Zen Hei",
    "Source Han Sans SC",
    "Microsoft YaHei",
    "PingFang SC",
    "SimHei",
    nullptr,
  };

  const QStringList families = QFontDatabase().families();
  QFont font = app.font();
  for (int i = 0; candidates[i] != nullptr; ++i) {
    if (families.contains(QString::fromUtf8(candidates[i]))) {
      font.setFamily(QString::fromUtf8(candidates[i]));
      app.setFont(font);
      return;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char *argv[])
{
    // ── Qt must own the main thread ─────────────────────────────────────────
    QApplication app(argc, argv);
    app.setApplicationName("sp_decision");
    app.setOrganizationName("SentinelProject");
    setup_chinese_font(app);

    // ── ROS2 init ────────────────────────────────────────────────────────────
    rclcpp::init(argc, argv);

    auto decision_node = std::make_shared<DecisionNode>();

    // Read parameters
    const auto tree_files =
        decision_node->get_parameter("tree_files").as_string_array();
    const auto bt_plugins =
        decision_node->get_parameter("bt_plugins").as_string_array();
    const double tick_hz =
        decision_node->get_parameter("tick_rate_hz").as_double();
    const bool cout_log =
        decision_node->get_parameter("cout_logger").as_bool();
    const bool file_log =
        decision_node->get_parameter("file_logger").as_bool();
    const bool zmq_pub =
        decision_node->get_parameter("zmq_publisher").as_bool();
    const std::string log_file_base =
        decision_node->get_parameter("log_file").as_string();

    // Append launch timestamp to the log filename: base_YYYYMMDD_HHMMSS.ext
    const std::string log_file = [&]() -> std::string {
        std::time_t t = std::time(nullptr);
        char ts[20];
        std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&t));
        const auto dot = log_file_base.rfind('.');
        if (dot == std::string::npos) {
            return log_file_base + "_" + ts;
        }
        return log_file_base.substr(0, dot) + "_" + ts + log_file_base.substr(dot);
    }();

    // ── Decision engine ──────────────────────────────────────────────────────
    const auto tick_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / std::max(tick_hz, 1.0)));

    auto engine = std::make_shared<sp_decision::DecisionEngine>(
        decision_node, tick_period);

    engine->set_plugin_libraries(bt_plugins);
    engine->set_tree_files(tree_files);
    engine->set_cout_logger(cout_log);
    engine->set_file_logger(file_log, log_file);
    engine->set_zmq_publisher(zmq_pub);

    // ── Blackboard bridge node ───────────────────────────────────────────────
    // Created *after* the engine so we can pass its blackboard.
    // The blackboard is only valid after engine->start(); the bridge tolerates
    // a null blackboard gracefully (write() checks for nullptr).
    auto bridge_node = std::make_shared<sp_decision::BlackboardBridge>(
        nullptr /* blackboard attached after start */);

    // Whenever the engine starts / switches trees, re-bind the blackboard to
    // the bridge so it always writes into the currently active tree's blackboard.
    engine->on_tree_initialized = [&bridge_node, &engine]()
    {
        auto bb = engine->get_blackboard();
        if (bb)
        {
            bridge_node->set_blackboard(bb);
        }
    };

    // ── ROS2 executor in a background thread ──────────────────────────────────
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(decision_node);
    executor.add_node(bridge_node);

    std::atomic<bool> ros_ok{true};
    std::thread ros_thread([&executor, &ros_ok]()
                           {
    executor.spin();
    ros_ok.store(false); });

    // ── Qt GUI (main thread) ──────────────────────────────────────────────────
    sp_decision::DecisionGui gui(engine);
    gui.show();

    // Auto-start: begin running the first tree as soon as the GUI is ready.
    if (!tree_files.empty()) {
        engine->start(tree_files.front());
    }

    const int qt_ret = app.exec(); // blocks until window is closed

    // ── Cleanup ───────────────────────────────────────────────────────────────
    engine->stop();
    executor.cancel();
    rclcpp::shutdown();
    if (ros_thread.joinable())
    {
        ros_thread.join();
    }

    return qt_ret;
}
