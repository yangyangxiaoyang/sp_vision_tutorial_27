#ifndef SP_DECISION_DECISION_GUI_HPP_
#define SP_DECISION_DECISION_GUI_HPP_

#include <memory>
#include <string>

#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QPushButton>

#include "sp_decision/decision_engine.hpp"

namespace sp_decision
{

/**
 * @class DecisionGui
 * @brief Qt5 main window for sp_decision.
 *
 * Layout
 * ──────
 *  ┌──────────────────────────────────────────────┐
 *  │  行为树列表  (QListWidget)                    │
 *  │  [浏览文件]  [刷新目录]                       │
 *  ├──────────────────────────────────────────────┤
 *  │  当前状态面板                                  │
 *  │    当前树:  <tree name>                       │
 *  │    运行状态: ● 运行中 / ■ 已停止 / tick msg   │
 *  ├──────────────────────────────────────────────┤
 *  │  [▶ 启动]   [⏹ 停止]   [⇄ 切换到选中]       │
 *  └──────────────────────────────────────────────┘
 *
 * Thread safety
 * ─────────────
 *  DecisionEngine::on_status_update is called from the BT background thread.
 *  We relay it to Qt via a signal→slot with Qt::QueuedConnection.
 */
class DecisionGui : public QMainWindow
{
  Q_OBJECT

public:
  explicit DecisionGui(
    std::shared_ptr<DecisionEngine> engine,
    QWidget * parent = nullptr);

  ~DecisionGui() override = default;

signals:
  /**
   * @brief Emitted (thread-safely) when the engine issues a status update.
   *        Connected to update_status_display() with Qt::QueuedConnection.
   */
  void engine_status_signal(const QString & msg);

private slots:
  void on_start_clicked();
  void on_stop_clicked();
  void on_switch_clicked();
  void on_browse_clicked();
  void on_reload_dir_clicked();

  /** @brief Thread-safe slot: update the status panel with a BT tick message. */
  void update_status_display(const QString & msg);

private:
  void build_ui();
  void connect_signals();
  void refresh_tree_list();
  void refresh_status_panel();
  void populate_from_engine();

  // ── Engine ─────────────────────────────────────────────────────────────
  std::shared_ptr<DecisionEngine> engine_;

  // ── Widgets ────────────────────────────────────────────────────────────
  QListWidget  * tree_list_         {nullptr};
  QPushButton  * btn_start_         {nullptr};
  QPushButton  * btn_stop_          {nullptr};
  QPushButton  * btn_switch_        {nullptr};
  QPushButton  * btn_browse_        {nullptr};
  QPushButton  * btn_reload_        {nullptr};
  QLabel       * lbl_current_tree_  {nullptr};
  QLabel       * lbl_bt_status_     {nullptr};

  // ── Directory being watched for XML files ────────────────────────────
  QString tree_dir_;
};

}  // namespace sp_decision

#endif  // SP_DECISION_DECISION_GUI_HPP_
