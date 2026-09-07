#include "sp_decision/decision_gui.hpp"

#include <QApplication>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QVBoxLayout>

namespace sp_decision
{

// ─────────────────────────────────────────────────────────────────────────────

DecisionGui::DecisionGui(
  std::shared_ptr<DecisionEngine> engine,
  QWidget * parent)
: QMainWindow(parent),
  engine_(engine)
{
  setWindowTitle("sp_decision – Behavior Tree Manager");
  setMinimumSize(560, 460);
  setMaximumWidth(720);

  build_ui();
  connect_signals();
  populate_from_engine();
  refresh_status_panel();
}

// ─────────────────────────────────────────────────────────────────────────────

void DecisionGui::build_ui()
{
  auto * central = new QWidget(this);
  setCentralWidget(central);

  auto * root_layout = new QVBoxLayout(central);
  root_layout->setContentsMargins(10, 10, 10, 10);
  root_layout->setSpacing(8);

  // ── 1. 行为树列表 ──────────────────────────────────────────────────────────
  auto * list_box    = new QGroupBox("行为树列表", central);
  auto * list_layout = new QVBoxLayout(list_box);

  tree_list_ = new QListWidget(list_box);
  tree_list_->setAlternatingRowColors(true);
  tree_list_->setToolTip("双击选中后点击 [切换到选中] 按钮激活");
  tree_list_->setMinimumHeight(120);
  // Larger font and row height for easier interaction
  QFont list_font = tree_list_->font();
  list_font.setPointSize(14);
  tree_list_->setFont(list_font);
  tree_list_->setStyleSheet("QListWidget::item { padding: 6px 4px; }");
  list_layout->addWidget(tree_list_);

  auto * file_btn_row = new QHBoxLayout();
  btn_browse_ = new QPushButton("浏览文件", list_box);
  btn_reload_ = new QPushButton("刷新目录", list_box);
  file_btn_row->addWidget(btn_browse_);
  file_btn_row->addWidget(btn_reload_);
  file_btn_row->addStretch();
  list_layout->addLayout(file_btn_row);

  root_layout->addWidget(list_box);

  // ── 2. 当前状态面板 ────────────────────────────────────────────────────────
  auto * status_box    = new QGroupBox("当前状态", central);
  auto * status_layout = new QVBoxLayout(status_box);
  status_layout->setSpacing(10);

  lbl_current_tree_ = new QLabel("当前行为树:  <i>（未选择）</i>", status_box);
  lbl_current_tree_->setStyleSheet("font-size: 13px; padding: 4px;");
  lbl_current_tree_->setWordWrap(true);

  lbl_bt_status_ = new QLabel("运行状态:  <font color='gray'>■ 已停止</font>", status_box);
  lbl_bt_status_->setStyleSheet(
    "font-size: 16px; font-weight: bold; padding: 8px;"
    "border: 1px solid #ccc; border-radius: 4px; background: #f7f7f7;");
  lbl_bt_status_->setAlignment(Qt::AlignCenter);
  lbl_bt_status_->setMinimumHeight(50);

  status_layout->addWidget(lbl_current_tree_);
  status_layout->addWidget(lbl_bt_status_);

  root_layout->addWidget(status_box);

  // ── 3. 控制按钮 ────────────────────────────────────────────────────────────
  auto * ctrl_box    = new QGroupBox("控制", central);
  auto * ctrl_layout = new QHBoxLayout(ctrl_box);
  ctrl_layout->setSpacing(10);

  btn_start_  = new QPushButton("▶  启动",    ctrl_box);
  btn_stop_   = new QPushButton("⏹  停止",    ctrl_box);
  btn_switch_ = new QPushButton("⇄  切换到选中", ctrl_box);

  const int btn_h = 40;
  btn_start_->setMinimumHeight(btn_h);
  btn_stop_->setMinimumHeight(btn_h);
  btn_switch_->setMinimumHeight(btn_h);

  btn_start_->setStyleSheet(
    "QPushButton { background:#27ae60; color:white; font-weight:bold;"
    "              font-size:13px; border-radius:5px; }"
    "QPushButton:hover { background:#2ecc71; }"
    "QPushButton:disabled { background:#95a5a6; }");
  btn_stop_->setStyleSheet(
    "QPushButton { background:#c0392b; color:white; font-weight:bold;"
    "              font-size:13px; border-radius:5px; }"
    "QPushButton:hover { background:#e74c3c; }"
    "QPushButton:disabled { background:#95a5a6; }");
  btn_switch_->setStyleSheet(
    "QPushButton { background:#2980b9; color:white; font-weight:bold;"
    "              font-size:13px; border-radius:5px; }"
    "QPushButton:hover { background:#3498db; }"
    "QPushButton:disabled { background:#95a5a6; }");

  ctrl_layout->addWidget(btn_start_);
  ctrl_layout->addWidget(btn_stop_);
  ctrl_layout->addWidget(btn_switch_);

  root_layout->addWidget(ctrl_box);
}

// ─────────────────────────────────────────────────────────────────────────────

void DecisionGui::connect_signals()
{
  connect(btn_start_,  &QPushButton::clicked, this, &DecisionGui::on_start_clicked);
  connect(btn_stop_,   &QPushButton::clicked, this, &DecisionGui::on_stop_clicked);
  connect(btn_switch_, &QPushButton::clicked, this, &DecisionGui::on_switch_clicked);
  connect(btn_browse_, &QPushButton::clicked, this, &DecisionGui::on_browse_clicked);
  connect(btn_reload_, &QPushButton::clicked, this, &DecisionGui::on_reload_dir_clicked);

  // Thread-safe: BT thread → Qt queued connection → GUI thread
  connect(this, &DecisionGui::engine_status_signal,
          this, &DecisionGui::update_status_display,
          Qt::QueuedConnection);

  // Wire engine callback → signal (called from BT tick thread)
  engine_->on_status_update = [this](const std::string & msg)
  {
    emit engine_status_signal(QString::fromStdString(msg));
  };
}

// ─────────────────────────────────────────────────────────────────────────────

void DecisionGui::populate_from_engine()
{
  const auto & files = engine_->get_tree_files();
  for (const auto & f : files) {
    const QString full = QString::fromStdString(f);
    auto * item = new QListWidgetItem(QFileInfo(full).fileName());
    item->setData(Qt::UserRole, full);  // store full path
    tree_list_->addItem(item);
  }
  if (!files.empty()) {
    tree_list_->setCurrentRow(0);
    QFileInfo fi(QString::fromStdString(files.front()));
    tree_dir_ = fi.absolutePath();
  }
}

// ─────────────────────────────────────────────────────────────────────────────

void DecisionGui::refresh_tree_list()
{
  if (tree_dir_.isEmpty()) { return; }
  QDir dir(tree_dir_);
  QStringList xml_files = dir.entryList({"*.xml"}, QDir::Files, QDir::Name);

  tree_list_->clear();
  std::vector<std::string> paths;
  for (const auto & name : xml_files) {
    const QString full = dir.filePath(name);
    auto * item = new QListWidgetItem(name);  // display filename only
    item->setData(Qt::UserRole, full);         // store full path
    tree_list_->addItem(item);
    paths.push_back(full.toStdString());
  }
  engine_->set_tree_files(paths);
}

// ─────────────────────────────────────────────────────────────────────────────

void DecisionGui::refresh_status_panel()
{
  const bool running = engine_->is_running();
  const std::string tree_file = engine_->current_tree_file();

  // Current tree label
  if (tree_file.empty()) {
    lbl_current_tree_->setText("当前行为树:  <i>（未选择）</i>");
  } else {
    const QString name = QFileInfo(QString::fromStdString(tree_file)).fileName();
    lbl_current_tree_->setText(
      "当前行为树:  <b>" + name + "</b>"
      "<br><font color='gray' style='font-size:11px;'>" +
      QString::fromStdString(tree_file) + "</font>");
  }

  // Running state label
  if (running) {
    lbl_bt_status_->setText("运行状态:  <font color='#27ae60'>● 运行中</font>");
    lbl_bt_status_->setStyleSheet(
      "font-size: 16px; font-weight: bold; padding: 8px;"
      "border: 1px solid #27ae60; border-radius: 4px; background: #eafaf1;");
  } else {
    lbl_bt_status_->setText("运行状态:  <font color='#c0392b'>■ 已停止</font>");
    lbl_bt_status_->setStyleSheet(
      "font-size: 16px; font-weight: bold; padding: 8px;"
      "border: 1px solid #ccc; border-radius: 4px; background: #f7f7f7;");
  }

  // Button states
  btn_start_->setEnabled(!running);
  btn_stop_->setEnabled(running);
  btn_switch_->setEnabled(true);
}

// ─────────────────────────────────────────────────────────────────────────────
// Slots

void DecisionGui::on_start_clicked()
{
  if (engine_->is_running()) { return; }

  std::string target;
  if (auto * item = tree_list_->currentItem()) {
    target = item->data(Qt::UserRole).toString().toStdString();
  }

  if (!engine_->start(target)) {
    QMessageBox::critical(this, "启动失败", "行为树引擎启动失败，请检查控制台输出。");
    return;
  }
  refresh_status_panel();
}

void DecisionGui::on_stop_clicked()
{
  engine_->stop();
  lbl_bt_status_->setText("运行状态:  <font color='#c0392b'>■ 已停止</font>");
  lbl_bt_status_->setStyleSheet(
    "font-size: 16px; font-weight: bold; padding: 8px;"
    "border: 1px solid #ccc; border-radius: 4px; background: #f7f7f7;");
  refresh_status_panel();
}

void DecisionGui::on_switch_clicked()
{
  auto * item = tree_list_->currentItem();
  if (!item) {
    QMessageBox::information(this, "提示", "请先在列表中选择一个行为树文件。");
    return;
  }

  const std::string xml = item->data(Qt::UserRole).toString().toStdString();

  if (!engine_->is_running()) {
    engine_->start(xml);
  } else {
    engine_->switch_tree(xml);
  }
  refresh_status_panel();
}

void DecisionGui::on_browse_clicked()
{
  const QString path = QFileDialog::getOpenFileName(
    this, "选择行为树 XML 文件", tree_dir_,
    "BehaviorTree XML (*.xml);;All files (*)");

  if (path.isEmpty()) { return; }

  bool exists = false;
  for (int i = 0; i < tree_list_->count(); ++i) {
    if (tree_list_->item(i)->data(Qt::UserRole).toString() == path) {
      exists = true;
      tree_list_->setCurrentRow(i);
      break;
    }
  }
  if (!exists) {
    const QFileInfo fi(path);
    auto * new_item = new QListWidgetItem(fi.fileName());  // display filename only
    new_item->setData(Qt::UserRole, path);                  // store full path
    tree_list_->addItem(new_item);
    tree_list_->setCurrentItem(new_item);
    auto files = engine_->get_tree_files();
    files.push_back(path.toStdString());
    engine_->set_tree_files(files);
    tree_dir_ = fi.absolutePath();
  }
}

void DecisionGui::on_reload_dir_clicked()
{
  refresh_tree_list();
}

void DecisionGui::update_status_display(const QString & msg)
{
  // Update the BT status label with the latest tick message
  const bool running = engine_->is_running();

  if (running) {
    lbl_bt_status_->setText(
      "运行状态:  <font color='#27ae60'>● 运行中</font>"
      "&nbsp;&nbsp;<font color='#555' style='font-size:12px;'>" + msg + "</font>");
    lbl_bt_status_->setStyleSheet(
      "font-size: 16px; font-weight: bold; padding: 8px;"
      "border: 1px solid #27ae60; border-radius: 4px; background: #eafaf1;");
  } else {
    lbl_bt_status_->setText(
      "运行状态:  <font color='#c0392b'>■ 已停止</font>");
    lbl_bt_status_->setStyleSheet(
      "font-size: 16px; font-weight: bold; padding: 8px;"
      "border: 1px solid #ccc; border-radius: 4px; background: #f7f7f7;");
  }

  // Also keep current-tree label in sync (tree may have switched)
  const std::string tree_file = engine_->current_tree_file();
  if (!tree_file.empty()) {
    const QString name = QFileInfo(QString::fromStdString(tree_file)).fileName();
    lbl_current_tree_->setText(
      "当前行为树:  <b>" + name + "</b>"
      "<br><font color='gray' style='font-size:11px;'>" +
      QString::fromStdString(tree_file) + "</font>");
  }

  // Sync button states
  btn_start_->setEnabled(!running);
  btn_stop_->setEnabled(running);
}

}  // namespace sp_decision
