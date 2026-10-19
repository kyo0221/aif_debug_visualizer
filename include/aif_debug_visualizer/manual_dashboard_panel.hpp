#pragma once
#include <QEvent>
#include <QWindow>
#include <map>
#include <memory>
#include <string>
#include <QLabel>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include "aif_debug_visualizer/vehicle_state.hpp"

namespace aif_debug_visualizer {

class ManualDashboardPanel : public rviz_common::Panel {
  Q_OBJECT

 public:
  explicit ManualDashboardPanel(QWidget * parent = nullptr);
  void onInitialize() override;

 private:
  void refresh();
  void set(const std::string & key, const std::string & text, const char * color);

  std::map<std::string, QLabel *> values_;
  std::shared_ptr<VehicleState> state_;
  rclcpp::Node::SharedPtr node_;
  QTimer * timer_;
};

}  // namespace aif_debug_visualizer
