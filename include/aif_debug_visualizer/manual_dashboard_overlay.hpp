#pragma once
#include <QEvent>
#include <QImage>
#include <QWindow>
#include <chrono>
#include <memory>
#include <string>
#include <OgreMaterial.h>
#include <OgreTexture.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgrePanelOverlayElement.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include "aif_debug_visualizer/format.hpp"
#include "aif_debug_visualizer/vehicle_state.hpp"

namespace aif_debug_visualizer {

class ManualDashboardOverlay : public rviz_common::Display {
  Q_OBJECT

 public:
  ManualDashboardOverlay();
  ~ManualDashboardOverlay() override;

 protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

 private Q_SLOTS:
  void reconnect();

 private:
  void ensureTexture(int w, int h);
  void destroyTexture();
  void draw(const VehicleState::Snapshot & s, int w, int h);

  rclcpp::Node::SharedPtr node_;
  rviz_common::properties::StringProperty * config_;
  rviz_common::properties::FloatProperty * scale_;
  rviz_common::properties::BoolProperty * show_speed_;
  rviz_common::properties::BoolProperty * show_steer_;
  rviz_common::properties::BoolProperty * show_drive_;
  rviz_common::properties::BoolProperty * show_chain_;
  rviz_common::properties::BoolProperty * show_status_;
  std::shared_ptr<VehicleState> state_;
  std::string name_;
  std::string shown_;
  std::chrono::steady_clock::time_point last_;
  int tex_w_ = 0;
  int tex_h_ = 0;
  QImage image_;
  Ogre::Overlay * overlay_ = nullptr;
  Ogre::PanelOverlayElement * panel_ = nullptr;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
};

}
