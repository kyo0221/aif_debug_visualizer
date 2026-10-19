#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <socketcan_interface_msg/msg/socketcan_if.hpp>
#include <vectornav_msgs/msg/ins_status.hpp>
#include "aif_debug_visualizer/can_decode.hpp"
#include "aif_debug_visualizer/decode_config.hpp"
#include "aif_debug_visualizer/freshness.hpp"
#include "aif_debug_visualizer/turn_chain.hpp"

namespace aif_debug_visualizer {

class VehicleState {
 public:
  using Clock = std::chrono::steady_clock;
  struct Snapshot {
    std::optional<double> cmd_velocity, cmd_steering;
    std::optional<std::array<double, 2>> joy;
    std::optional<bool> autonomous;
    std::optional<DriveRpm> drive_cmd, drive_meas;
    std::optional<std::array<double, 3>> caster;
    std::optional<int16_t> caster_angle_raw;
    std::optional<double> caster_angle, caster_rot;
    std::optional<bool> estop;
    bool estop_active = false;
    std::optional<uint8_t> odrive_axis_state;
    std::optional<FloatPair> odrive_encoder, odrive_bus;
    std::optional<std::array<double, 3>> vbody_linear;
    std::optional<double> yaw_rate_raw;
    std::optional<vectornav_msgs::msg::InsStatus> ins;
    std::array<StageView, TurnChain::kStages> chain;
    std::map<std::string, std::pair<double, Freshness::Level>> freshness;
  };

  static std::shared_ptr<VehicleState> get(rclcpp::Node::SharedPtr node, const std::string & config_path);
  static std::shared_ptr<VehicleState> latest();

  VehicleState(rclcpp::Node::SharedPtr node, DecodeConfig config);
  ~VehicleState();

  const DecodeConfig & config() const { return config_; }
  Snapshot snapshot(Clock::time_point now) const;

 private:
  struct Impl;

  rclcpp::Node::SharedPtr node_;
  DecodeConfig config_;
  std::shared_ptr<Impl> impl_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace aif_debug_visualizer
