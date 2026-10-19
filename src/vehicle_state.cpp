#include "aif_debug_visualizer/vehicle_state.hpp"
#include <filesystem>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <socketcan_interface_msg/msg/socketcan_if.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <steered_drive_msg/msg/steered_drive.hpp>
#include <vectornav_msgs/msg/common_group.hpp>

namespace aif_debug_visualizer {

namespace {
std::mutex registry_mutex;
std::map<std::string, std::weak_ptr<VehicleState>> registry;
std::weak_ptr<VehicleState> latest_state;
using socketcan_interface_msg::msg::SocketcanIF;

Frame toFrame(const SocketcanIF & m) {
  Frame f{};
  std::copy(m.candata.begin(), m.candata.end(), f.begin());
  return f;
}
}  // namespace

std::shared_ptr<VehicleState> VehicleState::get(rclcpp::Node::SharedPtr node, const std::string & config_path) {
  std::lock_guard<std::mutex> lock(registry_mutex);
  const std::string key = std::filesystem::absolute(config_path).string();
  auto it = registry.find(key);
  if (it != registry.end()) {
    if (auto alive = it->second.lock()) return alive;
  }
  auto created = std::make_shared<VehicleState>(node, DecodeConfig::load(config_path));
  registry[key] = created;
  latest_state = created;
  return created;
}

std::shared_ptr<VehicleState> VehicleState::latest() {
  std::lock_guard<std::mutex> lock(registry_mutex);
  return latest_state.lock();
}

template <typename Msg, typename Fn>
void VehicleState::subscribe(const std::string & topic, Fn fn) {
  subs_.push_back(node_->create_subscription<Msg>(topic, rclcpp::SensorDataQoS(), fn));
}

VehicleState::VehicleState(rclcpp::Node::SharedPtr node, DecodeConfig config)
    : node_(std::move(node)), config_(std::move(config)), chain_(config_.chain), caster_rot_calc_(config_.caster_rot) {
  for (const auto & [key, hz] : config_.expected_hz) {
    fresh_.emplace(key, Freshness(hz, config_.stale_sec));
  }
  const auto & t = config_.topics;

  subscribe<steered_drive_msg::msg::SteeredDrive>(t.at("cmd_vel"), [this](steered_drive_msg::msg::SteeredDrive::ConstSharedPtr m) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = Clock::now();
    s_.cmd_velocity = m->velocity;
    s_.cmd_steering = m->steering_angle;
    fresh_.at("cmd_vel").tick(now);
    chain_.setCommand(m->velocity, config_.steering_cmd_sign * m->steering_angle, now);
  });

  subscribe<sensor_msgs::msg::Joy>(t.at("joy"), [this](sensor_msgs::msg::Joy::ConstSharedPtr m) {
    if (m->axes.size() < 3) return;
    std::lock_guard<std::mutex> lock(mutex_);
    s_.joy = {m->axes[1], m->axes[2]};
    fresh_.at("joy").tick(Clock::now());
  });

  subscribe<std_msgs::msg::Bool>(t.at("autonomous"), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_.autonomous = m->data;
  });

  subscribe<SocketcanIF>(t.at("can_tx"), [this](SocketcanIF::ConstSharedPtr m) {
    if (m->canid != config_.drive_cmd.id) return;
    auto d = decodeDrive(toFrame(*m), m->candlc, config_.drive_cmd);
    if (!d) return;
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = Clock::now();
    s_.drive_cmd = d;
    fresh_.at("drive_cmd").tick(now);
    chain_.push(1, d->right - d->left, now);
  });

  const uint32_t ids[] = {
    config_.drive_meas.id, config_.caster_angle.id, config_.caster_rot.id, config_.estop.id,
    odriveCanId(config_.odrive_node_id, 0x01), odriveCanId(config_.odrive_node_id, 0x09),
    odriveCanId(config_.odrive_node_id, 0x17)};
  for (uint32_t id : ids) {
    subscribe<SocketcanIF>(config_.canRxTopic(id), [this, id](SocketcanIF::ConstSharedPtr m) { onCan(*m, id); });
  }

  subscribe<std_msgs::msg::Float64MultiArray>(t.at("caster_data"), [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) {
    if (m->data.size() < 3) return;
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = Clock::now();
    s_.caster = {m->data[0], m->data[1], m->data[2]};
    fresh_.at("caster_data").tick(now);
    chain_.push(3, config_.caster_meas_sign * m->data[1], now);
  });

  subscribe<geometry_msgs::msg::TwistWithCovarianceStamped>(t.at("velocity_body"), [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = Clock::now();
    const auto & tw = m->twist.twist;
    s_.vbody_linear = {tw.linear.x, tw.linear.y, tw.linear.z};
    s_.yaw_rate_raw = tw.angular.z;
    fresh_.at("velocity_body").tick(now);
    chain_.push(4, config_.yaw_rate_sign * tw.angular.z, now);
  });

  subscribe<vectornav_msgs::msg::CommonGroup>(t.at("ins_status"), [this](vectornav_msgs::msg::CommonGroup::ConstSharedPtr m) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_.ins = m->insstatus;
    fresh_.at("ins_status").tick(Clock::now());
  });
}

void VehicleState::onCan(const SocketcanIF & m, uint32_t id) {
  const Frame f = toFrame(m);
  const uint8_t dlc = m.candlc;
  std::lock_guard<std::mutex> lock(mutex_);
  auto now = Clock::now();
  if (id == config_.drive_meas.id) {
    if (auto d = decodeDrive(f, dlc, config_.drive_meas)) {
      s_.drive_meas = d;
      fresh_.at("drive_meas").tick(now);
      chain_.push(2, d->right - d->left, now);
    }
  } else if (id == config_.caster_angle.id) {
    if (auto a = decodeCasterAngle(f, dlc, config_.caster_angle)) {
      s_.caster_angle = a;
      s_.caster_angle_raw = readInt16(f, config_.caster_angle.offset);
      fresh_.at("caster_angle").tick(now);
    }
  } else if (id == config_.caster_rot.id) {
    if (auto r = caster_rot_calc_.update(f, dlc)) s_.caster_rot = r;
  } else if (id == config_.estop.id) {
    if (auto e = decodeEstop(f, dlc, config_.estop)) {
      s_.estop = e;
      chain_.setEstop(*e);
    }
  } else if (id == odriveCanId(config_.odrive_node_id, 0x01)) {
    if (auto v = decodeOdriveAxisState(f, dlc)) s_.odrive_axis_state = v;
  } else if (id == odriveCanId(config_.odrive_node_id, 0x09)) {
    if (auto v = decodeFloatPair(f, dlc)) s_.odrive_encoder = v;
  } else if (id == odriveCanId(config_.odrive_node_id, 0x17)) {
    if (auto v = decodeFloatPair(f, dlc)) s_.odrive_bus = v;
  }
}

VehicleState::Snapshot VehicleState::snapshot(Clock::time_point now) const {
  std::lock_guard<std::mutex> lock(mutex_);
  Snapshot s = s_;
  auto stale = [&](const char * key) { return fresh_.at(key).level(now) == Freshness::Level::Stale; };
  if (stale("cmd_vel")) { s.cmd_velocity.reset(); s.cmd_steering.reset(); }
  if (stale("joy")) s.joy.reset();
  if (stale("drive_cmd")) s.drive_cmd.reset();
  if (stale("drive_meas")) s.drive_meas.reset();
  if (stale("caster_data")) s.caster.reset();
  if (stale("caster_angle")) { s.caster_angle_raw.reset(); s.caster_angle.reset(); }
  if (stale("velocity_body")) { s.vbody_linear.reset(); s.yaw_rate_raw.reset(); }
  if (stale("ins_status")) s.ins.reset();
  s.chain = chain_.view(now);
  for (const auto & [key, f] : fresh_) s.freshness[key] = {f.hz(now), f.level(now)};
  return s;
}

}  // namespace aif_debug_visualizer
