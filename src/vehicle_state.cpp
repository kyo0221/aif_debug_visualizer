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
std::vector<std::weak_ptr<VehicleState>> creation_order;
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
  creation_order.push_back(created);
  return created;
}

std::shared_ptr<VehicleState> VehicleState::latest() {
  std::lock_guard<std::mutex> lock(registry_mutex);
  for (auto it = creation_order.rbegin(); it != creation_order.rend(); ++it) {
    if (auto alive = it->lock()) return alive;
  }
  return nullptr;
}

struct VehicleState::Impl {
  explicit Impl(const DecodeConfig & c)
      : config(c), chain(c.chain), caster_rot_calc(c.caster_rot) {
    for (const auto & [key, hz] : config.expected_hz) {
      fresh.emplace(key, Freshness(hz, config.stale_sec));
    }
  }

  DecodeConfig config;
  mutable std::mutex mutex;
  std::map<std::string, Freshness> fresh;
  TurnChain chain;
  CasterRotation caster_rot_calc;
  Snapshot s;
  std::map<std::string, Clock::time_point> last_rx;

  void onCmdVel(const steered_drive_msg::msg::SteeredDrive & m) {
    std::lock_guard<std::mutex> lock(mutex);
    auto now = Clock::now();
    s.cmd_velocity = m.velocity;
    s.cmd_steering = m.steering_angle;
    fresh.at("cmd_vel").tick(now);
    chain.setCommand(m.velocity, config.steering_cmd_sign * m.steering_angle, now);
  }

  void onJoy(const sensor_msgs::msg::Joy & m) {
    if (m.axes.size() < 3) return;
    std::lock_guard<std::mutex> lock(mutex);
    s.joy = {m.axes[1], m.axes[2]};
    fresh.at("joy").tick(Clock::now());
  }

  void onAutonomous(const std_msgs::msg::Bool & m) {
    std::lock_guard<std::mutex> lock(mutex);
    s.autonomous = m.data;
  }

  void onCanTx(const SocketcanIF & m) {
    if (m.canid != config.drive_cmd.id) return;
    auto d = decodeDrive(toFrame(m), m.candlc, config.drive_cmd);
    if (!d) return;
    std::lock_guard<std::mutex> lock(mutex);
    auto now = Clock::now();
    s.drive_cmd = d;
    fresh.at("drive_cmd").tick(now);
    chain.push(1, d->right - d->left, now);
  }

  void onCasterData(const std_msgs::msg::Float64MultiArray & m) {
    if (m.data.size() < 3) return;
    std::lock_guard<std::mutex> lock(mutex);
    auto now = Clock::now();
    s.caster = {m.data[0], m.data[1], m.data[2]};
    fresh.at("caster_data").tick(now);
    chain.push(3, config.caster_meas_sign * m.data[1], now);
  }

  void onVelocityBody(const geometry_msgs::msg::TwistWithCovarianceStamped & m) {
    std::lock_guard<std::mutex> lock(mutex);
    auto now = Clock::now();
    const auto & tw = m.twist.twist;
    s.vbody_linear = {tw.linear.x, tw.linear.y, tw.linear.z};
    s.yaw_rate_raw = tw.angular.z;
    fresh.at("velocity_body").tick(now);
    chain.push(4, config.yaw_rate_sign * tw.angular.z, now);
  }

  void onIns(const vectornav_msgs::msg::CommonGroup & m) {
    std::lock_guard<std::mutex> lock(mutex);
    s.ins = m.insstatus;
    fresh.at("ins_status").tick(Clock::now());
  }

  void onCan(const SocketcanIF & m, uint32_t id) {
    const Frame f = toFrame(m);
    const uint8_t dlc = m.candlc;
    std::lock_guard<std::mutex> lock(mutex);
    auto now = Clock::now();
    if (id == config.drive_meas.id) {
      if (auto d = decodeDrive(f, dlc, config.drive_meas)) {
        s.drive_meas = d;
        fresh.at("drive_meas").tick(now);
        chain.push(2, d->right - d->left, now);
      }
    } else if (id == config.caster_angle.id) {
      if (auto a = decodeCasterAngle(f, dlc, config.caster_angle)) {
        s.caster_angle = a;
        s.caster_angle_raw = readInt16(f, config.caster_angle.offset);
        fresh.at("caster_angle").tick(now);
      }
    } else if (id == config.caster_rot.id) {
      if (auto r = caster_rot_calc.update(f, dlc)) { s.caster_rot = r; last_rx["caster_rot"] = now; }
    } else if (id == config.estop.id) {
      if (auto e = decodeEstop(f, dlc, config.estop)) {
        s.estop = e;
        last_rx["estop"] = now;
        chain.setEstop(*e);
      }
    } else if (id == odriveCanId(config.odrive_node_id, 0x01)) {
      if (auto v = decodeOdriveAxisState(f, dlc)) { s.odrive_axis_state = v; last_rx["odrive_axis_state"] = now; }
    } else if (id == odriveCanId(config.odrive_node_id, 0x09)) {
      if (auto v = decodeFloatPair(f, dlc)) { s.odrive_encoder = v; last_rx["odrive_encoder"] = now; }
    } else if (id == odriveCanId(config.odrive_node_id, 0x17)) {
      if (auto v = decodeFloatPair(f, dlc)) { s.odrive_bus = v; last_rx["odrive_bus"] = now; }
    }
  }

  Snapshot snapshot(Clock::time_point now) const {
    std::lock_guard<std::mutex> lock(mutex);
    Snapshot out = s;
    auto stale = [&](const char * key) { return fresh.at(key).level(now) == Freshness::Level::Stale; };
    if (stale("cmd_vel")) { out.cmd_velocity.reset(); out.cmd_steering.reset(); }
    if (stale("joy")) out.joy.reset();
    if (stale("drive_cmd")) out.drive_cmd.reset();
    if (stale("drive_meas")) out.drive_meas.reset();
    if (stale("caster_data")) out.caster.reset();
    if (stale("caster_angle")) { out.caster_angle_raw.reset(); out.caster_angle.reset(); }
    if (stale("velocity_body")) { out.vbody_linear.reset(); out.yaw_rate_raw.reset(); }
    if (stale("ins_status")) out.ins.reset();
    auto old = [&](const char * key) {
      auto it = last_rx.find(key);
      return it == last_rx.end() || std::chrono::duration<double>(now - it->second).count() > config.stale_sec;
    };
    if (old("estop")) out.estop.reset();
    if (old("odrive_axis_state")) out.odrive_axis_state.reset();
    if (old("odrive_encoder")) out.odrive_encoder.reset();
    if (old("odrive_bus")) out.odrive_bus.reset();
    if (old("caster_rot")) out.caster_rot.reset();
    out.estop_active = out.estop.value_or(false);
    if (stale("cmd_vel")) {
      out.chain.fill({Dir::None, Color::Off, 0});
    } else {
      TurnChain c = chain;
      c.setEstop(out.estop_active);
      out.chain = c.view(now);
    }
    for (const auto & [key, f] : fresh) out.freshness[key] = {f.hz(now), f.level(now)};
    return out;
  }
};

VehicleState::VehicleState(rclcpp::Node::SharedPtr node, DecodeConfig config)
    : node_(std::move(node)), config_(std::move(config)), impl_(std::make_shared<Impl>(config_)) {
  const auto & t = config_.topics;
  std::weak_ptr<Impl> weak = impl_;
  auto qos = rclcpp::SensorDataQoS();
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = group_;

  subs_.push_back(node_->create_subscription<steered_drive_msg::msg::SteeredDrive>(
    t.at("cmd_vel"), qos, [weak](steered_drive_msg::msg::SteeredDrive::ConstSharedPtr m) {
      if (auto i = weak.lock()) i->onCmdVel(*m);
    }, opts));
  subs_.push_back(node_->create_subscription<sensor_msgs::msg::Joy>(
    t.at("joy"), qos, [weak](sensor_msgs::msg::Joy::ConstSharedPtr m) {
      if (auto i = weak.lock()) i->onJoy(*m);
    }, opts));
  subs_.push_back(node_->create_subscription<std_msgs::msg::Bool>(
    t.at("autonomous"), qos, [weak](std_msgs::msg::Bool::ConstSharedPtr m) {
      if (auto i = weak.lock()) i->onAutonomous(*m);
    }, opts));
  subs_.push_back(node_->create_subscription<SocketcanIF>(
    t.at("can_tx"), qos, [weak](SocketcanIF::ConstSharedPtr m) {
      if (auto i = weak.lock()) i->onCanTx(*m);
    }, opts));

  const uint32_t ids[] = {
    config_.drive_meas.id, config_.caster_angle.id, config_.caster_rot.id, config_.estop.id,
    odriveCanId(config_.odrive_node_id, 0x01), odriveCanId(config_.odrive_node_id, 0x09),
    odriveCanId(config_.odrive_node_id, 0x17)};
  for (uint32_t id : ids) {
    subs_.push_back(node_->create_subscription<SocketcanIF>(
      config_.canRxTopic(id), qos, [weak, id](SocketcanIF::ConstSharedPtr m) {
        if (auto i = weak.lock()) i->onCan(*m, id);
      }, opts));
  }

  subs_.push_back(node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    t.at("caster_data"), qos, [weak](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) {
      if (auto i = weak.lock()) i->onCasterData(*m);
    }, opts));
  subs_.push_back(node_->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    t.at("velocity_body"), qos, [weak](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m) {
      if (auto i = weak.lock()) i->onVelocityBody(*m);
    }, opts));
  subs_.push_back(node_->create_subscription<vectornav_msgs::msg::CommonGroup>(
    t.at("ins_status"), qos, [weak](vectornav_msgs::msg::CommonGroup::ConstSharedPtr m) {
      if (auto i = weak.lock()) i->onIns(*m);
    }, opts));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(group_, node_->get_node_base_interface());
  thread_ = std::thread([this]() {
    auto context = node_->get_node_base_interface()->get_context();
    while (!stop_ && rclcpp::ok(context)) {
      try {
        executor_->spin_once(std::chrono::milliseconds(50));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "dashboard executor stopped: %s", e.what());
        break;
      }
    }
  });
}

VehicleState::~VehicleState() {
  stop_ = true;
  if (thread_.joinable()) thread_.join();
}

VehicleState::Snapshot VehicleState::snapshot(Clock::time_point now) const {
  return impl_->snapshot(now);
}

}  // namespace aif_debug_visualizer
