#include "aif_debug_visualizer/decode_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace aif_debug_visualizer
{

namespace
{

YAML::Node node(const YAML::Node & parent, const std::string & key)
{
  YAML::Node n = parent[key];
  if (!n) {
    throw std::runtime_error("missing key: " + key);
  }
  return n;
}

template<class T>
T req(const YAML::Node & parent, const std::string & key)
{
  return node(parent, key).as<T>();
}

FieldSpec field(const YAML::Node & n)
{
  return {req<int>(n, "offset"), req<std::string>(n, "type")};
}

DriveSpec drive(const YAML::Node & n)
{
  return {req<uint32_t>(n, "id"), field(node(n, "right")), field(node(n, "left"))};
}

}  // namespace

DecodeConfig DecodeConfig::load(const std::string & path)
{
  YAML::Node root = YAML::LoadFile(path);
  DecodeConfig c;

  for (const auto & kv : node(root, "topics")) {
    c.topics[kv.first.as<std::string>()] = kv.second.as<std::string>();
  }
  for (const char * k : {"cmd_vel", "joy", "autonomous", "can_tx", "can_rx_prefix",
      "caster_data", "velocity_body", "ins_status"})
  {
    if (!c.topics.count(k)) {
      throw std::runtime_error(std::string("missing key: ") + k);
    }
  }

  YAML::Node v = node(root, "vehicle");
  c.wheel_radius = req<double>(v, "wheel_radius");
  c.linear_max_vel = req<double>(v, "linear_max_vel");

  YAML::Node can = node(root, "can");
  c.drive_cmd = drive(node(can, "drive_cmd"));
  c.drive_meas = drive(node(can, "drive_meas"));
  YAML::Node ca = node(can, "caster_angle");
  c.caster_angle = {
    req<uint32_t>(ca, "id"), req<int>(ca, "offset"), req<int>(ca, "counts_per_rev"),
    req<double>(ca, "zero_offset_deg")};
  YAML::Node cr = node(can, "caster_rot");
  c.caster_rot = {
    req<uint32_t>(cr, "id"), req<int>(cr, "offset"), req<int>(cr, "counts_per_rev"),
    req<double>(cr, "gear_ratio"), req<int>(cr, "sign")};
  YAML::Node es = node(can, "estop");
  c.estop = {req<uint32_t>(es, "id"), req<int>(es, "byte")};
  c.odrive_node_id = req<uint32_t>(node(can, "odrive"), "node_id");

  YAML::Node s = node(root, "signs");
  c.yaw_rate_sign = req<int>(s, "yaw_rate");
  c.steering_cmd_sign = req<int>(s, "steering_cmd");
  c.caster_meas_sign = req<int>(s, "caster_meas");

  YAML::Node ch = node(root, "chain");
  YAML::Node db = node(ch, "deadband");
  c.chain = {
    req<double>(ch, "min_speed"), req<double>(ch, "min_steer_deg"), req<double>(ch, "window_sec"),
    req<double>(ch, "good_ratio"), req<double>(ch, "bad_ratio"),
    req<double>(db, "rpm_diff"), req<double>(db, "caster_deg"), req<double>(db, "yaw_rate")};

  YAML::Node f = node(root, "freshness");
  c.stale_sec = req<double>(f, "stale_sec");
  for (const auto & kv : node(f, "expected_hz")) {
    c.expected_hz[kv.first.as<std::string>()] = kv.second.as<double>();
  }
  for (const char * k : {"cmd_vel", "joy", "drive_cmd", "drive_meas", "caster_data",
      "caster_angle", "velocity_body", "ins_status"})
  {
    if (!c.expected_hz.count(k)) {
      throw std::runtime_error(std::string("missing key: freshness.expected_hz.") + k);
    }
  }
  return c;
}

std::string DecodeConfig::defaultPath()
{
  const char * env = std::getenv("AIF_DEBUG_DECODE_CONFIG");
  if (env && *env) {
    return env;
  }
  return ament_index_cpp::get_package_share_directory("aif_debug_visualizer") +
         "/config/vehicle_decode.yaml";
}

std::string DecodeConfig::canRxTopic(uint32_t id) const
{
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%03X", id);
  return topics.at("can_rx_prefix") + buf;
}

}  // namespace aif_debug_visualizer
