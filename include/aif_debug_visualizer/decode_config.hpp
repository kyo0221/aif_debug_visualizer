#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace aif_debug_visualizer
{

struct FieldSpec { int offset; std::string type; };
struct DriveSpec { uint32_t id; FieldSpec right; FieldSpec left; };
struct CasterAngleSpec { uint32_t id; int offset; int counts_per_rev; double zero_offset_deg; };
struct CasterRotSpec { uint32_t id; int offset; int counts_per_rev; double gear_ratio; int sign; };
struct EstopSpec { uint32_t id; int byte; };
struct ChainSpec
{
  double min_speed, min_steer_deg, window_sec, good_ratio, bad_ratio,
    deadband_rpm_diff, deadband_caster_deg, deadband_yaw_rate;
};

struct DecodeConfig
{
  std::map<std::string, std::string> topics;
  double wheel_radius, linear_max_vel, steering_max_deg;
  DriveSpec drive_cmd, drive_meas;
  CasterAngleSpec caster_angle;
  CasterRotSpec caster_rot;
  EstopSpec estop;
  uint32_t odrive_node_id;
  int yaw_rate_sign, steering_cmd_sign, caster_meas_sign;
  ChainSpec chain;
  double stale_sec;
  std::map<std::string, double> expected_hz;

  static DecodeConfig load(const std::string & path);
  static std::string defaultPath();
  std::string canRxTopic(uint32_t id) const;
};

}  // namespace aif_debug_visualizer
