#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>

#include "aif_debug_visualizer/decode_config.hpp"

namespace aif_debug_visualizer
{

using Frame = std::array<uint8_t, 8>;

inline int16_t readInt16(const Frame & f, int offset)
{
  uint16_t v = static_cast<uint16_t>(f[offset] | (f[offset + 1] << 8));
  return static_cast<int16_t>(v);
}

inline int32_t readInt32(const Frame & f, int offset)
{
  uint32_t v = 0;
  for (int i = 0; i < 4; ++i) {
    v |= static_cast<uint32_t>(f[offset + i]) << (8 * i);
  }
  return static_cast<int32_t>(v);
}

inline float readFloat32(const Frame & f, int offset)
{
  uint32_t v = static_cast<uint32_t>(readInt32(f, offset));
  float out;
  std::memcpy(&out, &v, sizeof(out));
  return out;
}

inline std::optional<double> readField(const Frame & f, uint8_t dlc, const FieldSpec & s)
{
  int size;
  if (s.type == "int16") {
    size = 2;
  } else if (s.type == "int32" || s.type == "float32") {
    size = 4;
  } else {
    return std::nullopt;
  }
  if (s.offset < 0 || s.offset + size > dlc || s.offset + size > 8) {
    return std::nullopt;
  }
  if (s.type == "int16") {
    return readInt16(f, s.offset);
  }
  if (s.type == "int32") {
    return readInt32(f, s.offset);
  }
  return readFloat32(f, s.offset);
}

struct DriveRpm { double right; double left; };

inline std::optional<DriveRpm> decodeDrive(const Frame & f, uint8_t dlc, const DriveSpec & s)
{
  auto r = readField(f, dlc, s.right);
  auto l = readField(f, dlc, s.left);
  if (!r || !l) {
    return std::nullopt;
  }
  return DriveRpm{*r, *l};
}

inline double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

inline std::optional<double> decodeCasterAngle(
  const Frame & f, uint8_t dlc, const CasterAngleSpec & s)
{
  if (s.offset < 0 || s.offset + 2 > dlc) {
    return std::nullopt;
  }
  double count = readInt16(f, s.offset);
  double zero = s.zero_offset_deg * M_PI / 180.0;
  return normalizeAngle(count / s.counts_per_rev * 2.0 * M_PI - zero);
}

class CasterRotation
{
public:
  explicit CasterRotation(const CasterRotSpec & s) : spec_(s) {}

  std::optional<double> update(const Frame & f, uint8_t dlc)
  {
    if (spec_.offset < 0 || spec_.offset + 2 > dlc) {
      return std::nullopt;
    }
    int count = readInt16(f, spec_.offset);
    int gap = initialized_ ? count - prev_ : 0;
    if (gap > spec_.counts_per_rev / 2) {
      gap -= spec_.counts_per_rev;
    } else if (gap < -spec_.counts_per_rev / 2) {
      gap += spec_.counts_per_rev;
    }
    total_ += gap;
    prev_ = count;
    initialized_ = true;
    return spec_.sign * static_cast<double>(total_) / spec_.counts_per_rev * 2.0 * M_PI *
           spec_.gear_ratio;
  }

private:
  CasterRotSpec spec_;
  bool initialized_ = false;
  int prev_ = 0;
  long total_ = 0;
};

inline std::optional<bool> decodeEstop(const Frame & f, uint8_t dlc, const EstopSpec & s)
{
  if (s.byte < 0 || s.byte >= dlc || s.byte >= 8) {
    return std::nullopt;
  }
  return f[s.byte] != 0;
}

inline std::optional<uint8_t> decodeOdriveAxisState(const Frame & f, uint8_t dlc)
{
  if (dlc < 5) {
    return std::nullopt;
  }
  return f[4];
}

struct FloatPair { float first; float second; };

inline std::optional<FloatPair> decodeFloatPair(const Frame & f, uint8_t dlc)
{
  if (dlc != 8) {
    return std::nullopt;
  }
  return FloatPair{readFloat32(f, 0), readFloat32(f, 4)};
}

constexpr uint32_t odriveCanId(uint32_t node_id, uint32_t cmd) { return node_id << 5 | cmd; }

inline double rpmToMps(double rpm, double wheel_radius)
{
  return rpm * 2.0 * M_PI / 60.0 * wheel_radius;
}

}  // namespace aif_debug_visualizer
