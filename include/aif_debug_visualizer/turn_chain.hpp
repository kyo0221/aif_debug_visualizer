#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include "aif_debug_visualizer/decode_config.hpp"

namespace aif_debug_visualizer {

enum class Dir { Left, Right, None };
enum class Color { Ref, Good, Warn, Bad, NoData, Off };
struct StageView { Dir dir; Color color; int samples; };

class TurnChain {
 public:
  using Clock = std::chrono::steady_clock;
  static constexpr int kStages = 5;

  explicit TurnChain(const ChainSpec & spec) : spec_(spec) {}

  void setCommand(double velocity, double steering_rad, Clock::time_point) {
    velocity_ = velocity;
    steering_ = steering_rad;
  }

  void push(int stage, double value, Clock::time_point now) {
    if (stage < 1 || stage >= kStages || !gateOpen()) return;
    if (std::abs(value) < deadband(stage)) return;
    auto & q = samples_[stage];
    q.push_back({now, value > 0, (value > 0) == (steering_ > 0)});
    const auto cutoff = now - window();
    while (!q.empty() && q.front().time < cutoff) q.pop_front();
  }

  void setEstop(bool active) { estop_ = active; }

  std::array<StageView, kStages> view(Clock::time_point now) const {
    std::array<StageView, kStages> out;
    out.fill({Dir::None, Color::Off, 0});
    if (estop_ || !gateOpen()) return out;
    out[0] = {steering_ > 0 ? Dir::Left : Dir::Right, Color::Ref, 0};
    const auto cutoff = now - window();
    for (int k = 1; k < kStages; ++k) {
      int n = 0, agree = 0;
      const Sample * latest = nullptr;
      for (const auto & s : samples_[k]) {
        if (s.time < cutoff || s.time > now) continue;
        ++n;
        if (s.agree) ++agree;
        if (!latest || s.time >= latest->time) latest = &s;
      }
      if (n == 0) {
        out[k] = {Dir::None, Color::NoData, 0};
        continue;
      }
      const double ratio = static_cast<double>(agree) / n;
      const Color c = ratio > spec_.good_ratio ? Color::Good
                    : ratio < spec_.bad_ratio ? Color::Bad : Color::Warn;
      out[k] = {latest->positive ? Dir::Left : Dir::Right, c, n};
    }
    return out;
  }

 private:
  struct Sample { Clock::time_point time; bool positive; bool agree; };

  static double deg2rad(double d) { return d * M_PI / 180.0; }

  bool gateOpen() const {
    return std::abs(velocity_) >= spec_.min_speed &&
           std::abs(steering_) >= deg2rad(spec_.min_steer_deg);
  }

  double deadband(int stage) const {
    if (stage == 3) return deg2rad(spec_.deadband_caster_deg);
    if (stage == 4) return spec_.deadband_yaw_rate;
    return spec_.deadband_rpm_diff;
  }

  Clock::duration window() const {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(spec_.window_sec));
  }

  ChainSpec spec_;
  double velocity_ = 0.0;
  double steering_ = 0.0;
  bool estop_ = false;
  std::array<std::deque<Sample>, kStages> samples_;
};

}  // namespace aif_debug_visualizer
