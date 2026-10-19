#pragma once
#include <chrono>
#include <cstddef>
#include <deque>

namespace aif_debug_visualizer {

class Freshness {
 public:
  enum class Level { Good, Warn, Stale };
  using Clock = std::chrono::steady_clock;

  Freshness(double expected_hz, double stale_sec, double window_sec = 1.0)
      : expected_hz_(expected_hz), stale_sec_(stale_sec), window_sec_(window_sec) {}

  void tick(Clock::time_point now) {
    ticks_.push_back(now);
    last_ = now;
    has_last_ = true;
    const auto cutoff = now - window();
    while (!ticks_.empty() && ticks_.front() < cutoff) ticks_.pop_front();
  }

  double hz(Clock::time_point now) const {
    const auto cutoff = now - window();
    std::size_t n = 0;
    for (const auto& t : ticks_) {
      if (t >= cutoff && t <= now) ++n;
    }
    return static_cast<double>(n) / window_sec_;
  }

  Level level(Clock::time_point now) const {
    if (!has_last_) return Level::Stale;
    if (std::chrono::duration<double>(now - last_).count() > stale_sec_) return Level::Stale;
    if (expected_hz_ > 0.0 && hz(now) < 0.5 * expected_hz_) return Level::Warn;
    return Level::Good;
  }

 private:
  Clock::duration window() const {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(window_sec_));
  }

  double expected_hz_;
  double stale_sec_;
  double window_sec_;
  std::deque<Clock::time_point> ticks_;
  Clock::time_point last_{};
  bool has_last_ = false;
};

}  // namespace aif_debug_visualizer
