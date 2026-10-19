#include <gtest/gtest.h>
#include "aif_debug_visualizer/freshness.hpp"
using aif_debug_visualizer::Freshness;
using namespace std::chrono_literals;
static const auto t0 = Freshness::Clock::time_point{} + 100s;

TEST(Freshness, NoTicksIsStale) {
  Freshness f(20, 2.0);
  EXPECT_EQ(f.level(t0), Freshness::Level::Stale);
  EXPECT_DOUBLE_EQ(f.hz(t0), 0.0);
}
TEST(Freshness, TwentyHzIsGood) {
  Freshness f(20, 2.0);
  for (int i = 0; i < 20; ++i) f.tick(t0 + i * 50ms);
  auto now = t0 + 999ms;
  EXPECT_DOUBLE_EQ(f.hz(now), 20.0);
  EXPECT_EQ(f.level(now), Freshness::Level::Good);
}
TEST(Freshness, BelowHalfIsWarn) {
  Freshness f(20, 2.0);
  for (int i = 0; i < 5; ++i) f.tick(t0 + i * 200ms);
  EXPECT_EQ(f.level(t0 + 999ms), Freshness::Level::Warn);
}
TEST(Freshness, OldLastTickIsStale) {
  Freshness f(20, 2.0);
  f.tick(t0);
  EXPECT_EQ(f.level(t0 + 2500ms), Freshness::Level::Stale);
}
TEST(Freshness, ZeroExpectedNeverWarns) {
  Freshness f(0, 2.0);
  f.tick(t0);
  EXPECT_EQ(f.level(t0 + 500ms), Freshness::Level::Good);
}
TEST(Freshness, DropsTicksOutsideWindow) {
  Freshness f(20, 5.0);
  for (int i = 0; i < 40; ++i) f.tick(t0 + i * 50ms);
  EXPECT_DOUBLE_EQ(f.hz(t0 + 2999ms), 0.0);
  EXPECT_EQ(f.level(t0 + 2999ms), Freshness::Level::Warn);
}
