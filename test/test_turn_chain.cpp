#include <gtest/gtest.h>
#include "aif_debug_visualizer/turn_chain.hpp"
using namespace aif_debug_visualizer;
using namespace std::chrono_literals;
static const ChainSpec kSpec{0.3, 2.0, 1.0, 0.7, 0.3, 5.0, 0.5, 0.02};
static const auto t0 = TurnChain::Clock::time_point{} + 100s;

static TurnChain leftTurn() { TurnChain c(kSpec); c.setCommand(3.0, 0.1, t0); return c; }
static void pushN(TurnChain & c, int stage, int n, double value) {
  for (int i = 0; i < n; ++i) c.push(stage, value, t0 + i * 10ms);
}

TEST(TurnChain, GateClosedIsOff) {
  TurnChain c(kSpec);
  c.setCommand(0.1, 0.1, t0);
  for (auto & s : c.view(t0)) EXPECT_EQ(s.color, Color::Off);
}
TEST(TurnChain, AllAgreeIsGood) {
  auto c = leftTurn();
  pushN(c, 1, 10, 28); pushN(c, 2, 10, 22); pushN(c, 3, 10, 0.1); pushN(c, 4, 10, 0.4);
  auto v = c.view(t0 + 100ms);
  EXPECT_EQ(v[0].dir, Dir::Left); EXPECT_EQ(v[0].color, Color::Ref);
  for (int k = 1; k < 5; ++k) { EXPECT_EQ(v[k].dir, Dir::Left); EXPECT_EQ(v[k].color, Color::Good); EXPECT_EQ(v[k].samples, 10); }
}
TEST(TurnChain, InvertedImuIsBad) {
  auto c = leftTurn();
  pushN(c, 4, 10, -0.4);
  auto v = c.view(t0 + 100ms);
  EXPECT_EQ(v[4].dir, Dir::Right); EXPECT_EQ(v[4].color, Color::Bad);
}
TEST(TurnChain, PidFlipIsWarnOrGood) {
  auto c = leftTurn();
  for (int i = 0; i < 10; ++i) c.push(1, i < 8 ? 28 : -28, t0 + i * 10ms);
  EXPECT_EQ(c.view(t0 + 100ms)[1].color, Color::Good);
  auto d = leftTurn();
  for (int i = 0; i < 10; ++i) d.push(1, i < 5 ? 28 : -28, t0 + i * 10ms);
  EXPECT_EQ(d.view(t0 + 100ms)[1].color, Color::Warn);
}
TEST(TurnChain, DeadSensorIsNoData) {
  auto c = leftTurn();
  pushN(c, 3, 10, 0.0);
  pushN(c, 1, 10, 2.0);
  auto v = c.view(t0 + 100ms);
  EXPECT_EQ(v[3].color, Color::NoData); EXPECT_EQ(v[3].dir, Dir::None);
  EXPECT_EQ(v[1].color, Color::NoData);
}
TEST(TurnChain, OldSamplesLeaveWindow) {
  auto c = leftTurn();
  pushN(c, 2, 10, 22);
  EXPECT_EQ(c.view(t0 + 2s)[2].color, Color::NoData);
}
TEST(TurnChain, SamplesWhileGateClosedAreDropped) {
  TurnChain c(kSpec);
  c.setCommand(0.1, 0.1, t0);
  pushN(c, 2, 10, 22);
  c.setCommand(3.0, 0.1, t0 + 50ms);
  EXPECT_EQ(c.view(t0 + 100ms)[2].color, Color::NoData);
}
TEST(TurnChain, EstopIsOff) {
  auto c = leftTurn();
  pushN(c, 2, 10, 22);
  c.setEstop(true);
  for (auto & s : c.view(t0 + 100ms)) EXPECT_EQ(s.color, Color::Off);
}
TEST(TurnChain, RightTurnAgrees) {
  TurnChain c(kSpec);
  c.setCommand(3.0, -0.1, t0);
  pushN(c, 2, 10, -22);
  auto v = c.view(t0 + 100ms);
  EXPECT_EQ(v[0].dir, Dir::Right); EXPECT_EQ(v[2].dir, Dir::Right); EXPECT_EQ(v[2].color, Color::Good);
}
