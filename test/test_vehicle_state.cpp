#include <gtest/gtest.h>
#include <algorithm>
#include <fstream>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <socketcan_interface_msg/msg/socketcan_if.hpp>
#include <steered_drive_msg/msg/steered_drive.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "aif_debug_visualizer/vehicle_state.hpp"
using namespace aif_debug_visualizer;
using namespace std::chrono_literals;
using socketcan_interface_msg::msg::SocketcanIF;

struct VehicleStateTest : ::testing::Test {
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("vs_test");
  std::shared_ptr<VehicleState> vs = std::make_shared<VehicleState>(node, DecodeConfig::load(CONFIG_PATH));
  void spinFor(std::chrono::milliseconds d) {
    auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) { rclcpp::spin_some(node); std::this_thread::sleep_for(5ms); }
  }
  SocketcanIF frame(uint32_t id, uint8_t dlc, std::array<uint8_t, 8> d) {
    SocketcanIF m; m.canid = id; m.candlc = dlc; std::copy(d.begin(), d.end(), m.candata.begin()); return m;
  }
  std::array<uint8_t, 8> i32pair(int32_t a, int32_t b) {
    std::array<uint8_t, 8> f{};
    for (int i = 0; i < 4; ++i) { f[i] = (a >> (8 * i)) & 0xFF; f[4 + i] = (b >> (8 * i)) & 0xFF; }
    return f;
  }
};

TEST_F(VehicleStateTest, EmptyAtStart) {
  auto s = vs->snapshot(VehicleState::Clock::now());
  EXPECT_FALSE(s.cmd_velocity); EXPECT_FALSE(s.drive_meas); EXPECT_FALSE(s.estop);
  for (auto & st : s.chain) EXPECT_EQ(st.color, Color::Off);
  EXPECT_EQ(s.freshness.at("drive_meas").second, Freshness::Level::Stale);
}
TEST_F(VehicleStateTest, DecodesDriveMeasAndFeedsChain) {
  auto cmd = node->create_publisher<steered_drive_msg::msg::SteeredDrive>("/cmd_vel", rclcpp::SensorDataQoS());
  auto rx = node->create_publisher<SocketcanIF>("/can_rx_711", rclcpp::SensorDataQoS());
  spinFor(200ms);
  steered_drive_msg::msg::SteeredDrive c; c.velocity = 3.0; c.steering_angle = 0.1;
  cmd->publish(c);
  for (int i = 0; i < 10; ++i) { rx->publish(frame(0x711, 8, i32pair(246, 218))); spinFor(10ms); }
  auto s = vs->snapshot(VehicleState::Clock::now());
  ASSERT_TRUE(s.drive_meas);
  EXPECT_DOUBLE_EQ(s.drive_meas->right, 246); EXPECT_DOUBLE_EQ(s.drive_meas->left, 218);
  EXPECT_EQ(s.chain[0].dir, Dir::Left);
  EXPECT_EQ(s.chain[2].dir, Dir::Left); EXPECT_EQ(s.chain[2].color, Color::Good);
  EXPECT_FALSE(vs->snapshot(VehicleState::Clock::now() + 3s).drive_meas);
}
TEST_F(VehicleStateTest, IgnoresOtherIdsOnCanTxAndShortFrames) {
  auto tx = node->create_publisher<SocketcanIF>("/can_tx", rclcpp::SensorDataQoS());
  auto rx = node->create_publisher<SocketcanIF>("/can_rx_711", rclcpp::SensorDataQoS());
  spinFor(200ms);
  tx->publish(frame(0x211, 8, i32pair(1, 2)));
  rx->publish(frame(0x711, 4, i32pair(1, 2)));
  spinFor(100ms);
  auto s = vs->snapshot(VehicleState::Clock::now());
  EXPECT_FALSE(s.drive_cmd);
  EXPECT_FALSE(s.drive_meas);
}
TEST_F(VehicleStateTest, IgnoresShortCasterData) {
  auto pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/caster_data", rclcpp::SensorDataQoS());
  spinFor(200ms);
  std_msgs::msg::Float64MultiArray m; m.data = {0.1, 0.2};
  pub->publish(m);
  spinFor(100ms);
  EXPECT_FALSE(vs->snapshot(VehicleState::Clock::now()).caster);
}
TEST_F(VehicleStateTest, RegistrySharesInstanceAndTracksLatest) {
  auto a = VehicleState::get(node, CONFIG_PATH);
  EXPECT_NE(a, vs);
  EXPECT_EQ(VehicleState::get(node, CONFIG_PATH), a);
  EXPECT_EQ(VehicleState::latest(), a);
  std::string other = "/tmp/aif_debug_vs_other.yaml";
  std::ofstream(other) << std::ifstream(CONFIG_PATH).rdbuf();
  auto b = VehicleState::get(node, other);
  EXPECT_EQ(VehicleState::latest(), b);
  b.reset();
  EXPECT_EQ(VehicleState::latest(), a);
}
TEST_F(VehicleStateTest, DestroyedInstanceIgnoresLateMessages) {
  auto rx = node->create_publisher<SocketcanIF>("/can_rx_711", rclcpp::SensorDataQoS());
  auto local = std::make_shared<VehicleState>(node, DecodeConfig::load(CONFIG_PATH));
  spinFor(200ms);
  rx->publish(frame(0x711, 8, i32pair(1, 2)));
  spinFor(50ms);
  local.reset();
  for (int i = 0; i < 5; ++i) { rx->publish(frame(0x711, 8, i32pair(1, 2))); spinFor(10ms); }
  SUCCEED();
}
TEST_F(VehicleStateTest, EstopBecomesUnknownWhenStale) {
  auto rx = node->create_publisher<SocketcanIF>("/can_rx_712", rclcpp::SensorDataQoS());
  spinFor(200ms);
  rx->publish(frame(0x712, 8, {0, 0, 0, 0, 0, 0, 1, 0}));
  spinFor(100ms);
  auto now = VehicleState::Clock::now();
  ASSERT_TRUE(vs->snapshot(now).estop);
  EXPECT_TRUE(*vs->snapshot(now).estop);
  EXPECT_FALSE(vs->snapshot(now + 3s).estop);
}
