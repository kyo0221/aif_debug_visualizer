#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include "aif_debug_visualizer/decode_config.hpp"
using aif_debug_visualizer::DecodeConfig;

TEST(DecodeConfig, LoadsShippedYaml) {
  auto c = DecodeConfig::load(CONFIG_PATH);
  EXPECT_EQ(c.topics.at("cmd_vel"), "/cmd_vel");
  EXPECT_EQ(c.topics.at("ins_status"), "/vectornav/raw/common");
  EXPECT_DOUBLE_EQ(c.wheel_radius, 0.124);
  EXPECT_EQ(c.drive_cmd.id, 0x210u);
  EXPECT_EQ(c.drive_meas.id, 0x711u);
  EXPECT_EQ(c.drive_meas.right.offset, 0);
  EXPECT_EQ(c.drive_meas.left.offset, 4);
  EXPECT_EQ(c.drive_meas.left.type, "int32");
  EXPECT_EQ(c.caster_angle.counts_per_rev, 16384);
  EXPECT_DOUBLE_EQ(c.caster_angle.zero_offset_deg, -86.24);
  EXPECT_EQ(c.caster_rot.sign, -1);
  EXPECT_EQ(c.estop.byte, 6);
  EXPECT_EQ(c.caster_angle.id, 0x12u);
  EXPECT_EQ(c.caster_rot.id, 0x13u);
  EXPECT_EQ(c.estop.id, 0x712u);
  EXPECT_EQ(c.odrive_node_id, 24u);
  EXPECT_EQ(c.yaw_rate_sign, -1);
  EXPECT_DOUBLE_EQ(c.chain.min_steer_deg, 2.0);
  EXPECT_DOUBLE_EQ(c.chain.deadband_rpm_diff, 5.0);
  EXPECT_DOUBLE_EQ(c.stale_sec, 2.0);
  EXPECT_DOUBLE_EQ(c.expected_hz.at("drive_meas"), 300.0);
}

TEST(DecodeConfig, CanRxTopicIsZeroPadded) {
  auto c = DecodeConfig::load(CONFIG_PATH);
  EXPECT_EQ(c.canRxTopic(0x12), "/can_rx_012");
  EXPECT_EQ(c.canRxTopic(0x711), "/can_rx_711");
}

TEST(DecodeConfig, MissingKeyThrows) {
  std::string p = "/tmp/aif_debug_missing.yaml";
  std::ofstream(p) << "topics: {cmd_vel: /cmd_vel}\n";
  EXPECT_THROW(DecodeConfig::load(p), std::runtime_error);
}

TEST(DecodeConfig, DefaultPathPrefersEnv) {
  setenv("AIF_DEBUG_DECODE_CONFIG", "/x/y.yaml", 1);
  EXPECT_EQ(DecodeConfig::defaultPath(), "/x/y.yaml");
  setenv("AIF_DEBUG_DECODE_CONFIG", "", 1);
  EXPECT_NE(DecodeConfig::defaultPath(), "");
  unsetenv("AIF_DEBUG_DECODE_CONFIG");
}
