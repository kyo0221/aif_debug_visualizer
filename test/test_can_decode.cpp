#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include "aif_debug_visualizer/can_decode.hpp"
using namespace aif_debug_visualizer;

static Frame i32pair(int32_t a, int32_t b) {
  Frame f{};
  for (int i = 0; i < 4; ++i) { f[i] = (a >> (8 * i)) & 0xFF; f[4 + i] = (b >> (8 * i)) & 0xFF; }
  return f;
}
static Frame i16at0(int16_t v) { Frame f{}; f[0] = v & 0xFF; f[1] = (v >> 8) & 0xFF; return f; }
static const DriveSpec kDrive{0x711, {0, "int32"}, {4, "int32"}};
static const CasterAngleSpec kAngle{0x012, 0, 16384, -86.24};
static const CasterRotSpec kRot{0x013, 0, 16384, 0.5909, -1};

TEST(CanDecode, ReadsLittleEndian) {
  Frame f{0x78, 0x56, 0x34, 0x12, 0, 0, 0, 0};
  EXPECT_EQ(readInt32(f, 0), 0x12345678);
  EXPECT_EQ(readInt16(f, 0), 0x5678);
}
TEST(CanDecode, DriveRightFirst) {
  auto d = decodeDrive(i32pair(246, 218), 8, kDrive);
  ASSERT_TRUE(d);
  EXPECT_DOUBLE_EQ(d->right, 246);
  EXPECT_DOUBLE_EQ(d->left, 218);
}
TEST(CanDecode, DriveSwappedByConfig) {
  DriveSpec s{0x711, {4, "int32"}, {0, "int32"}};
  EXPECT_DOUBLE_EQ(decodeDrive(i32pair(246, 218), 8, s)->right, 218);
}
TEST(CanDecode, DriveNegative) {
  EXPECT_DOUBLE_EQ(decodeDrive(i32pair(-100, 7), 8, kDrive)->right, -100);
}
TEST(CanDecode, ShortDlcIsNullopt) {
  EXPECT_FALSE(decodeDrive(i32pair(1, 2), 4, kDrive));
  EXPECT_FALSE(decodeEstop(Frame{}, 6, EstopSpec{0x712, 6}));
  EXPECT_FALSE(decodeFloatPair(Frame{}, 7));
  EXPECT_FALSE(decodeOdriveAxisState(Frame{}, 4));
}
TEST(CanDecode, UnknownTypeIsNullopt) {
  EXPECT_FALSE(readField(Frame{}, 8, FieldSpec{0, "uint64"}));
}
TEST(CanDecode, CasterAngleZeroAtOffset) {
  EXPECT_NEAR(*decodeCasterAngle(i16at0(-3925), 8, kAngle), 0.0, 1e-3);
  EXPECT_NEAR(*decodeCasterAngle(i16at0(0), 8, kAngle), 86.24 * M_PI / 180.0, 1e-9);
  EXPECT_NEAR(*decodeCasterAngle(i16at0(8192), 8, kAngle), M_PI + 86.24 * M_PI / 180.0 - 2 * M_PI, 1e-9);
}
TEST(CanDecode, CasterRotationUnwraps) {
  CasterRotation r(kRot);
  EXPECT_NEAR(*r.update(i16at0(16380), 8), 0.0, 1e-12);
  EXPECT_NEAR(*r.update(i16at0(3), 8), -7.0 / 16384 * 2 * M_PI * 0.5909, 1e-12);
}
TEST(CanDecode, Estop) {
  Frame f{}; f[6] = 1;
  EXPECT_TRUE(*decodeEstop(f, 8, EstopSpec{0x712, 6}));
  EXPECT_FALSE(*decodeEstop(Frame{}, 8, EstopSpec{0x712, 6}));
}
TEST(CanDecode, Odrive) {
  Frame hb{}; hb[4] = 8;
  EXPECT_EQ(*decodeOdriveAxisState(hb, 8), 8);
  float a = 47.8f, b = 1.25f; Frame f{};
  std::memcpy(f.data(), &a, 4); std::memcpy(f.data() + 4, &b, 4);
  EXPECT_FLOAT_EQ(decodeFloatPair(f, 8)->first, 47.8f);
  EXPECT_FLOAT_EQ(decodeFloatPair(f, 8)->second, 1.25f);
  EXPECT_EQ(odriveCanId(24, 0x17), 0x317u);
  EXPECT_EQ(odriveCanId(24, 0x01), 0x301u);
  EXPECT_EQ(odriveCanId(24, 0x09), 0x309u);
}
TEST(CanDecode, RpmToMps) {
  EXPECT_NEAR(rpmToMps(77.0, 0.124), 1.0, 1e-3);
}

TEST(CanDecode, CasterAngleRejectsOffsetPastFrame) {
  CasterAngleSpec s{0x012, 7, 16384, 0.0};
  Frame f{};
  EXPECT_FALSE(decodeCasterAngle(f, 8, s));
  CasterRotSpec r{0x013, 7, 16384, 0.5909, -1};
  CasterRotation rot(r);
  EXPECT_FALSE(rot.update(f, 8));
}
