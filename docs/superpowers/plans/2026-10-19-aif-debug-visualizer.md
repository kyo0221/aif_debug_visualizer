# aif_debug_visualizer 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 手動操縦時の指令→駆動輪→舵→IMU の値を rviz2 の 3D ビュー上と細いパネルに数値表示し、旋回方向の符号が反転した段を赤で示す rviz プラグイン集を、launch 1 本で起動できる形で作る。

**Architecture:** 純粋ロジック（CAN デコード・鮮度・旋回方向の鎖）は header-only で gtest により固め、`VehicleState` が購読とデコードをまとめてプロセス内で共有する。`ManualDashboardOverlay`（`rviz_common::Display`、Ogre オーバーレイに QPainter 描画）と `ManualDashboardPanel`（`rviz_common::Panel`、Qt 表）がそれを読んで描く。左右・符号・スケール・トピック名は `config/vehicle_decode.yaml` だけに書く。

**Tech Stack:** ROS 2 Humble、C++17、ament_cmake、rviz_common / rviz_rendering（Ogre overlay）、Qt5 Widgets、yaml-cpp、ament_cmake_gtest、pluginlib

**Spec:** `docs/superpowers/specs/2026-10-19-aif-debug-visualizer-design.md`

## Global Constraints

- **コード内コメントを書かない**（ユーザー指示）。docstring・説明コメント・区切りコメントも無し。
- **最小構成**: `visibility_control.h` なし、ライブラリは `aif_debug_visualizer` 1 つ（SHARED）、純粋ロジックは header-only、余計な抽象層を作らない。
- ROS 2 Humble、C++17、`namespace aif_debug_visualizer`、ヘッダガードは `#pragma once`。
- 依存 msg は `socketcan_interface_msg` / `steered_drive_msg` / `vectornav_msgs` のみ。`odrive_can` に依存しない。
- 購読 QoS は `rclcpp::SensorDataQoS()`。時刻は `std::chrono::steady_clock`。
- 符号規約: 舵角・ヨーレートは左旋回 +。`velocity_body.angular.z` には `yaw_rate_sign`（既定 −1）を掛ける。
- 0x210 / 0x711 とも `byte0-3 = 右輪 rpm`、`byte4-7 = 左輪 rpm`（int32 LE）。`wheel_radius = 0.124`。
- ビルド・テストは次のコマンドで行う（underlay は `~/formula_ws/install`）:
  ```bash
  cd ~/aif_debug_ws && source /opt/ros/humble/setup.bash && source ~/formula_ws/install/setup.bash
  colcon build --symlink-install --packages-select aif_debug_visualizer
  colcon test --packages-select aif_debug_visualizer && colcon test-result --verbose
  ```
- コミットメッセージは日本語の `feat:` / `test:` / `docs:` 形式、末尾に `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`。

## Review Focus

1. `/can_tx` には 0x210 以外（ODrive 制御など）も流れる → `canid != drive_cmd.id` のフレームを無視すること（Task 5 のテスト）。
2. `candlc < 8` のフレーム（別機器の短いフレーム）→ デコーダは `nullopt` を返し落ちないこと（Task 2・Task 5 のテスト）。
3. yaml の `type` が未知の文字列（例 `uint64`）→ `readField` が `nullopt`（Task 2 のテスト）。
4. `/caster_data` の `data` が 3 要素未満 → 無視し `caster` は `nullopt` のまま（Task 5 のテスト）。
5. publisher が 1 つも無い状態で rviz を先に起動 → 全項目 `—`、鎖は `Off`、鮮度は `Stale`、例外なし（Task 5 のテスト）。

---

## ファイル構成

```
CMakeLists.txt  package.xml  plugin_description.xml  README.md
include/aif_debug_visualizer/
  decode_config.hpp   can_decode.hpp   freshness.hpp   turn_chain.hpp
  vehicle_state.hpp   manual_dashboard_overlay.hpp   manual_dashboard_panel.hpp
src/
  decode_config.cpp   vehicle_state.cpp   manual_dashboard_overlay.cpp   manual_dashboard_panel.cpp
config/vehicle_decode.yaml   config/dashboard.rviz   launch/dashboard.launch.py
assets/font/Quicksand/{LICENSE,Quicksand-Regular.ttf,Quicksand-Bold.ttf}
deps/socketcan_interface_msg/{COLCON_IGNORE,README.md,package.xml,CMakeLists.txt,msg/SocketcanIF.msg}
test/
  test_decode_config.cpp  test_can_decode.cpp  test_freshness.cpp  test_turn_chain.cpp  test_vehicle_state.cpp
```

---

### Task 1: パッケージ骨格と DecodeConfig

**Files:**
- Create: `package.xml`, `CMakeLists.txt`, `config/vehicle_decode.yaml`
- Create: `include/aif_debug_visualizer/decode_config.hpp`, `src/decode_config.cpp`
- Test: `test/test_decode_config.cpp`

**Interfaces:**
- Produces:
  ```cpp
  namespace aif_debug_visualizer {
  struct FieldSpec { int offset; std::string type; };
  struct DriveSpec { uint32_t id; FieldSpec right; FieldSpec left; };
  struct CasterAngleSpec { uint32_t id; int offset; int counts_per_rev; double zero_offset_deg; };
  struct CasterRotSpec { uint32_t id; int offset; int counts_per_rev; double gear_ratio; int sign; };
  struct EstopSpec { uint32_t id; int byte; };
  struct ChainSpec { double min_speed, min_steer_deg, window_sec, good_ratio, bad_ratio,
                     deadband_rpm_diff, deadband_caster_deg, deadband_yaw_rate; };
  struct DecodeConfig {
    std::map<std::string, std::string> topics;
    double wheel_radius, linear_max_vel, steering_max_deg;
    DriveSpec drive_cmd, drive_meas;
    CasterAngleSpec caster_angle; CasterRotSpec caster_rot; EstopSpec estop;
    uint32_t odrive_node_id;
    int yaw_rate_sign, steering_cmd_sign, caster_meas_sign;
    ChainSpec chain;
    double stale_sec;
    std::map<std::string, double> expected_hz;
    static DecodeConfig load(const std::string & path);
    static std::string defaultPath();
    std::string canRxTopic(uint32_t id) const;
  };
  }
  ```
  `load` は必須キー欠落で `std::runtime_error("missing key: <key>")`。既定値はコードに持たない。
  `defaultPath` は環境変数 `AIF_DEBUG_DECODE_CONFIG`（未設定または空文字なら無視）→ `ament_index_cpp::get_package_share_directory("aif_debug_visualizer") + "/config/vehicle_decode.yaml"`。
  `canRxTopic(0x12)` は `topics["can_rx_prefix"] + "012"`（`%03X`）。

- [ ] **Step 1: `package.xml` を書く**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>aif_debug_visualizer</name>
  <version>0.1.0</version>
  <description>rviz2 dashboard for manual driving of the AI Formula vehicle</description>
  <maintainer email="s23c1036dv@chibatech.ac.jp">kyo0221</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>pluginlib</depend>
  <depend>qtbase5-dev</depend>
  <depend>rclcpp</depend>
  <depend>rviz_common</depend>
  <depend>rviz_rendering</depend>
  <depend>ament_index_cpp</depend>
  <depend>yaml-cpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>socketcan_interface_msg</depend>
  <depend>steered_drive_msg</depend>
  <depend>vectornav_msgs</depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>ros2launch</exec_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

- [ ] **Step 2: `CMakeLists.txt` を書く**（後の Task で `add_library` のソースとテストを追記していく）

```cmake
cmake_minimum_required(VERSION 3.8)
project(aif_debug_visualizer)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(socketcan_interface_msg REQUIRED)
find_package(steered_drive_msg REQUIRED)
find_package(vectornav_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/decode_config.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib rclcpp rviz_common rviz_rendering ament_index_cpp
  std_msgs geometry_msgs sensor_msgs socketcan_interface_msg steered_drive_msg vectornav_msgs)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets yaml-cpp)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY config launch assets DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(CONFIG_PATH "${CMAKE_CURRENT_SOURCE_DIR}/config/vehicle_decode.yaml")
  foreach(t decode_config)
    ament_add_gtest(test_${t} test/test_${t}.cpp)
    target_link_libraries(test_${t} ${PROJECT_NAME})
    target_compile_definitions(test_${t} PRIVATE CONFIG_PATH="${CONFIG_PATH}")
  endforeach()
endif()

ament_package()
```

`launch` と `assets` は Task 6・Task 8 で作るまで空ディレクトリが無いと `install(DIRECTORY)` が失敗するため、この Task では `launch/.gitkeep` と `assets/.gitkeep` を置く（Task 6/8 で中身ができたら消す）。

- [ ] **Step 3: `config/vehicle_decode.yaml` を書く**（設計書 §8 と同内容。コロン後の空白を揃える）

```yaml
topics:
  cmd_vel: /cmd_vel
  joy: /joy
  autonomous: /autonomous
  can_tx: /can_tx
  can_rx_prefix: /can_rx_
  caster_data: /caster_data
  velocity_body: /vectornav/velocity_body
  ins_status: /vectornav/raw/common
vehicle:
  wheel_radius: 0.124
  linear_max_vel: 3.0
  steering_max_deg: 15.0
can:
  drive_cmd: {id: 0x210, right: {offset: 0, type: int32}, left: {offset: 4, type: int32}}
  drive_meas: {id: 0x711, right: {offset: 0, type: int32}, left: {offset: 4, type: int32}}
  caster_angle: {id: 0x012, offset: 0, counts_per_rev: 16384, zero_offset_deg: -86.24}
  caster_rot: {id: 0x013, offset: 0, counts_per_rev: 16384, gear_ratio: 0.5909, sign: -1}
  estop: {id: 0x712, byte: 6}
  odrive: {node_id: 24}
signs:
  yaw_rate: -1
  steering_cmd: 1
  caster_meas: 1
chain:
  min_speed: 0.3
  min_steer_deg: 2.0
  window_sec: 1.0
  good_ratio: 0.7
  bad_ratio: 0.3
  deadband: {rpm_diff: 5, caster_deg: 0.5, yaw_rate: 0.02}
freshness:
  stale_sec: 2.0
  expected_hz: {cmd_vel: 20, joy: 20, drive_cmd: 500, drive_meas: 300, caster_data: 500,
                caster_angle: 100, velocity_body: 20, ins_status: 20}
```

- [ ] **Step 4: 失敗するテストを書く** `test/test_decode_config.cpp`

```cpp
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
```

- [ ] **Step 5: テストが失敗することを確認する**

Run: ビルドコマンド
Expected: `decode_config.hpp` が無く FAIL（コンパイルエラー）

- [ ] **Step 6: `decode_config.hpp` / `decode_config.cpp` を実装する**

`load` は yaml-cpp で読み、`template <class T> T req(const YAML::Node &, const std::string & key)` で欠落時に throw。`id` は yaml-cpp が整数として読む（`0x210` は YAML 1.1 の 16 進）。

- [ ] **Step 7: テストが通ることを確認する**

Run: ビルド + テストコマンド
Expected: `test_decode_config` の 4 ケースが PASS

- [ ] **Step 8: Commit**

```bash
git add package.xml CMakeLists.txt config include src test launch assets
git commit -m "feat: パッケージ骨格と DecodeConfig（yaml 読み込み）"
```

---

### Task 2: CAN デコード（header-only）

**Files:**
- Create: `include/aif_debug_visualizer/can_decode.hpp`
- Modify: `CMakeLists.txt`（`foreach(t decode_config can_decode)`）
- Test: `test/test_can_decode.cpp`

**Interfaces:**
- Consumes: Task 1 の `FieldSpec`, `DriveSpec`, `CasterAngleSpec`, `CasterRotSpec`, `EstopSpec`
- Produces:
  ```cpp
  using Frame = std::array<uint8_t, 8>;
  inline int16_t readInt16(const Frame &, int offset);
  inline int32_t readInt32(const Frame &, int offset);
  inline float readFloat32(const Frame &, int offset);
  inline std::optional<double> readField(const Frame &, uint8_t dlc, const FieldSpec &);
  struct DriveRpm { double right; double left; };
  inline std::optional<DriveRpm> decodeDrive(const Frame &, uint8_t dlc, const DriveSpec &);
  inline double normalizeAngle(double a);
  inline std::optional<double> decodeCasterAngle(const Frame &, uint8_t dlc, const CasterAngleSpec &);
  class CasterRotation {
   public:
    explicit CasterRotation(const CasterRotSpec &);
    std::optional<double> update(const Frame &, uint8_t dlc);
  };
  inline std::optional<bool> decodeEstop(const Frame &, uint8_t dlc, const EstopSpec &);
  inline std::optional<uint8_t> decodeOdriveAxisState(const Frame &, uint8_t dlc);
  struct FloatPair { float first; float second; };
  inline std::optional<FloatPair> decodeFloatPair(const Frame &, uint8_t dlc);
  constexpr uint32_t odriveCanId(uint32_t node_id, uint32_t cmd) { return node_id << 5 | cmd; }
  inline double rpmToMps(double rpm, double wheel_radius);
  ```
  すべてリトルエンディアン。`readField` は `type` が `int16` / `int32` / `float32` 以外、または `offset + size > dlc` で `nullopt`。
  `decodeCasterAngle` = `normalizeAngle(count / counts_per_rev · 2π − deg2rad(zero_offset_deg))`、`normalizeAngle(a) = atan2(sin a, cos a)`。
  `CasterRotation::update` は dell `chassis_driver_node.cpp:232-250` と同じ unwrap（初回は差 0、`gap > counts/2` で `−counts`、`gap < −counts/2` で `+counts`）、`rot = sign · total / counts_per_rev · 2π · gear_ratio`。
  `decodeOdriveAxisState` は `dlc ≥ 5` で `frame[4]`。`decodeFloatPair` は `dlc == 8` で `(float32@0, float32@4)`。

- [ ] **Step 1: 失敗するテストを書く** `test/test_can_decode.cpp`

```cpp
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
```

- [ ] **Step 2: テストが失敗することを確認する**

Run: ビルドコマンド（`CMakeLists.txt` の `foreach` に `can_decode` を足してから）
Expected: `can_decode.hpp` が無く FAIL

- [ ] **Step 3: `can_decode.hpp` を実装する**

`readInt32` 等は `std::memcpy` でバイト列から組み立てる（`static_assert` でリトルエンディアン前提にせず、シフトで組む）。

- [ ] **Step 4: テストが通ることを確認する**

Run: ビルド + テストコマンド
Expected: `test_can_decode` 11 ケース PASS

- [ ] **Step 5: Commit**

```bash
git add include/aif_debug_visualizer/can_decode.hpp test/test_can_decode.cpp CMakeLists.txt
git commit -m "feat: CAN フレームのデコード（駆動輪 rpm・舵角・従動輪回転・E-stop・ODrive）"
```

---

### Task 3: 鮮度（Hz / stale）

**Files:**
- Create: `include/aif_debug_visualizer/freshness.hpp`
- Modify: `CMakeLists.txt`（`foreach` に `freshness`）
- Test: `test/test_freshness.cpp`

**Interfaces:**
- Produces:
  ```cpp
  class Freshness {
   public:
    enum class Level { Good, Warn, Stale };
    using Clock = std::chrono::steady_clock;
    Freshness(double expected_hz, double stale_sec, double window_sec = 1.0);
    void tick(Clock::time_point now);
    double hz(Clock::time_point now) const;
    Level level(Clock::time_point now) const;
  };
  ```
  `hz` = `[now − window, now]` の tick 数 / window。`Stale` = tick 無し、または最後の tick から `stale_sec` 超。`Warn` = `expected_hz > 0` かつ `hz < 0.5 · expected_hz`。それ以外 `Good`。tick は `std::deque` に保持し窓外は捨てる。

- [ ] **Step 1: 失敗するテストを書く** `test/test_freshness.cpp`

```cpp
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
```

- [ ] **Step 2: テストが失敗することを確認する**

Run: ビルドコマンド
Expected: FAIL（ヘッダ無し）

- [ ] **Step 3: `freshness.hpp` を実装する**

- [ ] **Step 4: テストが通ることを確認する**

Run: ビルド + テストコマンド
Expected: `test_freshness` 6 ケース PASS

- [ ] **Step 5: Commit**

```bash
git add include/aif_debug_visualizer/freshness.hpp test/test_freshness.cpp CMakeLists.txt
git commit -m "feat: トピック鮮度（Hz・stale）の判定"
```

---

### Task 4: 旋回方向の鎖

**Files:**
- Create: `include/aif_debug_visualizer/turn_chain.hpp`
- Modify: `CMakeLists.txt`（`foreach` に `turn_chain`）
- Test: `test/test_turn_chain.cpp`

**Interfaces:**
- Consumes: Task 1 の `ChainSpec`
- Produces:
  ```cpp
  enum class Dir { Left, Right, None };
  enum class Color { Ref, Good, Warn, Bad, NoData, Off };
  struct StageView { Dir dir; Color color; int samples; };
  class TurnChain {
   public:
    using Clock = std::chrono::steady_clock;
    static constexpr int kStages = 5;
    explicit TurnChain(const ChainSpec & spec);
    void setCommand(double velocity, double steering_rad, Clock::time_point now);
    void push(int stage, double value, Clock::time_point now);
    void setEstop(bool active);
    std::array<StageView, kStages> view(Clock::time_point now) const;
  };
  ```
  段: 0 = cmd、1 = rpm tx（`right − left`）、2 = rpm rx（同）、3 = δ meas [rad]、4 = IMU ω [rad/s]（呼び出し側で符号調整済み）。
  不感帯: 段 1・2 は `deadband_rpm_diff`、段 3 は `deg2rad(deadband_caster_deg)`、段 4 は `deadband_yaw_rate`。不感帯内の `push` は捨てる。
  ゲート: `|velocity| ≥ min_speed && |steering| ≥ deg2rad(min_steer_deg)`。閉じている間の `push` は捨てる。
  サンプルは `(time, value, agree)`、`agree = (value > 0) == (steering > 0)`。
  `view`: estop またはゲート閉 → 全段 `{None, Off, 0}`。段 0 → `{steering > 0 ? Left : Right, Ref, 0}`。段 k ≥ 1 → 窓 `[now − window_sec, now]` 内のサンプル数 `n`。`n == 0` → `{None, NoData, 0}`。`ratio = agree/n`、`ratio > good_ratio` → Good、`ratio < bad_ratio` → Bad、それ以外 Warn。`dir` は窓内の最新サンプルの符号。

- [ ] **Step 1: 失敗するテストを書く** `test/test_turn_chain.cpp`

```cpp
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
```

- [ ] **Step 2: テストが失敗することを確認する**

Run: ビルドコマンド
Expected: FAIL（ヘッダ無し）

- [ ] **Step 3: `turn_chain.hpp` を実装する**

- [ ] **Step 4: テストが通ることを確認する**

Run: ビルド + テストコマンド
Expected: `test_turn_chain` 9 ケース PASS

- [ ] **Step 5: Commit**

```bash
git add include/aif_debug_visualizer/turn_chain.hpp test/test_turn_chain.cpp CMakeLists.txt
git commit -m "feat: 旋回方向の鎖（段ごとの符号一致率）"
```

---

### Task 5: VehicleState（購読・デコード・プロセス内共有）

**Files:**
- Create: `include/aif_debug_visualizer/vehicle_state.hpp`, `src/vehicle_state.cpp`
- Modify: `CMakeLists.txt`（`add_library` に `src/vehicle_state.cpp`、`foreach` に `vehicle_state`）
- Test: `test/test_vehicle_state.cpp`

**Interfaces:**
- Consumes: Task 1〜4 すべて
- Produces:
  ```cpp
  class VehicleState {
   public:
    using Clock = std::chrono::steady_clock;
    struct Snapshot {
      std::optional<double> cmd_velocity, cmd_steering;
      std::optional<std::array<double, 2>> joy;
      std::optional<bool> autonomous;
      std::optional<DriveRpm> drive_cmd, drive_meas;
      std::optional<std::array<double, 3>> caster;
      std::optional<int16_t> caster_angle_raw;
      std::optional<double> caster_angle, caster_rot;
      std::optional<bool> estop;
      std::optional<uint8_t> odrive_axis_state;
      std::optional<FloatPair> odrive_encoder, odrive_bus;
      std::optional<std::array<double, 3>> vbody_linear;
      std::optional<double> yaw_rate_raw;
      std::optional<vectornav_msgs::msg::InsStatus> ins;
      std::array<StageView, TurnChain::kStages> chain;
      std::map<std::string, std::pair<double, Freshness::Level>> freshness;
    };
    static std::shared_ptr<VehicleState> get(rclcpp::Node::SharedPtr node, const std::string & config_path);
    static std::shared_ptr<VehicleState> latest();
    VehicleState(rclcpp::Node::SharedPtr node, DecodeConfig config);
    const DecodeConfig & config() const;
    Snapshot snapshot(Clock::time_point now) const;
  };
  ```
  - registry: 静的 `std::map<std::string, std::weak_ptr<VehicleState>>` と `std::mutex`。キーは `std::filesystem::absolute(config_path)`。`get` は生きていれば返し、無ければ `load` して作る。`latest()` は最後に `get` で作られた生きているものを返す（無ければ `nullptr`）。
  - 購読（すべて `SensorDataQoS`）: `topics.cmd_vel`（`steered_drive_msg::msg::SteeredDrive`）、`topics.joy`（`sensor_msgs::msg::Joy`、`axes[1]`・`axes[2]`、要素不足なら無視）、`topics.autonomous`（`std_msgs::msg::Bool`）、`topics.can_tx`（`SocketcanIF`、`canid == drive_cmd.id` のみ）、`canRxTopic(id)` を `drive_meas.id`、`caster_angle.id`、`caster_rot.id`、`estop.id`、`odriveCanId(node, 0x01)`、`odriveCanId(node, 0x09)`、`odriveCanId(node, 0x17)` の 7 本、`topics.caster_data`（`std_msgs::msg::Float64MultiArray`、`data.size() < 3` は無視）、`topics.velocity_body`（`geometry_msgs::msg::TwistWithCovarianceStamped`）、`topics.ins_status`（`vectornav_msgs::msg::CommonGroup`、`insstatus` を保持）。
  - 鮮度キー（`expected_hz` と同じ）: `cmd_vel`, `joy`, `drive_cmd`, `drive_meas`, `caster_data`, `caster_angle`, `velocity_body`, `ins_status`。各 `Freshness(expected_hz.at(key), stale_sec)`。
  - `snapshot`: 鮮度が `Stale` のキーに属する値は `nullopt` にする（`cmd_*` ← cmd_vel、`joy`、`drive_cmd`、`drive_meas`、`caster` ← caster_data、`caster_angle_raw`/`caster_angle` ← caster_angle、`vbody_linear`/`yaw_rate_raw` ← velocity_body、`ins`）。`autonomous`・`estop`・`odrive_*`・`caster_rot` は最後の値を保持。
  - 鎖への供給: cmd_vel → `setCommand(velocity, steering_cmd_sign · steering_angle)`、can_tx(0x210) → `push(1, right − left)`、drive_meas → `push(2, right − left)`、caster_data → `push(3, caster_meas_sign · data[1])`、velocity_body → `push(4, yaw_rate_sign · angular.z)`、estop → `setEstop`。
  - すべての更新は 1 つの `std::mutex` で守る。

- [ ] **Step 1: 失敗するテストを書く** `test/test_vehicle_state.cpp`

```cpp
#include <gtest/gtest.h>
#include <algorithm>
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
}
```

- [ ] **Step 2: テストが失敗することを確認する**

Run: ビルドコマンド
Expected: FAIL（ヘッダ無し）

- [ ] **Step 3: `vehicle_state.hpp` / `vehicle_state.cpp` を実装する**

- [ ] **Step 4: テストが通ることを確認する**

Run: ビルド + テストコマンド
Expected: `test_vehicle_state` 5 ケース PASS（DDS 探索のため各ケース 1 秒前後かかる）

- [ ] **Step 5: Commit**

```bash
git add include/aif_debug_visualizer/vehicle_state.hpp src/vehicle_state.cpp test/test_vehicle_state.cpp CMakeLists.txt
git commit -m "feat: VehicleState（購読・デコード・鎖・鮮度をプロセス内で共有）"
```

---

### Task 6: オーバーレイ Display

**Files:**
- Create: `include/aif_debug_visualizer/manual_dashboard_overlay.hpp`, `src/manual_dashboard_overlay.cpp`, `plugin_description.xml`
- Create: `assets/font/Quicksand/{LICENSE,Quicksand-Regular.ttf,Quicksand-Bold.ttf}`（`/tmp/claude-1000/-home-kyo-formula-sub-ws-src/45dbf7a4-d497-44d3-b6f6-c79ac640fae1/scratchpad/rsf_ref/rsf_rviz_plugins/assets/font/Quicksand/` からコピー。`assets/.gitkeep` は削除）
- Modify: `CMakeLists.txt`（`add_library` に `src/manual_dashboard_overlay.cpp` と `include/aif_debug_visualizer/manual_dashboard_overlay.hpp`（AUTOMOC 用）、`pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)`）

**Interfaces:**
- Consumes: Task 5 の `VehicleState::get` / `Snapshot`、Task 1 の `DecodeConfig::defaultPath`
- Produces: rviz Display クラス `aif_debug_visualizer/ManualDashboardOverlay`（`plugin_description.xml` の `name`）、型 `aif_debug_visualizer::ManualDashboardOverlay : rviz_common::Display`

参照実装: `rsf_rviz_plugins/src/cmd_vel_display.cpp`（Ogre overlay の生成・破棄、`QImage` → `blitFromMemory`、`getOgreViewport` によるビューポート幅取得）。Humble でも `rviz_rendering::RenderSystem::get()->prepareOverlays` と `RenderWindowOgreAdapter::getOgreViewport` は存在する。

- [ ] **Step 1: `plugin_description.xml` を書く**

```xml
<library path="aif_debug_visualizer">
  <class name="aif_debug_visualizer/ManualDashboardOverlay"
         type="aif_debug_visualizer::ManualDashboardOverlay"
         base_class_type="rviz_common::Display">
    <description>Manual driving dashboard drawn over the 3D view.</description>
  </class>
</library>
```

- [ ] **Step 2: ヘッダを書く**

```cpp
class ManualDashboardOverlay : public rviz_common::Display {
  Q_OBJECT
 public:
  ManualDashboardOverlay();
  ~ManualDashboardOverlay() override;
 protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
 private Q_SLOTS:
  void reconnect();
 private:
  void ensureTexture(int w, int h);
  void draw(const VehicleState::Snapshot & s, int w, int h);
  rviz_common::properties::StringProperty * config_;
  rviz_common::properties::FloatProperty * scale_;
  rviz_common::properties::BoolProperty * show_speed_, * show_steer_, * show_drive_, * show_chain_, * show_status_;
  std::shared_ptr<VehicleState> state_;
  ...
};
```
プロパティ名と既定値: `"Decode Config"` = `DecodeConfig::defaultPath()`、`"Scale"` = 1.0、`"Show Speed"` / `"Show Steering"` / `"Show Drive"` / `"Show Chain"` / `"Show Status"` = true。`Decode Config` 変更時は `reconnect()`（`state_ = VehicleState::get(...)`、失敗は `setStatus(Error, "Config", what())`）。

- [ ] **Step 3: `manual_dashboard_overlay.cpp` を実装する**

- `onInitialize`: ノード取得、`prepareOverlays`、フォント登録（`get_package_share_directory("aif_debug_visualizer") + "/assets/font/Quicksand/"`）。
- `ensureTexture(w, h)`: ビューポートと同じ寸法のテクスチャ・マテリアル・パネル（`GMM_PIXELS`、位置 0,0）を作り、寸法が変わったら作り直す。
- `update`: 100 ms に 1 回だけ `snapshot` を取り、描画する文字列群（`std::vector<std::string>`）を作って前回と異なるときだけ `draw`。
- `draw` の配置（`scale_` を乗じる。基準は設計書 §7.1 のモックアップ）:
  - 上中央: 丸型ピル（幅 220、高さ 110）。`cmd_velocity` を 38 pt `"%.1f"`、下に `"m/s CMD"` 10 pt、その下に `vbody_linear[0]` を 22 pt `"%.2f"` と `"IMU"`。
  - 上左（x = 14 %）: `"δ CMD"` と `cmd_steering` を deg で `"%+.1f°"` 22 pt。上右（x = 86 %）: `"δ MEAS"` と `caster[1]`。目標との差が 3° 以内なら緑、それ以外は白。
  - 左下: `"DRIVE rpm  target │ actual"`、L/R 2 本の横バー（幅 110、高さ 8、フルスケール = `linear_max_vel / (2π · wheel_radius / 60)` rpm、緑 = `drive_meas`、白線 = `drive_cmd`）、右に `"%d │ %d"`。
  - 中央下: `"TURN CHAIN"` と 5 段。矢印は `Left → "◀"`, `Right → "▶"`, `None → "─"`、`Off → "—"`。色は `Ref` 白、`Good` 緑、`Warn` 黄、`Bad` 赤（赤枠付き）、`NoData` 灰（段名の横に `"no data"`）。段名 `cmd`, `rpm tx`, `rpm rx`, `δ meas`, `IMU ω`。
  - 右下: 4 行。`autonomous` → `"MANUAL"` / `"AUTO"` / `"UNKNOWN"`、`estop` → `"E-STOP clear"` 緑 / `"E-STOP ACTIVE"` 赤 / `"E-STOP —"` 灰、`ins` → `"INS <mode名> · fix|nofix"`（mode 名: `NOT_TRACKING`, `ALIGNING`, `TRACKING`, `NO_GPS`。TRACKING かつ fix で緑、それ以外は黄）、`odrive_axis_state` → `"ODrive CLOSED_LOOP %.1f V"`（8 のとき。それ以外は `"ODrive state %d"` 黄）。
  - `estop == true` のとき全面に `rgba(231,76,60,0.18)` のベール、中央に `"E-STOP"` 44 pt 太字の赤箱。
  - 値が `nullopt` の要素は灰色で `"—"`。
  - 色: 緑 (46,204,113)、黄 (241,196,15)、赤 (231,76,60)、灰 (127,140,141)、文字 (232,232,232)、背景 rgba(0,0,0,0.38)、角丸 10。
- `onDisable`: `state_.reset()`、オーバーレイ hide。デストラクタで Ogre 資源を破棄（rsf と同じ順）。

- [ ] **Step 4: ビルドが通り、プラグインが登録されることを確認する**

Run: ビルドコマンド、続けて
```bash
cat ~/aif_debug_ws/install/aif_debug_visualizer/share/ament_index/resource_index/rviz_common__pluginlib__plugin/aif_debug_visualizer
```
Expected: `share/aif_debug_visualizer/plugin_description.xml` が出力される

- [ ] **Step 5: Commit**

```bash
git add include src plugin_description.xml assets CMakeLists.txt
git commit -m "feat: 3D ビュー上のオーバーレイ Display"
```

---

### Task 7: 詳細パネル

**Files:**
- Create: `include/aif_debug_visualizer/manual_dashboard_panel.hpp`, `src/manual_dashboard_panel.cpp`
- Modify: `plugin_description.xml`（Panel クラスを追加）、`CMakeLists.txt`（`add_library` にソースとヘッダを追加）

**Interfaces:**
- Consumes: Task 5 の `VehicleState::latest` / `get`、`DecodeConfig::defaultPath`
- Produces: rviz Panel クラス `aif_debug_visualizer/ManualDashboardPanel`、型 `aif_debug_visualizer::ManualDashboardPanel : rviz_common::Panel`

- [ ] **Step 1: `plugin_description.xml` に追記する**

```xml
  <class name="aif_debug_visualizer/ManualDashboardPanel"
         type="aif_debug_visualizer::ManualDashboardPanel"
         base_class_type="rviz_common::Panel">
    <description>Raw values and topic rates for the manual dashboard.</description>
  </class>
```

- [ ] **Step 2: ヘッダとソースを実装する**

```cpp
class ManualDashboardPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit ManualDashboardPanel(QWidget * parent = nullptr);
  void onInitialize() override;
 private:
  void refresh();
  std::map<std::string, QLabel *> values_;
  std::shared_ptr<VehicleState> state_;
  QTimer * timer_;
};
```
- 行（この順、左が名前ラベル、右が値ラベル、等幅フォント）: `/cmd_vel`, `/joy`, `0x210 tx`, `0x711 rx`, `/caster_data`, `0x012`, `velocity_body`, `ins_status`（以上 8 行は Hz を `"%.0f Hz"`、`Good` 緑・`Warn` 黄・`Stale` 灰で `"—"`）、`0x012 raw`（`"%d  %+.2f°"`）、`0x013 rot`（`"%.3f rad"`）、`vx / vy`（`"%.2f / %.2f"`）、`ω raw (NED)`（`"%+.3f"`）、`INS flags`（`"mode=%d fix=%d imu_err=%d gps_err=%d time_err=%d mag_err=%d"`）、`ODrive`（`"state=%d pos=%.2f vbus=%.1f ibus=%.2f"`）、`joy`（`"%.2f / %.2f"`）、`rpm→m/s L/R`（`"%.2f / %.2f"`）。
- `onInitialize`: ノード取得、`timer_->start(200)`。
- `refresh`: `state_ = VehicleState::latest()`、`nullptr` なら `VehicleState::get(node, DecodeConfig::defaultPath())`（例外時は最上段に赤字で `what()`）。`snapshot` を取り各ラベルを更新。`nullopt` は灰 `"—"`。

- [ ] **Step 3: ビルドが通り、Panel が登録されることを確認する**

Run: ビルドコマンド、続けて
```bash
grep -c "rviz_common::Panel" ~/aif_debug_ws/install/aif_debug_visualizer/share/aif_debug_visualizer/plugin_description.xml
```
Expected: `1`

- [ ] **Step 4: Commit**

```bash
git add include src plugin_description.xml CMakeLists.txt
git commit -m "feat: 生値と受信周波数の詳細パネル"
```

---

### Task 8: rviz 設定・launch・同梱物・README と bag 再生検証

**Files:**
- Create: `config/dashboard.rviz`, `launch/dashboard.launch.py`（`launch/.gitkeep` は削除）, `README.md`
- Create: `deps/socketcan_interface_msg/{COLCON_IGNORE,README.md,package.xml,CMakeLists.txt,msg/SocketcanIF.msg}`
- Modify: `.gitignore`（`.superpowers/` を追加）

**Interfaces:**
- Consumes: Task 6・7 のプラグイン名、Task 1 の環境変数 `AIF_DEBUG_DECODE_CONFIG`

- [ ] **Step 1: `config/dashboard.rviz` を書く**

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: aif_debug_visualizer/ManualDashboardPanel
    Name: ManualDashboardPanel
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
      Plane Cell Count: 10
    - Class: aif_debug_visualizer/ManualDashboardOverlay
      Enabled: true
      Name: ManualDashboardOverlay
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 8
      Name: Current View
      Pitch: 0.6
      Yaw: 3.14
    Saved: ~
Window Geometry:
  Height: 900
  Width: 1500
```

- [ ] **Step 2: `launch/dashboard.launch.py` を書く**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('aif_debug_visualizer')
    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([share, 'config', 'dashboard.rviz'])),
        DeclareLaunchArgument('decode_config', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            additional_env={'AIF_DEBUG_DECODE_CONFIG': LaunchConfiguration('decode_config')},
            output='screen'),
    ])
```

- [ ] **Step 3: `deps/socketcan_interface_msg` を同梱する**

`/home/kyo/dell_ws/src/socketcan_interface/socketcan_interface_msg/` の `package.xml`、`msg/SocketcanIF.msg` をコピー。`CMakeLists.txt` は `rosidl_generate_interfaces` までの最小形に書き直す（lint 節は含めない）。`COLCON_IGNORE`（空ファイル）と `README.md`（「Jetson の formula_ws に `socketcan_interface_msg` が無いときだけ `COLCON_IGNORE` を消す」の 2 行）を置く。

- [ ] **Step 4: `README.md` を書く**

内容: 目的（1 段落）、ビルド手順（Global Constraints のコマンド）、起動 `ros2 launch aif_debug_visualizer dashboard.launch.py [decode_config:=/path.yaml]`、bag 再生での確認手順、`config/vehicle_decode.yaml` の各キーの意味（左右バイト順・`yaw_rate` 符号・鎖のしきい値）、rviz 上で Panel を右にドッキングして `File → Save Config` で `dashboard.rviz` を上書きする手順。

- [ ] **Step 5: ビルド・テストが通ることを確認する**

Run: ビルド + テストコマンド
Expected: 全テスト PASS、`install/aif_debug_visualizer/share/aif_debug_visualizer/{config,launch,assets}` が存在

- [ ] **Step 6: bag 再生で表示を確認する**

ターミナル A:
```bash
source /opt/ros/humble/setup.bash && source ~/formula_ws/install/setup.bash
ros2 bag play /media/kyo/24B46936B4690B9C/aif_bags/4_0ms_1
```
ターミナル B:
```bash
source /opt/ros/humble/setup.bash && source ~/aif_debug_ws/install/setup.bash
ros2 launch aif_debug_visualizer dashboard.launch.py
```
Expected（設計書 §12）:
- 上中央の速度が走行に追従し、左右 rpm のバー・数値が動く。左旋回中は R > L
- 鎖は段 2〜5 に赤が一度も出ない（段 2 の黄は許容）
- パネルで `ins_status` と `0x013 rot` が灰 `—`、他は Hz が期待値付近（`/cmd_vel` と `/joy` は 50 Hz 程度）
- rviz を閉じて `yaw_rate: 1` にした yaml を `decode_config:=` で渡して再起動すると IMU 段だけ赤

再生中に `import -window root /tmp/aif_dashboard.png`（ImageMagick）または `gnome-screenshot -f /tmp/aif_dashboard.png` で画面を保存し、Read で確認する。

- [ ] **Step 7: Commit**

```bash
git add -A config launch deps README.md .gitignore
git commit -m "feat: rviz 設定・launch・socketcan_interface_msg 同梱・README"
```
