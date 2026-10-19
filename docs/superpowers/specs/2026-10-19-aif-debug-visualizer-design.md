# aif_debug_visualizer 設計書（手動操縦用 rviz 計器盤）

作成: 2026-10-19

## 1. 目的

AI Formula 車両の **手動操縦時** に、指令 → アクチュエータ → センサの鎖を数値で同時に眺め、
「joy で右旋回を送ったのに左へ曲がった」のような症状を **その場で切り分ける** ための rviz2 プラグイン集。

発端: 右旋回指令で左旋回した際、エンコーダが不正なのか `/vectornav/velocity_body` が不正なのか、
数値が見えていないために切り分けられなかった。既存の `debug_printer`（dell_ws）は左右のバイト順が
逆で、見ていればむしろ誤診していた（§5.1）。

Jetson 側（`formula_ws`、ROS 2 Humble）で launch 1 本を叩くだけで、計器盤付きの rviz2 が立ち上がる。

## 2. ゴール / 非ゴール / アンチゴール

**ゴール**

- `ros2 launch aif_debug_visualizer dashboard.launch.py` だけで表示が始まる
- 指令値と実測値を **同じ単位で隣に** 置く（rpm と m/s を併記、舵角は deg）
- 「旋回方向の鎖」で、符号が反転した段が一目で分かる
- 未受信（2 秒）と受信周波数の低下が見える
- rosbag 再生でも同じ画面が出る（録画されている生トピックだけで動く）

**非ゴール**

- 車両への指令送信（完全に読み取り専用）
- 自律走行時の地図・経路の可視化（既存の `~/.rviz2/formula_cfg.rviz` の役目）
- ログの記録・事後解析（bag と `aif1012_analysis` の役目）
- aiformula 本体・dell_ws のコード変更（見つけた不具合は §10 に別件として記す）

**アンチゴール（こうなったら失敗）**

- 左右の対応や符号の変換がコードの奥に埋まり、また誤診の元になる
  → **変換はすべて 1 つの yaml に明示**し、画面にも「ROS+」「NED」のように規約を書く
- rviz が重くなって操縦の邪魔になる → 描画は表示文字列が変わったときだけ

## 3. 前提と制約

| 項目 | 内容 |
|---|---|
| 実行環境 | Jetson Orin、Ubuntu 22.04、ROS 2 Humble、rviz2（`rviz_common` / `rviz_rendering`）、Qt5、C++17 |
| 配置 | `formula_ws/src/aif_debug_visualizer` に clone（または `formula_ws` を underlay にした別 ws）。aiformula 本体には手を入れない |
| 通信 | Dell PC（CAN・chassis_driver・joy）と Jetson は同一 `ROS_DOMAIN_ID` の DDS 共有。CAN 生フレームは `socketcan_interface_node` が `/can_rx_XXX`（ID ごと）と `/can_tx` に流している |
| 依存 msg | `socketcan_interface_msg`、`steered_drive_msg`、`vectornav_msgs`、`std_msgs`、`geometry_msgs`、`sensor_msgs`。**`odrive_can` には依存しない**（ODrive の値も生 CAN から読む） |
| 依存の保険 | Jetson の `formula_ws` に `socketcan_interface_msg` が無い場合に備え、`deps/socketcan_interface_msg/` に同梱（`COLCON_IGNORE` 付き。必要なときだけ外す）。定義は `SocketcanIF.msg`（`std_msgs/Header header, uint32 canid, bool eff, uint8 candlc, uint8[8] candata`） |
| 参考実装 | `rsf_rviz_plugins`（kyo0221）。Display 型は Ogre オーバーレイに QPainter で描いたテクスチャ、Panel 型は Qt の表。フォント Quicksand（OFL）を同梱する流儀も踏襲 |

## 4. 購読トピックと解釈（データ仕様）

すべて読み取り専用。多バイト整数は **リトルエンディアン**、CAN データは `candata[0..7]`。

### 4.1 指令系

| トピック | 型 | 使う値 | 規約 |
|---|---|---|---|
| `/cmd_vel` | `steered_drive_msg/SteeredDrive` | `velocity` [m/s]、`steering_angle` [rad] | 前進 +、**左旋回 +**（ROS 慣例）。手動時は dell の `controller_node` が joy から生成、自動時は Jetson の `controller_server` |
| `/joy` | `sensor_msgs/Joy` | `axes[1]`（L_y → 速度）、`axes[2]`（R_x → 舵角） | スティック左倒し・上倒しが + |
| `/autonomous` | `std_msgs/Bool` | true = 自動 | 起動後に届いた値のみ。未受信は「UNKNOWN」と表示 |

### 4.2 駆動輪（Roboteq FBLG2360T、CAN 2 系統基板経由）

| トピック | CAN ID | デコード | 備考 |
|---|---|---|---|
| `/can_tx` | **0x210**（この ID のみ拾う） | `right_rpm = int32(b0..3)`、`left_rpm = int32(b4..7)` | `chassis_driver_node::send_rpm` のエンコードと同じ。500 Hz |
| `/can_rx_711` | 0x711 | `right_rpm = int32(b0..3)`、`left_rpm = int32(b4..7)` | **bag で実測確認済み**（§5.1）。車輪 rpm。約 300〜500 Hz |

rpm → m/s: `v = rpm · 2π · wheel_radius / 60`、`wheel_radius = 0.124`（`send_rpm` と同じ値）。
左右差 `R − L` が + なら左旋回方向。

### 4.3 舵（従動輪 = 車体中心のキャスタ 1 本）

| トピック | デコード | 備考 |
|---|---|---|
| `/caster_data` (`std_msgs/Float64MultiArray`) | `data[0]` = 目標舵角、`data[1]` = 実舵角、`data[2]` = キャスタ積算回転（すべて rad） | 500 Hz。停止モード中は出ない。実舵角は指令と同じ符号系（左 +、bag で 98 % 一致） |
| `/can_rx_012` | `count = int16(b0..1)`、`angle = normalize(count / 16384 · 2π − offset)`、`offset = −86.24°` | 舵角センサ生値。100 Hz。パネルには生カウントも出す |
| `/can_rx_013` | `int16(b0..1)` を 16384 で unwrap して積算、`rot = −count_total / 16384 · 2π · 0.5909` [rad] | 従動輪回転。2026-10-12 の bag には未記録 |

### 4.4 ODrive（舵角プリロード用リール 1 軸、node_id = 24）

ODrive CAN simple protocol、`id = node_id << 5 | cmd`。

| トピック | cmd | デコード |
|---|---|---|
| `/can_rx_301` | 0x01 Heartbeat | `axis_state = b4`（8 = CLOSED_LOOP_CONTROL） |
| `/can_rx_309` | 0x09 Get_Encoder_Estimates | `pos = float32(b0..3)` [turn]、`vel = float32(b4..7)` |
| `/can_rx_317` | 0x17 Get_Bus_Voltage_Current | `vbus = float32(b0..3)` [V]、`ibus = float32(b4..7)` [A] |

### 4.5 安全

| トピック | デコード |
|---|---|
| `/can_rx_712` | `estop = (b6 != 0)` |

### 4.6 vectornav（Jetson 側で publish）

| トピック | 型 | 使う値 | 規約 |
|---|---|---|---|
| `/vectornav/velocity_body` | `geometry_msgs/TwistWithCovarianceStamped` | `twist.twist.linear.x`（車体前後速度）、`twist.twist.angular.z`（ヨーレート） | `linear.x` は前進 +。**`angular.z` はセンサ座標（z 下向き）のままで右旋回 +** = ROS と逆（§5.2）。計器盤では `yaw_sign = −1` を掛けて ROS 慣例に揃え、「ROS+」と表示。生値はパネルに「NED」と添えて出す |
| `/vectornav/raw/common` | `vectornav_msgs/CommonGroup` | `insstatus.mode`（0 NOT_TRACKING / 1 ALIGNING / 2 TRACKING / 3 NO_GPS）、`insstatus.gps_fix`、`imu_error`、`gps_error`、`time_error`、`mag_pres_error` | 2026-10-12 の bag には未記録。`/vectornav/raw/ins` に切り替え可能（同じ `InsStatus` が入っている） |

### 4.7 期待周波数（鮮度判定）

| トピック | 期待 Hz | 根拠 |
|---|---|---|
| `/can_tx`(0x210) | 500 | `interval_ms = 2` |
| `/can_rx_711` | 300〜500 | bag 実測 |
| `/caster_data` | 500 | `system_check.sh` |
| `/can_rx_012` | 100 | bag 実測 |
| `/cmd_vel`, `/joy` | 20 | `system_check.sh`（bag では 50 Hz。期待値超えは問題にしない） |
| `/vectornav/velocity_body` | 20 | `system_check.sh` |

Hz は直近 1 秒の受信数。期待の 50 % 未満で黄、2 秒未受信で灰（stale）。期待値を上回る分は警告しない。

## 5. 実測で確定した事実（bag 2026-10-12 `aif_bags/4_0ms_1` ほか）

### 5.1 0x711 のバイト順は 0x210 と同じ「右 → 左」

- `rx[b0..3]` は `tx_right` と相関 0.43（`tx_left` とは 0.25）、`rx[b4..7]` は `tx_left` と 0.44
- 左旋回指令中（|δ| > 4°、v > 1 m/s、32,669 サンプル）に `rx[b0..3] > rx[b4..7]`（外輪が速い）が **100 %**
- 値のスケールは車輪 rpm（`velocity_body.linear.x` に対し 74.7·v、理論 77·v）
- → dell_ws `debug_printer.cpp` の「b0..3 = 左」は誤り。この計器盤では **右 → 左** を既定にし、yaml で入れ替え可能にする

### 5.2 `velocity_body.angular.z` は ROS 慣例と符号が逆

左旋回指令・実舵角 +・右輪が速い、のとき `angular.z` は負（一致率 0.00〜0.02）。
`vn_sensor_msgs` が `CommonGroup.angularrate` をセンサ座標のまま詰めているため。`linear.x` は問題なし。

### 5.3 指令 rpm の左右差は舵角 PID の出力

`sign(tx_right − tx_left)` が指令舵角と一致するのは 79 %（キャスタが目標に追いつくと逆符号になる）。
瞬時値で赤くすると誤報になるため、鎖の判定は **直近 1 秒の一致率** で行う（§6）。

## 6. 「旋回方向の鎖」の定義

| 段 | 符号の元 | 表示名 |
|---|---|---|
| 1 | `sign(cmd.steering_angle)` | cmd |
| 2 | `sign(tx_right − tx_left)` | rpm tx |
| 3 | `sign(rx_right − rx_left)` | rpm rx |
| 4 | `sign(caster_data[1])` | δ meas |
| 5 | `sign(yaw_sign · velocity_body.angular.z)` | IMU ω |

- 矢印: + → `◀`（左旋回）、− → `▶`、判定不能 → `─`
- 判定を行う条件: `|cmd.velocity| ≥ 0.3 m/s` かつ `|cmd.steering_angle| ≥ 2°`。満たさない間は全段 `─`、E-stop 中は `—`
- 各段の色: 直近 1 秒の自サンプルのうち **段 1 と符号が一致した割合** `a` で、`a > 0.7` 緑、`0.3 ≤ a ≤ 0.7` 黄、`a < 0.3` 赤枠（反転）
- 段 1 自身は常に白（基準）。段 2〜5 で最初に赤になった段が「反転が起きた場所」
- **符号 0 の扱い**: 各段に不感帯を置き（`|R − L| < 5 rpm`、`|δ_meas| < 0.5°`、`|ω| < 0.02 rad/s`）、不感帯内のサンプルは一致率の分母に **入れない**。
  舵角センサが死んで `caster_data[1]` が常に 0 のような段は、赤ではなく灰の `─`（根拠なし）になる。生値はパネルで見える
- **窓が空の段**（有効サンプルが 1 つもない。stale や不感帯のみ）は灰の `─`。stale と区別するため段名の横に `no data` / `0` を添える
- 判定条件（速度・舵角のゲート）を満たさない間に届いたサンプルは窓に **入れない**。窓は「ゲートを満たしていた直近 1 秒分」の有効サンプルで構成する
- 段 2（rpm tx）は舵角 PID の出力なので瞬間的に反転する（§5.3）。黄になることは正常で、赤だけを異常として扱う
- しきい値・窓長・不感帯・`yaw_sign` は yaml（§8）

## 7. 画面構成（採用案 C: オーバーレイ + 細いパネル）

### 7.1 3D ビュー上のオーバーレイ（`ManualDashboardOverlay`、`rviz_common::Display`）

ビューポートの 6 箇所に固定。要素ごとに表示 ON/OFF と倍率をプロパティで持つ。

| 位置 | 内容 |
|---|---|
| 上中央（丸型ピル） | 指令速度 `cmd.velocity` を大きく（m/s、小数 1 桁）。その下に IMU 実速度 `linear.x` を小さく |
| 上左 | 目標舵角 `cmd.steering_angle` [deg、小数 1 桁、符号付き] |
| 上右 | 実舵角 `caster_data[1]` [deg]。目標との差が 3° 以内なら緑、それ以外は白 |
| 左下 | 左右 rpm。L/R 2 本の横バー（緑 = 実測 0x711、白線 = 目標 0x210、フルスケール = `linear_max.vel` 相当の rpm）＋ `target │ actual` の数値 |
| 中央下 | 旋回方向の鎖（§6）。5 段の矢印と段名 |
| 右下 | 4 行の状態: `MANUAL / AUTO / UNKNOWN`、`E-STOP clear / ACTIVE`、`INS <mode> · fix/nofix`、`ODrive <axis_state 名> <vbus> V` |
| 全面 | E-stop 中は半透明の赤いベールと中央に `E-STOP`。stale（2 秒未受信）の要素は灰色で `—` |

描画は rsf `CmdVelDisplay` と同じ方式（Ogre overlay + `QImage` を `QPainter` で描いて `blitFromMemory`）。
文字列が変わったときだけ再描画。フォントは Quicksand を同梱。

### 7.2 詳細パネル（`ManualDashboardPanel`、`rviz_common::Panel`）

右ドックの `QGridLayout` 表。「止めてから見る」生値と鮮度。

| 行 | 値 |
|---|---|
| 各トピック | 受信 Hz（期待値との比で色） — `/cmd_vel`、`0x210`、`0x711`、`/caster_data`、`0x012`、`velocity_body`、`raw/common` |
| 0x012 raw | 生カウント、換算角 [deg] |
| 0x013 rot | 積算回転 [rad] |
| vx / vy | `velocity_body.linear` |
| ω raw (NED) | `angular.z` の生値 |
| INS flags | mode 名、gps_fix、imu/gps/time/mag_pres error |
| ODrive | axis_state、pos [turn]、vbus、ibus |
| joy | `axes[1]` / `axes[2]` 生値 |
| rpm → m/s | 左右の実測 m/s |

トピック名・デコード設定は yaml（§8）だけが正本。Panel は自前のトピック名やパスを持たず、`.rviz` にも保存しない
（rsf `RsfStatusPanel` の `load/save` は踏襲しない）。

## 8. 設定ファイル `config/vehicle_decode.yaml`

左右の対応・符号・スケール・トピック名は **ここだけ** に書く。パスの決まり方（優先順）:

1. Display（`ManualDashboardOverlay`）のプロパティ `Decode Config`（`.rviz` に保存される）
2. 環境変数 `AIF_DEBUG_DECODE_CONFIG`（launch の `decode_config` 引数はこれを rviz2 プロセスに渡す。rviz2 は launch 引数をプラグインへ直接渡せないため）
3. パッケージ share の `config/vehicle_decode.yaml`

Panel はパスを持たない。すでに `VehicleState` が作られていればそれを共有し、無ければ 2 → 3 の順で決めて作る（§9）。

```yaml
topics:
  cmd_vel: /cmd_vel
  joy: /joy
  autonomous: /autonomous
  can_tx: /can_tx
  can_rx_prefix: /can_rx_        # + "%03X"
  caster_data: /caster_data
  velocity_body: /vectornav/velocity_body
  ins_status: /vectornav/raw/common   # InsStatus を持つ CommonGroup

vehicle:
  wheel_radius: 0.124            # [m] rpm → m/s
  linear_max_vel: 3.0            # [m/s] バーのフルスケール
  steering_max_deg: 15.0

can:
  drive_cmd:   {id: 0x210, right: {offset: 0, type: int32}, left: {offset: 4, type: int32}}
  drive_meas:  {id: 0x711, right: {offset: 0, type: int32}, left: {offset: 4, type: int32}}
  caster_angle:{id: 0x012, offset: 0, type: int16, counts_per_rev: 16384, zero_offset_deg: -86.24}
  caster_rot:  {id: 0x013, offset: 0, type: int16, counts_per_rev: 16384, gear_ratio: 0.5909, sign: -1}
  estop:       {id: 0x712, byte: 6}
  odrive:      {node_id: 24}     # heartbeat 0x301 / encoder 0x309 / bus 0x317

signs:
  yaw_rate: -1                   # velocity_body.angular.z を ROS 慣例（左 +）に揃える
  steering_cmd: 1
  caster_meas: 1

chain:
  min_speed: 0.3                 # [m/s] ゲート
  min_steer_deg: 2.0
  window_sec: 1.0
  good_ratio: 0.7
  bad_ratio: 0.3
  deadband: {rpm_diff: 5, caster_deg: 0.5, yaw_rate: 0.02}   # 不感帯内は分母に入れない

freshness:
  stale_sec: 2.0
  expected_hz: {cmd_vel: 20, joy: 20, drive_cmd: 500, drive_meas: 300, caster_data: 500,
                caster_angle: 100, velocity_body: 20, ins_status: 20}
```

## 9. ソフトウェア構成

```
aif_debug_visualizer/
├── package.xml / CMakeLists.txt      # ament_cmake、1 つの SHARED ライブラリに全プラグイン
├── plugin_description.xml            # rviz_common::Display ×1、rviz_common::Panel ×1
├── include/aif_debug_visualizer/
│   ├── decode_config.hpp             # yaml → 構造体
│   ├── can_decode.hpp                # 純粋関数: フレーム → 右/左 rpm、舵角、E-stop、ODrive
│   ├── turn_chain.hpp                # 純粋ロジック: 符号履歴 → 段ごとの矢印と色
│   ├── freshness.hpp                 # 受信時刻リング → Hz / stale
│   ├── vehicle_state.hpp             # 購読 + デコード + 最新値（mutex）。プロセス内共有
│   ├── manual_dashboard_overlay.hpp  # Display
│   └── manual_dashboard_panel.hpp    # Panel
├── src/ ...
├── config/vehicle_decode.yaml
├── config/dashboard.rviz             # Grid + Overlay + Panel（右ドック）、Fixed Frame = base_link
├── launch/dashboard.launch.py        # rviz2 -d dashboard.rviz。引数: rviz_config, decode_config, use_sim_time
├── assets/font/Quicksand/            # OFL
├── deps/socketcan_interface_msg/     # 同梱コピー + COLCON_IGNORE + README
├── test/                             # gtest（§11）
└── docs/superpowers/specs/           # 本書
```

**設計上の決め**

- `VehicleState` は **yaml パスごとに 1 つ**（registry のキーは解決済みの絶対パス。`std::weak_ptr` を静的に持ち `shared_ptr` を返す）。
  通常は Display と Panel が同じ 1 つを共有し、どちらかが生きている間だけ購読が続く。Display の `Decode Config` を変えると
  Display は新しい `VehicleState` を取り直し、Panel は「registry に残っている最新の 1 つ」を追従する（Panel は 1 秒周期の
  `QTimer` で registry を見に行く）。ノードは `context_->getRosNodeAbstraction().lock()->get_raw_node()`
- 購読 QoS は `rclcpp::SensorDataQoS()`（bag の `/can_rx_*` は reliability=1 / durability=2 = BEST_EFFORT / VOLATILE。`/cmd_vel` 等 RELIABLE の publisher とも互換）
- `can_decode` / `turn_chain` / `freshness` は ROS・Qt・Ogre に依存しない純粋関数・クラスにして gtest で固める。描画は手動確認のみ
- 時刻は `std::chrono::steady_clock`（壁時計）。bag 再生でも `--clock` 不要で鮮度判定が動く。`use_sim_time` は rviz 自身の TF 用に launch 引数で渡せるだけ
- 既存 rviz 設定（`formula_cfg.rviz`）に Display と Panel を足せば自律走行画面にも同居できる（やるかは利用者の判断）

## 10. 別件として報告する不具合（本リポジトリでは直さない）

1. dell_ws `chassis_driver/src/debug_printer.cpp:37-66`: 0x711 / 0x210 の左右が逆（b0..3 を左として読んでいる）
2. aiformula `vectornav/vn_sensor_msgs.cc`: `/vectornav/velocity_body.angular.z` がセンサ座標のまま（右旋回 +）。`ekf_localizer`、`pose_estimater`、`local_planner` など `angular.z` の利用側に符号の前提を確認する必要がある
3. dell_ws `controller_node.hpp` の Buttons enum 名とコメント、`system_check.sh` の「Share で自律切替」の記述が実装（View ボタン）と食い違う

## 11. テスト（TDD）

`ament_cmake_gtest`。先にテストを書いて失敗を確認してから実装する。

- `test_can_decode.cpp`: 0x210/0x711 の右→左（既定）と yaml で入れ替えた場合、負値、0x012 のオフセットと正規化（−π〜π）、0x013 の unwrap（16384 跨ぎ）と符号、0x712 byte6、ODrive heartbeat/encoder/bus の float、DLC 不足フレームの無視
- `test_turn_chain.cpp`: 判定条件未満で `─`、左指令中に全段一致で緑、PID 由来の瞬間反転（一致率 0.79）が黄〜緑に収まること、IMU 段が `yaw_sign=+1` のとき赤・`−1` のとき緑、E-stop で `—`、
  常に 0 の段（死んだ舵角センサ）が赤ではなく灰 `─` になること、stale の段が灰になること、ゲート外のサンプルが窓に入らないこと
- `test_freshness.cpp`: 1 秒窓の Hz、期待の 50 % 未満で warn、2 秒未受信で stale
- `test_decode_config.cpp`: yaml 読み込みの既定値と必須キー欠落時のエラー

## 12. 検証手順

1. ビルド: `cd ~/aif_debug_ws && colcon build --symlink-install`（underlay は `~/formula_ws/install`）。`colcon test` で gtest が通ること
2. **bag 再生（開発 PC）**: `ros2 bag play /media/kyo/24B46936B4690B9C/aif_bags/4_0ms_1` と
   `ros2 launch aif_debug_visualizer dashboard.launch.py` を並走。期待:
   - 左右 rpm のバーと数値が走行に追従し、左旋回中は R > L
   - 鎖の合格基準: `yaw_sign=−1`（既定）で再生全体を通して **段 2〜5 に赤が一度も出ない**こと。段 2（rpm tx）が黄になる時間帯は許容、
     段 3〜5 は判定中の大半で緑。yaml を `+1` にすると IMU 段だけ赤になること
   - 上の基準に入らないほど段 2 の黄が長い場合は、`chain.window_sec` / `good_ratio` を bag で調整してよい（コードではなく yaml で）
   - `raw/common`（INS）と `0x013` の行は灰色（未記録）、それ以外は Hz が期待値付近
   - `control_20261012_155226` でも同様（右旋回区間が少ないので左旋回で確認）
3. **実機（Jetson）**: `ros2 interface show socketcan_interface_msg/msg/SocketcanIF` が通ることを確認してから clone・ビルド。通らなければ `deps/` の `COLCON_IGNORE` を外す。手動操縦で左右に切って鎖が緑のまま追従することを確認

## 13. builder に委ねる点

- オーバーレイの具体的なフォントサイズ・余白・テクスチャ寸法（rsf の 1600×100 を参考に、6 要素に合わせて決める）
- rviz プロパティの名前と並び
- Hz 推定の実装（リングバッファ長など）
- `deps/socketcan_interface_msg` の README 文面
- CMake の細部（`CMAKE_AUTOMOC`、`pluginlib_export_plugin_description_file`、assets/config/launch の install）
