# aif_debug_visualizer

AI Formula 車両の手動運転時に、操作入力・左右輪 rpm・舵角・ヨーレートの対応を rviz2 上で確認するダッシュボード。
画面上部の速度・rpm バー・舵角表示と、操舵から旋回までの因果の鎖（段 1〜5）を表示し、右ドックのパネルに各トピックの受信状況と生値を出す。

## ビルド

```bash
cd ~/aif_debug_ws && source /opt/ros/humble/setup.bash && source ~/formula_ws/install/setup.bash
colcon build --symlink-install --packages-select aif_debug_visualizer
```

Jetson の formula_ws に `socketcan_interface_msg` が無い場合は `deps/socketcan_interface_msg/README.md` を参照する。

## 起動

```bash
source ~/aif_debug_ws/install/setup.bash
ros2 launch aif_debug_visualizer debug_printer_rviz.launch.py [decode_config:=/path.yaml]
```

`decode_config` を省略すると `config/vehicle_decode.yaml` を使う。環境変数 `AIF_DEBUG_DECODE_CONFIG` でも指定できる。

## bag 再生での確認

ターミナル A:

```bash
source /opt/ros/humble/setup.bash && source ~/formula_ws/install/setup.bash
export ROS_DOMAIN_ID=87
ros2 bag play /media/kyo/24B46936B4690B9C/aif_bags/4_0ms_1
```

ターミナル B:

```bash
source /opt/ros/humble/setup.bash && source ~/aif_debug_ws/install/setup.bash
export ROS_DOMAIN_ID=87
ros2 launch aif_debug_visualizer debug_printer_rviz.launch.py
```

- 上中央の速度が走行に追従し、左右 rpm のバー・数値が動く。左旋回中は R > L
- 鎖の段 2〜5 に赤が出ない（段 2 の黄は許容）
- パネルで `ins_status` と `0x013 rot` が灰 `—`、他は期待 Hz 付近
- `yaw_rate: 1` にした yaml を `decode_config:=` で渡すと IMU 段だけ赤になる

## config/vehicle_decode.yaml

- `topics`: 購読するトピック名
- `vehicle`: 輪半径 `wheel_radius`、速度上限 `linear_max_vel`
- `can`: 各 CAN ID のバイト位置。0x210 / 0x711 は `right` が byte0-3、`left` が byte4-7（int32 リトルエンディアン、rpm）。左右を入れ替えるときは `offset` を交換する
- `signs`: 符号規約。舵角・ヨーレートは左旋回が正。`yaw_rate` は `velocity_body.angular.z` に掛ける係数（既定 -1）で、鎖の IMU 段が赤になる場合は符号を反転する
- `chain`: 鎖の判定。`min_speed` 未満・`min_steer_deg` 未満では判定しない。`window_sec` 内の一致率が `good_ratio` 超で緑、`bad_ratio` 未満で赤、`deadband` 内の差は無視する
- `freshness`: `stale_sec` 以上受信が無いと灰 `—`。`expected_hz` は各トピックの期待周波数

## レイアウトの保存

rviz 上で ManualDashboardPanel を右にドッキングし、`File → Save Config` で `config/dashboard.rviz` を上書きする。
