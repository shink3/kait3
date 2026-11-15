# kait3_teleop

Bluetooth 接続された PlayStation 4 用コントローラー DualShock 4 を使用して、ロボットへの速度指令 (`cmd_vel`) の送信、ボタン押下イベントの配信、およびコントローラーへのハードウェアフィードバック（振動・ライトバー制御）を行う ROS 2 パッケージです。

## 主な機能

- **3段階の速度モード切替** 
  低速（Slow）・通常（Normal）・高速（Turbo）の3段階で速度倍率を切り替えます。各モードにイネーブルボタンを割り当てることで、ボタンを押している間だけ速度指令を有効にできます。

- **ボタンイベントの配信** 
  YAML で指定したボタンが押された瞬間（OFF から ON への変化時）に、`std_msgs/msg/Empty` を1回配信します。

- **ハードウェアフィードバック** 
  振動モーターの強度やライトバーの色・点滅を ROS 2 トピックから制御できます。
  
- **実行中の速度倍率調整**
  `ros2 param set` コマンドを使用して、ノードの再起動なしに速度倍率を変更できます。

## 動作要件

- **Ubuntu 24.04 LTS**
- **ROS2 jazzy**
- **DualShock4とのBluetooth接続**

## 事前準備

### DualShock 4 の接続

DualShock 4 を Bluetooth で PC に接続してください。

### udev ルールの設定（推奨）

振動やライトバーの制御機能を使用する場合、DualShock 4 の `hidraw` デバイスへの書き込み権限が必要です。

YAML ファイル（`params/teleop_params.yaml`）で `/dev/hidrawX` を直接指定することもできますが、デバイス番号は接続順によって変わる可能性があります。

そのため、udev ルールを使用して `/dev/dualshock4` という固定名でデバイスにアクセスする方法を推奨します。

```bash
# パッケージのディレクトリに移動してルールをコピー
sudo cp udev/99-dualshock4.rules /etc/udev/rules.d/

# udev ルールを再読み込み
sudo udevadm control --reload-rules
sudo udevadm trigger
```

DualShock 4 を Bluetooth で再接続したあと、シンボリックリンクが作成されたことを確認してください。

```bash
ls -l /dev/dualshock4
```

振動・ライトバー制御を使用しない場合、この設定は不要です。その場合は `feedback.enabled` を `false` にしてください。

## 起動方法

```bash
ros2 launch kait3_teleop teleop_joy.launch.py
```

### Launch 引数

設定ファイルや出力先を変更する場合は、`<引数名>:=<値>` の形式で指定できます。

```bash
ros2 launch kait3_teleop teleop_joy.launch.py \
  cmd_vel:=/robot/cmd_vel \
  deadzone:=0.1
```

| 引数名 | デフォルト値 | 説明 |
| :--- | :--- | :--- |
| `deadzone` | `0.05` | アナログスティックの微小なドリフトやノイズを除去する不感帯のしきい値 |
| `params_file` | `params/teleop_params.yaml` | 読み込むパラメータ YAML ファイルのパス |
| `cmd_vel` | `cmd_vel` | 速度指令の出力先トピック名 |

### 動作確認・パラメータ調整

ノード起動後、別のターミナルから以下のコマンドで動作確認や設定変更を行えます。

#### 速度倍率の変更

```bash
# 通常モード時の並進速度倍率を 0.6 に変更
ros2 param set /ds4_teleop_node scale_linear 0.6
```

#### 振動のテスト

```bash
# 振動強度を 0.5 に設定
ros2 topic pub --once \
  /dualshock4/rumble_intensity \
  std_msgs/msg/Float32 "{data: 0.5}"
```

#### ライトバーの色変更

```bash
# ライトバーを赤色に設定
ros2 topic pub --once \
  /dualshock4/lightbar_color \
  std_msgs/msg/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0, a: 1.0}"
```

#### ライトバーの点滅

```bash
# ライトバーの点滅を有効化
ros2 topic pub --once \
  /dualshock4/enable_flash \
  std_msgs/msg/Bool "{data: true}"
```

点滅を停止する場合は、`data` を `false` に設定してください。

```bash
ros2 topic pub --once \
  /dualshock4/enable_flash \
  std_msgs/msg/Bool "{data: false}"
```

## トピック一覧

### Subscribed Topics（入力）

| トピック名 | メッセージ型 | 説明 |
| :--- | :--- | :--- |
| `joy` | `sensor_msgs/msg/Joy` | `joy_node` から配信されるコントローラー入力 |
| `dualshock4/rumble_intensity` | `std_msgs/msg/Float32` | バイブレーション強度の設定（`0.0` ～ `1.0`）※1 |
| `dualshock4/lightbar_color` | `std_msgs/msg/ColorRGBA` | ライトバーの RGB カラー設定（各成分 `0.0` ～ `1.0`）※1 |
| `dualshock4/enable_flash` | `std_msgs/msg/Bool` | ライトバーの点滅の有効化・無効化 ※1 |

※1 `feedback.enabled: true` の場合のみ有効です。

### Published Topics（出力）

| トピック名 | メッセージ型 | 説明 |
| :--- | :--- | :--- |
| `cmd_vel` | `geometry_msgs/msg/Twist` または `geometry_msgs/msg/TwistStamped` | ロボットへの速度指令（`stamped` パラメータで切替） |
| `button_events/<event_name>` | `std_msgs/msg/Empty` | 指定したボタンが押された瞬間に1回配信されるイベント ※2 |

※2 `button_events.enabled: true` の場合のみ生成されます。

トピック名は ROS 2 の名前解決規則に従います。上記はノードの名前空間が `/` の場合の例です。

## パラメータ一覧

設定ファイルは `params/teleop_params.yaml` です。

### 基本設定・速度制御

| パラメータ名 | 型 | デフォルト値 | 動的変更 | 説明 |
| :--- | :--- | :--- | :--- | :--- |
| `stamped` | `bool` | `false` | 不可 | `true` で `TwistStamped`、`false` で `Twist` を配信 |
| `base_frame_id` | `string` | `"base_link"` | 不可 | `TwistStamped` 使用時の `header.frame_id` |
| `slow_button` | `int` | `-1` | 不可 | 低速モード用イネーブルボタン番号。負数で無効 |
| `normal_button` | `int` | `9` | 不可 | 通常モード用イネーブルボタン番号（L1） |
| `turbo_button` | `int` | `10` | 不可 | 高速モード用イネーブルボタン番号（R1） |
| `axes_linear` | `int` | `1` | 不可 | 前後移動（`linear.x`）に割り当てる軸番号 |
| `axes_angular` | `int` | `2` | 不可 | 旋回（`angular.z`）に割り当てる軸番号 |
| `scale_linear_slow` | `double` | `0.0` | 可 | 低速モード時の並進速度倍率（`>= 0.0`） |
| `scale_angular_slow` | `double` | `0.0` | 可 | 低速モード時の角速度倍率（`>= 0.0`） |
| `scale_linear` | `double` | `0.4` | 可 | 通常モード時の並進速度倍率（`>= 0.0`） |
| `scale_angular` | `double` | `1.0` | 可 | 通常モード時の角速度倍率（`>= 0.0`） |
| `scale_linear_turbo` | `double` | `0.7` | 可 | 高速モード時の並進速度倍率（`>= 0.0`） |
| `scale_angular_turbo` | `double` | `2.0` | 可 | 高速モード時の角速度倍率（`>= 0.0`） |

### フィードバック設定（`feedback`）

| パラメータ名 | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| `feedback.enabled` | `bool` | `true` | フィードバック機能の有効化。`false` の場合、フィードバック関連の設定を読み込まない |
| `feedback.device_path` | `string` | `"/dev/dualshock4"` | DualShock 4 の `hidraw` デバイスパス |
| `feedback.rumble_weak_multiplier` | `double` | `1.0` | 弱振動モーターの強度倍率（`0.0` ～ `1.0`） |
| `feedback.rumble_strong_multiplier` | `double` | `1.0` | 強振動モーターの強度倍率（`0.0` ～ `1.0`） |
| `feedback.flash_bright_duration` | `double` | `0.5` | 点滅時の点灯時間（秒、`0.0` ～ `2.55`） |
| `feedback.flash_dark_duration` | `double` | `0.5` | 点滅時の消灯時間（秒、`0.0` ～ `2.55`） |

### ボタンイベント設定（`button_events`）

| パラメータ名 | 型 | デフォルト値 | 説明 |
| :--- | :--- | :--- | :--- |
| `button_events.enabled` | `bool` | `false` | ボタンイベント機能の有効化。`false` の場合、イベント関連の設定を読み込まない |
| `button_events.event_names` | `string[]` | なし（有効時必須） | 有効にするイベント名のリスト。英数字と `_` のみ使用可能で、先頭に数字は使用不可 |
| `button_events.<name>.button` | `int` | なし（有効時必須） | 各イベントに割り当てるボタン番号（`0` ～ `15`） |

イベント名ごとにボタン番号を設定します。ボタンが押された瞬間に対応するイベントトピックへ `std_msgs/msg/Empty` が1回配信されます。

## 設定例

以下は `params/teleop_params.yaml` の設定例です。

```yaml
ds4_teleop_node:
  ros__parameters:
    stamped: true
    base_frame_id: "base_footprint"

    # Speed mode buttons
    slow_button: -1   # Disabled
    normal_button: 9  # L1
    turbo_button: 10  # R1

    # Joystick axes
    axes_linear: 1    # Left Stick Vertical
    axes_angular: 2   # Right Stick Horizontal

    # Speed scales
    scale_linear_slow: 0.0
    scale_angular_slow: 0.0
    scale_linear: 0.4
    scale_angular: 1.0
    scale_linear_turbo: 0.7
    scale_angular_turbo: 2.0

    # Hardware feedback
    feedback:
      enabled: true
      device_path: "/dev/dualshock4"

      rumble_weak_multiplier: 1.0
      rumble_strong_multiplier: 1.0

      flash_bright_duration: 0.5
      flash_dark_duration: 0.5

    # Button events
    button_events:
      enabled: true
      event_names:
        - "share_button_pressed"
        - "touch_pad_pressed"
        - "options_button_pressed"

      share_button_pressed:
        button: 4

      touch_pad_pressed:
        button: 15

      options_button_pressed:
        button: 6
```

## DualShock 4 のボタン・軸番号

`joy_node` が配信する `sensor_msgs/msg/Joy` の配列インデックスを使用します。

### ボタン番号

| 番号 | ボタン |
| :---: | :--- |
| `0` | ×（Cross） |
| `1` | ○（Circle） |
| `2` | □（Square） |
| `3` | △（Triangle） |
| `4` | SHARE |
| `5` | PS |
| `6` | OPTIONS |
| `7` | L3 |
| `8` | R3 |
| `9` | L1 |
| `10` | R1 |
| `11` | 十字キー上 |
| `12` | 十字キー下 |
| `13` | 十字キー左 |
| `14` | 十字キー右 |
| `15` | タッチパッド |

### 軸番号

| 番号 | 軸 |
| :---: | :--- |
| `0` | 左スティック左右 |
| `1` | 左スティック上下 |
| `2` | 右スティック左右 |
| `3` | 右スティック上下 |
| `4` | L2 トリガー |
| `5` | R2 トリガー |

※ ボタン・軸番号は `joy_node` の設定によって異なる場合があります。実際の入力値は `ros2 topic echo /joy` で確認してください。

## 注意事項
- `slow_button` などの速度モード用ボタンに負数を指定すると、そのモードのイネーブルボタンは無効になります。
- 速度倍率は `0.0` 以上の値を指定してください。
- ボタン番号・軸番号は、使用するコントローラーと `joy_node` の設定に合わせてください。
- `cmd_vel` のメッセージ型は `stamped` パラメータによって変わります。受信側ノードと一致させてください。
- 実機で使用する前に、低速設定で動作確認を行ってください。
