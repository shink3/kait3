# kait3_roboclaw_hardware

Basicmicro RoboClaw V5 モータードライバを使用して、対向二輪（差動二輪）ロボット を制御するための `ros2_control` ハードウェアインターフェース実装です。

## 動作要件

- **Ubuntu 24.04 LTS**
- **ROS 2 Jazzy**
- **Basicmicro RoboClaw V5**
- **RoboClaw との適切なシリアル通信接続**

## udev ルールの設定

RoboClaw を USB シリアル経由で PC に直接接続する場合、OS のデフォルト設定では USB のレイテンシタイマー（latency timer）が働き、シリアル通信に遅延が発生します。
通信の遅延を防ぐためにudev ルールを適用することを推奨します。

```bash
sudo cp udev/99-roboclaw.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

※ ルール適用後、RoboClaw の USB ケーブルを一度抜き差しして設定を反映させてください。

## ハードウェアパラメータ

このハードウェアインターフェースは、`<ros2_control>` の設定を通じて以下のパラメータを受け取ります。

### シリアル通信

| パラメータ | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `device_path` | `string` | `/dev/ttyUSB0` | シリアルデバイスのパス |
| `baud_rate` | `uint32` | `115200` | シリアル通信のボーレート |
| `address` | `uint8` | `0x80` | RoboClaw コントローラのアドレス |
| `read_timeout_ms` | `uint32` | `5` | シリアル読み込みのタイムアウト時間（ミリ秒） |
| `write_timeout_ms` | `uint32` | `5` | シリアル書き込みのタイムアウト時間（ミリ秒） |
| `exclusive_access` | `bool` | `false` | シリアルデバイスへの排他アクセスを要求するかどうか |
| `unacknowledged` | `bool` | `false` | コマンドの応答確認（ACK）を無視するかどうか |
| `max_attempts` | `uint32` | `1` | 通信の最大試行回数 |

### 対向二輪制御

| パラメータ | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `wheel_radius` | `double` | `0.231` | 車輪の半径（メートル） |
| `counts_per_revolution` | `uint32` | `2000` | モーター1回転あたりのエンコーダカウント数 (4逓倍) |　
| `gear_ratio` | `double` | `22.75` | ギア減速比 |
| `motion_strategy` | `string` | `speed_accel` | モーター制御方式（`speed` または `speed_accel`） |
| `default_acceleration` | `uint32` | `15700` | `speed_accel` 方式で使用されるデフォルト加速度 |

### オプションの安全設定・モーター設定

| パラメータ | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `watchdog_timeout_ms` | `uint32` | 未設定 | ウォッチドッグタイマーのタイムアウト時間（ミリ秒） |
| `m1_max_current` | `double` | 未設定 | モーター M1 の最大電流値 |
| `m2_max_current` | `double` | 未設定 | モーター M2 の最大電流値 |
| `m1_kp` | `double` | 未設定 | モーター M1 の PID 比例ゲイン（Kp） |
| `m1_ki` | `double` | 未設定 | モーター M1 の PID 積分ゲイン（Ki） |
| `m1_kd` | `double` | 未設定 | モーター M1 の PID 微分ゲイン（Kd） |
| `m1_qpps` | `uint32` | 未設定 | モーター M1 の最大速度（QPPS: 1秒あたりの4逓倍パルス数） |
| `m2_kp` | `double` | 未設定 | モーター M2 の PID 比例ゲイン（Kp） |
| `m2_ki` | `double` | 未設定 | モーター M2 の PID 積分ゲイン（Ki） |
| `m2_kd` | `double` | 未設定 | モーター M2 の PID 微分ゲイン（Kd） |
| `m2_qpps` | `uint32` | 未設定 | モーター M2 の最大速度（QPPS: 1秒あたりの4逓倍パルス数） |

※ オプションパラメータは指定された場合のみ設定が反映されます。詳細な挙動はハードウェアインターフェースの実装に依存します。

## Xacro 設定例

URDF / Xacro ファイルでこのハードウェアインターフェースを設定する記述例は以下の通りです。

```xml
<ros2_control name="RoboClawSystem" type="system">
  <hardware>
    <plugin>kait3_roboclaw_hardware/RoboClawHardware</plugin>

    <!-- Serial configurations -->
    <param name="device_path">/dev/ttyUSB0</param>
    <param name="baud_rate">115200</param>
    <param name="address">128</param>
    <param name="read_timeout_ms">5</param>
    <param name="write_timeout_ms">5</param>
    <param name="exclusive_access">false</param>
    <param name="unacknowledged">false</param>
    <param name="max_attempts">1</param>

    <!-- diff-drive configurations -->
    <param name="wheel_radius">0.231</param>
    <param name="counts_per_revolution">2000</param>
    <param name="gear_ratio">22.75</param>
    <param name="motion_strategy">speed_accel</param>
    <param name="default_acceleration">15700</param>

    <!-- Options：watchdog configuration -->
    <!-- <param name="watchdog_timeout_ms">1000</param> -->

    <!-- Option：motor 1 configurations -->
    <!-- <param name="m1_max_current">...</param> -->
    <!-- <param name="m1_kp">...</param> -->
    <!-- <param name="m1_ki">...</param> -->
    <!-- <param name="m1_kd">...</param> -->
    <!-- <param name="m1_qpps">...</param> -->
	
    <!-- Option：motor 2 configurations -->
    <!-- <param name="m2_max_current">...</param> -->
    <!-- <param name="m2_kp">...</param> -->
    <!-- <param name="m2_ki">...</param> -->
    <!-- <param name="m2_kd">...</param> -->
    <!-- <param name="m2_qpps">...</param> -->
  </hardware>

  <!-- Define the robot's joint interfaces (command_interface / state_interface) here -->

</ros2_control>
```

