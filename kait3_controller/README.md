# kait3_controller

`ros2_control` フレームワーク上で動作するカスタムコントローラーパッケージです。
現在は主に、外部からの非常停止信号及び通信タイムアウトを監視しハードウェアへ安全停止命令を送る `EmergencyStopController` を提供しています。

## 主な機能

### `EmergencyStopController` (`kait3_controller/EmergencyStopController`)

* **ハートビート監視**: micro-ROS等の外部ノードから非常停止状態 (`std_msgs/msg/Bool`) を受信します。メッセージの未受信時や指定時間以上の通信途絶発生時、及び非常停止信号 (`true`) の受信時に自動的にハードウェアの緊急停止コマンドインターフェイスをアクティブ (`1.0`) にします。
* **イベント通知**: 上位のノード向け (state machine など) に非常停止の作動・解除や通信タイムアウト・復帰の状態遷移 (エッジ) を検知し、`std_msgs/msg/Empty` トピックを配信します。

## 動作要件

* Ubuntu 24.04 LTS
* ROS2 Jazzy

## インターフェイス

### Subscribed Topic (入力)

| トピック | メッセージ型 | 説明 |
| :--- | :--- | :--- |
| `emergency_stop` | `std_msgs/msg/Bool` | 常に配信される緊急停止状態 (`true`: 非常停止作動 / `false`: 正常) を受信 ※1 |

※1 QoS (Reliability: `best effort`, Depth: `1`)

### Published Topics (出力)

| トピック | メッセージ型 | 説明 |
| :--- | :--- | :--- |
| `~/emergency_stop/activated` | `std_msgs/msg/Empty` | 非常停止が作動した瞬間 (`false` -> `true`) ※2 |
| `~/emergency_stop/deactivated` | `std_msgs/msg/Empty` | 非常停止が解除された瞬間 (`true` -> `false`) ※2 |
| `~/communication/timed_out` | `std_msgs/msg/Empty` | 定期配信の受信が途絶え、通信タイムアウトが発生した瞬間 ※2 |
| `~/communication/recovered` | `std_msgs/msg/Empty` | タイムアウト状態から定期配信の受信が復帰した瞬間 ※2 |

※2 `enable_notifications: true` の場合のみ生成されます。QoS (Reliability: `reliable`, Depth: `1`)

### Command Interfaces (ros2_control)

| インターフェース名 | 出力値 | 説明 |
| :--- | :--- | :--- |
| `emergency_stop/active` | `1.0` / `0.0` | `1.0`: 非常停止作動、通信タイムアウト、または未受信 / `0.0`: 正常稼働（正常通信かつ非常停止解除） |

## パラメータ一覧

| パラメータ名 | 型 | デフォルト値 |
| :--- | :--- | :--- |
| `emergency_stop_topic_name` | `string` | `emergency_stop` |
| `communication_timeout` | `double` | `0.5` |
| `enable_notifications` | `bool` | `true` |

## 設定例

コントローラーを使用するには、`ros2_control` の設定ファイルとURDFの両方に記述が必要です。

### コントローラー設定

```yaml
controller_manager:
  ros__parameters:
    # Other controllers are defined here...

    emergency_stop_controller:
      type: kait3_controller/EmergencyStopController

emergency_stop_controller:
  ros__parameters:
    emergency_stop_topic_name: "emergency_stop"
    communication_timeout: 0.5
    enable_notifications: true
```

### URDF

ハードウェアインターフェイス側で `emergency_stop/active` を公開するため、URDFの `<ros2_control>` ブロック内に `<gpio>` を定義します。

```xml
<ros2_control name="<your_system_name>" type="system" is_async="false">
  <hardware>
    <!-- hardware plugin is defined here... -->
  </hardware>

  <!-- other joints / interfaces are defined here... -->

  <gpio name="emergency_stop">
    <command_interface name="active"/>
  </gpio>
</ros2_control>
```

