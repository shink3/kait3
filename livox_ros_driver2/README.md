# livox_ros_driver2

Livox ROS Driver 2 は、Livox製のLiDAR製品を接続するための第2世代ドライバパッケージです。

本パッケージはオリジナルの Livox ROS Driver 2 をベースに、kait3 で使用するために改変を加えたものです。

## 主な変更点

オリジナル版からの主な変更点は以下の通りです。

- **JSON設定ファイルから `lidar_frame_id` を設定可能にしました。**
- **JSON設定ファイルから `imu_frame_id` を設定可能にしました。**
- **JSON設定ファイルから `lidar_topic_name` を設定可能にしました。**
- **JSON設定ファイルから `imu_topic_name` を設定可能にしました。**
- **ドライバ設定用の `multi_MID360s_config.json` を追加しました。**
- **ドライバ起動用の `MID360s.launch.py` を追加しました。**

## 動作要件

本パッケージはオリジナルの Livox ROS Driver 2 をベースにしています。

オリジナル版のドライバについては、[オリジナルのREADME](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/README.md)を参照してください。

本改変版は現在、以下の環境で開発および動作確認を行っています。

- **Ubuntu 24.04 LTS**
- **ROS 2 Jazzy**
- **Livox Mid-360S**

## ビルドと実行

一般的なビルドおよび実行手順については、オリジナルREADMEの [Build & Run Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2#2-build--run-livox-ros-driver-2) セクションを参照してください。

### ビルド

本パッケージは `colcon` または付属の `build.sh` スクリプトを使用してビルドできます。

#### colcon を使用する場合

```bash
cd ~/ros2_ws

colcon build --symlink-install --packages-select livox_ros_driver2
```

#### build.sh を使用する場合

```bash
cd ~/ros2_ws/src/livox_ros_driver2

./build.sh jazzy
```

### 実行

kait3 用の Livox ROS Driver 2 を起動します。

```bash
ros2 launch livox_ros_driver2 MID360s.launch.py
```

このlaunchファイルでは、以下の起動引数（launch arguments）を指定できます。

| 引数 | デフォルト値 | 説明 |
| :--- | :--- | :--- |
| `xfer_format` | `0` | 点群の出力フォーマット。`0`: PointCloud2 (`PointXYZRTL`)、`1`: カスタム点群フォーマット |
| `multi_topic` | `1` | トピック構成。`0`: 全LiDARで同じトピックを共有、`1`: LiDARごとに個別のトピックを使用 |
| `publish_freq` | `10.0` | 点群の配信周波数（Hz） |
| `config_file` | `config/multi_MID360s_config.json` | パッケージの share ディレクトリからの相対パスによるLivox設定JSONファイルのパス |
| `rviz` | `false` | RViz2 を同時に起動するかどうか |
| `rviz_config_file` | `config/display_point_cloud.rviz` | パッケージの share ディレクトリからの相対パスによるRViz設定ファイルのパス |

例：RViz2 を同時に起動する場合

```bash
ros2 launch livox_ros_driver2 MID360s.launch.py rviz:=true
```

配信周波数を変更する場合

```bash
ros2 launch livox_ros_driver2 MID360s.launch.py publish_freq:=20.0
```

利用可能なすべての起動引数を表示する場合

```bash
ros2 launch livox_ros_driver2 MID360s.launch.py --show-args
```

## 4. LiDAR 設定（LiDAR config）

以下のJSONファイルでは、LiDARデバイスのネットワーク設定、ROS 2トピック、フレームID、および外部パラメータ（extrinsic parameters）を定義します。

```jsonc
{
  "lidar_summary_info": {
    "lidar_type": 8                                 // LiDARのタイプ
  },
  "Mid360s": {
    "lidar_net_info": {
      "cmd_data_port": 56100,                       // LiDAR コマンドポート
      "push_msg_port": 56200,                       // LiDAR メッセージポート
      "point_data_port": 56300,                     // LiDAR 点群データポート
      "imu_data_port": 56400,                       // LiDAR IMUデータポート
      "log_data_port": 56500                        // LiDAR ログデータポート
    },
    "host_net_info": [
      {
        "host_ip": "192.168.1.xxx",                 // ホストPCのIPアドレス
        "cmd_data_port": 56101,                     // ホスト コマンドポート
        "push_msg_port": 56201,                     // ホスト メッセージポート
        "point_data_port": 56301,                   // ホスト 点群データポート
        "imu_data_port": 56401,                     // ホスト IMUデータポート
        "log_data_port": 56501                      // ホスト ログデータポート
      }
    ]
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.1xx",                         // LiDARのIPアドレス
      "pcl_data_type": 1,                            // 点群データタイプ
      "pattern_mode": 0,                             // スキャンパターンモード
      "lidar_topic_name": "/livox/front/pointcloud", // LiDAR点群トピック名
      "imu_topic_name": "/livox/front/imu",          // IMUトピック名
      "lidar_frame_id": "front_lidar_link",          // LiDARのTFフレームID
      "imu_frame_id": "front_imu_link",              // IMUのTFフレームID
      "extrinsic_parameter": {
        "roll": 0.0,                                 // X軸周りの回転
        "pitch": 0.0,                                // Y軸周りの回転
        "yaw": 0.0,                                  // Z軸周りの回転
        "x": 0,                                      // X軸方向の並進
        "y": 0,                                      // Y軸方向の並進
        "z": 0                                       // Z軸方向の並進
      }
    },
    {
      "ip": "192.168.1.1xx",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "lidar_topic_name": "/livox/rear/pointcloud",
      "imu_topic_name": "/livox/rear/imu",
      "lidar_frame_id": "rear_lidar_link",
      "imu_frame_id": "rear_imu_link",
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

