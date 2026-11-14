# Kait3 RoboClaw Hardware

A `ros2_control` hardware interface implementation for the Kait3 differential-drive robot using the Basicmicro RoboClaw V5 motor controller.

This package has been tested with **ROS 2 Jazzy**.

## Requirements

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Basicmicro RoboClaw V5
- A compatible serial connection to the RoboClaw

## Hardware Parameters

The hardware interface accepts the following parameters through the `<ros2_control>` configuration.

### Serial Communication

| Parameter | Type | Default | Description |
|---|---|---|---|
| `device_path` | `string` | `/dev/ttyUSB0` | Serial device path |
| `baud_rate` | `uint32` | `115200` | Serial communication baud rate |
| `address` | `uint8` | `0x80` | RoboClaw controller address |
| `read_timeout_ms` | `uint32` | `5` | Serial read timeout in milliseconds |
| `write_timeout_ms` | `uint32` | `5` | Serial write timeout in milliseconds |
| `exclusive_access` | `bool` | `false` | Whether to request exclusive access to the serial device |
| `unacknowledged` | `bool` | `false` | Whether to ignore command acknowledgement |
| `max_attempts` | `uint32` | `1` | Maximum number of communication attempts |

### Differential Drive

| Parameter | Type | Default | Description |
|---|---|---|---|
| `wheel_radius` | `double` | `0.231` | Wheel radius in meters |
| `counts_per_revolution` | `uint32` | `2000` | Encoder counts per motor revolution |
| `gear_ratio` | `double` | `22.75` | Gear reduction ratio |
| `motion_strategy` | `string` | `speed_accel` | Motor command strategy: `speed` or `speed_accel` |
| `default_acceleration` | `uint32` | `15700` | Default acceleration used by the `speed_accel` strategy |


### Optional Safety and Motor Configuration

| Parameter | Type | Default | Description |
|---|---|---|---|
| `watchdog_timeout_ms` | `uint32` | Not set | Optional watchdog timeout in milliseconds |
| `m1_max_current` | `double` | Not set | Optional maximum current setting for motor M1 |
| `m2_max_current` | `double` | Not set | Optional maximum current setting for motor M2 |
| `m1_kp` | `double` | Not set | Optional proportional PID gain for motor M1 |
| `m1_ki` | `double` | Not set | Optional integral PID gain for motor M1 |
| `m1_kd` | `double` | Not set | Optional derivative PID gain for motor M1 |
| `m1_qpps` | `uint32` | Not set | Optional maximum speed setting in quadrature pulses per second for motor M1 |
| `m2_kp` | `double` | Not set | Optional proportional PID gain for motor M2 |
| `m2_ki` | `double` | Not set | Optional integral PID gain for motor M2 |
| `m2_kd` | `double` | Not set | Optional derivative PID gain for motor M2 |
| `m2_qpps` | `uint32` | Not set | Optional maximum speed setting in quadrature pulses per second for motor M2 |

Optional parameters are only configured when provided. Their effects depend on the hardware interface implementation.

## Xacro Configuration Example

The following example shows how to configure the hardware interface in a URDF/Xacro file.

```xml
<ros2_control name="RoboClawHardware" type="system">
  <hardware>
    <plugin>kait3_roboclaw_hardware/RoboClawHardware</plugin>

    <!-- Serial communication -->
    <param name="device_path">/dev/ttyUSB0</param>
    <param name="baud_rate">115200</param>
    <param name="address">128</param>
    <param name="read_timeout_ms">5</param>
    <param name="write_timeout_ms">5</param>
    <param name="exclusive_access">false</param>
    <param name="unacknowledged">false</param>
    <param name="max_attempts">1</param>

    <!-- Differential drive -->
    <param name="wheel_radius">0.231</param>
    <param name="counts_per_revolution">2000</param>
    <param name="gear_ratio">22.75</param>
    <param name="motion_strategy">speed_accel</param>
    <param name="default_acceleration">1000</param>

    <!-- Optional watchdog -->
    <!-- <param name="watchdog_timeout_ms">1000</param> -->

    <!-- Optional motor M1 configuration -->
    <!-- <param name="m1_max_current">...</param> -->
    <!-- <param name="m1_kp">...</param> -->
    <!-- <param name="m1_ki">...</param> -->
    <!-- <param name="m1_kd">...</param> -->
    <!-- <param name="m1_qpps">...</param> -->

    <!-- Optional motor M2 configuration -->
    <!-- <param name="m2_max_current">...</param> -->
    <!-- <param name="m2_kp">...</param> -->
    <!-- <param name="m2_ki">...</param> -->
    <!-- <param name="m2_kd">...</param> -->
    <!-- <param name="m2_qpps">...</param> -->
  </hardware>

  <!-- Define the robot's joint interfaces here. -->
</ros2_control>
```

## Notes

This markdown may be wrong because it was written by AI.
If you want to know more details, Please read the source code.
