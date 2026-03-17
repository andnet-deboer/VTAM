# VTAM Firmware

Firmware for the QT Py microcontroller that streams eFlesh tactile sensor data to the robot over USB serial.

## Credits

The eFlesh board code (`5x_eflesh_stream.ino`) is adapted from the [eFlesh project](https://github.com/notvenky/eFlesh) by Venkatesh P, licensed under the Beerware license. The MLX90393 magnetometer library is based on [arduino-MLX90393](https://github.com/tedyapo/arduino-MLX90393) by Theodore Yapo (maintained by Tess Hellebrekers). The sensor design draws from [ReSkin](https://reskin.dev) and [AnySkin](https://any-skin.github.io).

## VTAM Modification

This firmware adds a **recording trigger button on pin A3** (active-low, internal pull-up). Pressing the button signals the start/stop of a demonstration episode, replacing the need to call the ROS2 service manually during teleoperation. This is the primary change from the upstream eFlesh firmware.

## Contents

- `5x_eflesh_stream.ino/` — streams 5 MLX90393 magnetometer sensors (15D per gripper finger) at ~100 Hz over USB serial; includes the A3 button trigger
- `arduino-MLX90393/` — required MLX90393 library

## Arduino Setup

1. Install the **Adafruit_LSM6DS** library via Arduino IDE > Sketch > Include Library > Manage Libraries
2. Install the local `arduino-MLX90393` library (Sketch > Include Library > Add .ZIP Library)
3. Flash `5x_eflesh_stream.ino` to the QT Py board

## ROS2 IMU Tools (optional)

```bash
sudo apt install ros-$ROS_DISTRO-imu-tools
sudo apt install ros-humble-imu-filter-madgwick
```