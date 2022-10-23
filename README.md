# ROS2 BNO055 #

ROS2 node for the [Bosch BNO055 IMU](https://www.adafruit.com/product/2472).


## Published Topics ##

| Topic | Type | Description |
| ----- | ---- | ----------- |
| `/bno055/diagnostics` | [diagnostic_msgs/msg/DiagnosticArray](https://docs.ros2.org/foxy/api/diagnostic_msgs/msg/DiagnosticArray.html) | Status of sensor calibration, mode, and axis remapping. |
| `/bno055/imu` | [sensor_msgs/msg/Imu](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html) | Absolute orientation quaternion, angular velocity vector (rad/s), linear acceleration vector (m/s^2) |
| `/bno055/magnetometer` | [sensor_msgs/msg/MagneticField](https://docs.ros2.org/foxy/api/sensor_msgs/msg/MagneticField.html) | Magnetic field strength vector (uT) |
| `/bno055/temperature` | [sensor_msgs/msg/Temperature](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Temperature.html) | Sensor temperature (C) |


## Parameters ##

| Parameter | Type | Default | Description |
| --------- | ---- | ------- | ----------- |
| `interface` | String | `i2c` | BNO055 interface. Can be `i2c` or `uart`. |
| `frame_id` | String | `imu` | Frame ID for published sensor data. |
| `axis_remap` | String | `P1` | Axis remap placement (see section 3.4 of the [datasheet](https://cdn-learn.adafruit.com/assets/assets/000/036/832/original/BST_BNO055_DS000_14.pdf)). Valid values are: `P0`, `P1`, ..., `P7` |
| `temp_update_rate` | Float | `1.0` | How often, in seconds, to publish the temperature. Default is 1 Hz. |
| `imu_update_rate` | Float | `0.1` | How often, in seconds, to publish the IMU data. Default is 10 Hz. |
| `mag_update_rate` | Float | `0.1` | How often, in seconds, to publish the magnetometer data. Default is 10 Hz. |


## Usage ##

1. Create a ROS2 workspace:
   ```
   mkdir -p ~/ws/src && cd ~/ws/src
   ```

2. Clone this repo:
   ```
   git clone https://github.com/process1183/ros2_bno055.git
   ```

3. Install required dependencies:
   ```
   cd ~/ws
   rosdep install --from-paths src -i
   ```

4. Build:
   ```
   colcon build
   ```

5. Run:
   ```
   . ~/ws/install/setup.bash
   ros2 run ros2_bno055 bno055
   ```
