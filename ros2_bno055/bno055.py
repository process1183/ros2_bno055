"""
ROS2 node for the Bosch BNO055 IMU
https://github.com/process1183/ros2_bno055
Copyright (C) 2022  Josh Gadeken

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""
import math

import rclpy
import rclpy.node
from rclpy.exceptions import InvalidParameterValueException
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu, MagneticField, Temperature
from tf_transformations import euler_from_quaternion

import board  # Adafruit Blinka
import adafruit_bno055  # https://pypi.org/project/adafruit-circuitpython-bno055/

# Covariance matrices copied from Sam 'bitmorse' Sulaimanov's BNO055 driver work:
# https://github.com/Octanis1/bosch_imu_driver/commit/d1132e27ecff46a63c128f7ecacc245c98b2811a

ORIENTATION_COVARIANCE = [
    0.0159, 0.0,    0.0,
    0.0,    0.0159, 0.0,
    0.0,    0.0,    0.0159
]

ANGULAR_VELOCITY_COVARIANCE = [
    0.04, 0.0,  0.0,
    0.0,  0.04, 0.0,
    0.0,  0.0,  0.04
]

LINEAR_ACCELERATION_COVARIANCE = [
    0.017, 0.0,   0.0,
    0.0,   0.017, 0.0,
    0.0,   0.0,   0.017
]

# BNO055 Data sheet, page 25
AXIS_REMAP_PARAMS = {
    # "placement": (AXIS_REMAP_CONFIG, AXIS_REMAP_SIGN)
    "P0": (0x21, 0x04),
    "P1": (0x24, 0x00),
    "P2": (0x24, 0x06),
    "P3": (0x21, 0x02),
    "P4": (0x24, 0x03),
    "P5": (0x21, 0x01),
    "P6": (0x21, 0x07),
    "P7": (0x24, 0x05),
}

OP_MODES = {
    adafruit_bno055.CONFIG_MODE: "CONFIG_MODE",
    adafruit_bno055.ACCONLY_MODE: "ACCONLY_MODE",
    adafruit_bno055.MAGONLY_MODE: "MAGONLY_MODE",
    adafruit_bno055.GYRONLY_MODE: "GYRONLY_MODE",
    adafruit_bno055.ACCMAG_MODE: "ACCMAG_MODE",
    adafruit_bno055.ACCGYRO_MODE: "ACCGYRO_MODE",
    adafruit_bno055.MAGGYRO_MODE: "MAGGYRO_MODE",
    adafruit_bno055.AMG_MODE: "AMG_MODE",
    adafruit_bno055.IMUPLUS_MODE: "IMUPLUS_MODE",
    adafruit_bno055.COMPASS_MODE: "COMPASS_MODE",
    adafruit_bno055.M4G_MODE: "M4G_MODE",
    adafruit_bno055.NDOF_FMC_OFF_MODE: "NDOF_FMC_OFF_MODE",
    adafruit_bno055.NDOF_MODE: "NDOF_MODE"
}


def axis_remap_translate(placement: str) -> tuple:
    """
    Convert an axis remap placement (e.g. 'P0', 'P3' to a remap configuration tuple
    suitable for passing to the axis_remap() method of adafruit_bno055.BNO055.
    """
    arc, ars = AXIS_REMAP_PARAMS[placement]
    x = arc & 0x03
    y = (arc >> 2) & 0x03
    z = (arc >> 4) & 0x03
    x_sign = (ars >> 2) & 0x01
    y_sign = (ars >> 1) & 0x01
    z_sign = ars & 0x01

    return (x, y, z, x_sign, y_sign, z_sign)


class BNO055Pub(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__("bno055", namespace="bno055")

        self.declare_parameters("", (
            ("interface", "i2c"),
            ("frame_id", "imu"),
            ("axis_remap", "P1"),
            ("temp_update_rate", 1.0),  # 1 Hz
            ("imu_update_rate", 0.1),  # 10 Hz
            ("mag_update_rate", 0.1),  # 10 Hz
        ))

        bno055_interface = self.get_parameter("interface").get_parameter_value().string_value.lower()
        if bno055_interface == "i2c":
            i2c = board.I2C()
            self.bno055 = adafruit_bno055.BNO055_I2C(i2c)
        elif bno055_interface == "uart":
            uart = board.UART()
            self.bno055 = adafruit_bno055.BNO055_UART(uart)
        else:
            raise InvalidParameterValueException

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        axis_remap = self.get_parameter("axis_remap").get_parameter_value().string_value.upper()
        if axis_remap not in AXIS_REMAP_PARAMS:
            raise InvalidParameterValueException
        self.bno055.axis_remap = axis_remap_translate(axis_remap)

        self._temp_pub = self.create_publisher(Temperature, "temperature", 10)
        temp_rate = self.get_parameter("temp_update_rate").get_parameter_value().double_value
        self._temp_timer = self.create_timer(temp_rate, self.temp_timer_callback)

        self._imu_pub = self.create_publisher(Imu, "imu", 10)
        imu_rate = self.get_parameter("imu_update_rate").get_parameter_value().double_value
        self._imu_timer = self.create_timer(imu_rate, self.imu_timer_callback)

        self._mag_pub = self.create_publisher(MagneticField, "magnetometer", 10)
        mag_rate = self.get_parameter("mag_update_rate").get_parameter_value().double_value
        self._mag_timer = self.create_timer(mag_rate, self.mag_timer_callback)

        self._diag_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self._diag_timer = self.create_timer(1, self.diag_timer_callback)

    def temp_timer_callback(self) -> None:
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.temperature = float(self.bno055.temperature)
        self._temp_pub.publish(msg)

    def imu_timer_callback(self) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        bno055_quaternion = self.bno055.quaternion
        bno055_gyro = self.bno055.gyro
        bno055_linear_accel = self.bno055.linear_acceleration
        # Convert to Euler angles (in radians)
        yaw, pitch, roll = euler_from_quaternion(bno055_quaternion)            
        
        print(f"roll: {roll}, pitch: {pitch}, yaw: {yaw}")  
        # Set the orientation in the message
        
        for i, a in enumerate(['x', 'y', 'z', 'w']):
            setattr(msg.orientation, a, bno055_quaternion[i])

        msg.orientation_covariance = ORIENTATION_COVARIANCE

        for i, a in enumerate(['x', 'y', 'z']):
            # Gyro data from IMU is in degrees/sec, but needs to be converted to rad/sec
            radps = math.radians(bno055_gyro[i])
            setattr(msg.angular_velocity, a, radps)

        msg.angular_velocity_covariance = ANGULAR_VELOCITY_COVARIANCE

        for i, a in enumerate(['x', 'y', 'z']):
            setattr(msg.linear_acceleration, a, bno055_linear_accel[i])

        msg.linear_acceleration_covariance = LINEAR_ACCELERATION_COVARIANCE

        self._imu_pub.publish(msg)

    def mag_timer_callback(self) -> None:
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        bno055_magnetic = self.bno055.magnetic

        for i, a in enumerate(['x', 'y', 'z']):
            # Magnetic field strength needs to be converted from mircoteslas to teslas
            fst = bno055_magnetic[i] * 0.000001
            setattr(msg.magnetic_field, a, fst)

        # TODO
        msg.magnetic_field_covariance = [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ]

        self._mag_pub.publish(msg)

    def diag_timer_callback(self) -> None:
        msg_arr = DiagnosticArray()
        msg_arr.header.stamp = self.get_clock().now().to_msg()
        msg_arr.header.frame_id = self.frame_id

        calib_msg = DiagnosticStatus()
        calib_msg.name = "Calibration Status"
        calib_msg.hardware_id = "BNO055"

        if self.bno055.calibrated:
            calib_msg.level = calib_msg.OK
            calib_msg.message = "Fully calibrated"
        else:
            calib_msg.level = calib_msg.WARN
            calib_msg.message = "Not fully calibrated"

        calib_names = ("System", "Gyroscope", "Accelerometer", "Magnetometer")
        for name, status in zip(calib_names, self.bno055.calibration_status):
            kv = KeyValue()
            kv.key = name
            kv.value = str(status)
            calib_msg.values.append(kv)

        mode_msg = DiagnosticStatus()
        mode_msg.name = "Mode"
        mode_msg.hardware_id = "BNO055"
        mode_msg.level = mode_msg.OK
        mode_msg.message = f"{OP_MODES[self.bno055.mode]} ({self.bno055.mode:#x})"

        ar_msg = DiagnosticStatus()
        ar_msg.name = "Axis Remap"
        ar_msg.hardware_id = "BNO055"
        ar_msg.level = mode_msg.OK
        ar_msg.message = str(self.bno055.axis_remap)

        msg_arr.status.extend([calib_msg, mode_msg, ar_msg])
        self._diag_pub.publish(msg_arr)


def main(args = None) -> None:
    rclpy.init(args=args)
    bno055_pub = BNO055Pub()

    try:
        rclpy.spin(bno055_pub)
    except KeyboardInterrupt:
        pass

    bno055_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
