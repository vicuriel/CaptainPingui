#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        try:
            self.sensor = MPU9250(
                address_ak=AK8963_ADDRESS,
                address_mpu_master=MPU9050_ADDRESS_68,
                address_mpu_slave=None,
                bus=1,
                gfs=GFS_250,
                afs=AFS_2G,
                mfs=AK8963_BIT_16,
                mode=AK8963_MODE_C100HZ
            )

            self.sensor.configure()

            self.get_logger().info('IMU MPU-9250 inicializado correctamente')

        except Exception as e:
            self.get_logger().error(f'Error inicializando el MPU-9250: {e}')
            self.sensor = None

        self.timer = self.create_timer(0.5, self.read_imu)

    def read_imu(self):
        if self.sensor is None:
            return

        try:
            accel = self.sensor.readAccelerometerMaster()
            gyro = self.sensor.readGyroscopeMaster()
            mag = self.sensor.readMagnetometerMaster()

            self.get_logger().info(
                f'Acel [g]: X={accel[0]:.2f}, Y={accel[1]:.2f}, Z={accel[2]:.2f} | '
                f'Giro [deg/s]: X={gyro[0]:.2f}, Y={gyro[1]:.2f}, Z={gyro[2]:.2f} | '
                f'Mag [uT]: X={mag[0]:.2f}, Y={mag[1]:.2f}, Z={mag[2]:.2f}'
            )

        except Exception as e:
            self.get_logger().warn(f'Error leyendo IMU: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
