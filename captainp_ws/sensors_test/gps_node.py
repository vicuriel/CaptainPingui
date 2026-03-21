#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        # Puerto UART de la Raspberry (GPIO)
        self.port = '/dev/serial0'
        self.baudrate = 9600

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            self.get_logger().info(f'GPS conectado en {self.port}')
        except Exception as e:
            self.get_logger().error(f'Error abriendo puerto: {e}')
            self.ser = None

        self.timer = self.create_timer(0.1, self.read_gps)

    def read_gps(self):
        if self.ser is None:
            return

        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                # Solo imprimimos si es válido
                if line.startswith('$'):
                    self.get_logger().info(line)

        except Exception as e:
            self.get_logger().warn(f'Error leyendo GPS: {e}')

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
