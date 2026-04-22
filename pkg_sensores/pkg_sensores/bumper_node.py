#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial


class BumperNode(Node):
    def __init__(self):
        super().__init__('bumper_node')

        self.front_center_pub = self.create_publisher(
            Bool,
            '/bumpers/front_center',
            10
        )

        self.ser = None

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info('bumper_node iniciado correctamente')
        except Exception as e:
            self.get_logger().error(f'No se pudo abrir el puerto serial: {e}')

        self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        if self.ser is None:
            return

        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            if not line:
                return

            msg = Bool()
            msg.data = (line == '1')
            self.front_center_pub.publish(msg)

            estado = "PRESIONADO" if msg.data else "LIBRE"
            self.get_logger().info(f'Bumper front_center: {estado}')


def main(args=None):
    rclpy.init(args=args)
    node = BumperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()