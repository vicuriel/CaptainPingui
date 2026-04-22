#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time


class UltrasonicSerialNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_node')

        self.publisher_ = self.create_publisher(Float32, '/ultrasonic_distance', 10)

        # Abrir puerto serial
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Esperar a que Arduino termine de reiniciarse al abrir el puerto
        time.sleep(2)

        # Limpiar basura inicial del buffer
        self.ser.reset_input_buffer()

        self.timer_ = self.create_timer(0.1, self.read_serial)

        self.get_logger().info('Nodo ultrasonico iniciado')

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                raw = self.ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()

                self.get_logger().info(f'RAW recibido: {repr(line)}')

                if not line:
                    return

                try:
                    distance_cm = float(line)
                except ValueError:
                    self.get_logger().warn(f'Dato invalido: {repr(line)}')
                    return

                msg = Float32()
                msg.data = distance_cm   # en centimetros

                self.publisher_.publish(msg)
                self.get_logger().info(f'Distancia publicada: {distance_cm:.2f} cm')

        except serial.SerialException as e:
            self.get_logger().error(f'Error serial: {e}')
        except Exception as e:
            self.get_logger().error(f'Error general: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()