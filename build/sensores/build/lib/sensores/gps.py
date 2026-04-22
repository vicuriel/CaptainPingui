#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial

class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')

        # 🔌 Configurar puerto serial (ajustar si es necesario)
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

        # ⏱ Timer para leer cada 0.5 seg
        self.timer = self.create_timer(0.5, self.read_serial)

        self.get_logger().info("Nodo GPS iniciado")

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            self.get_logger().info(f"RAW: {line}")

            try:
                # 🧩 Parseo simple
                parts = line.split('|')

                sat = int(parts[0].split(':')[1].strip())
                lat = float(parts[1].split(':')[1].strip())
                lon = float(parts[2].split(':')[1].strip())
                alt = float(parts[3].split(':')[1].replace('m', '').strip())

                self.get_logger().info(
                    f"SAT: {sat} | LAT: {lat} | LON: {lon} | ALT: {alt}"
                )

            except Exception as e:
                self.get_logger().warn(f"Error parseando: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()