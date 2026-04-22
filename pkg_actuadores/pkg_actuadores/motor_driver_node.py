#!/usr/bin/env python3

import math
import serial
import rclpy
import sys
import threading

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.32)      # metros
        self.declare_parameter('wheel_radius', 0.10)    # metros

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Serial
        self.ser = None
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            self.get_logger().info(f'Conectado a {self.port}')
        except Exception as e:
            self.get_logger().error(f'No se pudo abrir puerto serial: {e}')
            raise

        # Subscripciones
        self.enable_sub = self.create_subscription(
            Bool,
            '/motors_enable',
            self.enable_cb,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_cb,
            10
        )

        self.motors_enabled = True

        # Timer para leer mensajes de debug del ESP32
        self.create_timer(0.02, self.read_serial)

        # Hilo para control manual por terminal
        threading.Thread(target=self.terminal_listener, daemon=True).start()
        print("\n--- CONTROL MANUAL ---")
        print("Escribe: RPM_IZQ RPM_DER   (ej: 40 40)")
        print("O escribe: STOP\n")

    def terminal_listener(self):
        while rclpy.ok():
            try:
                line = sys.stdin.readline().strip()
                if not line:
                    continue

                if line.upper() == "STOP":
                    self.send("STOP")
                    print(">> Enviado: STOP")
                    continue

                parts = line.split()
                if len(parts) == 2:
                    rpm_izq = parts[0]
                    rpm_der = parts[1]
                    self.send(f"CMD_RPM {rpm_izq} {rpm_der}")
                    print(f">> Enviado: CMD_RPM {rpm_izq} {rpm_der}")
                else:
                    print("Formato inválido. Usar: RPM_IZQ RPM_DER o STOP")

            except Exception as e:
                print(f"Error en terminal_listener: {e}")

    def send(self, msg_str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((str(msg_str) + '\n').encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Error enviando por serial: {e}')

    def enable_cb(self, msg):
        self.motors_enabled = msg.data
        self.send('ENABLE' if msg.data else 'STOP')

        if msg.data:
            self.get_logger().info('Motores habilitados')
        else:
            self.get_logger().info('Motores detenidos')

    def cmd_vel_cb(self, msg):
        if not self.motors_enabled:
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Cinemática diferencial
        l_mps = linear_x - (angular_z * self.wheel_base / 2.0)
        r_mps = linear_x + (angular_z * self.wheel_base / 2.0)

        # m/s -> RPM
        l_rpm = (l_mps / (2 * math.pi * self.wheel_radius)) * 60.0
        r_rpm = (r_mps / (2 * math.pi * self.wheel_radius)) * 60.0

        self.send(f'CMD_RPM {l_rpm:.2f} {r_rpm:.2f}')

        self.get_logger().info(
            f'/cmd_vel -> RPM izq={l_rpm:.2f}, der={r_rpm:.2f}'
        )

    def read_serial(self):
        try:
            if self.ser and self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.get_logger().info(f'ESP32: {line}')
        except Exception as e:
            self.get_logger().warn(f'Error leyendo serial: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser and node.ser.is_open:
            node.send('STOP')
            node.ser.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()