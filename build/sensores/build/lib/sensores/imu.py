#!/usr/bin/env python3

import re
import math
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')

        # ==============================
        # CONFIGURACION DEL PUERTO SERIAL
        # ==============================
        self.serial_port = '/dev/ttyUSB0'   # Cambiar a /dev/ttyACM0 si hace falta
        self.baudrate = 9600                # Debe ser igual al Serial.begin(...) del Arduino

        # ==============================
        # VARIABLES PARA GUARDAR DATOS
        # ==============================
        self.accel_data = None   # (ax, ay, az) en g
        self.gyro_data = None    # (gx, gy, gz) en grados/s
        self.temp_data = None    # temperatura en C

        # ==============================
        # PUBLICADOR ROS2
        # ==============================
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        # ==============================
        # ABRIR PUERTO SERIAL
        # ==============================
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(
                f'Puerto serial abierto: {self.serial_port} a {self.baudrate} baud'
            )
        except Exception as e:
            self.get_logger().error(f'No se pudo abrir el puerto serial: {e}')
            raise

        # ==============================
        # TIMERS
        # ==============================
        # Lee el serial rapido
        self.read_timer = self.create_timer(0.05, self.read_serial)

        # Imprime cada 1 segundo
        self.print_timer = self.create_timer(1.0, self.print_data)

        self.get_logger().info('Nodo IMU iniciado correctamente')

    def read_serial(self):
        """Lee las líneas que llegan desde el Arduino por serial."""
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if not line:
                    continue

                # Ignorar lineas decorativas
                if '----- MPU 9250 -----' in line or '--------------------' in line:
                    continue

                # Parsear acelerometro
                if 'Acelerometro X:' in line:
                    self.accel_data = self.parse_accel(line)

                # Parsear giroscopio
                elif 'Giroscopio' in line:
                    self.gyro_data = self.parse_gyro(line)

                # Parsear temperatura
                elif 'Temperatura :' in line:
                    self.temp_data = self.parse_temp(line)

                    # Cuando ya tenemos accel + gyro, publicamos
                    if self.accel_data is not None and self.gyro_data is not None:
                        self.publish_imu()

            except Exception as e:
                self.get_logger().warn(f'Error leyendo/parsing serial: {e}')

    def parse_accel(self, line):
        """
        Espera una linea como:
        Acelerometro X: 0.010 g | Y: -0.020 g | Z: 0.980 g
        """
        pattern = r'Acelerometro X:\s*([-+]?\d*\.?\d+)\s*g\s*\|\s*Y:\s*([-+]?\d*\.?\d+)\s*g\s*\|\s*Z:\s*([-+]?\d*\.?\d+)\s*g'
        match = re.search(pattern, line)

        if match:
            ax = float(match.group(1))
            ay = float(match.group(2))
            az = float(match.group(3))
            return (ax, ay, az)

        self.get_logger().warn('No se pudo parsear la línea del acelerómetro')
        return None

    def parse_gyro(self, line):
        """
        Espera una linea como:
        Giroscopio   X: 0.450 °/s | Y: -0.120 °/s | Z: 0.080 °/s
        """
        pattern = r'Giroscopio\s*X:\s*([-+]?\d*\.?\d+)\s*°/s\s*\|\s*Y:\s*([-+]?\d*\.?\d+)\s*°/s\s*\|\s*Z:\s*([-+]?\d*\.?\d+)\s*°/s'
        match = re.search(pattern, line)

        if match:
            gx = float(match.group(1))
            gy = float(match.group(2))
            gz = float(match.group(3))
            return (gx, gy, gz)

        self.get_logger().warn('No se pudo parsear la línea del giroscopio')
        return None

    def parse_temp(self, line):
        """
        Espera una linea como:
        Temperatura : 28.41 C
        """
        pattern = r'Temperatura\s*:\s*([-+]?\d*\.?\d+)'
        match = re.search(pattern, line)

        if match:
            temp_c = float(match.group(1))
            return temp_c

        self.get_logger().warn('No se pudo parsear la línea de temperatura')
        return None

    def publish_imu(self):
        """Publica los datos de la IMU en /imu/data_raw."""
        if self.accel_data is None or self.gyro_data is None:
            return

        ax_g, ay_g, az_g = self.accel_data
        gx_dps, gy_dps, gz_dps = self.gyro_data

        # Convertir aceleracion de g a m/s^2
        ax = ax_g * 9.80665
        ay = ay_g * 9.80665
        az = az_g * 9.80665

        # Convertir velocidad angular de grados/s a rad/s
        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)

        msg = Imu()

        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Orientacion no disponible todavia
        msg.orientation_covariance[0] = -1.0

        # Velocidad angular
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Aceleracion lineal
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # Covarianzas simples de prueba
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02

        msg.linear_acceleration_covariance[0] = 0.04
        msg.linear_acceleration_covariance[4] = 0.04
        msg.linear_acceleration_covariance[8] = 0.04

        self.imu_pub.publish(msg)

    def print_data(self):
        """Imprime en consola los ultimos datos recibidos cada 1 segundo."""
        if self.accel_data is None or self.gyro_data is None:
            return

        ax, ay, az = self.accel_data
        gx, gy, gz = self.gyro_data
        temp = self.temp_data if self.temp_data is not None else 0.0

        self.get_logger().info(
            f'\n[IMU cada 1s]\n'
            f'Acelerometro (g):\n'
            f'  Eje X: {ax:.3f}\n'
            f'  Eje Y: {ay:.3f}\n'
            f'  Eje Z: {az:.3f}\n'
            f'Giroscopio (grados/s):\n'
            f'  Eje X: {gx:.3f}\n'
            f'  Eje Y: {gy:.3f}\n'
            f'  Eje Z: {gz:.3f}\n'
            f'Temperatura: {temp:.2f} C\n'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()