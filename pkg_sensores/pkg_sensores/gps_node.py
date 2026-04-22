#!/usr/bin/env python3

import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.gps_fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)

        self.ser = None
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info('gps_node iniciado correctamente')
        except Exception as e:
            self.get_logger().error(f'No se pudo abrir el puerto serial: {e}')

        self.timer = self.create_timer(0.5, self.read_serial)

    def read_serial(self):
        if self.ser is None:
            return

        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            if not line:
                return

            self.get_logger().info(f'RAW: {line}')

            try:
                data = self.parse_line(line)
                if data is None:
                    self.get_logger().warn('No se pudo parsear la línea del GPS')
                    return

                sat, lat, lon, alt = data

                self.publish_fix(lat, lon, alt)

                self.get_logger().info(
                    f"SAT: {sat} | LAT: {lat} | LON: {lon} | ALT: {alt}"
                )

            except Exception as e:
                self.get_logger().warn(f"Error parseando: {e}")

    def parse_line(self, line):
        """
        Espera una línea como:
        SAT: 8 | LAT: -25.33057 | LON: -57.51849 | ALT: 120.3m
        """
        try:
            parts = line.split('|')

            sat = int(parts[0].split(':')[1].strip())
            lat = float(parts[1].split(':')[1].strip())
            lon = float(parts[2].split(':')[1].strip())
            alt = float(parts[3].split(':')[1].replace('m', '').strip())

            return sat, lat, lon, alt

        except Exception:
            return None

    def publish_fix(self, lat, lon, alt):
        msg = NavSatFix()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt

        # Covarianza simple de prueba
        msg.position_covariance = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 4.0
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.gps_fix_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()