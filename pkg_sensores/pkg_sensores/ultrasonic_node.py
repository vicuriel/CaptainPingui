#!/usr/bin/env python3

import serial
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        self.publisher_ = self.create_publisher(
            Range,
            '/ultrasonic/front_center',
            10
        )

        self.ser = None

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

            # Esperar a que el microcontrolador reinicie
            time.sleep(2)

            # Limpiar buffer inicial
            self.ser.reset_input_buffer()

            self.get_logger().info('ultrasonic_node iniciado correctamente')

        except Exception as e:
            self.get_logger().error(f'No se pudo abrir el puerto serial: {e}')

        self.timer_ = self.create_timer(0.1, self.read_serial)

        # Parámetros del mensaje Range
        self.frame_id = 'ultrasonic_front_center_link'
        self.radiation_type = Range.ULTRASOUND
        self.field_of_view = 0.26   # aprox 15°
        self.min_range = 0.02       # 2 cm
        self.max_range = 4.00       # 4 m

    def create_range_msg(self, distance_cm):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = self.radiation_type
        msg.field_of_view = self.field_of_view
        msg.min_range = self.min_range
        msg.max_range = self.max_range

        # convertir cm a metros
        msg.range = distance_cm / 100.0
        return msg

    def read_serial(self):
        if self.ser is None:
            return

        try:
            if self.ser.in_waiting > 0:
                raw = self.ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()

                if not line:
                    return

                self.get_logger().info(f'RAW recibido: {repr(line)}')

                try:
                    distance_cm = float(line)
                except ValueError:
                    self.get_logger().warn(f'Dato inválido: {repr(line)}')
                    return

                msg = self.create_range_msg(distance_cm)
                self.publisher_.publish(msg)

                self.get_logger().info(
                    f'Distancia publicada en /ultrasonic/front_center: {distance_cm:.2f} cm'
                )

        except serial.SerialException as e:
            self.get_logger().error(f'Error serial: {e}')
        except Exception as e:
            self.get_logger().error(f'Error general: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()

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