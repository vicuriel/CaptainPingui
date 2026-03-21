#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO


class BumperNode(Node):
    def __init__(self):
        super().__init__('bumper_node')

        # Pin GPIO donde conectas el bumper
        self.BUMPER_PIN = 17

        # Configuración GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.BUMPER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Timer para leer cada 0.2 segundos
        self.timer = self.create_timer(0.2, self.read_bumper)

        self.get_logger().info('Nodo bumper_node iniciado')

    def read_bumper(self):
        state = GPIO.input(self.BUMPER_PIN)

        # Invertimos lógica
        if state == 0:
            value = 1  # presionado
        else:
            value = 0  # no presionado

        self.get_logger().info(f'Final de carrera: {value}')

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BumperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
