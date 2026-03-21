#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # Pines GPIO en numeración BCM
        self.TRIG_PIN = 23
        self.ECHO_PIN = 24

        # Configuración GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG_PIN, GPIO.OUT)
        GPIO.setup(self.ECHO_PIN, GPIO.IN)

        # Aseguramos que TRIG empiece en LOW
        GPIO.output(self.TRIG_PIN, False)
        time.sleep(1.0)

        # Timer para leer el sensor cada 0.5 segundos
        self.timer = self.create_timer(0.5, self.read_and_print_distance)

        self.get_logger().info('Nodo ultrasonic_node iniciado')

    def read_distance(self):
        # Pulso de disparo de 10 microsegundos
        GPIO.output(self.TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG_PIN, False)

        pulse_start = time.time()
        pulse_end = time.time()

        timeout = time.time() + 0.04  # 40 ms de timeout

        # Espera a que ECHO suba a HIGH
        while GPIO.input(self.ECHO_PIN) == 0:
            pulse_start = time.time()
            if time.time() > timeout:
                return None

        timeout = time.time() + 0.04  # nuevo timeout

        # Espera a que ECHO vuelva a LOW
        while GPIO.input(self.ECHO_PIN) == 1:
            pulse_end = time.time()
            if time.time() > timeout:
                return None

        pulse_duration = pulse_end - pulse_start

        # Velocidad del sonido: 34300 cm/s
        distance = pulse_duration * 34300 / 2

        return round(distance, 2)

    def read_and_print_distance(self):
        distance = self.read_distance()

        if distance is None:
            self.get_logger().warn('No se pudo medir la distancia')
        else:
            self.get_logger().info(f'Distancia medida: {distance} cm')

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
