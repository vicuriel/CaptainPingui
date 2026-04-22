#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial

class SwitchBridgeNode(Node):
    def __init__(self):
        super().__init__('switch_node')
        self.publisher_ = self.create_publisher(Bool, '/limit_switch_status', 10)
        
        # Misma configuración que el ultrasonido
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info('Nodo de Final de Carrera iniciado')
        except Exception as e:
            self.get_logger().error(f'No se pudo abrir el puerto: {e}')

        self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                msg = Bool()
                # Si recibimos '1', el switch está activado
                msg.data = (line == '1')
                self.publisher_.publish(msg)
                status = "PRESIONADO" if msg.data else "LIBRE"
                self.get_logger().info(f'Estado: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = SwitchBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()