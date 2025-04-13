import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'imu_data', 10)

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Serial порт открыт")
        except serial.SerialException as e:
            self.get_logger().error(f"Ошибка открытия serial порта: {e}")
            exit(1)

        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            if ',' in line:
                try:
                    pitch, roll = map(float, line.split(','))
                    msg = Float32MultiArray()
                    msg.data = [pitch, roll]
                    self.publisher_.publish(msg)
                except ValueError:
                    self.get_logger().warn(f"Неверный формат строки: {line}")

def main():
    rclpy.init()
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
