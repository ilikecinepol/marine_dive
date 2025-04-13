import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray
import serial
import threading

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.create_subscription(Int16, 'pump1_cmd', self.pump1_callback, 10)
        self.create_subscription(Int16, 'pump2_cmd', self.pump2_callback, 10)
        self.pub_imu = self.create_publisher(Float32MultiArray, 'imu_data', 10)
        self.get_logger().info("Serial bridge initialized on /dev/ttyUSB0")

        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

    def pump1_callback(self, msg):
        command = f'P1:{msg.data}\n'
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent to serial: {command.strip()}")

    def pump2_callback(self, msg):
        command = f'P2:{msg.data}\n'
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent to serial: {command.strip()}")

    def read_serial_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode().strip()
                if ',' in line:
                    parts = line.split(',')
                    if len(parts) == 2:
                        pitch = float(parts[0])
                        roll = float(parts[1])
                        msg = Float32MultiArray()
                        msg.data = [pitch, roll]
                        self.pub_imu.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Error reading serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
