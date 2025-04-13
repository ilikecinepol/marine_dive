import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray, Bool
import serial
import threading
import time

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.create_subscription(Int16, 'pump1_cmd', self.pump1_callback, 10)
        self.create_subscription(Int16, 'pump2_cmd', self.pump2_callback, 10)
        self.create_subscription(Bool, 'auto_level', self.auto_level_callback, 10)
        self.pub_imu = self.create_publisher(Float32MultiArray, 'imu_data', 10)
        self.get_logger().info("Serial bridge initialized on /dev/ttyUSB0")

        self.pitch = 0.0
        self.roll = 0.0
        self.auto_level = False
        self.kp = 3.0
        self.ki = 0.1
        self.kd = 1.0
        self.integral_pitch = 0.0
        self.prev_error_pitch = 0.0
        self.integral_roll = 0.0
        self.prev_error_roll = 0.0

        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.control_thread = threading.Thread(target=self.auto_control_loop, daemon=True)
        self.read_thread.start()
        self.control_thread.start()

    def pump1_callback(self, msg):
        self.auto_level = False
        value = int(msg.data)
        value = max(-255, min(255, value))
        command = f'P1:{value}\n'
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent to serial: {command.strip()}")

    def pump2_callback(self, msg):
        self.auto_level = False
        value = int(msg.data)
        value = max(-255, min(255, value))
        command = f'P2:{value}\n'
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent to serial: {command.strip()}")

    def auto_level_callback(self, msg):
        self.auto_level = msg.data
        self.get_logger().info(f"\U0001F9ED Auto-level: {'ON' if self.auto_level else 'OFF'}")

    def read_serial_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode().strip()
                if ',' in line:
                    parts = line.split(',')
                    if len(parts) == 2:
                        self.pitch = float(parts[0])
                        self.roll = float(parts[1])
                        msg = Float32MultiArray()
                        msg.data = [self.pitch, self.roll]
                        self.pub_imu.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Error reading serial: {e}")

    def auto_control_loop(self):
        rate = 0.1  # 10 Hz
        while rclpy.ok():
            if self.auto_level:
                # --- Pitch (Pump1) PID ---
                error_pitch = -self.pitch
                self.integral_pitch += error_pitch * rate
                derivative_pitch = (error_pitch - self.prev_error_pitch) / rate
                output_pitch = self.kp * error_pitch + self.ki * self.integral_pitch + self.kd * derivative_pitch
                self.prev_error_pitch = error_pitch
                output_pitch = max(-255, min(255, int(output_pitch)))
                self.ser.write(f'P1:{output_pitch}\n'.encode())

                # --- Roll (Pump2) PID ---
                error_roll = -self.roll
                self.integral_roll += error_roll * rate
                derivative_roll = (error_roll - self.prev_error_roll) / rate
                output_roll = self.kp * error_roll + self.ki * self.integral_roll + self.kd * derivative_roll
                self.prev_error_roll = error_roll
                output_roll = max(-255, min(255, int(output_roll)))
                self.ser.write(f'P2:{output_roll}\n'.encode())

            time.sleep(rate)

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
