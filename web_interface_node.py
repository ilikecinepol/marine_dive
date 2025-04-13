import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray
from flask import Flask, request, jsonify, send_from_directory
import threading
import os

CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))
WEBUI_DIR = '/home/trevor/ros2_ws/src/submarine_ros2/webui'
print("📁 HTML ищется в:", WEBUI_DIR)

app = Flask(__name__)
imu_data = [0.0, 0.0]  # [pitch, roll]

class WebInterfaceNode(Node):
    def __init__(self):
        super().__init__('web_interface_node')
        self.sub_imu = self.create_subscription(Float32MultiArray, 'imu_data', self.imu_callback, 10)
        self.pub_p1 = self.create_publisher(Int16, 'pump1_cmd', 10)
        self.pub_p2 = self.create_publisher(Int16, 'pump2_cmd', 10)

    def imu_callback(self, msg):
        global imu_data
        imu_data = msg.data

    def start_flask(self):
        threading.Thread(target=app.run, kwargs={"host": "0.0.0.0", "port": 5000}).start()

@app.route('/')
def index():
    return send_from_directory(WEBUI_DIR, 'index.html')

@app.route('/static/<path:path>')
def send_static(path):
    return send_from_directory(os.path.join(WEBUI_DIR, 'static'), path)

gyro_zero = [0.0, 0.0]

@app.route('/gyro')
def gyro():
    corrected_pitch = imu_data[0] - gyro_zero[0]
    corrected_roll = imu_data[1] - gyro_zero[1]
    return jsonify({"gx": corrected_pitch, "gy": corrected_roll})

@app.route('/control', methods=['POST'])
def control():
    data = request.get_json()
    pump1 = int(data.get('pump1_speed', 0))
    pump2 = int(data.get('pump2_speed', 0))
    node.pub_p1.publish(Int16(data=pump1))
    node.pub_p2.publish(Int16(data=pump2))
    return '', 204

def main():
    global node
    rclpy.init()
    node = WebInterfaceNode()
    node.start_flask()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
