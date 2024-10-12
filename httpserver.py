import io
import logging
import socketserver
from http import server
from threading import Condition
import serial
import json
import os

from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

# Serial port configuration for reading gyroscope data
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
PAGE = """\
<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>MARINE DIVE</title>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@900&display=swap');

        body {
            font-family: 'Orbitron', sans-serif;
            background-image: url('/static/scale.jpg'); /* Path to background image */
            background-size: cover;
            background-position: center;
            margin: 0;
            padding: 0;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            height: 100vh;
            color: #fff;
        }

        h1 {
            font-size: 60px;
            text-shadow: 0 0 10px #00ffcc, 0 0 20px #00ffcc, 0 0 40px #00ffcc;
            margin-bottom: 20px;
            letter-spacing: 3px;
            font-weight: 900;
        }

        .container {
            display: flex;
            justify-content: space-between;
            width: 80%;
            max-width: 1200px;
        }

        .video-placeholder {
            width: 640px;
            height: 480px;
            background-color: rgba(0, 0, 0, 0.7);
            border-radius: 15px;
            box-shadow: 0 0 20px rgba(0, 255, 204, 0.5);
            text-align: center;
            color: #fff;
            padding: 10px;
        }

        canvas {
            width: 300px;
            height: 300px;
        }

        .controls {
            width: 80%;
            max-width: 600px;
            margin: 20px auto;
            padding: 20px;
            background: rgba(0, 0, 0, 0.7);
            border-radius: 15px;
            box-shadow: 0 0 20px rgba(0, 255, 204, 0.5);
            text-align: center;
        }

        input[type="range"] {
            width: 80%;
            margin: 10px 0;
        }

        label, span {
            font-size: 22px;
            color: #00ffcc;
            margin-right: 10px;
            text-shadow: 0 0 5px #00ffcc;
        }

        button {
            padding: 10px 20px;
            background-color: #00ffcc;
            color: #000;
            border: none;
            border-radius: 5px;
            font-size: 20px;
            font-weight: bold;
            cursor: pointer;
            transition: background-color 0.3s ease;
        }

        button:hover {
            background-color: #009999;
        }
    </style>
</head>
<body>

<h1>MARINE DIVE</h1>

<div class="container">
    <!-- Video Stream -->
    <div class="video-placeholder">
        <img src="/stream.mjpg" width="640" height="480" alt="Видеопоток" />
    </div>

    <!-- Gyroscope canvas -->
    <canvas id="horizonCanvas" width="300" height="300"></canvas>
</div>

<!-- Pump controls -->
<div class="controls">
    <label for="pump1">Помпа 1:</label>
    <input type="range" id="pump1" min="-100" max="100" step="5" value="0">
    <span id="pump1_value">0%</span>
    <br><br>

    <label for="pump2">Помпа 2:</label>
    <input type="range" id="pump2" min="-100" max="100" step="5" value="0">
    <span id="pump2_value">0%</span>
    <br><br>

    <button onclick="stopAll()">Остановить все помпы</button>
</div>

<script>
    function updateGyroData() {
        fetch('/gyro')
        .then(response => response.json())
        .then(data => {
            drawHorizon(data.gy, data.gx);
        });
    }

    function drawHorizon(pitch, roll) {
        const canvas = document.getElementById('horizonCanvas');
        const ctx = canvas.getContext('2d');

        ctx.clearRect(0, 0, canvas.width, canvas.height);

        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;

        // Horizon drawing logic
        ctx.save();
        ctx.translate(centerX, centerY);
        ctx.rotate(-roll * Math.PI / 180);
        ctx.translate(-centerX, -centerY);

        ctx.beginPath();
        ctx.arc(centerX, centerY, 150, 0, Math.PI, true);
        ctx.closePath();
        ctx.fillStyle = '#1E90FF';
        ctx.fill();

        ctx.beginPath();
        ctx.arc(centerX, centerY, 150, 0, Math.PI, false);
        ctx.closePath();
        ctx.fillStyle = '#654321';
        ctx.fill();

        ctx.restore();
        const offsetY = pitch * 2;
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(centerX - 150, centerY + offsetY);
        ctx.lineTo(centerX + 150, centerY + offsetY);
        ctx.stroke();

        // Draw pitch scale numbers
        for (let i = -90; i <= 90; i += 10) {
            const tickY = centerY + offsetY + i * 2;
            ctx.beginPath();
            ctx.moveTo(centerX - 10, tickY);
            ctx.lineTo(centerX + 10, tickY);
            ctx.stroke();

            if (i % 30 === 0) {
                ctx.font = '14px Orbitron';
                ctx.fillStyle = '#ffffff';
                ctx.fillText(`${i}`, centerX + 15, tickY + 5);
            }
        }
    }

    setInterval(updateGyroData, 100);

    function stopAll() {
        document.getElementById('pump1').value = 0;
        document.getElementById('pump2').value = 0;
        sendPumpData();
    }

    function sendPumpData() {
        const pump1Speed = document.getElementById('pump1').value;
        const pump2Speed = document.getElementById('pump2').value;
        document.getElementById('pump1_value').innerText = `${pump1Speed}%`;
        document.getElementById('pump2_value').innerText = `${pump2Speed}%`;

        fetch('/control', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ pump1_speed: pump1Speed, pump2_speed: pump2Speed })
        });
    }

    document.getElementById('pump1').addEventListener('input', sendPumpData);
    document.getElementById('pump2').addEventListener('input', sendPumpData);
</script>

</body>
</html>

"""
# Define the root directory for static files (like the background image)
STATIC_DIR = os.path.join(os.path.dirname(__file__), 'static')

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning('Removed streaming client %s: %s', self.client_address, str(e))
        elif self.path == '/gyro':
            if ser.in_waiting > 0:
                try:
                    gyro_data = ser.readline().decode('utf-8').strip()
                    gx, gy = map(float, gyro_data.split(','))  # Expecting two values
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({'gx': gx, 'gy': gy}).encode())
                except Exception as e:
                    logging.error(f"Error reading gyroscope data: {e}")
                    self.send_response(500)
                    self.end_headers()
            else:
                self.send_response(204)  # No content (empty)
                self.end_headers()
        elif self.path == '/control':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            data = json.loads(post_data)
            pump1_speed = data.get('pump1_speed', 0)
            pump2_speed = data.get('pump2_speed', 0)

            # Send pump1_speed and pump2_speed to Arduino via serial
            try:
                ser.write(f'P1:{pump1_speed},P2:{pump2_speed}\n'.encode('utf-8'))
                self.send_response(200)
                self.end_headers()
            except Exception as e:
                logging.error(f"Error sending pump data: {e}")
                self.send_response(500)
                self.end_headers()
        elif self.path.startswith('/static/'):
            # Serve static files like images
            file_path = os.path.join(STATIC_DIR, self.path[8:])
            if os.path.exists(file_path):
                with open(file_path, 'rb') as f:
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.end_headers()
                    self.wfile.write(f.read())
            else:
                self.send_error(404, "File Not Found")
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

# Initialize and configure the Picamera2 object
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
output = StreamingOutput()
picam2.start_recording(JpegEncoder(), FileOutput(output))

try:
    address = ('', 8000)
    server = StreamingServer(address, StreamingHandler)
    print("Server started on port 8000...")
    server.serve_forever()
finally:
    picam2.stop_recording()
