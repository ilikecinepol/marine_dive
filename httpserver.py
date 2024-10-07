import io
import logging
import socketserver
from http import server
from threading import Condition
import serial
import json

from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

# Настройка последовательного порта для чтения данных с гироскопа
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

PAGE = """\
<html>
<head>
<title>picamera2 MJPEG streaming demo with Gyroscope</title>
<script>
    function updateGyroData() {
        fetch('/gyro')
        .then(response => response.json())
        .then(data => {
            document.getElementById('gyro').innerText = 
                'Gyro X: ' + data.gx + ', Gyro Y: ' + data.gy;
        });
    }

    setInterval(updateGyroData, 500); // Обновление каждые 500мс
</script>
</head>
<body>
<h1>Picamera2 MJPEG Streaming Demo with Gyroscope</h1>
<img src="stream.mjpg" width="640" height="480" />
<h2>Gyroscope Readings</h2>
<p id="gyro">Gyro X: N/A, Gyro Y: N/A</p>
</body>
</html>
"""

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
                    gx, gy = map(float, gyro_data.split(','))  # Ожидаем два значения
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(json.dumps({'gx': gx, 'gy': gy}).encode())
                except Exception as e:
                    logging.error(f"Ошибка чтения данных гироскопа: {e}")
                    self.send_response(500)
                    self.end_headers()
            else:
                self.send_response(204)  # Нет данных
                self.end_headers()
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
output = StreamingOutput()
picam2.start_recording(JpegEncoder(), FileOutput(output))

try:
    address = ('', 8000)
    server = StreamingServer(address, StreamingHandler)
    server.serve_forever()
finally:
    picam2.stop_recording()
