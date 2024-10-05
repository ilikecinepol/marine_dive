import serial
from flask import Flask, render_template, jsonify, request

# Настройка последовательного порта
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/gyro')
def gyro():
    if ser.in_waiting > 0:
        try:
            gyro_data = ser.readline().decode('utf-8').strip()
            gx, gy = map(float, gyro_data.split(','))  # Разделение данных по запятой
            return jsonify({'gx': gx, 'gy': gy})
        except Exception as e:
            print(f"Ошибка чтения данных: {e}")
            return jsonify({'gx': 'N/A', 'gy': 'N/A'})
    return jsonify({'gx': 'N/A', 'gy': 'N/A'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, debug=True)
