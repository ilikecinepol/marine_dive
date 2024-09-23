import serial
from flask import Flask, render_template, jsonify, request

# Настройка последовательного порта
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Замените '/dev/ttyUSB0' на ваш последовательный порт

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/control', methods=['POST'])
def control():
    """
    Принимаем команды для управления помпами
    """
    data = request.json
    pump1_speed = data.get('pump1_speed')
    pump2_speed = data.get('pump2_speed')

    if pump1_speed is not None:
        command = f'P1:{pump1_speed}\n'
        ser.write(command.encode())

    if pump2_speed is not None:
        command = f'P2:{pump2_speed}\n'
        ser.write(command.encode())

    return jsonify({'status': 'OK'})

@app.route('/gyro')
def gyro():
    """
    Возвращаем данные гироскопа
    """
    if ser.in_waiting > 0:
        try:
            # Чтение данных с гироскопа
            gyro_data = ser.readline().decode('utf-8').strip()
            return jsonify({'gyro_data': gyro_data})
        except Exception as e:
            print(f"Ошибка чтения данных: {e}")
            return jsonify({'gyro_data': 'Ошибка'})
    return jsonify({'gyro_data': 'Нет данных'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, debug=True)
