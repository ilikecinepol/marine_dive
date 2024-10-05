import serial
from flask import Flask, render_template, jsonify, request

# Настройка последовательного порта
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Укажите правильный последовательный порт

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
            # Разделим данные на три компонента
            gx, gy, gz = parse_gyro_data(gyro_data)
            return jsonify({'gx': gx, 'gy': gy, 'gz': gz})
        except Exception as e:
            print(f"Ошибка чтения данных: {e}")
            return jsonify({'gx': 'N/A', 'gy': 'N/A', 'gz': 'N/A'})
    return jsonify({'gx': 'N/A', 'gy': 'N/A', 'gz': 'N/A'})

def parse_gyro_data(gyro_data):
    """
    Преобразование строки с гироскопическими данными в три компонента
    Пример входных данных: "Gx: 0.73 Gy: 0.11 Gz: 0.07"
    """
    try:
        data = gyro_data.split()
        gx = data[1] if len(data) > 1 else 'N/A'
        gy = data[3] if len(data) > 3 else 'N/A'
        gz = data[5] if len(data) > 5 else 'N/A'
        return gx, gy, gz
    except:
        return 'N/A', 'N/A', 'N/A'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, debug=True)
