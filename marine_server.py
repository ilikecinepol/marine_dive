import serial
import time
from flask import Flask, jsonify, render_template

# Настройка последовательного порта
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Увеличение скорости передачи данных
time.sleep(2)  # Задержка для установления связи

# Создание веб-приложения на Flask
app = Flask(__name__)

# Функция для чтения данных с Arduino
def read_serial_data():
    if ser.in_waiting > 0:
        try:
            # Чтение данных с последовательного порта
            data = ser.readline().decode('utf-8').rstrip()
            return data
        except Exception as e:
            print(f"Ошибка чтения данных: {e}")
            return None

@app.route('/')
def index():
    return render_template('index.html')  # Отображение HTML-страницы

@app.route('/data')
def data():
    """
    Эндпоинт для получения трёх переменных с Arduino через Serial
    """
    serial_data = read_serial_data()
    if serial_data:
        try:
            # Ожидаем, что данные будут в формате "var1,var2,var3"
            var1, var2, var3 = serial_data.split(',')
            return jsonify({'var1': var1, 'var2': var2, 'var3': var3})
        except ValueError:
            return jsonify({'var1': 'N/A', 'var2': 'N/A', 'var3': 'N/A'})
    
    return jsonify({'var1': 'N/A', 'var2': 'N/A', 'var3': 'N/A'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, debug=True)
