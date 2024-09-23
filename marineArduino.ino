#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float alpha = 0.05;  // Коэффициент сглаживания для фильтрации

const int ENA = 3;  // ШИМ для первого насоса
const int ENB = 5;  // ШИМ для второго насоса
const int In1 = 2;
const int In2 = 4;
const int In3 = 7;
const int In4 = 8;

// Переменные для хранения предыдущих значений
float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

void setup() {
  Serial.begin(115200);  // Увеличение скорости передачи данных
  Wire.begin();          // Инициализация I2C для MPU6050
  mpu.initialize();      // Инициализация MPU6050

  // Проверка подключения MPU6050
  if (mpu.testConnection()) {
    Serial.println("MPU6050 подключен");
  } else {
    Serial.println("Ошибка подключения к MPU6050");
    while (1)
      ;  // Остановить, если датчик не подключен
  }

  // Калибровка акселерометра и гироскопа
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();  // Вывод смещений

  // Настройка цифрового фильтра (DLPF)
  mpu.setDLPFMode(6);  // Установка фильтра с частотой 5 Гц

  // Настройка выводов для L293N
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
}

void loop() {
  // Чтение данных с датчика MPU6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Фильтрация данных гироскопа с использованием экспоненциального скользящего среднего
  filtered_gx = (alpha * gx) + ((1 - alpha) * filtered_gx);
  filtered_gy = (alpha * gy) + ((1 - alpha) * filtered_gy);
  filtered_gz = (alpha * gz) + ((1 - alpha) * filtered_gz);

  // Отправляем отфильтрованные данные по Serial
  Serial.print(filtered_gx);
  Serial.print(",");
  Serial.print(filtered_gy);
  Serial.print(",");
  Serial.println(filtered_gz);
  // pump1(-255);

  delay(50);  // Уменьшение задержки для ускорения передачи данных
}

void stopPump1() {
  analogWrite(ENA, 0);  // Останавливаем первый насос
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
}

void stopPump2() {
  analogWrite(ENB, 0);  // Останавливаем второй насос
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}

// Функция для управления первым насосом
void pump1(int speed) {
  if (speed >= 0) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);  // Накачка
  } else {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);  // Выкачка
    speed = -speed;          // Меняем знак скорости
  }
  analogWrite(ENA, speed);  // Управляем скоростью через ШИМ
}

// Функция для управления вторым насосом
void pump2(int speed) {
  if (speed >= 0) {
    digitalWrite(In3, LOW);
    digitalWrite(In4, HIGH);  // Накачка
  } else {
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);  // Выкачка
    speed = -speed;          // Меняем знак скорости
  }
  analogWrite(ENB, speed);  // Управляем скоростью через ШИМ
}
