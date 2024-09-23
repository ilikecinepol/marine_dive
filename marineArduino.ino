#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float alpha = 0.05;  // Коэффициент сглаживания для фильтрации

const int ENA = 3;  // ШИМ для первой помпы
const int ENB = 5;  // ШИМ для второй помпы
const int In1 = 2;
const int In2 = 4;
const int In3 = 7;
const int In4 = 8;

// Переменные для хранения предыдущих значений
float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

void setup() {
  Serial.begin(115200);  // Инициализация последовательного порта для связи с Raspberry Pi
  Wire.begin();          // Инициализация I2C для MPU6050
  mpu.initialize();      // Инициализация MPU6050

  // Проверка подключения MPU6050
  if (mpu.testConnection()) {
    Serial.println("MPU6050 подключен");
  } else {
    Serial.println("Ошибка подключения к MPU6050");
    while (1)
      ;  // Остановка, если датчик не подключен
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

  // Остановка обоих насосов
  stopPump1();
  stopPump2();
}

void loop() {
  // Чтение команд для управления помпами через Serial
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Читаем строку до новой строки
    if (input.startsWith("P1:")) {
      int speed1 = input.substring(3).toInt();  // Извлекаем значение для первой помпы
      controlPump1(speed1);
    } else if (input.startsWith("P2:")) {
      int speed2 = input.substring(3).toInt();  // Извлекаем значение для второй помпы
      controlPump2(speed2);
    }
  }

  // Чтение данных с гироскопа и отправка их на Raspberry Pi через Serial
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  filtered_gx = (alpha * gx) + ((1 - alpha) * filtered_gx);
  filtered_gy = (alpha * gy) + ((1 - alpha) * filtered_gy);
  filtered_gz = (alpha * gz) + ((1 - alpha) * filtered_gz);

  Serial.print("Gx: "); Serial.print(filtered_gx);
  Serial.print(" Gy: "); Serial.print(filtered_gy);
  Serial.print(" Gz: "); Serial.println(filtered_gz);

  delay(100);  // Частота отправки данных с гироскопа
}

void controlPump1(int speed) {
  if (speed == 0) {
    stopPump1();  // Останавливаем насос, если скорость 0
    return;
  }

  // Если скорость положительная - накачиваем, если отрицательная - выкачиваем
  if (speed > 0) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);  // Накачка
  } else {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);   // Выкачка
    speed = -speed;           // Меняем знак скорости на положительный
  }
  
  // Устанавливаем скорость насоса через ШИМ
  analogWrite(ENA, map(speed, 150, 255, 150, 255));  // Скорость от 150 до 255
}

void controlPump2(int speed) {
  if (speed == 0) {
    stopPump2();  // Останавливаем насос, если скорость 0
    return;
  }

  // Если скорость положительная - накачиваем, если отрицательная - выкачиваем
  if (speed > 0) {
    digitalWrite(In3, LOW);
    digitalWrite(In4, HIGH);  // Накачка
  } else {
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);   // Выкачка
    speed = -speed;           // Меняем знак скорости на положительный
  }
  
  // Устанавливаем скорость насоса через ШИМ
  analogWrite(ENB, map(speed, 150, 255, 150, 255));  // Скорость от 150 до 255
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
