#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float alpha = 0.5;  // Коэффициент сглаживания для фильтрации
Servo motor;

const int ENA = 3;  // ШИМ для первой помпы
const int ENB = 5;  // ШИМ для второй помпы
const int In1 = 2;
const int In2 = 4;
const int In3 = 7;
const int In4 = 8;

float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;  // Смещения акселерометра
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;  // Смещения гироскопа

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
      ;  // Остановить, если датчик не подключен
  }

  // Калибровка датчика (установка нулей)
  calibrateMPU();

  // Настройка цифрового фильтра (DLPF)
  mpu.setDLPFMode(10);  // Установка фильтра с частотой 5 Гц

  motor.attach(11);
  motor.writeMicroseconds(2300);
  delay(2000);
  motor.writeMicroseconds(800);
  delay(6000);

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

  // Чтение данных с гироскопа и акселерометра, корректировка на смещения
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;
  gx -= gx_offset;
  gy -= gy_offset;
  gz -= gz_offset;

  // Применение фильтрации данных гироскопа
  filtered_gx = (alpha * gx) + ((1 - alpha) * filtered_gx);
  filtered_gy = (alpha * gy) + ((1 - alpha) * filtered_gy);
  filtered_gz = (alpha * gz) + ((1 - alpha) * filtered_gz);

  // Отправка данных по Serial
  Serial.print("Gx: ");
  Serial.print(filtered_gx);
  Serial.print(" Gy: ");
  Serial.print(filtered_gy);
  Serial.print(" Gz: ");
  Serial.println(filtered_gz);

  motor.writeMicroseconds(1000);
  delay(100);  // Частота отправки данных
}

// Калибровка MPU6050 - вычисление смещений
void calibrateMPU() {
  const int calibration_samples = 1000;
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  Serial.println("Калибровка MPU6050...");
  for (int i = 0; i < calibration_samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    delay(3);
  }

  ax_offset = ax_sum / calibration_samples;
  ay_offset = ay_sum / calibration_samples;
  az_offset = az_sum / calibration_samples;
  gx_offset = gx_sum / calibration_samples;
  gy_offset = gy_sum / calibration_samples;
  gz_offset = gz_sum / calibration_samples;

  Serial.println("Калибровка завершена.");
}

void controlPump1(int speed) {
  if (speed == 0) {
    stopPump1();  // Останавливаем насос, если скорость 0
    return;
  }

  if (speed > 0) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);  // Накачка
  } else {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);  // Выкачка
    speed = -speed;
  }

  analogWrite(ENA, map(speed, 150, 255, 150, 255));  // Скорость от 150 до 255
}

void controlPump2(int speed) {
  if (speed == 0) {
    stopPump2();
    return;
  }

  if (speed > 0) {
    digitalWrite(In3, LOW);
    digitalWrite(In4, HIGH);  // Накачка
  } else {
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);  // Выкачка
    speed = -speed;
  }

  analogWrite(ENB, map(speed, 150, 255, 150, 255));
}

void stopPump1() {
  analogWrite(ENA, 0);
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
}

void stopPump2() {
  analogWrite(ENB, 0);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}
