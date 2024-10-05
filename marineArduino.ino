#include "MPU6050.h"
MPU6050 mpu;

float ax_offset = 0, ay_offset = 0, az_offset = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  mpu.initialize();  // Запускаем датчик
  
  // Калибровка датчика
  calibrateMPU();
}

void loop() {
  int16_t ax = mpu.getAccelerationX();  // ускорение по оси X
  int16_t ay = mpu.getAccelerationY();  // ускорение по оси Y

  // Ограничиваем диапазон для оси X
  ax = constrain(ax, -16384, 16384);
  float angle_x = ax / 16384.0;  // Переводим в диапазон +-1.0
  if (angle_x < 0) angle_x = 90 - degrees(acos(angle_x));
  else angle_x = degrees(acos(-angle_x)) - 90;

  // Ограничиваем диапазон для оси Y
  ay = constrain(ay, -16384, 16384);
  float angle_y = ay / 16384.0;  // Переводим в диапазон +-1.0
  if (angle_y < 0) angle_y = 90 - degrees(acos(angle_y));
  else angle_y = degrees(acos(-angle_y)) - 90;

  // Отправка углов через сериал
  Serial.print(angle_x, 2);  // 2 знака после запятой
  Serial.print(",");
  Serial.println(angle_y, 2);

  delay(100);  // Задержка для снижения частоты отправки данных
}

void calibrateMPU() {
  const int calibration_samples = 1000;
  long ax_sum = 0, ay_sum = 0, az_sum = 0;

  Serial.println("Калибровка MPU6050...");
  for (int i = 0; i < calibration_samples; i++) {
    ax_sum += mpu.getAccelerationX();
    ay_sum += mpu.getAccelerationY();
    az_sum += mpu.getAccelerationZ();
    delay(3);
  }

  ax_offset = ax_sum / calibration_samples;
  ay_offset = ay_sum / calibration_samples;
  az_offset = az_sum / calibration_samples;
  
  Serial.println("Калибровка завершена.");
}
