#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float alpha_gyro = 0.05;
Servo motor;

const int ENA = 3;
const int ENB = 5;
const int In1 = 2;
const int In2 = 4;
const int In3 = 7;
const int In4 = 8;
const int ledPin = 10;

float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

float pitch = 0, roll = 0;
unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("Ошибка подключения к MPU6050");
    while (1);
  }

  Serial.println("MPU6050 подключен");

  calibrateMPU();
  mpu.setDLPFMode(6);

  motor.attach(11);
  motor.writeMicroseconds(2300);
  delay(2000);
  motor.writeMicroseconds(800);
  delay(6000);
  motor.detach();

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  stopPump1();
  stopPump2();

  prevTime = millis();
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("P1:")) {
      int speed1 = input.substring(3).toInt();
      controlPump1(speed1);
    } else if (input.startsWith("P2:")) {
      int speed2 = input.substring(3).toInt();
      controlPump2(speed2);
    }
  }

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;

  // Углы через акселерометр и арксинус (как в примере)
  float ax_norm = constrain(ax, -16384, 16384) / 16384.0;
  float ay_norm = constrain(ay, -16384, 16384) / 16384.0;

  if (ax_norm < 0) pitch = 90 - degrees(acos(ax_norm));
  else pitch = degrees(acos(-ax_norm)) - 90;

  if (ay_norm < 0) roll = 90 - degrees(acos(ay_norm));
  else roll = degrees(acos(-ay_norm)) - 90;

  Serial.print(pitch);
  Serial.print(",");
  Serial.println(-roll);

  delay(100);
}

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
    stopPump1();
    return;
  }
  if (speed > 0) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
  } else {
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    speed = -speed;
  }
  analogWrite(ENA, map(speed, 150, 255, 150, 255));
}

void controlPump2(int speed) {
  if (speed == 0) {
    stopPump2();
    return;
  }
  if (speed > 0) {
    digitalWrite(In3, LOW);
    digitalWrite(In4, HIGH);
  } else {
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
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
