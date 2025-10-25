// esp32_i2c_slave_fixed_channels.ino
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>

// I2C
static const uint8_t I2C_ADDR = 0x42;
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;
static const uint32_t I2C_FREQ = 100000;

// Pins (как у тебя)
const int P1_A_PIN = 16;   // помпа1 A-input (PWM)
const int P1_B_PIN = 4;    // помпа1 B-input (PWM)

const int P2_A_PIN = 17;   // помпа2 A-input (PWM)
const int P2_B_PIN = 5;    // помпа2 B-input (PWM)

const int MOTOR_PIN = 14;  // ESC signal (Servo)
const int LIGHT_PIN = 23;  // свет

static const bool LIGHT_ACTIVE_LOW = true;

// PWM + channels
static const uint32_t PWM_FREQ = 1000;
static const uint8_t PWM_BITS = 8; // 0..255

// moved channels away from low numbers (0..3) to reduce chance of conflicts
static const int CH_P1A = 4;
static const int CH_P1B = 5;
static const int CH_P2A = 6;
static const int CH_P2B = 7;

// ESC (servo)
static const int ESC_MIN_US  = 1000;
static const int ESC_MAX_US  = 2000;
static const int ESC_NEUTRAL = 1500;
static const int ESC_ARM_US  = 1000;
static const int ESC_ARM_MS  = 1500;

static const int ESC_SLEW_US_PER_STEP = 25;
static const uint32_t ESC_SLEW_INTERVAL_MS = 15;

// globals
Servo esc;
unsigned long lastSlewStamp = 0;
int targetEscUs = ESC_NEUTRAL;
int currentEscUs = ESC_NEUTRAL;

volatile int g_P1 = 0;
volatile int g_P2 = 0;
volatile int g_M  = 0;
volatile bool g_light = false;

// I2C RX buffer
static const size_t RXBUF_SIZE = 128;
volatile uint8_t rxbuf[RXBUF_SIZE];
volatile size_t rxlen = 0;
volatile bool rx_ready = false;

char resp_buf[32] = "ACK;L:0;M:0;P1:0;P2:0";

void controlPumpRaw(int pwmA, int pwmB, int chA, int chB) {
  pwmA = constrain(pwmA, 0, 255);
  pwmB = constrain(pwmB, 0, 255);
  ledcWrite(chA, pwmA);
  ledcWrite(chB, pwmB);
}

void controlPump(int speed, int a_pin, int b_pin, int chA, int chB) {
  int s = constrain(speed, -255, 255);
  int pA = 0, pB = 0;
  if (s > 0) { pA = s; pB = 0; }
  else if (s < 0) { pA = 0; pB = -s; }
  else { pA = 0; pB = 0; }
  controlPumpRaw(pA, pB, chA, chB);
}

void controlLight(bool on) {
  if (LIGHT_ACTIVE_LOW) digitalWrite(LIGHT_PIN, on ? LOW : HIGH);
  else digitalWrite(LIGHT_PIN, on ? HIGH : LOW);
  g_light = on;
}

void controlMotorPWM(int pwmValue) {
  int pwm = constrain(pwmValue, -255, 255);
  int pulse = map(pwm, -255, 255, ESC_MIN_US, ESC_MAX_US);
  targetEscUs = constrain(pulse, ESC_MIN_US, ESC_MAX_US);
  g_M = pwmValue;
}

void applyEscSlew() {
  if (currentEscUs == targetEscUs) return;
  int delta = targetEscUs - currentEscUs;
  int step = ESC_SLEW_US_PER_STEP;
  if (abs(delta) <= step) currentEscUs = targetEscUs;
  else currentEscUs += (delta > 0) ? step : -step;
  currentEscUs = constrain(currentEscUs, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(currentEscUs);
}

void updateResp() {
  int l = g_light ? 1 : 0;
  snprintf(resp_buf, sizeof(resp_buf), "ACK;L:%d;M:%d;P1:%d;P2:%d", l, g_M, g_P1, g_P2);
}

void handleToken(const String& token) {
  if (token.startsWith("P1:")) {
    int v = token.substring(3).toInt();
    g_P1 = constrain(v, -255, 255);
    controlPump(g_P1, P1_A_PIN, P1_B_PIN, CH_P1A, CH_P1B);
    Serial.printf("ACT: set P1=%d\n", g_P1);
  } else if (token.startsWith("P2:")) {
    int v = token.substring(3).toInt();
    g_P2 = constrain(v, -255, 255);
    controlPump(g_P2, P2_A_PIN, P2_B_PIN, CH_P2A, CH_P2B);
    Serial.printf("ACT: set P2=%d\n", g_P2);
  } else if (token.startsWith("M:")) {
    int v = token.substring(2).toInt();
    controlMotorPWM(v);
    Serial.printf("ACT: set M=%d => targetPulse=%d\n", v, targetEscUs);
  } else if (token.startsWith("L:")) {
    String s = token.substring(2); s.toLowerCase();
    bool on = (s == "1" || s == "on" || s == "true");
    controlLight(on);
    Serial.printf("ACT: set L=%d\n", on ? 1 : 0);
  } else if (token.equalsIgnoreCase("STOP")) {
    g_P1 = g_P2 = 0;
    controlPump(0, P1_A_PIN, P1_B_PIN, CH_P1A, CH_P1B);
    controlPump(0, P2_A_PIN, P2_B_PIN, CH_P2A, CH_P2B);
    targetEscUs = ESC_NEUTRAL;
    controlLight(false);
    Serial.println("ACT: STOP");
  }
  updateResp();
}

void processLine(const String& line) {
  int start = 0;
  while (start < (int)line.length()) {
    int sep = line.indexOf('|', start);
    String token = (sep == -1) ? line.substring(start) : line.substring(start, sep);
    token.trim();
    if (token.length() > 0) handleToken(token);
    if (sep == -1) break;
    start = sep + 1;
  }
}

void onReceiveISR(int len) {
  size_t n = 0;
  bool first = true;
  while (Wire.available() && n < RXBUF_SIZE) {
    uint8_t b = (uint8_t)Wire.read();
    if (first) {
      first = false;
      if (b == 0x00) continue;
    }
    if (b == 0x00) continue;
    rxbuf[n++] = b;
  }
  rxlen = n;
  rx_ready = true;
}

void onRequestISR() {
  Wire.write((const uint8_t*)resp_buf, strnlen(resp_buf, sizeof(resp_buf)));
}

void setup() {
  Serial.begin(115200);
  delay(20);
  Serial.println("ESP32 I2C SLAVE (channels changed) @0x42 starting...");

  pinMode(P1_A_PIN, OUTPUT); pinMode(P1_B_PIN, OUTPUT);
  pinMode(P2_A_PIN, OUTPUT); pinMode(P2_B_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);

  // use channels 4..7 for pumps to avoid low-number conflicts
  ledcSetup(CH_P1A, PWM_FREQ, PWM_BITS);
  ledcSetup(CH_P1B, PWM_FREQ, PWM_BITS);
  ledcSetup(CH_P2A, PWM_FREQ, PWM_BITS);
  ledcSetup(CH_P2B, PWM_FREQ, PWM_BITS);

  ledcAttachPin(P1_A_PIN, CH_P1A);
  ledcAttachPin(P1_B_PIN, CH_P1B);
  ledcAttachPin(P2_A_PIN, CH_P2A);
  ledcAttachPin(P2_B_PIN, CH_P2B);

  esc.setPeriodHertz(50);
  esc.attach(MOTOR_PIN, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(ESC_ARM_US);
  delay(ESC_ARM_MS);
  esc.writeMicroseconds(ESC_NEUTRAL);
  currentEscUs = targetEscUs = ESC_NEUTRAL;

  controlPump(0, P1_A_PIN, P1_B_PIN, CH_P1A, CH_P1B);
  controlPump(0, P2_A_PIN, P2_B_PIN, CH_P2A, CH_P2B);
  controlLight(false);
  updateResp();

  Serial.println("I2C begin...");
  Wire.begin(I2C_ADDR, I2C_SDA, I2C_SCL, I2C_FREQ);
  Wire.onReceive(onReceiveISR);
  Wire.onRequest(onRequestISR);
  Serial.println("Ready.");
}

void loop() {
  if (rx_ready) {
    noInterrupts();
    size_t n = rxlen; rxlen = 0; rx_ready = false;
    uint8_t tmp[RXBUF_SIZE];
    memcpy(tmp, (const void*)rxbuf, n);
    interrupts();

    String line; line.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      uint8_t b = tmp[i];
      if (b == '\n' || b == '\r') break;
      if (b < 0x20 || b > 0x7E) continue;
      line += (char)b;
    }
    line.trim();
    if (line.length() > 0) {
      Serial.print("CMD: '"); Serial.print(line); Serial.println("'");
      processLine(line);
      Serial.print("RESP: "); Serial.println(resp_buf);
    }
  }

  unsigned long now = millis();
  if (now - lastSlewStamp >= ESC_SLEW_INTERVAL_MS) {
    applyEscSlew();
    lastSlewStamp = now;
  }
  delay(1);
}
