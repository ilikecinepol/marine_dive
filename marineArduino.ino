// ===== ESP32 I²C-Slave control firmware (addr 0x42) =====
// Управляет: 2 помпы (H-бридж ENA/ENB), ESC, руль (опционально), свет.
// Принимает команды по I²C от Raspberry Pi (мастер):
//   "P1:-255..255|P2:-255..255|M:-255..255|R:-100..100|L:0/1\n"
// Отвечает на чтение мастера (<=32 байт) строкой статуса вида:
//   "ACK;L:1;M:0;P1:0;P2:0"
//
// Совместимо и с i2c_msg.write (сырые байты),
// и с write_i2c_block_data(addr, 0, [...]) — в ISR игнорируем первый «регистр».

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>

/* ================== I2C ================== */
static const uint8_t  I2C_ADDR  = 0x42;
static const int      I2C_SDA   = 21;
static const int      I2C_SCL   = 22;
static const uint32_t I2C_FREQ  = 100000;   // 100 кГц — надёжнее на длинных проводах

/* ================== PWM/ESC ================== */
static const uint32_t PWM_FREQ  = 1000;
static const uint8_t  PWM_BITS  = 8;

static const int ESC_MIN_US   = 1000;
static const int ESC_MAX_US   = 2000;
static const int ESC_NEUTRAL  = 1500;
static const int ESC_ARM_US   = 1000;
static const int ESC_ARM_MS   = 3000;

static const int ESC_SLEW_US_PER_STEP      = 25;
static const uint32_t ESC_SLEW_INTERVAL_MS = 15;

/* ========== Пины (как у тебя) ========== */
const int IN1 = 18;
const int IN2 = 5;
const int IN3 = 17;
const int IN4 = 16;
const int ENA = 19;     // PWM ch0
const int ENB = 4;      // PWM ch1

const int MOTOR_PIN  = 14;   // ESC сигнал
const int LIGHT_PIN  = 23;   // свет (active-HIGH)

#define USE_RUDDER 1
#if USE_RUDDER
const int RUDDER_PIN    = 27;
const int RUDDER_MIN_US = 1000;
const int RUDDER_MAX_US = 2000;
#endif

/* ================== Глобальные ================== */
Servo esc;
#if USE_RUDDER
Servo rudder;
#endif

static const int CH_ENA = 0;
static const int CH_ENB = 1;

unsigned long lastSlewStamp = 0;
int targetEscUs  = ESC_NEUTRAL;
int currentEscUs = ESC_NEUTRAL;

volatile bool g_lightOn = false;
volatile int  g_M = 0, g_P1 = 0, g_P2 = 0, g_R = 0;

/* ===== I2C rx буфер из ISR → парсим в loop() ===== */
static const size_t RXBUF_SIZE = 128;
volatile uint8_t rxbuf[RXBUF_SIZE];
volatile size_t  rxlen = 0;
volatile bool    rx_ready = false;

/* Ответ мастеру при onRequest (<= 32 байт) */
char resp_buf[32] = "ACK";

/* ================== Исполнители ================== */
void controlPump(int speed, int inA, int inB, int pwm_ch) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    ledcWrite(pwm_ch, 0);
  } else if (speed > 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    ledcWrite(pwm_ch, speed);
  } else {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    ledcWrite(pwm_ch, -speed);
  }
}

void controlMotorPWM(int pwmValue) {
  int pwm = constrain(pwmValue, -255, 255);
  int pulse = map(pwm, -255, 255, ESC_MIN_US, ESC_MAX_US);
  targetEscUs = pulse; // плавно дойдём
}

void controlLight(bool on) {
  // Аппарат активный-низкий: логическая «1 = ВКЛ» -> уровень LOW на пине
  digitalWrite(LIGHT_PIN, on ? LOW : HIGH);
}


#if USE_RUDDER
void controlRudderVal(int valNeg100to100) {
  int v = constrain(valNeg100to100, -100, 100);
  int pulse = map(v, -100, 100, RUDDER_MIN_US, RUDDER_MAX_US);
  rudder.writeMicroseconds(pulse);
}
#endif

void applyEscSlew() {
  if (currentEscUs == targetEscUs) return;
  int delta = targetEscUs - currentEscUs;
  int step  = ESC_SLEW_US_PER_STEP;
  if (abs(delta) <= step) currentEscUs = targetEscUs;
  else currentEscUs += (delta > 0) ? step : -step;
  currentEscUs = constrain(currentEscUs, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(currentEscUs);
}

/* ================== Парсер команд ================== */
int safeToInt(const String& s, int def = 0) {
  String x = s; x.trim();
  if (!x.length()) return def;
  bool neg = (x[0] == '-');
  long val = 0;
  for (int i = neg ? 1 : 0; i < x.length(); ++i) {
    char c = x[i];
    if (c < '0' || c > '9') break;
    val = val * 10 + (c - '0');
    if (val > 100000) break;
  }
  if (neg) val = -val;
  if (val >  32767) val =  32767;
  if (val < -32768) val = -32768;
  return (int)val;
}

void handleToken(const String& token) {
  if (token.startsWith("P1:")) {
    g_P1 = safeToInt(token.substring(3));
    controlPump(g_P1, IN1, IN2, CH_ENA);
  }
  else if (token.startsWith("P2:")) {
    g_P2 = safeToInt(token.substring(3));
    controlPump(g_P2, IN3, IN4, CH_ENB);
  }
  else if (token.startsWith("M:")) {
    g_M = safeToInt(token.substring(2));
    controlMotorPWM(g_M);
  }
  else if (token.startsWith("L:")) {
    String s = token.substring(2); s.toLowerCase();
    bool on = (s == "1" || s == "on" || s == "true");
    controlLight(on);
  }
#if USE_RUDDER
  else if (token.startsWith("R:")) {
    g_R = safeToInt(token.substring(2));
    controlRudderVal(g_R);
  }
#endif
  else if (token.equalsIgnoreCase("STOP")) {
    controlPump(0, IN1, IN2, CH_ENA);
    controlPump(0, IN3, IN4, CH_ENB);
    targetEscUs = ESC_NEUTRAL;
    esc.writeMicroseconds(ESC_NEUTRAL);
    controlLight(false);
#if USE_RUDDER
    rudder.writeMicroseconds((RUDDER_MIN_US + RUDDER_MAX_US) / 2);
#endif
  }
  // PING/HB — ответим статусом в onRequest
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
  // Обновим короткий статус (<=32 байт!)
  snprintf(resp_buf, sizeof(resp_buf), "ACK;L:%d;M:%d;P1:%d;P2:%d",
           g_lightOn ? 1 : 0, g_M, g_P1, g_P2);
}

/* ================== I2C ISR-хендлеры ================== */
// Совместимо с двумя стилями записи мастера:
// 1) «Сырые» байты (i2c_msg.write):       [ 'L',':','1','\n' ]
// 2) write_i2c_block_data(addr, 0, data): [ 0x00, 'L',':','1','\n' ]
void onReceiveISR(int len) {
  size_t n = 0;
  bool first = true;
  while (Wire.available() && n < RXBUF_SIZE) {
    uint8_t b = (uint8_t)Wire.read();
    if (first) {               // отбрасываем первый «регистр» (если он был)
      first = false;
      if (b == 0x00) continue; // типичный случай для write_block_data
      // если мастер шлёт сырые байты — первый символ может быть полезным
      // тогда просто записываем его:
    }
    if (b == 0x00) continue;   // игнорируем случайные нули
    rxbuf[n++] = b;
  }
  rxlen = n;
  rx_ready = true;
}

void onRequestISR() {
  Wire.write((const uint8_t*)resp_buf, strnlen(resp_buf, sizeof(resp_buf)));
}

/* ================== setup/loop ================== */
void setup() {
  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, LOW);

  // PWM для помп
  ledcSetup(CH_ENA, PWM_FREQ, PWM_BITS);
  ledcAttachPin(ENA, CH_ENA);
  ledcSetup(CH_ENB, PWM_FREQ, PWM_BITS);
  ledcAttachPin(ENB, CH_ENB);

  // ESC
  esc.setPeriodHertz(50);
  esc.attach(MOTOR_PIN, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(ESC_ARM_US);
  delay(ESC_ARM_MS);
  esc.writeMicroseconds(ESC_NEUTRAL);
  currentEscUs = targetEscUs = ESC_NEUTRAL;

#if USE_RUDDER
  rudder.setPeriodHertz(50);
  rudder.attach(RUDDER_PIN, RUDDER_MIN_US, RUDDER_MAX_US);
  rudder.writeMicroseconds((RUDDER_MIN_US + RUDDER_MAX_US) / 2);
#endif

  Serial.begin(115200);
  Serial.println("ESP32 I2C SLAVE READY @0x42");

  // Важно: сначала begin, затем onReceive/onRequest
  Wire.begin((uint8_t)I2C_ADDR, I2C_SDA, I2C_SCL, I2C_FREQ);
  Wire.onReceive(onReceiveISR);
  Wire.onRequest(onRequestISR);
}

void loop() {
  // Обрабатываем полученную строку вне ISR
  if (rx_ready) {
    noInterrupts();
    size_t n = rxlen; rxlen = 0; rx_ready = false;
    uint8_t tmp[RXBUF_SIZE];
    memcpy(tmp, (const void*)rxbuf, n);
    interrupts();

    // Собираем печатную ASCII-строку до '\n'
    String line; line.reserve(n);
    for (size_t i = 0; i < n; i++) {
      uint8_t b = tmp[i];
      if (b == '\n' || b == '\r') break;
      if (b < 0x20 || b > 0x7E) continue; // только печатные ASCII
      line += (char)b;
    }
    line.trim();
    if (line.length() > 0) {
      processLine(line);
      // Лёгкий лог (вне ISR)
      Serial.print("CMD: ");  Serial.println(line);
      Serial.print("RESP: "); Serial.println(resp_buf);
    }
  }

  // ESC плавность
  unsigned long now = millis();
  if (now - lastSlewStamp >= ESC_SLEW_INTERVAL_MS) {
    applyEscSlew();
    lastSlewStamp = now;
  }

  delay(1);
}
