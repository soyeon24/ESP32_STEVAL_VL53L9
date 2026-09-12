// ---------------------------------------------------------------------------
// STEVAL-VL53L9 (VL53L9CX) I2C 버스 진단 스케치
//
// 증상: 0x29 로 나간 주소 바이트 직후 9클럭에서 트랜잭션이 끝난다 = 주소 NACK.
//       (정상이라면 addr-W + reg_hi + reg_lo + ReSTART + addr-R + data = 45클럭)
//
// 이 스케치가 순서대로 확인하는 것:
//   [1] SDA/SCL 에 외부 풀업(R7/R8 = 2.2k)이 실제로 붙어 있는가
//   [2] 버스를 능동적으로 LOW 로 물고 있는 놈이 없는가 (stuck bus)
//   [3] XSHUT = LOW  상태에서 ACK 하는 주소 목록 (센서를 뺀 나머지. 예: EEPROM)
//   [4] XSHUT = HIGH 상태에서 ACK 하는 주소 목록
//       [4] - [3] 의 차집합이 ToF 센서다. 데이터시트 주소를 몰라도 잡힌다.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include "board_config.h"

#define MAX_FOUND 16

static uint8_t g_found[MAX_FOUND];   // XSHUT HIGH 에서 마지막으로 찾은 주소들
static size_t  g_foundCount = 0;

static void banner(const char *title) {
  Serial.println();
  Serial.println(F("==================================================="));
  Serial.print(F("  "));
  Serial.println(title);
  Serial.println(F("==================================================="));
}

// ---------------------------------------------------------------------------
// [1][2] Wire 가 핀을 가져가기 전에 물리 레벨부터 본다.
//
// 세 가지 조건으로 읽어서 라인 상태를 확정한다.
//   float    : 아무것도 안 걸고 읽기
//   pull-up  : ESP32 내부 풀업 (약 45k)
//   pull-down: ESP32 내부 풀다운 (약 45k)
//
//   pulldown 에서 HIGH         -> 외부 2.2k 풀업이 45k 를 이김. 배선 정상.
//   pulldown LOW + pullup HIGH -> 외부 풀업 없음. 선 미연결이거나 다른 핀에 꽂힘.
//   pulldown LOW + pullup LOW  -> 누가 능동적으로 LOW 로 잡고 있음 (단락/stuck).
// ---------------------------------------------------------------------------
typedef enum { LINE_OK, LINE_NO_PULLUP, LINE_STUCK_LOW } line_state_t;

static line_state_t probeLine(int pin, const char *name) {
  pinMode(pin, INPUT);
  delayMicroseconds(500);
  const int vFloat = digitalRead(pin);

  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(500);
  const int vUp = digitalRead(pin);

  pinMode(pin, INPUT_PULLDOWN);
  delayMicroseconds(500);
  const int vDown = digitalRead(pin);

  pinMode(pin, INPUT);

  line_state_t st;
  if (vDown == HIGH)    st = LINE_OK;
  else if (vUp == HIGH) st = LINE_NO_PULLUP;
  else                  st = LINE_STUCK_LOW;

  const char *verdict =
      (st == LINE_OK)        ? "외부 풀업 확인 -> 배선 정상"
    : (st == LINE_NO_PULLUP) ? "외부 풀업 없음 -> 선 미연결 / 오배선 의심"
                             : "능동 LOW -> GND 단락 또는 stuck bus";

  Serial.printf("  %-3s (GPIO%02d)  float=%d  pullup=%d  pulldown=%d   %s\n",
                name, pin, vFloat, vUp, vDown, verdict);
  return st;
}

// SDA 가 물려 있을 때 SCL 을 9번 때려서 슬레이브를 풀어주는 표준 복구 루틴.
static void recoverBus() {
  Serial.println(F("  -> SDA stuck. SCL 9클럭 + STOP 으로 복구 시도"));
  pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_SDA, INPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
    if (digitalRead(PIN_SDA) == HIGH) break;
  }
  // STOP 컨디션: SCL HIGH 인 동안 SDA 를 LOW -> HIGH
  pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_SDA, HIGH); delayMicroseconds(5);
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
}

// ---------------------------------------------------------------------------
// [3][4] 주소 스윕. 0x08 ~ 0x77 (예약 주소 제외)
//
// endTransmission() 리턴 코드 집계가 핵심이다.
//   전부 2 (addr NACK) -> 버스는 멀쩡한데 아무도 안 받는다 = 전기적 문제 아님
//   전부 5 (timeout)   -> 버스가 죽었다 = 배선/풀업/전원 문제
// ---------------------------------------------------------------------------
static size_t scanBus(uint8_t *found, size_t maxFound) {
  size_t n = 0;
  uint16_t tally[8] = {0};

  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    const uint8_t rc = Wire.endTransmission(true);
    if (rc < 8) tally[rc]++;
    if (rc == 0) {
      Serial.printf("  [ACK] 7-bit 0x%02X   (8-bit  W=0x%02X  R=0x%02X)\n",
                    addr, (uint8_t)(addr << 1), (uint8_t)((addr << 1) | 1));
      if (n < maxFound) found[n++] = addr;
    }
  }

  Serial.printf("  ACK %u | addr-NACK %u | data-NACK %u | bus-err %u | timeout %u\n",
                (unsigned)tally[0], (unsigned)tally[2], (unsigned)tally[3],
                (unsigned)tally[4], (unsigned)tally[5]);
  if (n == 0) Serial.println(F("  (응답 없음)"));
  return n;
}

static void setXshut(bool enable) {
  if (PIN_XSHUT < 0) {
    Serial.println(F("  XSHUT 미정의 (-1). 건너뜀."));
    return;
  }
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, enable ? HIGH : LOW);
  Serial.printf("  XSHUT (GPIO%d) = %s\n",
                PIN_XSHUT, enable ? "HIGH (동작)" : "LOW (셧다운)");
}

// 16-bit 주소 레지스터 읽기. 스코프로 45클럭 전체 트랜잭션을 보기 위한 용도.
static bool readReg16(uint8_t devAddr, uint16_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(devAddr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission(false) != 0) return false;

  if ((int)Wire.requestFrom((int)devAddr, (int)len, (int)true) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  banner("[1/4] I2C 라인 물리 상태 (Wire.begin 이전)");
  const line_state_t sda = probeLine(PIN_SDA, "SDA");
  const line_state_t scl = probeLine(PIN_SCL, "SCL");
  if (sda == LINE_STUCK_LOW) recoverBus();

  if (sda != LINE_OK || scl != LINE_OK) {
    Serial.println(F("  *** 라인이 정상이 아니다. 아래 스캔 결과는 의미 없다. ***"));
    Serial.println(F("      SDA/SCL 배선, GND 공통, J3 점퍼(3V3 쪽) 부터 확인할 것."));
  }

  Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ_HZ);
  Wire.setTimeOut(20);
  Serial.printf("\n  I2C %lu Hz, SDA=GPIO%d, SCL=GPIO%d\n",
                (unsigned long)I2C_FREQ_HZ, PIN_SDA, PIN_SCL);

  banner("[2/4] XSHUT = LOW 스캔 (센서 셧다운)");
  setXshut(false);
  delay(50);
  uint8_t base[MAX_FOUND];
  const size_t baseCount = scanBus(base, MAX_FOUND);

  banner("[3/4] XSHUT = HIGH 스캔 (센서 동작)");
  setXshut(true);
  delay(200);                       // 부팅 대기
  g_foundCount = scanBus(g_found, MAX_FOUND);

  banner("[4/4] 판정");
  bool newDevice = false;
  for (size_t i = 0; i < g_foundCount; i++) {
    bool wasThere = false;
    for (size_t j = 0; j < baseCount; j++) {
      if (base[j] == g_found[i]) { wasThere = true; break; }
    }
    if (!wasThere) {
      Serial.printf("  >>> XSHUT HIGH 에서만 나타난 주소: 0x%02X  <-- ToF 센서\n",
                    g_found[i]);
      newDevice = true;
    }
  }

  if (g_foundCount == 0) {
    Serial.println(F("  아무것도 ACK 하지 않음."));
    Serial.println(F("   -> 보드 레벨 문제. GND 공통 누락, J3 점퍼가 1V8 쪽,"));
    Serial.println(F("      레벨시프터(U1/U2) 전원/OE 를 의심."));
  } else if (!newDevice) {
    Serial.println(F("  응답은 있으나 XSHUT 로 변하는 장치가 없다."));
    Serial.println(F("   -> 버스/전원/레벨시프터는 정상. 센서만 안 깨어난다."));
    Serial.println(F("      XSHUT 배선(GPIO32 <-> J2), 센서측 1V8/2V8 레일,"));
    Serial.println(F("      또는 I2C/SPI 인터페이스 선택 스트랩을 확인할 것."));
  }

  bool seenTof = false, seenEep = false;
  for (size_t i = 0; i < g_foundCount; i++) {
    if (g_found[i] == TOF_I2C_ADDR_7BIT)    seenTof = true;
    if (g_found[i] == EEPROM_I2C_ADDR_7BIT) seenEep = true;
  }
  Serial.printf("  기대 주소 점검:  ToF 0x%02X = %s   EEPROM 0x%02X = %s\n",
                TOF_I2C_ADDR_7BIT,    seenTof ? "응답 O" : "응답 X",
                EEPROM_I2C_ADDR_7BIT, seenEep ? "응답 O" : "응답 X");

  banner("루프 진입 (3초마다 재스캔)");
}

void loop() {
  delay(3000);

  g_foundCount = scanBus(g_found, MAX_FOUND);

  // 응답한 주소가 있으면 실제 레지스터 읽기까지 해본다 (스코프로 45클럭 확인용).
  for (size_t i = 0; i < g_foundCount; i++) {
    uint8_t data = 0;
    if (readReg16(g_found[i], 0x010F, &data, 1)) {
      Serial.printf("  0x%02X  reg 0x010F = 0x%02X\n", g_found[i], data);
    } else {
      Serial.printf("  0x%02X  ACK 는 하는데 reg 0x010F 읽기 실패\n", g_found[i]);
    }
  }
  Serial.println(F("---------------------------------------------------"));
}
