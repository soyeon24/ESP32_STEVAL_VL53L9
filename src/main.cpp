// ---------------------------------------------------------------------------
// STEVAL-VL53L9 (VL53L9CX) 브링업 진단 스케치 - 거리 측정 기능 없음
//
// 직접 납땜한 보드의 전원 / 배선 / I2C 통신만 단계별로 확인한다.
//
//   0단계  버스 유휴 레벨 (J3 점퍼가 3V3 인지 여기서 드러난다)
//   1단계  XSHUT 시퀀스로 센서 인에이블
//   2단계  I2C 주소 스캔 (0x29 센서 / 0x55 EEPROM)
//   3단계  EEPROM 읽기 - 실제 데이터가 오가는지 증명
//   4단계  센서 레지스터 읽기 시도
//   5단계  200회 연속 ACK 로 납땜 안정성 확인
//
// 안전: 센서와 EEPROM 모두 쓰기(write)를 하지 않는다. EEPROM 에는 공장
//       캘리브레이션 데이터가 들어 있을 수 있어 한 번 덮어쓰면 복구 불가다.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include "board_config.h"

static bool g_sensorSeen = false;
static bool g_eepromSeen = false;

// --- 유틸 -----------------------------------------------------------------

static void banner(const char *title) {
  Serial.println();
  Serial.println(F("==================================================="));
  Serial.print(F("  "));
  Serial.println(title);
  Serial.println(F("==================================================="));
}

// --- 0단계: 버스 유휴 레벨 ------------------------------------------------
// 보드에 2.2k 풀업(R7/R8 -> HOST_IOVDD)이 있으므로 정상이면 둘 다 HIGH 다.
// LOW 로 읽히는 가장 흔한 원인이 J3 점퍼가 1V8 쪽에 꽂힌 경우인데,
// 1.8V 는 ESP32 입력 문턱(약 2.48V)에 못 미쳐 LOW 로 보인다.
// 한 선을 3단계로 판정한다.
//   (a) 풀업 없이 읽기      : 보드의 2.2k 풀업이 그 핀까지 도달하는가
//   (b) 내부 풀업(약 45k)   : HIGH 로 올라오면 단선, LOW 면 GND 단락
//   (c) SCL 클럭 9회 후 재측정 : 슬레이브가 버스를 물고 있었는지
//
// (b) 가 핵심이다. 단선과 단락은 둘 다 LOW 로 보이지만 조치가 정반대다.
// 보드의 2.2k 가 이미 LOW 인 상태라면 내부 45k 는 더더욱 못 올린다.
// 그런데도 HIGH 가 되면, 그 핀은 애초에 보드와 연결돼 있지 않다는 뜻이다.
enum LineVerdict { LINE_OK, LINE_OPEN, LINE_SHORT, LINE_STUCK };

static LineVerdict diagnoseLine(int pin, const char *name) {
  pinMode(pin, INPUT);                 // (a) 외부 풀업만
  delay(5);
  const int raw = digitalRead(pin);

  pinMode(pin, INPUT_PULLUP);          // (b) 내부 풀업 추가
  delay(5);
  const int pulled = digitalRead(pin);

  pinMode(pin, INPUT);
  Serial.printf("  %-3s (GPIO%-2d)  풀업없음=%-4s  내부풀업=%-4s  -> ",
                name, pin, raw ? "HIGH" : "LOW", pulled ? "HIGH" : "LOW");

  if (raw) {
    Serial.println(F("정상"));
    return LINE_OK;
  }
  if (pulled) {
    Serial.println(F("단선 (연결 안 됨)"));
    return LINE_OPEN;
  }
  Serial.println(F("LOW 고착 (GND 단락 또는 슬레이브가 물고 있음)"));
  return LINE_SHORT;
}

// SCL 을 9번 두드려 전송 도중 멈춘 슬레이브가 SDA 를 놓게 만든다.
static bool recoverBus() {
  Serial.println(F("  버스 복구 시도: SCL 클럭 9회 + STOP"));

  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(5);
  }
  digitalWrite(PIN_SCL, HIGH);   delayMicroseconds(5);  // STOP 조건
  pinMode(PIN_SDA, OUTPUT);
  digitalWrite(PIN_SDA, LOW);    delayMicroseconds(5);
  pinMode(PIN_SDA, INPUT);       delayMicroseconds(5);

  pinMode(PIN_SCL, INPUT);
  delay(5);
  const bool freed = digitalRead(PIN_SDA);
  Serial.printf("  복구 후 SDA = %s\n", freed ? "HIGH" : "LOW");
  return freed;
}

static bool checkBusIdle() {
  banner("0단계: I2C 버스 유휴 레벨 확인");

  LineVerdict vSda = diagnoseLine(PIN_SDA, "SDA");
  LineVerdict vScl = diagnoseLine(PIN_SCL, "SCL");

  if (vSda == LINE_SHORT && recoverBus()) {
    Serial.println(F("  -> 슬레이브가 버스를 물고 있던 것이었다. 해제됨."));
    vSda = LINE_STUCK;
  }

  Serial.println();
  Serial.println(F("  [판정]"));

  // 한 선만 HIGH 면 전원과 풀업 전압은 정상이라는 뜻이다.
  if (vScl == LINE_OK || vSda == LINE_OK) {
    Serial.println(F("   * 보드 전원 정상, J3 점퍼도 3V3 쪽으로 확인됨."));
    Serial.println(F("     (풀업 전압이 낮으면 두 선 모두 LOW 로 보였을 것)"));
  }

  if (vSda == LINE_OK && vScl == LINE_OK) {
    Serial.println(F("   * 두 선 모두 정상."));
    return true;
  }

  for (int i = 0; i < 2; i++) {
    const LineVerdict v = i ? vScl : vSda;
    const char *n = i ? "SCL" : "SDA";
    if (v == LINE_OK || v == LINE_STUCK) continue;

    if (v == LINE_OPEN) {
      Serial.printf("   * %s 단선. 전원 끄고 확인할 것:\n", n);
      Serial.printf("       - ESP32 핀 <-> J2 의 %s 패드 도통 테스트\n", n);
      Serial.println(F("       - 냉납 의심 패드 재납땜 (플럭스 쓰고 충분히 가열)"));
      Serial.println(F("       - 점퍼선 단선 / 브레드보드 접촉 불량"));
    } else {
      Serial.printf("   * %s 가 GND 에 단락. 전원 끄고 확인할 것:\n", n);
      Serial.printf("       - %s 패드와 인접 GND 패드 사이 납 브릿지\n", n);
      Serial.printf("       - 멀티미터 저항: %s <-> GND 가 100옴 이하면 단락\n", n);
      Serial.println(F("       - 확대경으로 패드 사이 납땜 확인"));
    }
  }
  return false;
}

// --- 1단계: XSHUT 인에이블 -------------------------------------------------
static void enableSensor() {
  banner("1단계: XSHUT 로 센서 인에이블");

  if (PIN_XSHUT < 0) {
    Serial.println(F("  XSHUT 을 -1 로 뒀다. 보드에서 HIGH 로 고정돼 있지 않으면"));
    Serial.println(F("  센서는 셧다운 상태로 남아 I2C 에 응답하지 않는다."));
    return;
  }

  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);          // 확실히 리셋
  Serial.printf("  XSHUT  GPIO%-2d -> LOW  (셧다운)\n", PIN_XSHUT);
  delay(10);

  digitalWrite(PIN_XSHUT, HIGH);         // 인에이블
  Serial.printf("  XSHUT  GPIO%-2d -> HIGH (동작)\n", PIN_XSHUT);
  delay(50);                             // 내부 부팅 대기

  if (PIN_INTR >= 0) {
    pinMode(PIN_INTR, INPUT);
    Serial.printf("  INTR   GPIO%-2d <- %s\n", PIN_INTR,
                  digitalRead(PIN_INTR) ? "HIGH" : "LOW");
  }
  if (PIN_SYNC_IN >= 0) {
    pinMode(PIN_SYNC_IN, OUTPUT);
    digitalWrite(PIN_SYNC_IN, LOW);      // 단독 사용이면 LOW 로 고정
    Serial.printf("  SYNC_IN GPIO%-2d -> LOW\n", PIN_SYNC_IN);
  }
}

// --- 2단계: I2C 스캔 -------------------------------------------------------
static int scanBus() {
  banner("2단계: I2C 주소 스캔 (0x08 ~ 0x77)");

  int found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found++;
      Serial.printf("  응답: 0x%02X (8-bit 0x%02X)", addr, addr << 1);
      if (addr == TOF_I2C_ADDR_7BIT) {
        Serial.print(F("  <== VL53L9CX 센서"));
        g_sensorSeen = true;
      } else if (addr == EEPROM_I2C_ADDR_7BIT) {
        Serial.print(F("  <== M24C64 EEPROM (U4)"));
        g_eepromSeen = true;
      }
      Serial.println();
    }
  }

  Serial.println();
  if (found == 0) {
    Serial.println(F("  응답 없음. 전원 또는 배선 문제다."));
    Serial.println(F("   1) J2 의 3V3 / GND 가 실제로 연결됐는가"));
    Serial.println(F("   2) J3 점퍼 = 3V3 인가"));
    Serial.println(F("   3) SDA / SCL 이 서로 바뀌지 않았는가"));
    Serial.println(F("   4) 냉납 / 미접촉 (도통 테스트)"));
  } else if (g_eepromSeen && !g_sensorSeen) {
    // 두 칩이 같은 버스에 있으므로 이 조합은 원인을 정확히 짚어준다.
    Serial.println(F("  EEPROM 은 보이는데 센서가 없다."));
    Serial.println(F("  -> I2C 버스와 레벨 시프터는 정상이다. 센서만 문제다."));
    Serial.println(F("     XSHUT 배선 / 센서 납땜 / 온보드 LDO 출력을 확인할 것."));
  } else if (g_sensorSeen && !g_eepromSeen) {
    Serial.println(F("  센서는 보이는데 EEPROM 이 없다. (통신 자체는 정상)"));
  } else if (g_sensorSeen && g_eepromSeen) {
    Serial.println(F("  두 칩 모두 응답. 배선 상태 양호."));
  }
  return found;
}

// --- 16-bit 주소 레지스터 읽기 (센서 / EEPROM 공통) ------------------------
static bool readReg16(uint8_t devAddr, uint16_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(devAddr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission(false) != 0) return false;   // repeated start

  const size_t got = Wire.requestFrom((int)devAddr, (int)len, (int)true);
  if (got != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static void dump(uint8_t devAddr, uint16_t from, uint16_t to) {
  for (uint16_t base = from; base < to; base += 8) {
    uint8_t b[8];
    if (!readReg16(devAddr, base, b, 8)) {
      Serial.printf("  0x%04X : 읽기 실패\n", base);
      continue;
    }
    Serial.printf("  0x%04X :", base);
    for (int i = 0; i < 8; i++) Serial.printf(" %02X", b[i]);
    Serial.print(F("   |"));
    for (int i = 0; i < 8; i++)
      Serial.printf("%c", (b[i] >= 0x20 && b[i] < 0x7F) ? b[i] : '.');
    Serial.println(F("|"));
  }
}

// --- 3단계: EEPROM 읽기 ----------------------------------------------------
// ACK 만으로는 배선이 확실하다고 말할 수 없다. 실제 바이트가 오가는지 본다.
// M24C64 는 16-bit 주소 체계라 센서와 접근 방식이 같다.
static void readEeprom() {
  banner("3단계: EEPROM(0x55) 읽기 - 데이터 전송 검증");
  Serial.println(F("  * 읽기 전용. 쓰기는 하지 않는다."));

  dump(EEPROM_I2C_ADDR_7BIT, 0x0000, 0x0020);

  Serial.println();
  Serial.println(F("  값이 전부 FF 면 빈 EEPROM 일 수 있다(정상)."));
  Serial.println(F("  단, 읽기가 성공했다는 것 자체가 SDA/SCL 양방향 통신과"));
  Serial.println(F("  레벨 시프터가 제대로 동작한다는 증거다."));
}

// --- 4단계: 센서 레지스터 읽기 --------------------------------------------
static void probeSensor() {
  banner("4단계: 센서(0x29) 레지스터 읽기 시도");
  Serial.println(F("  * 읽기 전용. 설정 레지스터를 건드리지 않는다."));

  dump(TOF_I2C_ADDR_7BIT, 0x0000, 0x0020);

  Serial.println();
  Serial.println(F("  참고: VL53L9CX 의 정확한 ID 레지스터 값은 데이터시트로"));
  Serial.println(F("  대조해야 한다. 여기서는 '00/FF 가 아닌 일정한 값이"));
  Serial.println(F("  반복해서 읽히는가' 만 봐도 통신 검증으로는 충분하다."));
  Serial.println(F("  읽기가 실패해도 2단계에서 ACK 가 왔다면 배선은 정상이고,"));
  Serial.println(F("  ULD 드라이버로 펌웨어를 올려야 응답하는 상태일 수 있다."));
}

// --- 5단계: 통신 안정성 ----------------------------------------------------
static void stressCheck() {
  banner("5단계: 200회 연속 ACK 테스트");

  int okSensor = 0, okEeprom = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(TOF_I2C_ADDR_7BIT);
    if (Wire.endTransmission() == 0) okSensor++;
    Wire.beginTransmission(EEPROM_I2C_ADDR_7BIT);
    if (Wire.endTransmission() == 0) okEeprom++;
    delay(1);
  }
  Serial.printf("  센서 0x29  : %3d / 200\n", okSensor);
  Serial.printf("  EEPROM 0x55: %3d / 200\n", okEeprom);

  const int worst = min(okSensor, okEeprom);
  if (worst == 200) {
    Serial.println(F("  -> 통신 완전 안정. 다음 단계로 진행 가능."));
  } else if (worst > 0) {
    Serial.println(F("  -> 간헐적 실패. 납땜 냉납, 긴 점퍼선, 전원 리플 의심."));
    Serial.println(F("     점퍼선을 10cm 이내로 줄이고, 3V3-GND 사이에 10uF 을"));
    Serial.println(F("     보드 가까이 붙여볼 것."));
  } else {
    Serial.println(F("  -> 응답 없음."));
  }
}

// --- Arduino 엔트리 --------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  banner("STEVAL-VL53L9 브링업 진단 시작");
  Serial.printf("  I2C  SDA=GPIO%d  SCL=GPIO%d  @ %lu Hz\n",
                PIN_SDA, PIN_SCL, I2C_FREQ_HZ);
  Serial.println(F("  [확인] J3 점퍼가 3V3 쪽인지 먼저 볼 것. 1V8 이면 레벨"));
  Serial.println(F("         시프터가 3.3V 입력에 손상될 수 있다."));

  checkBusIdle();
  enableSensor();

  Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ_HZ);
  Wire.setTimeOut(50);   // 버스가 물려도 무한 대기하지 않도록

  if (scanBus() > 0) {
    if (g_eepromSeen) readEeprom();
    if (g_sensorSeen) probeSensor();
    stressCheck();
  }

  banner("진단 종료 - 이후 5초마다 재확인");
}

void loop() {
  delay(5000);

  Wire.beginTransmission(TOF_I2C_ADDR_7BIT);
  const bool s = (Wire.endTransmission() == 0);
  Wire.beginTransmission(EEPROM_I2C_ADDR_7BIT);
  const bool e = (Wire.endTransmission() == 0);

  Serial.printf("[재확인] 센서 0x29=%s  EEPROM 0x55=%s",
                s ? "ACK" : "무응답", e ? "ACK" : "무응답");
  if (PIN_INTR >= 0) Serial.printf("  | INTR=%s", digitalRead(PIN_INTR) ? "H" : "L");
  Serial.println();
}
