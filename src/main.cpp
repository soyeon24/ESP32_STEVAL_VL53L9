// ---------------------------------------------------------------------------
// VL53L9CX I2C 통신 계층 + 레지스터 맵 탐색
//
// 0x29 응답이 확인됐다 (R25 제거로 Y1 12MHz 활성화 후). 이제 통신 자체를 쓸 수
// 있게 만든다.
//
// 문제: VL53L9CX 의 레지스터 맵을 모른다. 이전 스케치가 읽던 0x010F 는
//       VL53L5CX/L8CX 관례에서 가져온 값이고 L9 에 대한 근거가 없다.
//       실제로 0x00 이 나온다.
//
// 그래서 이 스케치는 두 가지를 한다:
//   1. 재사용 가능한 I2C 읽기/쓰기 API (16-bit 레지스터 주소, 다중 바이트)
//   2. 맵을 실측으로 찾기 위한 탐색 루틴
//        [1] 존재 확인
//        [2] 다중 바이트 읽기가 주소 자동증가를 하는지
//        [3] 전체 스윕에서 non-zero 영역이 어디인지
//        [4] 읽을 때마다 값이 변하는 레지스터가 있는지 (살아있는 레지스터 파일)
//
// 안전: 이 보드에는 VCSEL 2개와 레이저 드라이버가 있다. 맵을 모르는 상태에서
//       임의 레지스터에 쓰기를 하면 레이저 구동 설정을 건드릴 수 있다.
//       tofWrite() 는 API 로 제공하되 이 스케치에서 호출하지 않는다.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include "board_config.h"

// ESP32 Arduino 코어의 Wire 버퍼는 128바이트. 그보다 작게 잡는다.
#define CHUNK            32

// 스윕 범위. 필요하면 좁혀서 다시 돌린다.
#define SWEEP_START      0x0000UL
#define SWEEP_END        0xFFFFUL

// non-zero 행이 너무 많으면 시리얼이 막힌다. 집계는 전부 하고 출력만 제한.
#define MAX_PRINT_ROWS   64

static void banner(const char *title) {
  Serial.println();
  Serial.println(F("==================================================="));
  Serial.print(F("  "));
  Serial.println(title);
  Serial.println(F("==================================================="));
}

// ---------------------------------------------------------------------------
// I2C 통신 계층
//
// 반환값 0 = 성공. 그 외는 진단에 쓰라고 원인을 구분해 돌려준다.
//   1..5  : Wire.endTransmission() 코드 (2=주소 NACK, 3=데이터 NACK, 5=타임아웃)
//   0xFE  : 요청한 길이만큼 못 받음
//   0xFD  : 인자 오류
// ---------------------------------------------------------------------------
static uint8_t tofRead(uint16_t reg, uint8_t *buf, size_t len) {
  if (!buf || len == 0 || len > CHUNK) return 0xFD;

  Wire.beginTransmission(TOF_I2C_ADDR_7BIT);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  const uint8_t rc = Wire.endTransmission(false);   // repeated start 유지
  if (rc != 0) return rc;

  const size_t got = Wire.requestFrom((int)TOF_I2C_ADDR_7BIT, (int)len, (int)true);
  if (got != len) return 0xFE;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return 0;
}

// 맵을 모르는 동안에는 호출하지 말 것. 위 안전 주석 참고.
static uint8_t tofWrite(uint16_t reg, const uint8_t *buf, size_t len) {
  if (!buf || len == 0) return 0xFD;

  Wire.beginTransmission(TOF_I2C_ADDR_7BIT);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  for (size_t i = 0; i < len; i++) Wire.write(buf[i]);
  return Wire.endTransmission(true);
}

static uint8_t tofRead8(uint16_t reg, uint8_t *out) {
  return tofRead(reg, out, 1);
}

static uint8_t tofRead16(uint16_t reg, uint16_t *out) {
  uint8_t b[2];
  const uint8_t rc = tofRead(reg, b, 2);
  if (rc == 0) *out = ((uint16_t)b[0] << 8) | b[1];   // big-endian 가정
  return rc;
}

static const char *rcText(uint8_t rc) {
  switch (rc) {
    case 0:    return "성공";
    case 2:    return "주소 NACK";
    case 3:    return "데이터 NACK";
    case 4:    return "버스 에러";
    case 5:    return "타임아웃";
    case 0xFE: return "길이 부족";
    case 0xFD: return "인자 오류";
    default:   return "기타";
  }
}

static void enableSensor() {
  if (PIN_XSHUT < 0) return;
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_XSHUT, HIGH);
  delay(200);                       // 부팅 대기
}

// ---------------------------------------------------------------------------
// [1] 존재 확인
// ---------------------------------------------------------------------------
static bool probe() {
  Wire.beginTransmission(TOF_I2C_ADDR_7BIT);
  const uint8_t rc = Wire.endTransmission(true);
  Serial.printf("  0x%02X 주소 응답: %s (rc=%u)\n", TOF_I2C_ADDR_7BIT, rcText(rc), rc);
  return rc == 0;
}

// ---------------------------------------------------------------------------
// [2] 다중 바이트 읽기가 주소 자동증가를 하는가
//
// 이게 안 되면 스윕 결과를 신뢰할 수 없다. 한 번에 4바이트 읽은 것과
// 1바이트씩 4번 읽은 것을 비교한다.
//   일치            -> 자동증가 O. 청크 스윕 유효
//   전부 같은 값    -> 자동증가 X. 같은 레지스터를 4번 읽은 것
// ---------------------------------------------------------------------------
static bool checkAutoIncrement(uint16_t base) {
  uint8_t blk[4], one[4];

  uint8_t rc = tofRead(base, blk, 4);
  if (rc != 0) {
    Serial.printf("  블록 읽기 실패: %s\n", rcText(rc));
    return false;
  }
  for (int i = 0; i < 4; i++) {
    rc = tofRead8(base + i, &one[i]);
    if (rc != 0) {
      Serial.printf("  단일 읽기 실패 @0x%04X: %s\n", base + i, rcText(rc));
      return false;
    }
  }

  Serial.printf("  블록 4B  @0x%04X: %02X %02X %02X %02X\n", base, blk[0], blk[1], blk[2], blk[3]);
  Serial.printf("  단일 4회 @0x%04X: %02X %02X %02X %02X\n", base, one[0], one[1], one[2], one[3]);

  const bool same = (memcmp(blk, one, 4) == 0);
  const bool flat = (blk[0] == blk[1] && blk[1] == blk[2] && blk[2] == blk[3]);

  if (same && !flat)      Serial.println(F("  -> 자동증가 동작. 청크 스윕 유효."));
  else if (same && flat)  Serial.println(F("  -> 값이 전부 같아 판정 불가. 다른 주소에서 재시도 필요."));
  else                    Serial.println(F("  -> !! 자동증가 안 함. 스윕은 1바이트씩 해야 한다."));
  return same;
}

// ---------------------------------------------------------------------------
// [3] 전체 스윕. non-zero 만 출력한다.
// ---------------------------------------------------------------------------
static void sweep(uint32_t start, uint32_t end) {
  uint8_t buf[CHUNK];
  uint32_t nonZero = 0, rows = 0, printed = 0, errs = 0, consecErr = 0;
  uint32_t firstNZ = 0xFFFFFFFF, lastNZ = 0;

  for (uint32_t a = start; a <= end; a += CHUNK) {
    const uint32_t remain = end - a + 1;
    const size_t n = (remain < CHUNK) ? (size_t)remain : (size_t)CHUNK;

    const uint8_t rc = tofRead((uint16_t)a, buf, n);
    if (rc != 0) {
      errs++;
      if (++consecErr >= 64) {
        Serial.printf("  0x%04lX 부터 연속 실패 64회. 스윕 중단.\n", (unsigned long)a);
        break;
      }
      continue;
    }
    consecErr = 0;

    bool any = false;
    for (size_t i = 0; i < n; i++) {
      if (buf[i]) {
        any = true;
        nonZero++;
        if (a + i < firstNZ) firstNZ = a + i;
        if (a + i > lastNZ)  lastNZ  = a + i;
      }
    }
    if (!any) continue;

    rows++;
    if (printed < MAX_PRINT_ROWS) {
      printed++;
      Serial.printf("  0x%04lX: ", (unsigned long)a);
      for (size_t i = 0; i < n; i++) Serial.printf("%02X ", buf[i]);
      Serial.println();
    }
  }

  if (rows > printed) Serial.printf("  ... non-zero 행 %lu개 더 있음 (출력 생략)\n",
                                    (unsigned long)(rows - printed));
  Serial.printf("  non-zero 바이트 %lu, 행 %lu, 읽기 실패 %lu\n",
                (unsigned long)nonZero, (unsigned long)rows, (unsigned long)errs);
  if (nonZero) Serial.printf("  non-zero 구간: 0x%04lX ~ 0x%04lX\n",
                             (unsigned long)firstNZ, (unsigned long)lastNZ);
  else         Serial.println(F("  전 구간 0x00. 레지스터 파일이 아직 초기화되지 않았을 수 있다"
                                " (FW 다운로드 필요 가능성)."));
}

// ---------------------------------------------------------------------------
// [4] 같은 구간을 두 번 읽어 값이 변하는 레지스터를 찾는다.
//     변하는 게 있으면 레지스터 파일이 살아서 돌고 있다는 뜻이다.
// ---------------------------------------------------------------------------
static void checkLiveRegisters(uint32_t start, uint32_t end) {
  uint8_t a1[CHUNK], a2[CHUNK];
  uint32_t changed = 0, shown = 0;

  for (uint32_t a = start; a <= end; a += CHUNK) {
    const uint32_t remain = end - a + 1;
    const size_t n = (remain < CHUNK) ? (size_t)remain : (size_t)CHUNK;

    if (tofRead((uint16_t)a, a1, n) != 0) continue;
    delay(20);
    if (tofRead((uint16_t)a, a2, n) != 0) continue;

    for (size_t i = 0; i < n; i++) {
      if (a1[i] != a2[i]) {
        changed++;
        if (shown < 16) {
          shown++;
          Serial.printf("  0x%04lX: %02X -> %02X\n", (unsigned long)(a + i), a1[i], a2[i]);
        }
      }
    }
  }

  if (changed) Serial.printf("  변하는 바이트 %lu개. 레지스터 파일이 살아 있다.\n",
                             (unsigned long)changed);
  else         Serial.println(F("  변하는 바이트 없음. 정적 상태."));
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  banner("VL53L9CX I2C 통신 + 레지스터 맵 탐색");

  enableSensor();
  Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ_HZ);
  Wire.setTimeOut(50);
  Serial.printf("  I2C %lu Hz, SDA=GPIO%d, SCL=GPIO%d, XSHUT=GPIO%d\n",
                (unsigned long)I2C_FREQ_HZ, PIN_SDA, PIN_SCL, PIN_XSHUT);

  banner("[1/4] 존재 확인");
  if (!probe()) {
    Serial.println(F("  센서가 응답하지 않는다. 아래 단계는 의미 없다."));
    Serial.println(F("  R25 가 제거되어 Y1(12MHz) 이 동작하는지, R24 가 장착됐는지 확인할 것."));
    return;
  }

  banner("[2/4] 다중 바이트 읽기 주소 자동증가 확인");
  checkAutoIncrement(0x0000);

  banner("[3/4] 레지스터 스윕 (non-zero 만 출력)");
  Serial.printf("  범위 0x%04lX ~ 0x%04lX, %d 바이트씩\n",
                (unsigned long)SWEEP_START, (unsigned long)SWEEP_END, CHUNK);
  sweep(SWEEP_START, SWEEP_END);

  banner("[4/4] 값이 변하는 레지스터 탐색");
  checkLiveRegisters(0x0000, 0x0FFF);

  banner("탐색 완료 - 루프 진입");
}

void loop() {
  delay(5000);

  uint8_t rc = 0;
  uint16_t v = 0;
  rc = tofRead16(0x0000, &v);
  Serial.printf("[생존 확인] 0x0000 = 0x%04X (%s)\n", v, rcText(rc));

  (void)tofWrite;   // API 로만 제공. 맵 확정 전까지 호출하지 않는다.
}
