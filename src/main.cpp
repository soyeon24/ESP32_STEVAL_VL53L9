// ---------------------------------------------------------------------------
// VL53L9CX I2C 통신 계층 + 전체 레지스터 공간 원시 덤프
//
// 확인된 사실:
//   - 0x29 응답 (R25 제거로 Y1 12MHz 활성화 후)
//   - 다중 바이트 읽기가 주소 자동증가를 한다
//   - 0x0000 이 디바이스 ID. 값 39 4C 33 53 ("9L3S", 역순 "S3L9").
//     이전에 읽던 0x010F 는 VL53L5CX/L8CX 관례에서 가져온 값이라 근거가 없었고
//     실제로 0x00 이 나온다
//   - 레지스터 파일이 이미 채워져 있다 (FW 다운로드 불필요)
//
// 이 스케치는 맵 분석을 위해 64KB 전 구간을 호스트로 넘긴다.
//   [1] ID 확인
//   [2] 64KB 전체를 RAM 으로 읽어들임
//   [3] 파싱 가능한 형식으로 덤프
//   [4] 다시 읽어 전 구간 안정성 확인 (미매핑 주소의 버스 노이즈와 구분)
//
// 안전: 보드에 VCSEL 2개와 레이저 드라이버가 있다. 맵을 모르는 상태의 임의
//       쓰기는 레이저 구동 설정을 건드릴 수 있다. tofWrite() 는 API 로만
//       제공하고 호출하지 않는다. 이 스케치는 전부 읽기 전용이다.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include "board_config.h"

#define CHUNK        32
#define SPACE_SIZE   65536UL

// 64KB 스냅샷. ESP32 는 320KB RAM 이라 여유 있다.
static uint8_t g_img[SPACE_SIZE];

static void banner(const char *title) {
  Serial.println();
  Serial.println(F("==================================================="));
  Serial.print(F("  "));
  Serial.println(title);
  Serial.println(F("==================================================="));
}

// ---------------------------------------------------------------------------
// I2C 통신 계층. 반환 0 = 성공, 그 외는 원인을 구분해 돌려준다.
// ---------------------------------------------------------------------------
static uint8_t tofRead(uint16_t reg, uint8_t *buf, size_t len) {
  if (!buf || len == 0 || len > CHUNK) return 0xFD;

  Wire.beginTransmission(TOF_I2C_ADDR_7BIT);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  const uint8_t rc = Wire.endTransmission(false);
  if (rc != 0) return rc;

  const size_t got = Wire.requestFrom((int)TOF_I2C_ADDR_7BIT, (int)len, (int)true);
  if (got != len) return 0xFE;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return 0;
}

// 맵 확정 전까지 호출 금지. 위 안전 주석 참고.
static uint8_t tofWrite(uint16_t reg, const uint8_t *buf, size_t len) {
  if (!buf || len == 0) return 0xFD;

  Wire.beginTransmission(TOF_I2C_ADDR_7BIT);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  for (size_t i = 0; i < len; i++) Wire.write(buf[i]);
  return Wire.endTransmission(true);
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
  delay(200);
}

// 64KB 전 구간을 dst 로 읽어들인다. 실패한 청크는 0xFF 로 채우고 개수를 센다.
static uint32_t readFullSpace(uint8_t *dst) {
  uint32_t fails = 0;
  for (uint32_t a = 0; a < SPACE_SIZE; a += CHUNK) {
    const uint8_t rc = tofRead((uint16_t)a, dst + a, CHUNK);
    if (rc != 0) {
      fails++;
      memset(dst + a, 0xFF, CHUNK);
    }
  }
  return fails;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  banner("VL53L9CX 전체 레지스터 공간 덤프");

  enableSensor();
  Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ_HZ);
  Wire.setTimeOut(50);
  Serial.printf("  I2C %lu Hz, SDA=GPIO%d, SCL=GPIO%d, XSHUT=GPIO%d\n",
                (unsigned long)I2C_FREQ_HZ, PIN_SDA, PIN_SCL, PIN_XSHUT);

  banner("[1/4] ID 확인");
  uint8_t id[4];
  const uint8_t rc = tofRead(0x0000, id, 4);
  if (rc != 0) {
    Serial.printf("  0x0000 읽기 실패: %s\n", rcText(rc));
    Serial.println(F("  R25 제거 / R24 장착 여부를 확인할 것."));
    return;
  }
  Serial.printf("  0x0000 = %02X %02X %02X %02X  (\"%c%c%c%c\")\n",
                id[0], id[1], id[2], id[3],
                isprint(id[0]) ? id[0] : '.', isprint(id[1]) ? id[1] : '.',
                isprint(id[2]) ? id[2] : '.', isprint(id[3]) ? id[3] : '.');

  banner("[2/4] 64KB 전 구간 읽기");
  const uint32_t fails = readFullSpace(g_img);
  Serial.printf("  완료. 실패 청크 %lu / %lu\n",
                (unsigned long)fails, (unsigned long)(SPACE_SIZE / CHUNK));

  banner("[3/4] 원시 덤프");
  Serial.println(F("---DUMP-BEGIN---"));
  for (uint32_t a = 0; a < SPACE_SIZE; a += CHUNK) {
    Serial.printf("D%04lX:", (unsigned long)a);
    for (int i = 0; i < CHUNK; i++) Serial.printf("%02X", g_img[a + i]);
    Serial.println();
  }
  Serial.println(F("---DUMP-END---"));

  banner("[4/4] 재읽기 안정성 확인");
  // 미매핑 주소의 버스 노이즈라면 두 번째 읽기에서 값이 달라진다.
  // 고엔트로피 구간이 실제 저장 데이터인지 노이즈인지 이걸로 가른다.
  uint8_t buf[CHUNK];
  uint32_t diffBytes = 0, diffRows = 0, shown = 0;
  for (uint32_t a = 0; a < SPACE_SIZE; a += CHUNK) {
    if (tofRead((uint16_t)a, buf, CHUNK) != 0) continue;
    bool rowDiff = false;
    for (int i = 0; i < CHUNK; i++) {
      if (buf[i] != g_img[a + i]) {
        diffBytes++;
        rowDiff = true;
        if (shown < 16) {
          shown++;
          Serial.printf("  0x%04lX: %02X -> %02X\n",
                        (unsigned long)(a + i), g_img[a + i], buf[i]);
        }
      }
    }
    if (rowDiff) diffRows++;
  }
  Serial.printf("  변한 바이트 %lu, 행 %lu\n",
                (unsigned long)diffBytes, (unsigned long)diffRows);
  if (diffBytes == 0)
    Serial.println(F("  전 구간 안정. 고엔트로피 영역도 실제 저장 데이터다."));
  else
    Serial.println(F("  변하는 구간이 있다. 위 주소들을 확인할 것."));

  banner("완료");
}

void loop() {
  delay(10000);
  uint8_t id[4];
  const uint8_t rc = tofRead(0x0000, id, 4);
  Serial.printf("[생존] 0x0000 = %02X %02X %02X %02X (%s)\n",
                id[0], id[1], id[2], id[3], rcText(rc));
  (void)tofWrite;
}
