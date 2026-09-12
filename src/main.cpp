// ---------------------------------------------------------------------------
// STEVAL-VL53L9 (VL53L9CX) Who Am I 강제 반복 진단 스케치
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include "board_config.h"

static void banner(const char *title) {
  Serial.println();
  Serial.println(F("==================================================="));
  Serial.print(F("  "));
  Serial.println(title);
  Serial.println(F("==================================================="));
}

// 16-bit 주소 레지스터 읽기 함수 (실제 Read 파형 발생)
static bool readReg16(uint8_t devAddr, uint16_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(devAddr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  
  // 센서가 ACK를 안 하더라도 강제로 Repeated Start 시도 (오실로스코프 관측용)
  Wire.endTransmission(false); 

  const size_t got = Wire.requestFrom((int)devAddr, (int)len, (int)true);
  if (got != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static void enableSensor() {
  banner("센서 인에이블 (XSHUT)");
  if (PIN_XSHUT < 0) return;

  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_XSHUT, HIGH);
  delay(200); // 부팅 대기
}

void setup() {
  Serial.begin(115200);
  delay(500);

  uint32_t i2c_freq = 50000;

  banner("VL53L9 Who Am I 강제 반복 진단 시작");
  
  enableSensor();

  Wire.begin(PIN_SDA, PIN_SCL, i2c_freq);
  Wire.setTimeOut(50);

  banner("진단 완료 - 루프 진입 (무조건 Read 시도)");
}

void loop() {
  delay(2000); // 오실로스코프 트리거를 잡기 쉽도록 2초마다 실행

  Serial.println(F("[반복 측정] 0x29 주소로 레지스터 강제 읽기 시도..."));
  
  uint16_t candidateRegs[] = { 0x010F, 0x0000 };
  
  for (size_t i = 0; i < 2; i++) {
    uint16_t reg = candidateRegs[i];
    uint8_t data = 0;

    if (readReg16(TOF_I2C_ADDR_7BIT, reg, &data, 1)) {
      Serial.printf(" -> 레지스터 0x%04X 읽기 성공! 값: 0x%02X\n", reg, data);
    } else {
      Serial.printf(" -> 레지스터 0x%04X 읽기 실패 (I2C 리턴 없음)\n", reg);
    }
  }
  Serial.println(F("---------------------------------------------------"));
}