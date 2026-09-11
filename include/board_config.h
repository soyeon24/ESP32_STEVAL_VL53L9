#pragma once

// ---------------------------------------------------------------------------
// ESP32 DevKit v1  <->  STEVAL-VL53L9 (VL53L9CX) 배선 설정
//
// 근거: STEVAL-VL53L9 Data brief DB5805 Rev 1 (2026-03) 회로도
//
// J2 헤더(8핀, 2.54mm)의 신호선:
//     3V3 / SYNC_IN / AP_CLK / INTR / XSHUT / SCL / SDA / GND
//
// 보드 특성:
//   * 3V3 만 넣으면 온보드 LDO 가 2V8 / 1V8 / 1V2 를 만든다.
//   * U1/U2 (PI4ULS3V204) 레벨 시프터가 호스트 <-> 센서(1.8V) 사이에 있다.
//   * J3 점퍼가 HOST_IOVDD 를 3V3 / 1V8 중에서 고른다. --> 반드시 3V3!
//   * R7/R8 = 2.2k 풀업이 호스트 쪽 SDA/SCL 에 이미 있다. 외부 풀업 금지.
//   * Y1 = 12MHz 온보드 발진기가 있으므로 AP_CLK 는 보통 연결 불필요.
// ---------------------------------------------------------------------------

// --- I2C ---
#define PIN_SDA        21
#define PIN_SCL        22

// 브링업 단계에서는 느리게 시작한다. 안정 확인 후 400k 로 올릴 것.
#define I2C_FREQ_HZ    100000UL

// --- I2C 주소 (회로도에 명시된 값) ---
#define TOF_I2C_ADDR_7BIT     0x29   // VL53L9CX     (8-bit 0x52)
#define EEPROM_I2C_ADDR_7BIT  0x55   // M24C64 (U4)  (8-bit 0xAA)

// --- 제어 핀 ---
// 연결하지 않은 핀은 -1 로 두면 코드가 건너뛴다.
#define PIN_XSHUT      32   // 필수. LOW = 셧다운, HIGH = 동작
#define PIN_INTR       27   // 입력. 측정 완료 인터럽트
#define PIN_SYNC_IN    -1   // 다중 센서 동기화용. 단독 사용이면 불필요
#define PIN_AP_CLK     -1   // 온보드 12MHz 발진기가 있으면 불필요

// --- 시리얼 ---
#define SERIAL_BAUD    115200

// EEPROM(U4) 에는 모듈 캘리브레이션 데이터가 들어 있을 수 있다.
// 이 스케치는 절대 쓰기를 하지 않는다. 읽기 전용으로만 접근할 것.
