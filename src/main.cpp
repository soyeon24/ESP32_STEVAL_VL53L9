// ---------------------------------------------------------------------------
// VL53L9CX 측거 — ST 드라이버(BSD-3-Clause) ESP32 포팅
//
// 데이터 경로는 MIPI CSI-2 가 아니라 I2C 다.
//   vl53l9_hw_config_t.output_interface 를 true(I3C/시리얼)로 두면 프레임이
//   레지스터 공간으로 나오고 0x1800 창에서 읽힌다. ESP32 에 CSI 수신기가
//   없어도 측거 데이터를 받을 수 있는 이유다.
//
// 프레임 버퍼 레이아웃 (vl53l9_get_frame 기준):
//   [0          .. res*2)     depth      zone 당 uint16, 리틀엔디안
//   [res*2      .. res*4)     amplitude
//   [res*4      .. res*6)     ambient
//   [res*6      .. +res/2)    DSS LUT 인덱스
//   [...        .. +100)      status line
//
// 설정 순서는 ST 예제의 vl53l9_utils_set_profile() 을 공개 API 로 그대로
// 재현한 것이다 (그 함수는 SLA0111 인 vl53l9_interface.h 에 의존해서 가져올
// 수 없다. 하는 일은 공개 API 호출 나열이라 재현이 어렵지 않다).
//
// 전제 조건: R25 제거 / R24 장착 (board_config.h 참고).
//
// === 현재 미해결: 측거 시작 시 FW 폴트 ===
//
// 동작하는 부분: init(패치 9865B 설치) / device id / 설정 6종 / start -> STREAMING
//
// 그런데 trigger 후 +0ms 에 즉시 폴트가 난다.
//
//   폴트 포착: trigger +0ms  fsm=0x03(STREAMING)  err=0x80  code=0x0F00
//
// err=0x80 은 FW_ERROR(BIT 7) 하나뿐이다. I_LIMIT / VHV_UNDERVOLTAGE /
// SPAD_SUPPLY_OVERLOAD / PLL_LOCK 은 전부 0 이므로 전원·전류 문제가 아니고,
// 부하가 걸릴 시간도 없이 즉시 나므로 FW 가 설정 자체를 거부하는 것으로 보인다.
//
// 검증한 것 (전부 정상):
//   - 플랫폼 계층 청크 분할 쓰기: 4096B write/readback 완전 일치
//   - start 직전 설정 레지스터 읽기: STREAM_STEP_NUMBER=7,
//     NB_SHOT_STEP 7단계(100/200/400/615/1231/1231/100) 정확,
//     binning/context/frame_period/sync/output_if 전부 의도대로
//
// 바꿔봐도 증상이 동일한 것:
//   context(SHORT/LONG), binning(2/8), exposure(1/10ms),
//   power_mode(REGULAR/ULTRA_LOW), sync(MANUAL/AUTONOMOUS), 폴링 간격,
//   set_hw_config 호출 여부
//
// 남은 미지수는 ERROR_CODE 0x0F00 의 의미 하나다. ST 가 코드표를 공개하지
// 않아 여기서 더 좁힐 수 없다. docs/vl53l9cx-i2c-map.md 참고.
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "board_config.h"
#include "vl53l9_esp32.h"

extern "C" {
#include "vl53l9.h"
#include "vl53l9_platform.h"   // vl53l9_read / vl53l9_write (청크 검증용)
}

// ST 의 AR_PRECISION 프로파일 값. sync 만 예제와 같이 MANUAL 로 덮어쓴다.
#define PROF_POWER        VL53L9_POWER_REGULAR   // ULTRA_LOW 는 I3C 웨이크 전제로 보임
#define PROF_CONTEXT      VL53L9_CONTEXT_SHORT
#define PROF_FRAME_PERIOD (1000000UL / 30UL)   // 30 fps
#define PROF_BINNING      8                    // 12x10. binning 2 는 start 가 60ms 를 넘긴다
#define PROF_EXPOSURE_MS  10

#define FRAME_BUF_MAX     15000                // binning 2 기준 14842B

static vl53l9_esp32_dev_t g_dev;
static uint8_t            g_frame[FRAME_BUF_MAX];
static uint16_t           g_frame_size = 0;
static uint16_t           g_w = 0, g_h = 0;

static void dumpStatus(const char *when);

static bool binningToWH(uint8_t b, uint16_t *w, uint16_t *h) {
  switch (b) {
    case 2:  *w = 54; *h = 42; return true;
    case 4:  *w = 24; *h = 24; return true;
    case 6:  *w = 18; *h = 14; return true;
    case 8:  *w = 12; *h = 10; return true;
    case 12: *w =  8; *h =  8; return true;
    case 24: *w =  4; *h =  4; return true;
    default: return false;
  }
}

static void banner(const char *title) {
  Serial.println();
  Serial.println(F("==================================================="));
  Serial.print(F("  "));
  Serial.println(title);
  Serial.println(F("==================================================="));
}

static const char *errText(int e) {
  switch (e) {
    case VL53L9_ERROR_NONE:              return "성공";
    case VL53L9_ERROR_PLATFORM:          return "플랫폼(I2C) 오류";
    case VL53L9_ERROR_INVALID_PARAM:     return "잘못된 인자";
    case VL53L9_ERROR_INVALID_STATE:     return "잘못된 상태";
    case VL53L9_ERROR_INVALID_OPERATION: return "잘못된 동작";
    case VL53L9_ERROR_TIMEOUT:           return "타임아웃";
    case VL53L9_ERROR_INTERNAL:          return "내부 오류";
    default:                             return "알 수 없음";
  }
}

#define STEP(call, what)                                                      \
  do {                                                                        \
    const int _e = (call);                                                    \
    Serial.printf("  %-36s %s\n", what, errText(_e));                         \
    if (_e != VL53L9_ERROR_NONE) { Serial.println(F("  중단.")); return; }    \
  } while (0)

static void enableSensor() {
  if (PIN_XSHUT < 0) return;
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_XSHUT, HIGH);
  delay(200);
}

static void dumpStatus(const char *when) {
  vl53l9_status_t st;
  const int e = vl53l9_get_status(&g_dev, &st);
  if (e != VL53L9_ERROR_NONE) {
    Serial.printf("  [%s] get_status 실패: %s\n", when, errText(e));
    return;
  }
  const char *fsmName = (st.fsm == 0) ? "NONE"
                      : (st.fsm == 1) ? "READY_TO_BOOT"
                      : (st.fsm == 2) ? "STANDBY"
                      : (st.fsm == 3) ? "STREAMING" : "?";
  Serial.printf("  [%s] fsm=0x%02X(%s) command_err=0x%02X firmware=0x%04X\n",
                when, st.fsm, fsmName, st.command, st.firmware);
  Serial.printf("    error: vhv_ov=%u vhv_uv=%u spad_ovl=%u hvboost=%u "
                "sof_blank=%u pll_lock=%u ref_array=%u internal_fw=%u\n",
                st.error.vhv_overvoltage, st.error.vhv_undervoltage,
                st.error.spad_supply_overload, st.error.hvboost_limit,
                st.error.sof_outside_blanking, st.error.pll_lock,
                st.error.ref_array, st.error.internal_fw);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  banner("VL53L9CX 측거 (ST 드라이버 ESP32 포팅)");

  g_dev.address = TOF_I2C_ADDR_7BIT;
  enableSensor();
  vl53l9_esp32_bus_begin();
  Serial.printf("  I2C %lu Hz, SDA=GPIO%d, SCL=GPIO%d, XSHUT=GPIO%d\n",
                (unsigned long)I2C_FREQ_HZ, PIN_SDA, PIN_SCL, PIN_XSHUT);

  // ---------------------------------------------------------------------
  // 플랫폼 계층 청크 분할 쓰기 검증
  //
  // 펌웨어 패치는 9865바이트이고 Wire 버퍼보다 커서 여러 청크로 쪼개 보낸다.
  // 이 경로가 미묘하게 틀리면 패치가 손상된 채 올라가고, 부팅은 되지만
  // 측거 코드가 돌 때 죽는다. init 전에 먼저 확인한다.
  //
  // 대상은 0x1800 (패치 영역). 어차피 init 이 곧 덮어쓰므로 안전하다.
  // ---------------------------------------------------------------------
  banner("[사전] 청크 쓰기 검증");
  {
    const uint32_t N = 4096;                       // 여러 청크에 걸치게
    static uint8_t tx[4096], rx[4096];
    for (uint32_t i = 0; i < N; i++) tx[i] = (uint8_t)(i * 7u + 3u);

    int we = vl53l9_write(&g_dev, 0x1800, tx, N);
    int re = vl53l9_read(&g_dev, 0x1800, rx, N);
    Serial.printf("  write %lu B -> %d,  read back -> %d\n", (unsigned long)N, we, re);

    uint32_t bad = 0; int32_t first = -1;
    for (uint32_t i = 0; i < N; i++) {
      if (tx[i] != rx[i]) { bad++; if (first < 0) first = (int32_t)i; }
    }
    if (bad == 0) {
      Serial.println(F("  일치. 청크 분할 쓰기 정상."));
    } else {
      Serial.printf("  !! 불일치 %lu / %lu, 첫 위치 오프셋 %ld (0x%04lX)\n",
                    (unsigned long)bad, (unsigned long)N,
                    (long)first, (unsigned long)(0x1800 + first));
      Serial.printf("     기대 %02X  실제 %02X\n", tx[first], rx[first]);
    }
  }

  banner("초기화");

  // 부팅 + 펌웨어 패치(9865B) 설치
  STEP(vl53l9_init(&g_dev), "vl53l9_init (패치 설치 포함)");

  uint32_t id = 0;
  STEP(vl53l9_get_device_id(&g_dev, &id), "vl53l9_get_device_id");
  Serial.printf("    device id = 0x%08lX\n", (unsigned long)id);

  // 출력 인터페이스 확인만 한다.
  //
  // 기본값이 이미 I3C(시리얼)라 되쓸 필요가 없다. 그리고 set_hw_config 는
  // output_interface 뿐 아니라 CSI 관련 필드(data_rate, frame_width/height,
  // virtual_channel, datatype, signaling_mode ...)를 한꺼번에 되쓴다.
  // ST 의 i3c 예제도 이 함수를 호출하지 않는다. 건드리지 않는다.
  vl53l9_hw_config_t hw;
  STEP(vl53l9_get_hw_config(&g_dev, &hw), "vl53l9_get_hw_config");
  Serial.printf("    output_interface = %s\n", hw.output_interface ? "I3C(시리얼)" : "CSI2");
  if (!hw.output_interface) {
    Serial.println(F("  기본값이 CSI2 다. 이 경로로는 프레임을 못 받는다."));
    return;
  }

  banner("프로파일 적용 (ST AR_PRECISION)");

  STEP(vl53l9_set_power_mode(&g_dev, PROF_POWER),   "vl53l9_set_power_mode (REGULAR)");
  STEP(vl53l9_set_frame_period(&g_dev, PROF_FRAME_PERIOD),
                                                     "vl53l9_set_frame_period (30fps)");
  STEP(vl53l9_set_context(&g_dev, PROF_CONTEXT),     "vl53l9_set_context (SHORT)");
  STEP(vl53l9_set_binning(&g_dev, PROF_CONTEXT, PROF_BINNING), "vl53l9_set_binning");
  STEP(vl53l9_set_exposure(&g_dev, PROF_CONTEXT, PROF_EXPOSURE_MS),
                                                     "vl53l9_set_exposure (10ms)");
  STEP(vl53l9_set_sync_mode(&g_dev, VL53L9_SYNC_MANUAL), "vl53l9_set_sync_mode (MANUAL)");

  STEP(vl53l9_get_raw_buffer_size(PROF_BINNING, &g_frame_size), "vl53l9_get_raw_buffer_size");
  if (!binningToWH(PROF_BINNING, &g_w, &g_h) || g_frame_size > FRAME_BUF_MAX) {
    Serial.println(F("  binning 설정이 버퍼와 맞지 않는다."));
    g_frame_size = 0;
    return;
  }
  Serial.printf("    binning %d -> %ux%u (%u존), 프레임 %u 바이트\n",
                PROF_BINNING, g_w, g_h, g_w * g_h, g_frame_size);

  // start 전 실제 설정값을 읽어본다. FW 가 +0ms 에 설정을 거부하므로
  // 무엇이 비어 있는지 보는 게 핵심이다.
  banner("[진단] start 직전 설정 레지스터");
  {
    uint8_t b[0x100];
    if (vl53l9_read(&g_dev, 0x0460, b, 0xC0) == VL53L9_ERROR_NONE) {
      for (uint16_t off = 0; off < 0xC0; off += 16) {
        bool any = false;
        for (int k = 0; k < 16; k++) if (b[off + k]) { any = true; break; }
        if (!any) continue;
        Serial.printf("  %04X  ", 0x0460 + off);
        for (int k = 0; k < 16; k++) Serial.printf("%02X ", b[off + k]);
        Serial.println();
      }
    }
    uint8_t sn = 0xAA;
    vl53l9_read8(&g_dev, 0x04CC, &sn);          // STREAM_STEP_NUMBER (SHORT)
    Serial.printf("  STREAM_STEP_NUMBER(SHORT, 0x04CC) = %u\n", sn);
    uint32_t shots0 = 0;
    vl53l9_read32(&g_dev, 0x0504, &shots0);     // NB_SHOT_STEP(1, SHORT)
    Serial.printf("  NB_SHOT_STEP(1,SHORT, 0x0504) = %lu\n", (unsigned long)shots0);
  }

  STEP(vl53l9_start(&g_dev), "vl53l9_start");
  delay(100);
  dumpStatus("start 직후");

  banner("측거 시작");
}

void loop() {
  if (g_frame_size == 0) { delay(2000); return; }

  int e = vl53l9_trigger_frame(&g_dev);
  if (e != VL53L9_ERROR_NONE) {
    Serial.printf("trigger_frame 실패: %s\n", errText(e));
    dumpStatus("trigger 실패");
    delay(2000);
    return;
  }

  // 폴트가 나는 순간을 잡는다. trigger 직후부터 1ms 간격으로 상태를 보면서
  // fsm 이 STREAMING 을 벗어나거나 에러 비트가 서는 첫 시점을 기록한다.
  {
    vl53l9_status_t st;
    for (int i = 0; i < 60; i++) {
      if (vl53l9_get_status(&g_dev, &st) != VL53L9_ERROR_NONE) break;
      const uint8_t *eb = (const uint8_t *)&st.error;
      if (st.fsm != 3 || *eb != 0) {
        Serial.printf("\n  폴트 포착: trigger +%dms  fsm=0x%02X err=0x%02X code=0x%04X\n",
                      i, st.fsm, *eb, st.firmware);
        Serial.printf("    laser_driver[0..4] = %02X %02X %02X %02X %02X\n",
                      st.laser_driver[0], st.laser_driver[1], st.laser_driver[2],
                      st.laser_driver[3], st.laser_driver[4]);
        break;
      }
      delay(1);
    }
  }

  uint8_t ready = 0;
  const uint32_t t0 = millis();
  // 딜레이 없이 폴링하면 측거 구간 내내 I2C 트랜잭션이 쉴 새 없이 들어가
  // 센서 SoC 의 내부 타이밍을 방해할 수 있다. 프레임 주기가 33ms 이므로
  // 2ms 간격이면 충분하다.
  while (!ready && (millis() - t0) < 2000) {
    delay(2);
    if (vl53l9_poll_frame(&g_dev, &ready) != VL53L9_ERROR_NONE) break;
  }
  if (!ready) {
    Serial.println(F("프레임 대기 타임아웃"));
    dumpStatus("프레임 타임아웃");
    delay(2000);
    return;
  }

  e = vl53l9_get_frame(&g_dev, g_frame, g_frame_size);
  if (e != VL53L9_ERROR_NONE) {
    Serial.printf("get_frame 실패: %s\n", errText(e));
    delay(2000);
    return;
  }

  const uint16_t *depth = (const uint16_t *)g_frame;
  const uint32_t n = (uint32_t)g_w * g_h;

  uint16_t mn = 0xFFFF, mx = 0;
  uint32_t sum = 0, cnt = 0;
  for (uint32_t i = 0; i < n; i++) {
    const uint16_t d = depth[i];
    if (d == 0) continue;
    if (d < mn) mn = d;
    if (d > mx) mx = d;
    sum += d;
    cnt++;
  }

  Serial.printf("\n--- depth %ux%u | 중앙 %u mm | 최소 %u | 최대 %u | 평균 %lu | 유효 %lu/%lu ---\n",
                g_w, g_h, depth[(g_h / 2) * g_w + (g_w / 2)],
                cnt ? mn : 0, mx, cnt ? (unsigned long)(sum / cnt) : 0UL,
                (unsigned long)cnt, (unsigned long)n);

  // 54열은 터미널에 너무 넓으므로 가로/세로를 솎아서 보여준다.
  const uint16_t stepX = (g_w > 18) ? (g_w / 18) : 1;
  const uint16_t stepY = (g_h > 14) ? (g_h / 14) : 1;
  for (uint16_t y = 0; y < g_h; y += stepY) {
    Serial.print("  ");
    for (uint16_t x = 0; x < g_w; x += stepX) Serial.printf("%6u", depth[y * g_w + x]);
    Serial.println();
  }

  delay(500);
}
