// BF-018A-WWVB: WWVB (60kHz, NIST) Simulator for ATOM Lite / ATOM Matrix / ATOMS3 Lite
//
// Based on BF-018A Rev.4 by BotanicFields, Inc. (https://github.com/botanicfields/BF-018A)
// WWVB timecode port: replaces JJY/MSF timecode generation with WWVB (60kHz).
//
// WWVB spec ref: https://en.wikipedia.org/wiki/WWVB
//               https://www.nist.gov/pml/time-and-frequency-division/time-realization/wwvb
//
// Key differences from JJY:
//   - Carrier: 60kHz (same as JJY west / MSF)
//   - Timezone: UTC (JJY: JST+9). DST flags sent for US time zones.
//   - OOK pattern: carrier ON at second-start, power reduced for data
//     - Marker:  800ms reduced + 200ms full  (same as JJY marker)
//     - Bit 1:   500ms reduced + 500ms full
//     - Bit 0:   200ms reduced + 800ms full
//   - 1 bit per second (like JJY, unlike MSF's 2 bits)
//   - Data: minute, hour, day-of-year, year, DUT1, leap second, DST
//   - BCD layout similar to JJY but uses day-of-year instead of month+day
//   - Position markers at seconds 0,9,19,29,39,49,59 (like JJY)
//   - Parity: none (unlike JJY even / MSF odd)
//
#include <M5Unified.h>
#include <FastLED.h>      // https://github.com/FastLED/FastLED
#include <Ticker.h>
#include <WiFi.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include "BF_Pcf8563.h"
#include "BF_RtcxNtp.h"

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// for TCO (Time Code Output)
const uint32_t wwvb_frequency(60000);  // WWVB: 60kHz
struct tm       td;  // time of day (UTC): year, month, day, wday, hour, min, sec
struct timespec ts;  // time spec: second, nano-second

// WWVB uses 1 bit per second: 0, 1, or marker (2)
const int WWVB_ZERO   = 0;
const int WWVB_ONE    = 1;
const int WWVB_MARKER = 2;

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// for NTP — UTC0: always UTC
const char* time_zone  = "UTC0";
const char* ntp_server = "pool.ntp.org";
bool rtcx_available(false);
bool localtime_valid(false);

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// for WiFi
const          int wifi_config_portal_timeout_sec(60);
const unsigned int wifi_retry_interval_ms(60000);
      unsigned int wifi_retry_last_ms(0);
const          int wifi_retry_max_times(3);
               int wifi_retry_times(0);

wl_status_t wifi_status(WL_NO_SHIELD);
const char* wl_status_str[] = {
  "WL_IDLE_STATUS",      // 0
  "WL_NO_SSID_AVAIL",    // 1
  "WL_SCAN_COMPLETED",   // 2
  "WL_CONNECTED",        // 3
  "WL_CONNECT_FAILED",   // 4
  "WL_CONNECTION_LOST",  // 5
  "WL_DISCONNECTED",     // 6
  "WL_NO_SHIELD",        // 7 <-- 255
  "wl_status invalid",   // 8
};

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// for FastLED
const int led_pin_atom(27);        // GPIO27 for ATOM Lite/Matrix
const int led_pin_atoms3lite(35);  // GPIO35 for ATOMS3 Lite
const unsigned char led_num(25);   // ATOM Matrix: 25, ATOM lite/ATOMS3 Lite: 1 --> 25
CRGB leds[led_num];

const int led_on_r(0x80);
const int led_on_g(0x80);
const int led_on_b(0x80);
const int led_brightness(40);
const unsigned int blink_slow_ms(1000);
const unsigned int blink_fast_ms( 200);

enum led_r_t {
  led_r_off,
  led_r_slow,
  led_r_fast,
  led_r_on,
};
enum led_g_t {
  led_g_off,
  led_g_slow,
  led_g_fast,
  led_g_on,
};
enum led_b_t {
  led_b_off,
  led_b_slow,
  led_b_fast,
  led_b_on,
};
led_r_t led_r(led_r_off);
led_g_t led_g(led_g_off);
led_b_t led_b(led_b_off);
bool led_enable(true);

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// LED
void LedShow()
{
  CRGB led(0);
  if (led_enable) {
    switch (led_r) {
      case led_r_on  : led.r = led_on_r;  break;
      case led_r_fast: led.r = LedBlink(blink_fast_ms) ? led_on_r : 0;  break;
      case led_r_slow: led.r = LedBlink(blink_slow_ms) ? led_on_r : 0;  break;
      default:  break;
    }
    switch (led_g) {
      case led_g_on  : led.g = led_on_g;  break;
      case led_g_fast: led.g = LedBlink(blink_fast_ms) ? led_on_g : 0;  break;
      case led_g_slow: led.g = LedBlink(blink_slow_ms) ? led_on_g : 0;  break;
      default:  break;
    }
    switch (led_b) {
      case led_b_on  : led.b = led_on_b;  break;
      case led_b_fast: led.b = LedBlink(blink_fast_ms) ? led_on_b : 0;  break;
      case led_b_slow: led.b = LedBlink(blink_slow_ms) ? led_on_b : 0;  break;
      default:  break;
    }
  }
  leds[0] = led;
  FastLED.show();
}

bool LedBlink(unsigned int period_ms)
{
  return millis() / period_ms % 2 != 0;
}

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// WiFi (unchanged from original)
const char* WlStatus(wl_status_t wl_status)
{
  if (wl_status >= 0 && wl_status <= 6) return wl_status_str[wl_status];
  if (wl_status == 255)                 return wl_status_str[7];
  return wl_status_str[8];
}

void WifiCheck()
{
  wl_status_t wifi_status_new = WiFi.status();
  if (wifi_status != wifi_status_new) {
    wifi_status = wifi_status_new;
    Serial.printf("[WiFi]%s\n", WlStatus(wifi_status));
    switch (wifi_status) {
      case WL_CONNECTED    : led_r = led_r_off;   break;
      case WL_NO_SSID_AVAIL: led_r = led_r_slow;  break;
      case WL_DISCONNECTED : led_r = led_r_fast;  break;
      default              : led_r = led_r_on;    break;
    }
  }
  if (millis() - wifi_retry_last_ms < wifi_retry_interval_ms) return;
  wifi_retry_last_ms = millis();
  if (wifi_status == WL_CONNECT_FAILED) {
    Serial.print("[WiFi]connect failed: rebooting..\n");
    ESP.restart();  return;
  }
  if (wifi_status != WL_DISCONNECTED) {
    wifi_retry_times = 0;  return;
  }
  if (++wifi_retry_times > wifi_retry_max_times) {
    Serial.print("[WiFi]disconnect timeout: rebooting..\n");
    ESP.restart();  return;
  }
  Serial.printf("[WiFi]reconnect %d\n", wifi_retry_times);
  if (!WiFi.reconnect()) {
    Serial.print("[WiFi]reconnect failed: rebooting..\n");
    ESP.restart();
  }
}

void WifiConfigModeCallback(WiFiManager *wm)
{
  led_g = led_g_on;
  LedShow();
}

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// TCO (Time Code Output) — WWVB version
Ticker tk;
const int ticker_interval_ms(100);  // 100ms resolution

// PWM for TCO signal
const uint8_t  ledc_pin_atom(22);
const uint8_t  ledc_pin_atoms3(5);
const uint32_t ledc_frequency(wwvb_frequency);
const uint8_t  ledc_resolution(8);
const uint32_t ledc_duty_on(128);
const uint32_t ledc_duty_off(0);
      uint8_t  ledc_pin(ledc_pin_atom);

void TcoInit()
{
  if (M5.getBoard() == m5::board_t::board_M5AtomS3Lite) {
    ledc_pin = ledc_pin_atoms3;
  }
  // Arduino LEDC API (v2.x)
  Serial.printf("ledcSetup result= %lf\n", ledcSetup(0, ledc_frequency, ledc_resolution));
  ledcAttachPin(ledc_pin, 0);
  ledcWrite(0, ledc_duty_on);
  Serial.printf("pin= %u, duty= %lu, freq= %lu\n", ledc_pin, ledcRead(0), ledcReadFreq(0));

  clock_gettime(CLOCK_REALTIME, &ts);
  delayMicroseconds((150000000 - ts.tv_nsec % 100000000) / 1000);

  clock_gettime(CLOCK_REALTIME, &ts);
  Serial.printf("ts.tv_nsec = %ld\n", ts.tv_nsec);

  tk.attach_ms(ticker_interval_ms, TcoGen);
}

// main TCO task (called every 100ms by Ticker)
void TcoGen()
{
  static int    tk_count(0);
  static int    tk_max(0);
  static int    tk_min(0);
  static double tk_sum(0.0);
  static double tk_sq_sum(0.0);
  static int    tk_distribution[9] = {0,0,0,0,0,0,0,0,0};
  static int    tk_last_nsec(0);

  if (!localtime_valid) {
    led_g = led_g_slow;
    return;
  }
  led_g = led_g_off;

  getLocalTime(&td);
  clock_gettime(CLOCK_REALTIME, &ts);
  int ts_100ms = ts.tv_nsec / 100000000;
  switch (ts_100ms) {
    case 0: Wwvb000ms();  break;  // all: TcOn (carrier full power at second start)
    case 2: Wwvb200ms();  break;  // bit 0: TcOn (end of 200ms reduced)
    case 5: Wwvb500ms();  break;  // bit 1: TcOn (end of 500ms reduced)
    case 8: Wwvb800ms();  break;  // marker: TcOn (end of 800ms reduced)
    default: break;
  }

  // statistics (same as original)
  if (tk_count++ != 0) {
    int tk_deviation = ts.tv_nsec - tk_last_nsec;
    if (tk_deviation < 0)   tk_deviation += 1000000000;
    tk_deviation -= 100000000;
    if (tk_max < tk_deviation) tk_max = tk_deviation;
    if (tk_min > tk_deviation) tk_min = tk_deviation;
    tk_sum    += (double)tk_deviation;
    tk_sq_sum += (double)tk_deviation * (double)tk_deviation;
    if      (tk_deviation < -50000000) ++tk_distribution[0];
    else if (tk_deviation <  -5000000) ++tk_distribution[1];
    else if (tk_deviation <   -500000) ++tk_distribution[2];
    else if (tk_deviation <    -50000) ++tk_distribution[3];
    else if (tk_deviation <     50000) ++tk_distribution[4];
    else if (tk_deviation <    500000) ++tk_distribution[5];
    else if (tk_deviation <   5000000) ++tk_distribution[6];
    else if (tk_deviation <  50000000) ++tk_distribution[7];
    else                               ++tk_distribution[8];
  }
  tk_last_nsec = ts.tv_nsec;

  if ((td.tm_sec == 0) && (ts.tv_nsec < 100000000)) {
    for (int i = 0; i < 9; ++i) Serial.printf("%d ", tk_distribution[i]);
    double tk_average   = tk_sum / (double)tk_count;
    double tk_variance  = (tk_sq_sum - tk_sum * tk_sum / (double)tk_count) / (double)tk_count;
    double tk_std_dev   = sqrt(tk_variance);
    Serial.printf("\nn= %d, ave= %.4f  sdv= %.4f  min= %d  max= %d\n",
                  tk_count, tk_average, tk_std_dev, tk_min, tk_max);
  }
}

// --- WWVB OOK modulation ---
//
// WWVB bit pattern within each second (carrier starts ON = full power):
//
//   Bit 0  :  |__200ms__|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾800ms‾‾‾‾|  (200ms reduced)
//   Bit 1  :  |_____500ms_____|‾‾‾‾‾‾‾‾‾‾‾‾‾500ms‾‾‾‾‾‾‾‾‾‾|  (500ms reduced)
//   Marker :  |________800ms________|‾‾‾‾‾200ms‾‾‾‾‾‾‾‾‾‾‾‾|  (800ms reduced)
//
// "Reduced" = power reduced by ~17dB. BF-018A uses OFF as approximation (same as JJY).
// Full power = carrier ON. All seconds start with carrier OFF (reduced = OFF on BF-018A).

void Wwvb000ms()
{
  TcOff();  // Start of every second: power reduced (OFF on BF-018A)
  if (td.tm_sec == 0) {
    Serial.print(&td, "\n[WWVB] %A %d %b %Y %H:%M:%S UTC\n");
  }
}

void Wwvb200ms()
{
  int bit = WwvbValue();
  if (bit == WWVB_ZERO) TcOn();  // bit 0: reduced ends at 200ms
}

void Wwvb500ms()
{
  int bit = WwvbValue();
  if (bit == WWVB_ONE) TcOn();   // bit 1: reduced ends at 500ms
}

void Wwvb800ms()
{
  int bit = WwvbValue();
  if (bit == WWVB_MARKER) TcOn();  // marker: reduced ends at 800ms
}

void TcOn()
{
  ledcWrite(0, ledc_duty_on);
  led_b = led_b_on;
}

void TcOff()
{
  ledcWrite(0, ledc_duty_off);
  led_b = led_b_off;
}

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// US DST detection (for WWVB DST flags)
// US DST: second Sunday in March 02:00 local → first Sunday in November 02:00 local
// WWVB sends DST status bits but the actual UTC offset is not transmitted.
// DST flags: bit 55 and bit 56
//   00 = DST not in effect
//   10 = DST begins today
//   11 = DST in effect
//   01 = DST ends today
// For simplicity, we just set 00 (no DST). The receiver clock handles local DST.

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// BCD helper

// 2-digit decimal -> BCD (e.g. 25 -> 0x25 = 37)
int Int2Bcd(int a)
{
  return (a % 10) + (a / 10 % 10 * 16);
}

// 3-digit decimal -> BCD (e.g. 125 -> 0x125 = 293)
int Int3Bcd(int a)
{
  return (a % 10) + (a / 10 % 10 * 16) + (a / 100 % 10 * 256);
}

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// Day of year (1-366)
int DayOfYear(const struct tm& t)
{
  return t.tm_yday + 1;  // tm_yday is 0-based
}

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// WWVB bit value for current second
//
// WWVB timecode frame (60 seconds per minute):
//
//   Sec  Content                        Type
//   ---  ----------------------------   ----
//    0   Frame reference marker          M
//    1   Minute 40 (BCD)                 D
//    2   Minute 20                       D
//    3   Minute 10                       D
//    4   0 (unused)                      0
//    5   Minute 8                        D
//    6   Minute 4                        D
//    7   Minute 2                        D
//    8   Minute 1                        D
//    9   Position marker P1              M
//   10   0 (unused)                      0
//   11   0 (unused)                      0
//   12   Hour 20 (BCD)                   D
//   13   Hour 10                         D
//   14   0 (unused)                      0
//   15   Hour 8                          D
//   16   Hour 4                          D
//   17   Hour 2                          D
//   18   Hour 1                          D
//   19   Position marker P2              M
//   20   0 (unused)                      0
//   21   0 (unused)                      0
//   22   Day-of-year 200 (BCD)           D
//   23   Day-of-year 100                 D
//   24   0 (unused)                      0
//   25   Day-of-year 80                  D
//   26   Day-of-year 40                  D
//   27   Day-of-year 20                  D
//   28   Day-of-year 10                  D
//   29   Position marker P3              M
//   30   Day-of-year 8                   D
//   31   Day-of-year 4                   D
//   32   Day-of-year 2                   D
//   33   Day-of-year 1                   D
//   34   0 (unused)                      0
//   35   0 (unused)                      0
//   36   DUT1 sign (+)                   D
//   37   DUT1 sign (-)                   D
//   38   DUT1 sign (+)                   D
//   39   Position marker P4              M
//   40   DUT1 magnitude 0.8              D
//   41   DUT1 magnitude 0.4              D
//   42   DUT1 magnitude 0.2              D
//   43   DUT1 magnitude 0.1              D
//   44   0 (unused)                      0
//   45   Year 80 (BCD)                   D
//   46   Year 40                         D
//   47   Year 20                         D
//   48   Year 10                         D
//   49   Position marker P5              M
//   50   Year 8                          D
//   51   Year 4                          D
//   52   Year 2                          D
//   53   Year 1                          D
//   54   0 (unused)                      0
//   55   Leap year indicator             D
//   56   Leap second warning             D
//   57   DST status bit 1                D
//   58   DST status bit 2                D
//   59   Position marker P0 (frame ref)  M

int WwvbValue()
{
  // UTC time fields
  int bcd_minute = Int2Bcd(td.tm_min);         // 0-59 as BCD
  int bcd_hour   = Int2Bcd(td.tm_hour);        // 0-23 as BCD
  int doy        = DayOfYear(td);              // 1-366
  int bcd_doy    = Int3Bcd(doy);               // day-of-year as 3-digit BCD
  int year_2d    = td.tm_year % 100;           // 2-digit year
  int bcd_year   = Int2Bcd(year_2d);           // e.g. 0x26

  // Leap year: tm_year is years since 1900
  int full_year = td.tm_year + 1900;
  int is_leap = ((full_year % 4 == 0) && (full_year % 100 != 0)) || (full_year % 400 == 0) ? 1 : 0;

  switch (td.tm_sec) {

    // Second 0: frame reference marker
    case  0: return WWVB_MARKER;

    // Seconds 1-8: Minute (BCD: 40,20,10, 0, 8,4,2,1)
    case  1: return (bcd_minute & 0x40) ? WWVB_ONE : WWVB_ZERO;  // 40
    case  2: return (bcd_minute & 0x20) ? WWVB_ONE : WWVB_ZERO;  // 20
    case  3: return (bcd_minute & 0x10) ? WWVB_ONE : WWVB_ZERO;  // 10
    case  4: return WWVB_ZERO;                                     // unused
    case  5: return (bcd_minute & 0x08) ? WWVB_ONE : WWVB_ZERO;  // 8
    case  6: return (bcd_minute & 0x04) ? WWVB_ONE : WWVB_ZERO;  // 4
    case  7: return (bcd_minute & 0x02) ? WWVB_ONE : WWVB_ZERO;  // 2
    case  8: return (bcd_minute & 0x01) ? WWVB_ONE : WWVB_ZERO;  // 1

    // Second 9: position marker P1
    case  9: return WWVB_MARKER;

    // Seconds 10-18: Hour (BCD: 0,0, 20,10, 0, 8,4,2,1)
    case 10: return WWVB_ZERO;                                     // unused
    case 11: return WWVB_ZERO;                                     // unused
    case 12: return (bcd_hour & 0x20) ? WWVB_ONE : WWVB_ZERO;    // 20
    case 13: return (bcd_hour & 0x10) ? WWVB_ONE : WWVB_ZERO;    // 10
    case 14: return WWVB_ZERO;                                     // unused
    case 15: return (bcd_hour & 0x08) ? WWVB_ONE : WWVB_ZERO;    // 8
    case 16: return (bcd_hour & 0x04) ? WWVB_ONE : WWVB_ZERO;    // 4
    case 17: return (bcd_hour & 0x02) ? WWVB_ONE : WWVB_ZERO;    // 2
    case 18: return (bcd_hour & 0x01) ? WWVB_ONE : WWVB_ZERO;    // 1

    // Second 19: position marker P2
    case 19: return WWVB_MARKER;

    // Seconds 20-28: Day of year high (BCD: 0,0, 200,100, 0, 80,40,20,10)
    case 20: return WWVB_ZERO;                                        // unused
    case 21: return WWVB_ZERO;                                        // unused
    case 22: return (bcd_doy & 0x200) ? WWVB_ONE : WWVB_ZERO;       // 200
    case 23: return (bcd_doy & 0x100) ? WWVB_ONE : WWVB_ZERO;       // 100
    case 24: return WWVB_ZERO;                                        // unused
    case 25: return (bcd_doy & 0x080) ? WWVB_ONE : WWVB_ZERO;       // 80
    case 26: return (bcd_doy & 0x040) ? WWVB_ONE : WWVB_ZERO;       // 40
    case 27: return (bcd_doy & 0x020) ? WWVB_ONE : WWVB_ZERO;       // 20
    case 28: return (bcd_doy & 0x010) ? WWVB_ONE : WWVB_ZERO;       // 10

    // Second 29: position marker P3
    case 29: return WWVB_MARKER;

    // Seconds 30-33: Day of year low (BCD: 8,4,2,1)
    case 30: return (bcd_doy & 0x008) ? WWVB_ONE : WWVB_ZERO;       // 8
    case 31: return (bcd_doy & 0x004) ? WWVB_ONE : WWVB_ZERO;       // 4
    case 32: return (bcd_doy & 0x002) ? WWVB_ONE : WWVB_ZERO;       // 2
    case 33: return (bcd_doy & 0x001) ? WWVB_ONE : WWVB_ZERO;       // 1

    // Seconds 34-35: unused
    case 34: return WWVB_ZERO;
    case 35: return WWVB_ZERO;

    // Seconds 36-38: DUT1 sign (positive: 1,0,1; negative: 0,1,0; zero: 1,0,1)
    // NTP-synced, DUT1 ≈ 0 → set positive sign (1,0,1)
    case 36: return WWVB_ONE;   // DUT1 sign +
    case 37: return WWVB_ZERO;  // DUT1 sign -
    case 38: return WWVB_ONE;   // DUT1 sign +

    // Second 39: position marker P4
    case 39: return WWVB_MARKER;

    // Seconds 40-43: DUT1 magnitude (BCD: 0.8, 0.4, 0.2, 0.1)
    // Set to 0.0 (NTP-synced)
    case 40: return WWVB_ZERO;  // 0.8
    case 41: return WWVB_ZERO;  // 0.4
    case 42: return WWVB_ZERO;  // 0.2
    case 43: return WWVB_ZERO;  // 0.1

    // Second 44: unused
    case 44: return WWVB_ZERO;

    // Seconds 45-48: Year tens (BCD: 80,40,20,10)
    case 45: return (bcd_year & 0x80) ? WWVB_ONE : WWVB_ZERO;  // 80
    case 46: return (bcd_year & 0x40) ? WWVB_ONE : WWVB_ZERO;  // 40
    case 47: return (bcd_year & 0x20) ? WWVB_ONE : WWVB_ZERO;  // 20
    case 48: return (bcd_year & 0x10) ? WWVB_ONE : WWVB_ZERO;  // 10

    // Second 49: position marker P5
    case 49: return WWVB_MARKER;

    // Seconds 50-53: Year units (BCD: 8,4,2,1)
    case 50: return (bcd_year & 0x08) ? WWVB_ONE : WWVB_ZERO;  // 8
    case 51: return (bcd_year & 0x04) ? WWVB_ONE : WWVB_ZERO;  // 4
    case 52: return (bcd_year & 0x02) ? WWVB_ONE : WWVB_ZERO;  // 2
    case 53: return (bcd_year & 0x01) ? WWVB_ONE : WWVB_ZERO;  // 1

    // Second 54: unused
    case 54: return WWVB_ZERO;

    // Second 55: leap year indicator
    case 55: return is_leap ? WWVB_ONE : WWVB_ZERO;

    // Second 56: leap second warning (0 = no leap second this month)
    case 56: return WWVB_ZERO;

    // Seconds 57-58: DST status (00 = DST not in effect)
    case 57: return WWVB_ZERO;
    case 58: return WWVB_ZERO;

    // Second 59: position marker P0 (frame reference for next minute)
    case 59: return WWVB_MARKER;

    default: return WWVB_ZERO;
  }
}

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// main
const unsigned int loop_period_ms(100);
      unsigned int loop_last_ms;
const int button_atom(39);
const int button_atoms3(41);

void setup()
{
  if (M5.getBoard() == m5::board_t::board_M5Atom) {
    pinMode(0, OUTPUT);
    digitalWrite(0, LOW);
  }

  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  cfg.external_rtc    = true;
  M5.begin(cfg);
  delay(3000);
  Serial.println();
  Serial.println("[BF-018A-WWVB] WWVB 60kHz Simulator starting...");

#if defined (CONFIG_IDF_TARGET_ESP32S3)
  FastLED.addLeds<WS2811, led_pin_atoms3lite, GRB>(leds, led_num);
#else
  FastLED.addLeds<WS2811, led_pin_atom, GRB>(leds, led_num);
#endif
  FastLED.setBrightness(led_brightness);
  for (int i = 0; i < led_num; ++i) leds[i] = 0;

  if (rtcx.Begin(Wire) == 0) {
    rtcx_available = true;
    if (SetTimeFromRtcx(time_zone)) {
      localtime_valid = true;
    }
  }
  if (!localtime_valid) {
    Serial.print("RTC not valid: set localtime temporarily\n");
    td.tm_year = 117;
    td.tm_mon  = 0;
    td.tm_mday = 1;
    td.tm_hour = 0;
    td.tm_min  = 0;
    td.tm_sec  = 0;
    struct timeval tv = { mktime(&td), 0 };
    settimeofday(&tv, NULL);
  }
  getLocalTime(&td);
  Serial.print(&td, "UTC localtime: %A, %B %d %Y %H:%M:%S\n");

  WiFiManager wm;
  int button_pin = button_atom;
  if (M5.getBoard() == m5::board_t::board_M5AtomS3Lite) {
    button_pin = button_atoms3;
  }
  if (digitalRead(button_pin) == LOW) {
    wm.resetSettings();
  }
  wm.setConfigPortalTimeout(wifi_config_portal_timeout_sec);
  wm.setAPCallback(WifiConfigModeCallback);
  wm.autoConnect();
  WiFi.setSleep(false);
  wifi_retry_last_ms = millis() - wifi_retry_interval_ms;

  NtpBegin(time_zone, ntp_server);
  TcoInit();

  M5.update();
  loop_last_ms = millis();
}

void loop()
{
  M5.update();
  LedShow();

  WifiCheck();
  if (RtcxUpdate(rtcx_available)) {
    localtime_valid = true;
  }

  if (M5.BtnA.wasReleased()) {
    led_enable = !led_enable;
  }

  unsigned int delay_ms(0);
  unsigned int elapse_ms = millis() - loop_last_ms;
  if (elapse_ms < loop_period_ms) {
    delay_ms = loop_period_ms - elapse_ms;
  }
  delay(delay_ms);
  loop_last_ms = millis();
}
