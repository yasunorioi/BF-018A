// BF-018A-MSF: MSF (60kHz, UK NPL) Simulator for ATOM Lite / ATOM Matrix / ATOMS3 Lite
//
// Based on BF-018A Rev.4 by BotanicFields, Inc. (https://github.com/botanicfields/BF-018A)
// MSF timecode port: replaces JJY timecode generation with MSF (Time from NPL, 60kHz).
//
// MSF spec ref: https://en.wikipedia.org/wiki/Time_from_NPL_(MSF)
//              https://www.npl.co.uk/products-services/time-frequency/msf-signal
//
// Key differences from JJY:
//   - Carrier: 60kHz only (JJY: 40kHz east / 60kHz west)
//   - Timezone: UTC (JJY: JST+9). BST flag set when UK summer time is active.
//   - OOK pattern: carrier OFF at second-start (JJY: ON at second-start)
//   - 2 bits per second (Bit A + Bit B). JJY is 1 bit per second.
//   - Data order: Year, Month, Day, DoW, Hour, Minute (JJY: Minute, Hour, Day-of-year, Year, DoW)
//   - Parity: 4x odd-parity bits P1-P4 in Bit B (JJY: 2x even-parity in Bit A)
//   - No position markers (JJY has them at seconds 9,19,29,39,49,59)
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
const uint32_t msf_frequency(60000);  // MSF: 60kHz only
struct tm       td;  // time of day (UTC): year, month, day, wday, hour, min, sec
struct timespec ts;  // time spec: second, nano-second

// MSF bit pair: Bit A carries time data, Bit B carries parity/flags
struct MsfBit {
  int a;  // 0 or 1
  int b;  // 0 or 1
};

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// for NTP — UTC0: always UTC (no automatic DST adjustment)
// BST detection is done manually in IsBst() below.
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
// TCO (Time Code Output) — MSF version
Ticker tk;
const int ticker_interval_ms(100);  // 100ms resolution

// PWM for TCO signal
const uint8_t  ledc_pin_atom(22);
const uint8_t  ledc_pin_atoms3(5);
const uint32_t ledc_frequency(msf_frequency);
const uint8_t  ledc_resolution(8);
const uint32_t ledc_duty_on(128);
const uint32_t ledc_duty_off(0);
      uint8_t  ledc_pin(ledc_pin_atom);

void TcoInit()
{
  if (M5.getBoard() == m5::board_t::board_M5AtomS3Lite) {
    ledc_pin = ledc_pin_atoms3;
  }
  Serial.printf("ledcAttach result= %d\n", ledcAttach(ledc_pin, ledc_frequency, ledc_resolution));
  Serial.printf("ledcWrite result= %d\n",  ledcWrite(ledc_pin, ledc_duty_on));
  Serial.printf("pin= %u, duty= %lu, freq= %lu\n", ledc_pin, ledcRead(ledc_pin), ledcReadFreq(ledc_pin));

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
    case 0: Msf000ms();  break;  // all: TcOff + minute marker print
    case 1: Msf100ms();  break;  // A=0: TcOn
    case 2: Msf200ms();  break;  // A=0,B=1: TcOff  |  A=1,B=0: TcOn
    case 3: Msf300ms();  break;  // B=1: TcOn
    case 5: Msf500ms();  break;  // sec==0 (minute marker): TcOn
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

// --- MSF OOK modulation ---
//
// MSF bit pattern within each second (100ms steps):
//
//   A=0, B=0 :  |_100ms|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|  (100ms OFF)
//   A=1, B=0 :  |___200ms___|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|  (200ms OFF)
//   A=0, B=1 :  |_100ms|‾100ms‾|_100ms|‾‾‾‾‾‾‾‾‾‾‾‾‾‾|  (split: 100-100-100ms)
//   A=1, B=1 :  |_______300ms_________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾|  (300ms OFF)
//   sec==0    :  |___________500ms___________|‾‾‾‾‾‾‾|  (minute marker: 500ms OFF)

void Msf000ms()
{
  TcOff();  // ALL seconds start with carrier OFF
  if (td.tm_sec == 0) {
    Serial.print(&td, "\n[MSF] %A %d %b %Y %H:%M:%S UTC\n");
  }
}

void Msf100ms()
{
  if (td.tm_sec == 0) return;  // minute marker: remain OFF until 500ms
  MsfBit bit = MsfValue();
  if (bit.a == 0) TcOn();      // A=0: carrier ON at 100ms  (both B=0 and B=1 cases)
}

void Msf200ms()
{
  if (td.tm_sec == 0) return;
  MsfBit bit = MsfValue();
  if      (bit.a == 0 && bit.b == 1) TcOff();  // A=0,B=1: OFF again at 200ms (split)
  else if (bit.a == 1 && bit.b == 0) TcOn();   // A=1,B=0: ON at 200ms
  // A=0,B=0: already ON since 100ms — no change
  // A=1,B=1: still OFF                — no change
}

void Msf300ms()
{
  if (td.tm_sec == 0) return;
  MsfBit bit = MsfValue();
  if (bit.b == 1) TcOn();  // A=0,B=1 and A=1,B=1: ON at 300ms
}

void Msf500ms()
{
  if (td.tm_sec == 0) TcOn();  // minute marker ends at 500ms
}

void TcOn()
{
  ledcWrite(ledc_pin, ledc_duty_on);
  led_b = led_b_on;
}

void TcOff()
{
  ledcWrite(ledc_pin, ledc_duty_off);
  led_b = led_b_off;
}

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// BST (British Summer Time) detection
// BST: last Sunday in March 01:00 UTC → last Sunday in October 01:00 UTC

// Returns the day-of-month of the last Sunday in the given month/year (UTC)
int LastSundayOfMonth(int year, int month)
{
  struct tm tmp = {};
  tmp.tm_year = year - 1900;
  tmp.tm_mon  = month - 1;   // 0-based
  tmp.tm_mday = 31;          // deliberately overflowing — mktime normalises
  tmp.tm_isdst = 0;
  mktime(&tmp);              // normalise: tmp.tm_mday → last day of month
  int last_day = tmp.tm_mday;
  int wday     = tmp.tm_wday;  // 0 = Sunday
  return last_day - wday;      // day of last Sunday (may need wrap: wday==0→last_day)
}

bool IsBst(const struct tm& t)
{
  int year  = t.tm_year + 1900;
  int month = t.tm_mon  + 1;   // 1-12
  int day   = t.tm_mday;
  int hour  = t.tm_hour;

  if (month < 3 || month > 10) return false;  // Jan, Feb, Nov, Dec → UTC
  if (month > 3 && month < 10) return true;   // Apr – Sep → BST

  int ls = LastSundayOfMonth(year, month);
  if (month == 3) {
    // BST starts on last Sunday of March at 01:00 UTC
    return (day > ls) || (day == ls && hour >= 1);
  } else {
    // BST ends on last Sunday of October at 01:00 UTC
    return (day < ls) || (day == ls && hour < 1);
  }
}

// Warning flag: set 1 hour before BST change (last Sunday Mar/Oct at 00:xx UTC)
bool IsBstChangeWarning(const struct tm& t)
{
  int year  = t.tm_year + 1900;
  int month = t.tm_mon  + 1;
  int day   = t.tm_mday;
  int hour  = t.tm_hour;

  if (month == 3 || month == 10) {
    int ls = LastSundayOfMonth(year, month);
    if (day == ls && hour == 0) return true;
  }
  return false;
}

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// BCD and parity helpers

// 2-digit decimal → BCD (e.g. 25 → 0x25 = 37)
int Int2Bcd(int a)
{
  return (a % 10) + (a / 10 % 10 * 16);
}

// Count the number of 1-bits in the lowest num_bits bits of val
int CountBits(int val, int num_bits)
{
  int count = 0;
  for (int i = 0; i < num_bits; i++) count += (val >> i) & 1;
  return count;
}

// Odd parity: returns 1 if current 1-count is even (need to add 1 to make it odd)
int OddParity(int val, int num_bits)
{
  return (CountBits(val, num_bits) % 2 == 0) ? 1 : 0;
}

//..:....1....:....2....:....3....:....4....:....5....:....6....:....7..
// MSF bit value for current second
//
// Bit A layout (seconds 17–51, MSB first):
//   17-24: year (2 digits, BCD 8 bits,  weights 80,40,20,10, 8,4,2,1)
//   25-29: month         (BCD 5 bits,  weights 10, 8, 4, 2,1)
//   30-35: day           (BCD 6 bits,  weights 20,10, 8, 4,2,1)
//   36-38: day-of-week   (3 bits,      weights  4, 2, 1  — 0=Sun…6=Sat)
//   39-44: hour          (BCD 6 bits,  weights 20,10, 8, 4,2,1)
//   45-51: minute        (BCD 7 bits,  weights 40,20,10, 8,4,2,1)
//   52-59: 0,1,1,1,1,1,1,0  (fixed end-of-minute identifier)
//
// Bit B layout (seconds 52–58):
//   52: 0
//   53: BST change warning (1 = change in next 1 hour)
//   54: P1 — odd parity over seconds 17A–24A  (year, 8 bits)
//   55: P2 — odd parity over seconds 25A–35A  (month 5 + day 6 = 11 bits)
//   56: P3 — odd parity over seconds 36A–38A  (day-of-week, 3 bits)
//   57: P4 — odd parity over seconds 39A–51A  (hour 6 + minute 7 = 13 bits)
//   58: BST flag (0 = UTC in effect, 1 = BST in effect)
//   59: 0

MsfBit MsfValue()
{
  // UTC time fields
  int year_2d    = td.tm_year % 100;          // 2-digit year (e.g. 26)
  int bcd_year   = Int2Bcd(year_2d);          // e.g. 0x26
  int bcd_month  = Int2Bcd(td.tm_mon + 1);   // 1–12 as BCD
  int bcd_day    = Int2Bcd(td.tm_mday);       // 1–31 as BCD
  int day_of_week = td.tm_wday;               // 0=Sun … 6=Sat
  int bcd_hour   = Int2Bcd(td.tm_hour);       // 0–23 as BCD
  int bcd_minute = Int2Bcd(td.tm_min);        // 0–59 as BCD

  // Parity (odd parity — result bit = 1 if 1-count is currently even)
  int p1 = OddParity(bcd_year  & 0xFF, 8);
  int p2 = OddParity((bcd_month & 0x1F) << 6 | (bcd_day & 0x3F), 11);
  int p3 = OddParity(day_of_week & 0x07, 3);
  int p4 = OddParity((bcd_hour & 0x3F) << 7 | (bcd_minute & 0x7F), 13);

  int bst      = IsBst(td)              ? 1 : 0;
  int bst_warn = IsBstChangeWarning(td) ? 1 : 0;

  MsfBit bit = {0, 0};

  switch (td.tm_sec) {

    // Second 0: minute marker — OOK handled by Msf500ms(), data bits unused
    case  0: bit = {0, 0};  break;

    // Seconds 1–16: DUT1 (UT1 – UTC correction, CCIR Method B)
    // Set to 0 — NTP keeps UTC accurate; DUT1 rarely exceeds ±0.9 s
    case  1: case  2: case  3: case  4:
    case  5: case  6: case  7: case  8:
    case  9: case 10: case 11: case 12:
    case 13: case 14: case 15: case 16:
      bit = {0, 0};  break;

    // Seconds 17–24: Year (BCD 8 bits, MSB first)
    case 17: bit = {(bcd_year & 0x80) ? 1 : 0, 0};  break;
    case 18: bit = {(bcd_year & 0x40) ? 1 : 0, 0};  break;
    case 19: bit = {(bcd_year & 0x20) ? 1 : 0, 0};  break;
    case 20: bit = {(bcd_year & 0x10) ? 1 : 0, 0};  break;
    case 21: bit = {(bcd_year & 0x08) ? 1 : 0, 0};  break;
    case 22: bit = {(bcd_year & 0x04) ? 1 : 0, 0};  break;
    case 23: bit = {(bcd_year & 0x02) ? 1 : 0, 0};  break;
    case 24: bit = {(bcd_year & 0x01) ? 1 : 0, 0};  break;

    // Seconds 25–29: Month (BCD 5 bits, weights 10,8,4,2,1)
    case 25: bit = {(bcd_month & 0x10) ? 1 : 0, 0};  break;
    case 26: bit = {(bcd_month & 0x08) ? 1 : 0, 0};  break;
    case 27: bit = {(bcd_month & 0x04) ? 1 : 0, 0};  break;
    case 28: bit = {(bcd_month & 0x02) ? 1 : 0, 0};  break;
    case 29: bit = {(bcd_month & 0x01) ? 1 : 0, 0};  break;

    // Seconds 30–35: Day (BCD 6 bits, weights 20,10,8,4,2,1)
    case 30: bit = {(bcd_day & 0x20) ? 1 : 0, 0};  break;
    case 31: bit = {(bcd_day & 0x10) ? 1 : 0, 0};  break;
    case 32: bit = {(bcd_day & 0x08) ? 1 : 0, 0};  break;
    case 33: bit = {(bcd_day & 0x04) ? 1 : 0, 0};  break;
    case 34: bit = {(bcd_day & 0x02) ? 1 : 0, 0};  break;
    case 35: bit = {(bcd_day & 0x01) ? 1 : 0, 0};  break;

    // Seconds 36–38: Day of week (3 bits, weights 4,2,1 — 0=Sun … 6=Sat)
    case 36: bit = {(day_of_week & 0x04) ? 1 : 0, 0};  break;
    case 37: bit = {(day_of_week & 0x02) ? 1 : 0, 0};  break;
    case 38: bit = {(day_of_week & 0x01) ? 1 : 0, 0};  break;

    // Seconds 39–44: Hour (BCD 6 bits, weights 20,10,8,4,2,1)
    case 39: bit = {(bcd_hour & 0x20) ? 1 : 0, 0};  break;
    case 40: bit = {(bcd_hour & 0x10) ? 1 : 0, 0};  break;
    case 41: bit = {(bcd_hour & 0x08) ? 1 : 0, 0};  break;
    case 42: bit = {(bcd_hour & 0x04) ? 1 : 0, 0};  break;
    case 43: bit = {(bcd_hour & 0x02) ? 1 : 0, 0};  break;
    case 44: bit = {(bcd_hour & 0x01) ? 1 : 0, 0};  break;

    // Seconds 45–51: Minute (BCD 7 bits, weights 40,20,10,8,4,2,1)
    case 45: bit = {(bcd_minute & 0x40) ? 1 : 0, 0};  break;
    case 46: bit = {(bcd_minute & 0x20) ? 1 : 0, 0};  break;
    case 47: bit = {(bcd_minute & 0x10) ? 1 : 0, 0};  break;
    case 48: bit = {(bcd_minute & 0x08) ? 1 : 0, 0};  break;
    case 49: bit = {(bcd_minute & 0x04) ? 1 : 0, 0};  break;
    case 50: bit = {(bcd_minute & 0x02) ? 1 : 0, 0};  break;
    case 51: bit = {(bcd_minute & 0x01) ? 1 : 0, 0};  break;

    // Seconds 52–59: End-of-minute identifier
    // Bit A fixed pattern: 0,1,1,1,1,1,1,0
    // Bit B: 0, BST_warn, P1, P2, P3, P4, BST, 0
    case 52: bit = {0, 0};         break;
    case 53: bit = {1, bst_warn};  break;
    case 54: bit = {1, p1};        break;
    case 55: bit = {1, p2};        break;
    case 56: bit = {1, p3};        break;
    case 57: bit = {1, p4};        break;
    case 58: bit = {1, bst};       break;
    case 59: bit = {0, 0};         break;

    default: bit = {0, 0};  break;
  }
  return bit;
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
  Serial.println("[BF-018A-MSF] MSF 60kHz Simulator starting...");

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
