/**
 * ads1256_encoder.ino
 *
 * Arduino Mega sketch: reads ADS1256 (load cell, DYLY-108) + EN25-Absolute
 * encoder, then streams combined data over USB serial to a ROS bridge node.
 *
 * ── Pin connections ──────────────────────────────────────────────────────
 *  EN25-Absolute  │ Mega
 *  Yellow (TTL TX)│ 19 (RX1)
 *  Red   (5V)     │ 5V
 *  Green (GND)    │ GND
 *
 *  ADS1256        │ Mega
 *  SCLK           │ 52 (SPI SCK)
 *  DIN            │ 51 (SPI MOSI)
 *  DOUT           │ 50 (SPI MISO)
 *  CS             │ 53
 *  DRDY           │ 48
 *  PDWN           │ 49
 *  5V / GND       │ 5V / GND
 *
 *  DYLY-108 load cell → ADS1256
 *  Red   (Exc+)   │ 5V (or external regulated supply)
 *  Black (Exc-)   │ GND
 *  Green (Sig+)   │ AIN0
 *  White (Sig-)   │ AIN1
 *
 * ── Serial output format (USB, OUT_BAUD) ─────────────────────────────────
 *  $SENSORS,<lc_raw>,<lc_mv>,<enc_deg>,<enc_rev>,<enc_rpm>\r\n
 *
 *    lc_raw   : signed 24-bit ADC count (differential AIN0−AIN1)
 *    lc_mv    : differential voltage [mV], 4 decimal places
 *    enc_deg  : absolute angle [0.0 – 359.9]°, 1 decimal place
 *    enc_rev  : cumulative revolutions, 3 decimal places
 *    enc_rpm  : rotation speed [RPM]
 *
 * ── ADS1256 configuration ────────────────────────────────────────────────
 *  PGA = 64  → full-scale input ±39.1 mV  (typical for mV-level load cells)
 *  DRATE = 1000 SPS (DRATE register 0xA1)
 *  MUX  = AIN0(+) vs AIN1(−)  (differential, 0x01)
 *  VREF = 2.5 V (onboard ADR03)
 *
 *  Adjust PGA_SEL if your load cell signal is larger or smaller.
 */

#include <SPI.h>

#if !defined(HAVE_HWSERIAL1)
#error "This sketch requires Arduino Mega 2560 or another board with Serial1. Select Tools > Board > Arduino Mega or Mega 2560."
#endif

// ─── User-configurable constants ─────────────────────────────────────────
#define OUT_BAUD   115200    // USB serial baud rate to ROS host
#define ROS_HZ      100       // ROS stream output rate [Hz]
#define MONITOR_HZ  5         // Human-readable Serial Monitor rate [Hz]
#define PGA_SEL     64        // ADS1256 PGA: 1/2/4/8/16/32/64

// Output mode:
//   1 = ROS mode:      $SENSORS,<lc_raw>,<lc_mv>,<enc_deg>,<enc_rev>,<enc_rpm>
//   2 = Monitor mode:  readable labels for Arduino Serial Monitor
#define ROS_MODE      1
#define MONITOR_MODE  2
#define OUTPUT_MODE   ROS_MODE

// ─── Pin assignments ──────────────────────────────────────────────────────
#define ADS_CS    53
#define ADS_DRDY  48
#define ADS_PDWN  49

// ─── ADS1256 constants ────────────────────────────────────────────────────
#define CMD_WAKEUP  0x00
#define CMD_RDATA   0x01
#define CMD_RDATAC  0x03
#define CMD_SDATAC  0x0F
#define CMD_RREG    0x10
#define CMD_WREG    0x50
#define CMD_SELFCAL 0xF0
#define CMD_SYNC    0xFC
#define CMD_RESET   0xFE

#define REG_STATUS  0x00
#define REG_MUX     0x01
#define REG_ADCON   0x02
#define REG_DRATE   0x03

// MUX: AIN0(+) vs AIN1(−)
#define MUX_DIFF_01 0x01

// DRATE 1000 SPS
#define DRATE_1000  0xA1

// PGA bits: 0→1, 1→2, 2→4, 3→8, 4→16, 5→32, 6→64
static uint8_t pga_bits(int pga) {
    switch (pga) {
        case  2: return 1;
        case  4: return 2;
        case  8: return 3;
        case 16: return 4;
        case 32: return 5;
        case 64: return 6;
        default: return 0;  // PGA = 1
    }
}

#define VREF_MV  2500.0f   // 2.5 V reference in mV

// ─── Global state ─────────────────────────────────────────────────────────
static long    lc_raw        = 0;
static float   lc_mv         = 0.0f;

static float   enc_deg       = 0.0f;
static float   enc_rev       = 0.0f;
static int     enc_rpm       = 0;

static char    enc_buf[64];
static uint8_t enc_len       = 0;

static unsigned long last_out_ms = 0;
static const unsigned long OUT_INTERVAL =
    1000UL / ((OUTPUT_MODE == ROS_MODE) ? ROS_HZ : MONITOR_HZ);

// ─── ADS1256 helpers ──────────────────────────────────────────────────────

inline void ads_cs_low()  { digitalWrite(ADS_CS, LOW);  }
inline void ads_cs_high() { digitalWrite(ADS_CS, HIGH); }

static void ads_wait_drdy() {
    // DRDY goes LOW when a new conversion result is ready.
    unsigned long t0 = millis();
    while (digitalRead(ADS_DRDY) == HIGH) {
        if (millis() - t0 > 2000UL) return;  // safety timeout
    }
}

static void ads_write_reg(uint8_t reg, uint8_t val) {
    ads_cs_low();
    SPI.transfer(CMD_WREG | reg);
    SPI.transfer(0x00);     // write 1 register
    SPI.transfer(val);
    ads_cs_high();
    delayMicroseconds(5);
}

static void ads_init() {
    // Hardware reset via PDWN
    digitalWrite(ADS_PDWN, LOW);
    delay(100);
    digitalWrite(ADS_PDWN, HIGH);
    delay(500);

    ads_wait_drdy();

    // Exit continuous mode (in case already in it)
    ads_cs_low();
    SPI.transfer(CMD_SDATAC);
    ads_cs_high();
    delayMicroseconds(5);

    // STATUS: auto-calibrate on, MSB first, buffer enabled
    ads_write_reg(REG_STATUS, 0x06);

    // ADCON: CLKOUT off (00), sensor detect off (00), PGA
    ads_write_reg(REG_ADCON, pga_bits(PGA_SEL));

    // DRATE: 1000 SPS
    ads_write_reg(REG_DRATE, DRATE_1000);

    // MUX: AIN0(+) vs AIN1(−)
    ads_write_reg(REG_MUX, MUX_DIFF_01);

    // Self-calibration
    ads_cs_low();
    SPI.transfer(CMD_SELFCAL);
    ads_cs_high();
    ads_wait_drdy();

    // Enter continuous-read mode (RDATAC)
    ads_cs_low();
    SPI.transfer(CMD_RDATAC);
    ads_cs_high();
    delayMicroseconds(5);
}

// Call only when DRDY is already LOW (in RDATAC mode)
static long ads_read_raw() {
    ads_cs_low();
    uint8_t b2 = SPI.transfer(0x00);  // MSB
    uint8_t b1 = SPI.transfer(0x00);
    uint8_t b0 = SPI.transfer(0x00);  // LSB
    ads_cs_high();

    long raw = ((long)b2 << 16) | ((long)b1 << 8) | b0;
    if (raw & 0x800000L) raw -= 0x1000000L;  // sign-extend to 32-bit
    return raw;
}

// Convert signed count to millivolts (differential, gain applied)
static float ads_to_mv(long raw) {
    return ((float)raw / 0x7FFFFF) * VREF_MV / (float)PGA_SEL;
}

// ─── Encoder parser ───────────────────────────────────────────────────────

// Expected: "$ANG,152.3,11.644,110"  (stripped of \r\n)
static void parse_encoder(const char *buf) {
    if (strncmp(buf, "$ANG,", 5) != 0) return;

    const char *p = buf + 5;
    char *end;

    enc_deg = strtod(p, &end);
    if (*end != ',') return;
    p = end + 1;

    enc_rev = strtod(p, &end);
    if (*end != ',') return;
    p = end + 1;

    enc_rpm = (int)strtol(p, nullptr, 10);
}

// ─── Output helpers ──────────────────────────────────────────────────────

static void print_ros_frame() {
    // Keep this exact format for the ROS parser.
    Serial.print(F("$SENSORS,"));
    Serial.print(lc_raw);
    Serial.print(',');
    Serial.print(lc_mv, 4);
    Serial.print(',');
    Serial.print(enc_deg, 1);
    Serial.print(',');
    Serial.print(enc_rev, 3);
    Serial.print(',');
    Serial.println(enc_rpm);
}

static void print_monitor_frame() {
    Serial.print(F("LoadCell | raw="));
    Serial.print(lc_raw);
    Serial.print(F("  voltage_mv="));
    Serial.print(lc_mv, 4);
    Serial.print(F("  ||  Encoder | angle_deg="));
    Serial.print(enc_deg, 1);
    Serial.print(F("  rev="));
    Serial.print(enc_rev, 3);
    Serial.print(F("  rpm="));
    Serial.println(enc_rpm);
}

static void print_output_frame() {
#if OUTPUT_MODE == ROS_MODE
    print_ros_frame();
#else
    print_monitor_frame();
#endif
}

// ─── Setup ────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(OUT_BAUD);
    Serial1.begin(38400);       // EN25-Absolute TTL on pin 19 (RX1)

    pinMode(ADS_CS,   OUTPUT);
    pinMode(ADS_DRDY, INPUT);
    pinMode(ADS_PDWN, OUTPUT);

    digitalWrite(ADS_CS,   HIGH);
    digitalWrite(ADS_PDWN, HIGH);

    SPI.begin();
    // ADS1256: SPI mode 1 (CPOL=0, CPHA=1), 1 MHz (safe below 1.92 MHz max)
    SPI.beginTransaction(SPISettings(1000000UL, MSBFIRST, SPI_MODE1));

    ads_init();

#if OUTPUT_MODE == ROS_MODE
    Serial.println(F("# enc_lc ready: ROS_MODE, $SENSORS stream"));
#else
    Serial.println(F("# enc_lc ready: MONITOR_MODE, human-readable output"));
#endif
}

// ─── Loop ─────────────────────────────────────────────────────────────────
void loop() {
    // ── 1. Read ADS1256 (blocking until DRDY, ~1 ms at 1000 SPS) ──
    ads_wait_drdy();
    lc_raw = ads_read_raw();
    lc_mv  = ads_to_mv(lc_raw);

    // ── 2. Read encoder (non-blocking byte accumulation) ──
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (c == '\n') {
            enc_buf[enc_len] = '\0';
            parse_encoder(enc_buf);
            enc_len = 0;
        } else if (c != '\r' && enc_len < 63) {
            enc_buf[enc_len++] = c;
        }
    }

    // ── 3. Output at fixed rate ──
    unsigned long now = millis();
    if (now - last_out_ms >= OUT_INTERVAL) {
        last_out_ms = now;

        print_output_frame();
    }
}
