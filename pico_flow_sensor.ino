/*
  Air-flow sensor: 3 RTDs + PI-controlled heater   (Raspberry Pi Pico, Arduino IDE)
  ===================================================================================
  How it works
    RTD_h  (glued to heater)   -> PI controller keeps the heater at T_up + OVERHEAT
    RTD_up (upstream)          -> air temperature
    RTD_dn (downstream)        -> warmed by the heater, cooled by the air: falls as v rises
    Velocity (King's law):
        Y = (T_h - T_up) / (T_dn - T_up) - 1          Y = A + B * v^n
        v = ((Y - A) / B)^(1/n)                       A, B, n from calibration ('p', 'f')

  Wiring
    RTD_up : bridge -> LM358 (gain 5) -> ADS1115 #1 (ADDR->GND, 0x48) A0
    RTD_dn : bridge -> LM358 (gain 5) -> ADS1115 #2 (ADDR->VDD, 0x49) A0
    RTD_h  : MAX31865 (3-wire, RREF 4300)  SPI0: SCK GP18, MOSI GP19, MISO GP16, CS GP17
             (or bridge -> LM358 gain 3 -> ADS #1 A1: set HEATER_RTD_ON_MAX false)
    ADS1115: SDA GP4, SCL GP5
    Heater : +V_H -> thermal fuse -> heater -> NMOS drain, source -> GND
             GP15 -> 100 ohm -> gate, 10k gate -> GND

  Board: "Raspberry Pi Pico/RP2040" (Earle Philhower). Library: "Adafruit MAX31865".
  Serial Monitor: 115200 baud, "Newline".

  Commands
    r <T>       calibrate the bridge channels: heater off, all RTDs at T degC (do it twice,
                at two temperatures >= 10 degC apart, e.g. room and warm)
    m on|off    heater PI on / off
    p <v>       store a calibration point at known air velocity v [m/s]
    f           fit King's law (A, B, n) to the stored points
    t           step test: prints suggested PI gains
    g <Kp> <Ki> set PI gains          o <dT>  set heater overheat [degC]
    s           show settings
*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_MAX31865.h>

// ------------------------------- settings ---------------------------------------
#define HEATER_RTD_ON_MAX true      // true: RTD_h on MAX31865, false: bridge on ADS #1 A1

const float VEXC = 3.3f;            // bridge supply [V]
const float GAIN_AIR = 5.0f;        // LM358 gain for RTD_up and RTD_dn
const float GAIN_HEATER = 3.0f;     // LM358 gain for RTD_h (bridge option only)
const float VO_MAX = 1.65f;         // LM358 on 3.3 V saturates near 1.8 V
const float RREF = 4300.0f;         // MAX31865 reference resistor (PT1000 board)
const int   PIN_HEATER = 15, PIN_CS = 17;
const float T_H_MAX = 220.0f;       // software trip [degC] (thermal fuse ~240 C)
const float CLAMP_C = 15.0f;        // heater cut while more than this above setpoint
const unsigned long LOOP_MS = 100;  // 10 Hz control loop

Adafruit_MAX31865 maxH(PIN_CS);

// ------------------------------- calibration (kept in flash) --------------------
struct Cal {
  uint32_t magic;
  float off[3], span[3];            // per bridge channel: Vo = off + span * x
  float maxOff;                     // MAX31865 correction [degC]
  float overheat;                   // heater setpoint above air [degC]
  float Kp, Ki;                     // PI gains (output = duty 0..1)
  float A, B, n;                    // King's law
} cal;
const uint32_t MAGIC = 0x33525444;
enum { UP = 0, DN = 1, HT = 2 };    // channel index

void defaults() {
  cal.magic = MAGIC;
  for (int i = 0; i < 3; i++) cal.off[i] = 0;
  cal.span[UP] = cal.span[DN] = VEXC * GAIN_AIR;
  cal.span[HT] = VEXC * GAIN_HEATER;
  cal.maxOff = 0;
  cal.overheat = 160;               // heater at about 180 C in a 20 C room
  cal.Kp = 0.008f; cal.Ki = 0.002f; // starting values: tune with 't'
  cal.A = 0.3f; cal.B = 0.5f; cal.n = 0.5f;   // placeholders: calibrate with 'p' + 'f'
}
void save() { EEPROM.put(0, cal); EEPROM.commit(); }

// ------------------------------- RTD maths --------------------------------------
// PT1000: R = 1000 (1 + a T + b T^2)          (Callendar-Van Dusen, T >= 0 C)
const float CA = 3.9083e-3f, CB = -5.775e-7f;
float rtdR(float T) { return 1000.0f * (1 + CA * T + CB * T * T); }
float rtdT(float R) { return (-CA + sqrtf(CA * CA - 4 * CB * (1 - R / 1000.0f))) / (2 * CB); }
// Quarter bridge with 1k resistors:  Vd = Vexc * x,  x = (R - 1k) / (2 (R + 1k))
float bridgeX(float R) { return (R - 1000.0f) / (2 * (R + 1000.0f)); }
float bridgeR(float x) { return 1000.0f * (1 + 2 * x) / (1 - 2 * x); }

// ------------------------------- ADS1115 (single-ended, +-2.048 V, 128 SPS) -----
bool adsRead(uint8_t addr, uint8_t input, float &volts) {
  uint16_t cfg = 0x8000 | ((4 + input) << 12) | (2 << 9) | 0x0100 | (4 << 5) | 3;
  Wire.beginTransmission(addr);
  Wire.write(1); Wire.write(cfg >> 8); Wire.write(cfg & 0xFF);
  if (Wire.endTransmission()) return false;
  unsigned long t0 = millis();
  for (;;) {                                       // wait until conversion done
    Wire.beginTransmission(addr); Wire.write(1);
    if (Wire.endTransmission() || Wire.requestFrom(addr, (uint8_t)2) != 2) return false;
    uint8_t hi = Wire.read(); Wire.read();
    if (hi & 0x80) break;
    if (millis() - t0 > 50) return false;
  }
  Wire.beginTransmission(addr); Wire.write(0);
  if (Wire.endTransmission() || Wire.requestFrom(addr, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read(), lo = Wire.read();
  volts = (int16_t)((hi << 8) | lo) * 2.048f / 32768.0f;
  return true;
}

// ------------------------------- state ------------------------------------------
float vo[3];                         // raw amplifier voltages
float Tup = NAN, Tdn = NAN, Th = NAN, rawMax = NAN;
float duty = 0, integ = 0, Y = NAN, v = NAN;
bool heaterOn = true;
const char *fault = nullptr;
float ptV[30], ptY[30]; int nPts = 0;
char job = 0; int jobI = 0; float jobRef, s1, s2, s3, rP[2][5]; int rN = 0;
float stepT0, stepLog[300];

void setDuty(float d) { duty = constrain(d, 0.0f, 1.0f); analogWrite(PIN_HEATER, (int)(duty * 1000)); }

// Bridge channel: volts -> x -> resistance -> temperature
float bridgeTemp(int ch) { return rtdT(bridgeR((vo[ch] - cal.off[ch]) / cal.span[ch])); }

bool readSensors() {
  if (!adsRead(0x48, 0, vo[UP]) || !adsRead(0x49, 0, vo[DN])) { fault = "ADS1115"; return false; }
  if (vo[UP] > VO_MAX || vo[DN] > VO_MAX) { fault = "AMP_SATURATED"; return false; }
  float tu = bridgeTemp(UP), td = bridgeTemp(DN), th;
#if HEATER_RTD_ON_MAX
  uint16_t raw = maxH.readRTD();                   // ~75 ms one-shot conversion
  if (maxH.readFault()) { maxH.clearFault(); fault = "RTD_h FAULT"; return false; }
  rawMax = rtdT(raw * RREF / 32768.0f);            // 15-bit ratio * Rref = resistance
  th = rawMax + cal.maxOff;
#else
  if (!adsRead(0x48, 1, vo[HT])) { fault = "ADS1115"; return false; }
  if (vo[HT] > VO_MAX) { fault = "AMP_SATURATED"; return false; }
  th = bridgeTemp(HT);
#endif
  // light smoothing (exponential moving average)
  Tup = isnan(Tup) ? tu : Tup + 0.2f * (tu - Tup);
  Tdn = isnan(Tdn) ? td : Tdn + 0.5f * (td - Tdn);
  Th = th;                                         // unfiltered: it is the PI input
  return true;
}

// ------------------------------- PI controller ----------------------------------
void runPI() {
  if (fault || !heaterOn || job == 'r') { setDuty(0); integ = 0; return; }
  if (job == 't') return;                          // step test drives the heater itself
  float dt = LOOP_MS / 1000.0f;
  float e = (Tup + cal.overheat) - Th;             // error = setpoint - heater temperature
  float u = cal.Kp * e + integ;                    // PI output (duty)
  if ((u < 1 || e < 0) && (u > 0 || e > 0))        // anti-windup: stop integrating at limits
    integ += cal.Ki * e * dt;
  // Overshoot clamp: if the airflow suddenly stops, the heater can overshoot because
  // RTD_h lags the coil. Cut the heater while it is more than CLAMP_C above setpoint.
  if (e < -CLAMP_C) setDuty(0);
  else setDuty(cal.Kp * e + integ);
}

// ------------------------------- velocity ---------------------------------------
void calcVelocity() {
  float dTh = Th - Tup, dTdn = Tdn - Tup;
  Y = NAN; v = NAN;
  if (!heaterOn || dTdn < 0.5f || dTh < 10) return;
  Y = dTh / dTdn - 1;                              // heat balance of RTD_dn
  v = (Y > cal.A) ? powf((Y - cal.A) / cal.B, 1 / cal.n) : 0;   // invert King's law
}

// ------------------------------- calibration jobs -------------------------------
void fitKing() {
  // For a trial n, Y = A + B*u (u = v^n) is a straight line -> least squares for A, B.
  if (nPts < 4) { Serial.println("# need at least 4 points"); return; }
  double best = 1e30;
  for (double n = 0.25; n <= 1.0; n += 0.005) {
    double Su = 0, SY = 0, Suu = 0, SuY = 0;
    for (int i = 0; i < nPts; i++) { double u = pow(ptV[i], n); Su += u; SY += ptY[i]; Suu += u * u; SuY += u * ptY[i]; }
    double B = (nPts * SuY - Su * SY) / (nPts * Suu - Su * Su), A = (SY - B * Su) / nPts, err = 0;
    for (int i = 0; i < nPts; i++) { double r = ptY[i] - A - B * pow(ptV[i], n); err += r * r; }
    if (B > 0 && err < best) { best = err; cal.A = A; cal.B = B; cal.n = n; }
  }
  save();
  Serial.printf("# King's law: Y = %.4f + %.4f * v^%.3f\n", cal.A, cal.B, cal.n);
  for (int i = 0; i < nPts; i++) {
    float vf = ptY[i] > cal.A ? pow((ptY[i] - cal.A) / cal.B, 1 / cal.n) : 0;
    Serial.printf("#   v = %.2f  ->  fit %.3f\n", ptV[i], vf);
  }
}

void runJob() {
  if (!job) return;
  jobI++;
  if (job == 'r') {                                // 3 s average, heater off
    if (jobI <= 10) return;                        // let readings settle first
    s1 += vo[UP]; s2 += vo[DN]; s3 += HEATER_RTD_ON_MAX ? rawMax : vo[HT];
    if (jobI < 40) return;
    rP[rN][0] = jobRef; rP[rN][1] = s1 / 30; rP[rN][2] = s2 / 30; rP[rN][3] = s3 / 30;
    Serial.printf("# point %d at %.1f C stored\n", rN + 1, jobRef);
    if (++rN == 2) {
      // two points per channel:  span = dVo / dx,  off = Vo1 - span * x1
      float x1 = bridgeX(rtdR(rP[0][0])), x2 = bridgeX(rtdR(rP[1][0]));
      for (int ch = 0; ch < (HEATER_RTD_ON_MAX ? 2 : 3); ch++) {
        cal.span[ch] = (rP[0][ch + 1] - rP[1][ch + 1]) / (x1 - x2);
        cal.off[ch] = rP[0][ch + 1] - cal.span[ch] * x1;
        Serial.printf("# channel %d: real gain %.3f\n", ch, cal.span[ch] / VEXC);
      }
      if (HEATER_RTD_ON_MAX)                        // MAX31865: average offset only
        cal.maxOff = ((rP[0][0] - rP[0][3]) + (rP[1][0] - rP[1][3])) / 2;
      save(); rN = 0;
    }
    job = 0;
  } else if (job == 'p') {                         // 5 s average of Y
    s1 += Y;
    if (jobI < 50) return;
    float Ym = s1 / 50;
    if (!isnan(Ym) && nPts < 30) { ptV[nPts] = jobRef; ptY[nPts] = Ym; nPts++; }
    Serial.printf("CAL,%.3f,%.3f,%.3f,%.3f,%.4f\n", jobRef, Tup, Tdn, Th, Ym);
    job = 0;
  } else if (job == 't') {                         // +5 % duty for 30 s
    if (jobI == 1) { stepT0 = Th; setDuty(duty + 0.05f); }
    stepLog[jobI - 1] = Th;
    if (jobI == 300) {
      float d = stepLog[299] - stepT0, K = d / 0.05f;   // gain: degC per unit duty
      int i = 0;
      while (i < 299 && stepLog[i] - stepT0 < 0.632f * d) i++;
      float tau = (i + 1) * LOOP_MS / 1000.0f;           // time constant [s]
      // lambda tuning, lambda = tau/2 (closed loop twice as fast as the heater):
      //   Kp = tau / (K * lambda) = 2/K,   Ki = Kp / tau
      Serial.printf("# K = %.1f C/duty, tau = %.1f s  ->  g %.4f %.5f\n", K, tau, 2 / K, 2 / (K * tau));
      integ = duty;
      job = 0;
    }
  }
}

// ------------------------------- commands ---------------------------------------
void command(char *line) {
  float a = 0, b = 0;
  int n = sscanf(line + 1, "%f %f", &a, &b);
  s1 = s2 = s3 = 0; jobI = 0;
  switch (line[0]) {
    case 'r': if (n >= 1) { job = 'r'; jobRef = a; } break;
    case 'p': if (n >= 1) { job = 'p'; jobRef = a; } break;
    case 'f': fitKing(); break;
    case 't': job = 't'; break;
    case 'g': if (n >= 2) { cal.Kp = a; cal.Ki = b; save(); } break;
    case 'o': if (n >= 1) { cal.overheat = a; save(); } break;
    case 'm': heaterOn = strstr(line, "on") != nullptr; fault = nullptr; break;
  }
  Serial.printf("# overheat %.0f C | Kp %.4f Ki %.5f | A %.4f B %.4f n %.3f | points %d\n",
                cal.overheat, cal.Kp, cal.Ki, cal.A, cal.B, cal.n, nPts);
}

// ------------------------------- setup / loop -----------------------------------
void setup() {
  pinMode(PIN_HEATER, OUTPUT);
  analogWriteFreq(1000); analogWriteRange(1000); setDuty(0);   // heater off first
  Serial.begin(115200);
  Wire.setSDA(4); Wire.setSCL(5); Wire.begin(); Wire.setClock(400000);
#if HEATER_RTD_ON_MAX
  maxH.begin(MAX31865_3WIRE);
  maxH.enable50Hz(true);
#endif
  EEPROM.begin(256);
  EEPROM.get(0, cal);
  if (cal.magic != MAGIC) defaults();
  Serial.println("# ms,v_mps,T_up,T_dn,T_heater,duty,Y,status");
}

void loop() {
  static char buf[32]; static int len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { buf[len] = 0; if (len) command(buf); len = 0; }
    else if (c != '\r' && len < 31) buf[len++] = c;
  }
  static unsigned long next = 0;
  if ((long)(millis() - next) < 0) return;
  next = millis() + LOOP_MS;

  if (!readSensors()) setDuty(0);
  else {
    if (Th > T_H_MAX) fault = "HEATER_TOO_HOT";
    runPI();
    calcVelocity();
    runJob();
  }
  Serial.printf("%lu,%.3f,%.2f,%.2f,%.1f,%.3f,%.4f,%s\n", millis(), v, Tup, Tdn, Th, duty, Y,
                fault ? fault : (job ? "CAL" : "OK"));
}
