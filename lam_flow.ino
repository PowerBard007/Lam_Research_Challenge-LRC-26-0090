 // =====================================================================
// LAM Research Challenge 3.0 - Stage 2 Practical Engineering Challenge
// Task 1 / Q1.1 : Case 1 (Control + Measurement) -> Case 2 (Flush + Shutdown)
// Target: NodeMCU-32S on "LAM Stage 2 PCB" (pin map in config.h)
//
// Serial log (115200): one CSV line per SAMPLE_MS, prefix "D,"
//   D,t_s,state,f_hz,q_lpm,q_mls,total_L,case1_L,case2_L,p1,p2,p3,sv,float
// =====================================================================
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

// ---------------------------------------------------------------- state
enum State : uint8_t { ST_IDLE, ST_CASE1, ST_ISOLATE, ST_CASE2, ST_DONE, ST_ESTOP, ST_MANUAL, ST_CAL };
static const char *STATE_NAME[] = { "IDLE", "CASE1", "ISOLATE", "CASE2", "DONE", "ESTOP", "MANUAL", "CAL" };
static State g_state = ST_IDLE;
static uint32_t g_stateMs = 0;            // millis() at state entry

// ---------------------------------------------------------------- pumps (BTS7960)
struct Pump {
  uint8_t rpwm, lpwm, ren, len, chR, chL;
  bool reverse;
  uint8_t duty;                           // percent
};
static Pump g_pump[3] = {
  { PIN_P1_RPWM, PIN_P1_LPWM, PIN_P1_REN, PIN_P1_LEN, 0, 1, P1_REVERSE, 0 },
  { PIN_P2_RPWM, PIN_P2_LPWM, PIN_P2_REN, PIN_P2_LEN, 2, 3, P2_REVERSE, 0 },
  { PIN_P3_RPWM, PIN_P3_LPWM, PIN_P3_REN, PIN_P3_LEN, 4, 5, P3_REVERSE, 0 },
};
static const uint32_t PWM_MAX = (1UL << PWM_RES_BITS) - 1;

// Debounced input type: declared above the first function so the Arduino IDE's
// automatic function prototypes (inserted there) can see it
struct Debounce { uint8_t pin; bool stable, last; uint32_t changedMs; uint16_t ms; };

static void pwmInit(uint8_t pin, uint8_t ch) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  ledcSetup(ch, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(pin, ch);
#endif
}
static void pwmWrite(uint8_t pin, uint8_t ch, uint32_t v) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, v);
#else
  (void)pin;
  ledcWrite(ch, v);
#endif
}

static void setPump(uint8_t i, uint8_t pct) {
  if (pct > 100) pct = 100;
  Pump &p = g_pump[i];
  if (p.duty == pct) return;
  p.duty = pct;
  uint32_t v = (uint32_t)pct * PWM_MAX / 100;
  uint8_t fwdPin = p.reverse ? p.lpwm : p.rpwm, fwdCh = p.reverse ? p.chL : p.chR;
  uint8_t revPin = p.reverse ? p.rpwm : p.lpwm, revCh = p.reverse ? p.chR : p.chL;
  pwmWrite(revPin, revCh, 0);
  pwmWrite(fwdPin, fwdCh, v);
  digitalWrite(p.ren, pct ? HIGH : LOW);  // EN low when off -> motor coasts
  digitalWrite(p.len, pct ? HIGH : LOW);
}
static void allPumpsOff() { for (uint8_t i = 0; i < 3; i++) setPump(i, 0); }

// ---------------------------------------------------------------- OV-01 solenoid
static bool g_svOpen = false;
static void setValve(bool open) {
  g_svOpen = open;
  digitalWrite(PIN_SV, open ? SV_OPEN_LEVEL : !SV_OPEN_LEVEL);
}

// ---------------------------------------------------------------- FT-01 flow transmitter
volatile uint32_t g_pulses = 0;
volatile uint32_t g_lastPulseUs = 0;
void IRAM_ATTR onFlowPulse() {
  uint32_t now = micros();
  if (now - g_lastPulseUs >= FLOW_PULSE_MIN_US) { g_pulses = g_pulses + 1; g_lastPulseUs = now; }
}

static Preferences g_prefs;
static float g_calA = FLOW_A_DEFAULT, g_calB = FLOW_B_DEFAULT;   // [Q2.2]

static uint32_t g_lastSampleMs = 0, g_lastSamplePulses = 0;
static float g_freqHz = 0, g_flowLpm = 0;
static double g_totalL = 0, g_case1L = 0, g_case2L = 0;          // [Q1.1 Case 1] totaliser

// rolling window for flow-loss detection [Q3.2-C/E]
static const uint8_t WIN_N = (FLOW_LOSS_WINDOW_S * 1000UL + SAMPLE_MS - 1) / SAMPLE_MS;
static uint32_t g_winPulses[WIN_N > 0 ? WIN_N : 1];
static float g_winDt[WIN_N > 0 ? WIN_N : 1];
static uint8_t g_winIdx = 0, g_winFill = 0;

static float flowFromFreq(float f) {                               // [Q2.2] Q = a f + b
  if (f <= 0.0f) return 0.0f;
  float q = g_calA * f + g_calB;
  return q > 0.0f ? q : 0.0f;
}
static float windowFlowLpm() {
  if (g_winFill < WIN_N) return -1.0f;                             // not enough data yet
  uint32_t n = 0; float dt = 0;
  for (uint8_t i = 0; i < WIN_N; i++) { n += g_winPulses[i]; dt += g_winDt[i]; }
  return dt > 0 ? flowFromFreq(n / dt) : 0.0f;
}
static void resetWindow() { g_winIdx = 0; g_winFill = 0; }

// trial counter for [Q2.3] (pulses N over time t)
static bool g_trialOn = false;
static uint32_t g_trialStartPulses = 0, g_trialStartMs = 0, g_trialNo = 0;

// ---------------------------------------------------------------- guided calibration [Q2.2][Q2.3]
enum CalPhase : uint8_t { CAL_WAIT_GO, CAL_RUN, CAL_TAIL, CAL_WAIT_VOL };
static const uint8_t CAL_DUTY_LIST[CAL_TRIALS] = CAL_DUTIES;
static uint8_t g_calPhase = CAL_WAIT_GO, g_calIdx = 0;
static uint32_t g_calStartPulses = 0, g_calStartMs = 0, g_calStopMs = 0;
static uint32_t g_calN[CAL_TRIALS];
static float g_calT[CAL_TRIALS], g_calV[CAL_TRIALS];
static float g_fitA = 0, g_fitB = 0;
static bool g_fitValid = false;

// ---------------------------------------------------------------- debounced inputs
static Debounce g_sw  = { PIN_SW,  HIGH, HIGH, 0, 30 };
static Debounce g_fsw = { PIN_FSW, HIGH, HIGH, 0, FLOAT_DEBOUNCE_MS };
static bool debounce(Debounce &d) {             // returns true on a stable change
  bool r = digitalRead(d.pin);
  uint32_t now = millis();
  if (r != d.last) { d.last = r; d.changedMs = now; }
  if (r != d.stable && now - d.changedMs >= d.ms) { d.stable = r; return true; }
  return false;
}
static bool floatHigh() { return FLOAT_ACTIVE_LOW ? (g_fsw.stable == LOW) : (g_fsw.stable == HIGH); }

// ---------------------------------------------------------------- sump protection
// [Q1.1 Case 1] external charge -> sump level high -> stop inflow pump(s), keep Pump-02 draining
static bool g_sumpHold = false;
static uint32_t g_floatClearMs = 0;
static bool g_floatWasHigh = false;
static void updateSumpHold() {
  bool hi = floatHigh();
  if (hi) {
    if (!g_sumpHold) Serial.println(F("# SUMP HIGH -> inflow pump(s) stopped, Pump-02 draining"));
    g_sumpHold = true;
  } else {
    if (g_floatWasHigh) g_floatClearMs = millis();
    if (g_sumpHold && millis() - g_floatClearMs >= SUMP_CLEAR_HOLD_MS) {
      g_sumpHold = false;
      Serial.println(F("# SUMP level restored -> inflow resumed"));
    }
  }
  g_floatWasHigh = hi;
}

// ---------------------------------------------------------------- guided calibration helpers [Q2.2][Q2.3]
static void enter(State s);

static void calPrompt() {
  Serial.printf("# CAL trial %u of %u (Pump-02 at %u %%): put the return hose in the EMPTY jug,\n",
                g_calIdx + 1, CAL_TRIALS, CAL_DUTY_LIST[g_calIdx]);
  Serial.println(F("#   make sure the sump holds at least 2.5 litres, then type go (or press the ON/OFF switch)"));
  g_calPhase = CAL_WAIT_GO;
}

static void calGo() {                                 // start one timed trial
  g_calStartPulses = g_pulses;
  g_calStartMs = millis();
  setPump(1, CAL_DUTY_LIST[g_calIdx]);
  g_calPhase = CAL_RUN;
  Serial.printf("# CAL trial %u running - keep the hose in the jug until told to stop\n", g_calIdx + 1);
}

// [Q2.2] least-squares fit Q = a f + b over trials with a measured volume
static void calFinish() {
  double sf = 0, sq = 0, sff = 0, sfq = 0;
  uint8_t n = 0;
  float f[CAL_TRIALS], q[CAL_TRIALS];
  for (uint8_t i = 0; i < CAL_TRIALS; i++) {
    if (g_calV[i] <= 0 || g_calT[i] <= 0) continue;
    f[n] = g_calN[i] / g_calT[i];                     // [Q2.3] f = N / t
    q[n] = g_calV[i] / g_calT[i] * 60.0f;             // [Q2.3] Q = V / t  (litres per minute)
    sf += f[n]; sq += q[n]; sff += (double)f[n] * f[n]; sfq += (double)f[n] * q[n];
    n++;
  }
  allPumpsOff();
  if (n < 2 || (n * sff - sf * sf) == 0) {
    Serial.println(F("# CAL needs at least 2 trials with different flows - nothing saved"));
    enter(ST_IDLE);
    return;
  }
  double a = (n * sfq - sf * sq) / (n * sff - sf * sf);
  double b = (sq - a * sf) / n;
  double qMean = sq / n, ssRes = 0, ssTot = 0, eMax = 0, eSum = 0, pMax = 0, pSum = 0;
  Serial.println(F("# ---------------- Q2.3 calibration table ----------------"));
  Serial.println(F("# Trial  Pulses N  Time t(s)  f=N/t(Hz)  Volume(L)  Actual Q(L/min)  Fitted Q(L/min)  Error(%)"));
  for (uint8_t i = 0; i < n; i++) {
    double qh = a * f[i] + b, e = fabs(qh - q[i]), p = q[i] > 0 ? e / q[i] * 100.0 : 0;
    ssRes += (q[i] - qh) * (q[i] - qh); ssTot += (q[i] - qMean) * (q[i] - qMean);
    eSum += e; pSum += p; if (e > eMax) eMax = e; if (p > pMax) pMax = p;
  }
  uint8_t k = 0;
  for (uint8_t i = 0; i < CAL_TRIALS; i++) {
    if (g_calV[i] <= 0 || g_calT[i] <= 0) continue;
    double qh = a * f[k] + b;
    Serial.printf("# %5u  %8lu  %9.2f  %9.2f  %9.3f  %15.3f  %15.3f  %8.2f\n", i + 1, (unsigned long)g_calN[i], g_calT[i],
                  f[k], g_calV[i], q[k], qh, q[k] > 0 ? fabs(qh - q[k]) / q[k] * 100.0 : 0);
    k++;
  }
  double r2 = ssTot > 0 ? 1.0 - ssRes / ssTot : 0;
  Serial.println(F("# ---------------- Q2.2 calibration report --------------"));
  Serial.printf("# R^2 = %.5f\n", r2);
  Serial.printf("# a   = %.6f litres per minute per hertz  (%.0f pulses per litre)\n", a, a > 0 ? 60.0 / a : 0);
  Serial.printf("# b   = %.5f litres per minute\n", b);
  Serial.printf("# Maximum calibration error = %.4f L/min (%.2f %%)\n", eMax, pMax);
  Serial.printf("# Average calibration error = %.4f L/min (%.2f %%)\n", eSum / n, pSum / n);
  g_fitA = a; g_fitB = b; g_fitValid = true;
  if (r2 >= CAL_AUTOSAVE_R2) {
    g_calA = a; g_calB = b;
    g_prefs.putFloat("a", g_calA); g_prefs.putFloat("b", g_calB);
    Serial.printf("# CAL saved a=%.6f b=%.6f\n", g_calA, g_calB);
  } else {
    Serial.printf("# R^2 below %.2f - NOT saved. Type calsave to save anyway, or calib to repeat\n", CAL_AUTOSAVE_R2);
  }
  enter(ST_IDLE);
}

static void calRun() {
  if (g_calPhase == CAL_RUN) {
    float el = (millis() - g_calStartMs) / 1000.0f;
    uint32_t n = g_pulses - g_calStartPulses;
    float estL = el > 0 ? flowFromFreq(n / el) * el / 60.0f : 0;   // estimate with current a, b
    if ((el >= CAL_MIN_S && estL >= CAL_TARGET_L) || el >= CAL_MAX_S) {
      setPump(1, 0);
      g_calStopMs = millis();
      g_calPhase = CAL_TAIL;
      Serial.println(F("# CAL Pump-02 stopped - keep the hose in the jug for 2 more seconds"));
    }
  } else if (g_calPhase == CAL_TAIL && millis() - g_calStopMs >= CAL_TAIL_MS) {
    g_calN[g_calIdx] = g_pulses - g_calStartPulses;                // pulses N
    g_calT[g_calIdx] = (millis() - g_calStartMs) / 1000.0f;        // time t (hose in jug throughout)
    float f = g_calN[g_calIdx] / g_calT[g_calIdx];
    Serial.printf("# CAL trial %u done: N=%lu pulses, t=%.2f s, f=%.2f Hz\n", g_calIdx + 1,
                  (unsigned long)g_calN[g_calIdx], g_calT[g_calIdx], f);
    Serial.println(F("# Lift the hose out, measure the jug, then type vol <litres>  (example: vol 1.482)"));
    g_calPhase = CAL_WAIT_VOL;
  }
  digitalWrite(PIN_LED, g_calPhase == CAL_RUN || g_calPhase == CAL_TAIL);
}

static void calVolume(float v) {                      // operator entered the jug volume
  if (v <= 0) { Serial.println(F("# volume must be above 0")); return; }
  uint8_t i = g_calIdx;
  g_calV[i] = v;
  float f = g_calN[i] / g_calT[i], q = v / g_calT[i] * 60.0f;
  // same columns as the manual t/e trial, plus volume, so logger.py saves it
  Serial.printf("TRIAL,%u,%lu,%.2f,%.3f,%.4f,%.4f\n", i + 1, (unsigned long)g_calN[i], g_calT[i], f, flowFromFreq(f), v);
  Serial.printf("# CAL trial %u: actual flow Q = V/t = %.3f L/min. Pour the jug back into the SUMP.\n", i + 1, q);
  g_calIdx++;
  if (g_calIdx < CAL_TRIALS) calPrompt();
  else calFinish();
}

// ---------------------------------------------------------------- state machine
static void enter(State s) {
  g_state = s;
  g_stateMs = millis();
  Serial.printf("# STATE -> %s\n", STATE_NAME[s]);
  switch (s) {
    case ST_IDLE:
    case ST_MANUAL:
      allPumpsOff(); setValve(false);
      break;
    case ST_CASE1:                                   // [Q1.1 Case 1]
      g_totalL = g_case1L = g_case2L = 0;
      g_sumpHold = false; g_floatWasHigh = false;
      setPump(2, 0);
      setValve(true);                                // 10 L tank -> OV-01 -> Pump-01
      setPump(1, P2_DUTY_CASE1);                     // Pump-02 100 %: sump -> FT-01 -> 10 L tank
      setPump(0, floatHigh() ? 0 : P1_DUTY_CASE1);   // Pump-01 50 %: 10 L tank -> sump
      break;
    case ST_ISOLATE:                                 // [Q1.1 Case 2] isolate 10 L tank, wait for ball valve
      allPumpsOff();
      setValve(false);
      Serial.printf("# OV-01 CLOSED. OPEN FLUSH-LINE BALL VALVE NOW - Pump-03 starts in %d s\n", BALL_VALVE_WAIT_S);
      break;
    case ST_CASE2:                                   // [Q1.1 Case 2] all three pumps
      g_sumpHold = false; g_floatWasHigh = false;
      resetWindow();
      setPump(1, P2_DUTY_CASE2);
      setPump(2, P3_DUTY_CASE2);                     // flush tank -> BV -> Pump-03 -> equal tee
      setPump(0, P1_DUTY_CASE2);
      break;
    case ST_DONE:
    case ST_ESTOP:
      allPumpsOff(); setValve(false);
      Serial.printf("# SUMMARY total_L=%.3f case1_L=%.3f case2_L=%.3f\n", g_totalL, g_case1L, g_case2L);
      break;
    case ST_CAL:                                     // [Q2.2][Q2.3] Pump-02 only, OV-01 closed
      allPumpsOff(); setValve(false);
      for (uint8_t i = 0; i < CAL_TRIALS; i++) { g_calN[i] = 0; g_calT[i] = 0; g_calV[i] = 0; }
      g_calIdx = 0;
      Serial.println(F("# CAL guided calibration: Pump-02 pumps sump -> FT-01 -> jug, 5 trials."));
      Serial.println(F("#   Hold the hose end above the water in the jug. Type calstop to abort."));
      calPrompt();
      break;
  }
}

static uint32_t inStateS() { return (millis() - g_stateMs) / 1000; }

static void runStateMachine() {
  switch (g_state) {
    case ST_CASE1: {
      updateSumpHold();
      setPump(0, g_sumpHold ? 0 : P1_DUTY_CASE1);
      setPump(1, P2_DUTY_CASE1);                     // Pump-02 always runs in Case 1
      bool timeUp = inStateS() >= CASE1_RUN_S;
      bool volUp = CASE1_TARGET_L > 0 && g_case1L >= CASE1_TARGET_L;
      if (timeUp || volUp) {
        Serial.printf("# CASE1 complete (%s)\n", volUp ? "target volume" : "run time");
        enter(ST_ISOLATE);
      }
      break;
    }
    case ST_ISOLATE: {
      static uint32_t lastTick = 0;
      digitalWrite(PIN_LED, (millis() / 150) & 1);   // fast blink = operator action
      uint32_t el = inStateS();
      uint32_t left = el >= BALL_VALVE_WAIT_S ? 0 : BALL_VALVE_WAIT_S - el;
      if (millis() - lastTick >= 1000) { lastTick = millis(); Serial.printf("# ball valve window: %lu s\n", (unsigned long)left); }
      if (left == 0) enter(ST_CASE2);
      break;
    }
    case ST_CASE2: {
      if (SUMP_PROTECT_CASE2) updateSumpHold();
      bool hold = SUMP_PROTECT_CASE2 && g_sumpHold;
      setPump(0, hold ? 0 : P1_DUTY_CASE2);          // both inflow paths stop on sump high
      setPump(2, hold ? 0 : P3_DUTY_CASE2);
      setPump(1, P2_DUTY_CASE2);
      float wq = windowFlowLpm();
      bool flowLost = inStateS() >= CASE2_GRACE_S && wq >= 0 && wq < FLOW_LOSS_LPM;
      bool volUp = CASE2_STOP_L > 0 && g_case2L >= CASE2_STOP_L;
      bool timeout = inStateS() >= CASE2_MAX_S;
      if (flowLost || volUp || timeout) {
        Serial.printf("# CASE2 stop: %s\n", flowLost ? "flush tank exhausted (flow loss)" : volUp ? "flush volume reached" : "TIMEOUT");
        enter(ST_DONE);
      }
      break;
    }
    case ST_CAL:
      calRun();
      return;                                        // calRun drives the LED itself
    default: break;
  }
  if (g_state != ST_ISOLATE)
    digitalWrite(PIN_LED, (g_state == ST_CASE1 || g_state == ST_CASE2) ? HIGH : LOW);
}

// ---------------------------------------------------------------- sampling + log
static void sampleFlow() {
  uint32_t now = millis();
  if (now - g_lastSampleMs < SAMPLE_MS) return;
  uint32_t pulses = g_pulses;
  uint32_t n = pulses - g_lastSamplePulses;
  float dt = (now - g_lastSampleMs) / 1000.0f;
  g_lastSamplePulses = pulses;
  g_lastSampleMs = now;

  g_freqHz = n / dt;                                  // [Q2.3] f = N / t
  g_flowLpm = flowFromFreq(g_freqHz);                 // [Q2.2] Q = a f + b
  double dV = g_flowLpm / 60.0 * dt;                  // litres in this gate
  g_totalL += dV;
  if (g_state == ST_CASE1) g_case1L += dV;
  if (g_state == ST_CASE2) g_case2L += dV;

  g_winPulses[g_winIdx] = n; g_winDt[g_winIdx] = dt;
  g_winIdx = (g_winIdx + 1) % WIN_N;
  if (g_winFill < WIN_N) g_winFill++;

  // [Q1.1 Case 1] flow vs time in mL/s and L/min, totalised litres back to 10 L tank
  Serial.printf("D,%.1f,%s,%.2f,%.3f,%.2f,%.4f,%.4f,%.4f,%u,%u,%u,%d,%d\n",
                now / 1000.0f, STATE_NAME[g_state], g_freqHz, g_flowLpm, g_flowLpm * 1000.0f / 60.0f,
                g_totalL, g_case1L, g_case2L,
                g_pump[0].duty, g_pump[1].duty, g_pump[2].duty, g_svOpen, floatHigh());
}

// ---------------------------------------------------------------- serial commands
static void printHelp() {
  Serial.println(F("# cmds: s=start  x=stop  m=manual  q=quit manual  p <1-3> <0-100>  v <0|1>"));
  Serial.println(F("#       t=trial start  e=trial end  cal <a> <b>  z=zero total  ?=status"));
  Serial.println(F("#       calib=guided calibration  go  vol <litres>  calsave  calstop"));
}
static void handleCommand(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *a1 = strtok(nullptr, " \t");
  char *a2 = strtok(nullptr, " \t");

  // ---- guided calibration [Q2.2][Q2.3]
  if (g_state == ST_CAL && g_calPhase == CAL_WAIT_VOL && (isdigit((unsigned char)cmd[0]) || cmd[0] == '.')) {
    calVolume(atof(cmd));                             // bare number accepted as the jug volume
    return;
  }
  if (!strcmp(cmd, "calib")) {
    if (g_state == ST_CASE1 || g_state == ST_ISOLATE || g_state == ST_CASE2) Serial.println(F("# stop the run first (x)"));
    else enter(ST_CAL);
    return;
  }
  if (!strcmp(cmd, "go")) {
    if (g_state == ST_CAL && g_calPhase == CAL_WAIT_GO) calGo();
    return;
  }
  if (!strcmp(cmd, "vol")) {
    if (g_state == ST_CAL && g_calPhase == CAL_WAIT_VOL && a1) calVolume(atof(a1));
    else Serial.println(F("# vol is used after a calibration trial: vol <litres>"));
    return;
  }
  if (!strcmp(cmd, "calsave")) {
    if (!g_fitValid) { Serial.println(F("# no calibration fit to save - run calib first")); return; }
    g_calA = g_fitA; g_calB = g_fitB;
    g_prefs.putFloat("a", g_calA); g_prefs.putFloat("b", g_calB);
    Serial.printf("# CAL saved a=%.6f b=%.6f\n", g_calA, g_calB);
    return;
  }
  if (!strcmp(cmd, "calstop")) {
    if (g_state == ST_CAL) { Serial.println(F("# CAL aborted - nothing saved")); enter(ST_IDLE); }
    return;
  }

  if (!strcmp(cmd, "s")) { if (g_state != ST_CASE1 && g_state != ST_ISOLATE && g_state != ST_CASE2 && g_state != ST_CAL) enter(ST_CASE1); }
  else if (!strcmp(cmd, "x")) { enter(ST_ESTOP); }
  else if (!strcmp(cmd, "m")) { enter(ST_MANUAL); }
  else if (!strcmp(cmd, "q")) { enter(ST_IDLE); }
  else if (!strcmp(cmd, "p") && a1 && a2) {           // [Q1.2][Q2.1] manual operating points
    if (g_state != ST_MANUAL) { Serial.println(F("# enter manual mode first (m)")); return; }
    int i = atoi(a1), d = atoi(a2);
    if (i < 1 || i > 3) return;
    setPump(i - 1, constrain(d, 0, 100));
    Serial.printf("# P%d = %d %%\n", i, g_pump[i - 1].duty);
  }
  else if (!strcmp(cmd, "v") && a1) {
    if (g_state != ST_MANUAL) { Serial.println(F("# enter manual mode first (m)")); return; }
    setValve(atoi(a1) != 0);
    Serial.printf("# OV-01 %s\n", g_svOpen ? "OPEN" : "CLOSED");
  }
  else if (!strcmp(cmd, "t")) {                       // [Q2.3] start timed collection
    g_trialOn = true; g_trialNo++;
    g_trialStartPulses = g_pulses; g_trialStartMs = millis();
    Serial.printf("# TRIAL %lu started - collect outflow now\n", (unsigned long)g_trialNo);
  }
  else if (!strcmp(cmd, "e")) {                       // [Q2.3] N, t, f = N/t
    if (!g_trialOn) return;
    g_trialOn = false;
    uint32_t n = g_pulses - g_trialStartPulses;
    float t = (millis() - g_trialStartMs) / 1000.0f;
    float f = t > 0 ? n / t : 0;
    Serial.printf("TRIAL,%lu,%lu,%.2f,%.3f,%.4f\n", (unsigned long)g_trialNo, (unsigned long)n, t, f, flowFromFreq(f));
  }
  else if (!strcmp(cmd, "cal") && a1 && a2) {         // [Q2.2] store fitted a, b
    g_calA = atof(a1); g_calB = atof(a2);
    g_prefs.putFloat("a", g_calA); g_prefs.putFloat("b", g_calB);
    Serial.printf("# CAL saved a=%.6f b=%.6f\n", g_calA, g_calB);
  }
  else if (!strcmp(cmd, "z")) { g_totalL = g_case1L = g_case2L = 0; Serial.println(F("# totals zeroed")); }
  else if (!strcmp(cmd, "?")) {
    Serial.printf("# state=%s a=%.6f b=%.6f float_high=%d sv=%d P=%u/%u/%u\n", STATE_NAME[g_state],
                  g_calA, g_calB, floatHigh(), g_svOpen, g_pump[0].duty, g_pump[1].duty, g_pump[2].duty);
    printHelp();
  }
}
static void pollSerial() {
  static char buf[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[len] = 0; handleCommand(buf); len = 0; }
    else if (len < sizeof(buf) - 1) buf[len++] = c;
  }
}

// ---------------------------------------------------------------- ON/OFF switch
// "The case-1 can be turned on using the available on/off Switch" - press: start; press while running: stop
static void pollSwitch() {
  if (debounce(g_sw) && g_sw.stable == LOW) {
    if (g_state == ST_CAL) {                         // calibration: press = go; press while pumping = abort
      if (g_calPhase == CAL_WAIT_GO) calGo();
      else if (g_calPhase == CAL_RUN || g_calPhase == CAL_TAIL) enter(ST_ESTOP);
      return;
    }
    if (g_state == ST_CASE1 || g_state == ST_ISOLATE || g_state == ST_CASE2) enter(ST_ESTOP);
    else if (g_state == ST_MANUAL) enter(ST_IDLE);
    else enter(ST_CASE1);
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SV, OUTPUT);
  setValve(false);
  pinMode(PIN_SW, INPUT);                  // G34/G35 input-only, external 10k pull-ups on PCB
  pinMode(PIN_FSW, INPUT);
  for (uint8_t i = 0; i < 3; i++) {
    Pump &p = g_pump[i];
    pinMode(p.ren, OUTPUT); pinMode(p.len, OUTPUT);
    digitalWrite(p.ren, LOW); digitalWrite(p.len, LOW);
    pwmInit(p.rpwm, p.chR); pwmInit(p.lpwm, p.chL);
    pwmWrite(p.rpwm, p.chR, 0); pwmWrite(p.lpwm, p.chL, 0);
    p.duty = 0;
  }
  pinMode(PIN_FS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FS), onFlowPulse, FALLING);

  g_prefs.begin("lamflow", false);
  g_calA = g_prefs.getFloat("a", FLOW_A_DEFAULT);
  g_calB = g_prefs.getFloat("b", FLOW_B_DEFAULT);

  g_sw.stable = g_sw.last = digitalRead(PIN_SW);
  g_fsw.stable = g_fsw.last = digitalRead(PIN_FSW);
  g_lastSampleMs = millis();

  delay(200);
  Serial.println(F("# LAM Stage 2 flow system ready - press ON/OFF switch to start Case 1"));
  Serial.printf("# CAL a=%.6f b=%.6f\n", g_calA, g_calB);
  Serial.println(F("H,t_s,state,f_hz,q_lpm,q_mls,total_L,case1_L,case2_L,p1,p2,p3,sv,float"));
  printHelp();
  enter(ST_IDLE);
}

void loop() {
  pollSerial();
  pollSwitch();
  debounce(g_fsw);
  sampleFlow();
  runStateMachine();
}
