#pragma once
// =====================================================================
// LAM Research Challenge 3.0 - Stage 2 : configuration
// Pin map read from "LAM Stage 2 PCB" schematic, REV 1.0 (U7 NodeMCU-32S)
// =====================================================================

// ---- Pump drivers (BTS7960) - HW list p.11 "Motor Driver Interfaces U2/U3/U4"
// Pump-01 -> U2 header
#define PIN_P1_RPWM   33   // U7 pin 8  (G33)  R_PWM1
#define PIN_P1_LPWM   32   // U7 pin 7  (G32)  L_PWM1
#define PIN_P1_REN    26   // U7 pin 10 (G26)  R_EN1
#define PIN_P1_LEN    13   // U7 pin 15 (G13)  L_EN1
// Pump-02 -> U3 header
#define PIN_P2_RPWM   19   // U7 pin 31 (G19)  R_PWM2
#define PIN_P2_LPWM   18   // U7 pin 30 (G18)  L_PWM2
#define PIN_P2_REN    17   // U7 pin 28 (G17)  R_EN2
#define PIN_P2_LEN    16   // U7 pin 27 (G16)  L_EN2
// Pump-03 -> U4 header
#define PIN_P3_RPWM   25   // U7 pin 9  (G25)  R_PWM3
#define PIN_P3_LPWM   23   // U7 pin 37 (G23)  L_PWM3
#define PIN_P3_REN    22   // U7 pin 36 (G22)  R_EN3
#define PIN_P3_LEN    21   // U7 pin 33 (G21)  L_EN3

// ---- I/O - HW list p.11 connectors
#define PIN_SV        27   // U7 pin 11 (G27)  SOLENOID_VALVE pin1 -> D4184 PWM terminal (OV-01 coil on + / LOAD)
#define PIN_FS        14   // U7 pin 12 (G14)  FLOW_SENSOR pin1 (FT-01, YF-S401)
#define PIN_SW        34   // U7 pin 5  (G34)  ON/OFF_SWITCH, 10k pull-up R2 -> pressed = LOW
#define PIN_FSW       35   // U7 pin 6  (G35)  FLOAT_SWITCH, 10k pull-up R10 -> closed = LOW
#define PIN_LED        2   // on-board LED of NodeMCU-32S (G2 is not routed on the PCB)

// ---- Pump PWM (BTS7960 accepts up to 25 kHz)
#define PWM_FREQ_HZ   20000
#define PWM_RES_BITS  10
// Set to 1 if a pump turns the wrong way (swaps R_PWM / L_PWM)
#define P1_REVERSE    0
#define P2_REVERSE    1
#define P3_REVERSE    0

// ---- [Q1.1 Case 1] duty cycles fixed by the problem statement
#define P1_DUTY_CASE1 50
#define P2_DUTY_CASE1 100
// ---- [Q1.1 Case 2] "all three pumps operate simultaneously"
#define P1_DUTY_CASE2 100
#define P2_DUTY_CASE2 100
#define P3_DUTY_CASE2 100

// ---- OV-01 solenoid: assumed normally-closed, energised = open
#define SV_OPEN_LEVEL HIGH

// ---- [Q2.2] FT-01 calibration  Q[L/min] = a*f[Hz] + b
// Defaults = YF-S401 datasheet (F = 98*Q). Replace with your fit via serial: cal <a> <b>
#define FLOW_A_DEFAULT    (1.0f / 98.0f)
#define FLOW_B_DEFAULT    0.0f
#define FLOW_PULSE_MIN_US 300      // glitch filter; 6 L/min ~ 588 Hz -> 1.7 ms period
#define SAMPLE_MS         1000     // [Q2.3] f = N/t gate time

// ---- [Q2.2][Q2.3] guided calibration (serial command: calib)
// Pump-02 alone pumps sump -> FT-01 -> jug; the ESP32 times each trial, counts pulses,
// asks for the jug volume, then fits Q = a*f + b and saves a, b.
#define CAL_TRIALS        5
#define CAL_DUTIES        { 35, 50, 65, 80, 100 }  // Pump-02 duty (%) for trials 1..5
#define CAL_TARGET_L      1.5f     // stop a trial once about this volume has passed (estimate)
#define CAL_MIN_S         15       // shortest trial
#define CAL_MAX_S         120      // longest trial (protects against a stalled pump)
#define CAL_TAIL_MS       2000     // keep counting while the rotor coasts after Pump-02 stops
#define CAL_AUTOSAVE_R2   0.99f    // save a, b automatically only if R^2 is at least this

// ---- [Q1.1 Case 1] sump level protection (float switch in 4.2 L sump)
#define FLOAT_ACTIVE_LOW    1      // 1: contact closes (LOW) when sump level is HIGH
#define FLOAT_DEBOUNCE_MS   300
#define SUMP_CLEAR_HOLD_MS  3000   // float must stay clear this long before Pump-01 restarts

// ---- Case 1 -> Case 2 automatic hand-over ("should run in a single go")
#define CASE1_RUN_S     60        // Case 1 duration
#define CASE1_TARGET_L  0.0f       // >0: also end Case 1 once this many litres returned

// ---- [Q1.1 Case 2] flush sequence
#define BALL_VALVE_WAIT_S   20     // window to open the flush-line ball valve before Pump-03
#define SUMP_PROTECT_CASE2  1      // keep float protection active (stops P1+P3) during flush
#define CASE2_GRACE_S       10     // ignore low flow while lines prime
#define FLOW_LOSS_LPM       0.30f  // [Q3.2-C/E] flush tank empty -> FT-01 flow collapses
#define FLOW_LOSS_WINDOW_S  3      // averaged window (air slugs give intermittent pulses)
#define CASE2_STOP_L        0.0f   // >0: also stop once this many litres measured in Case 2
#define CASE2_MAX_S         1600    // safety timeout

#define SERIAL_BAUD   115200
