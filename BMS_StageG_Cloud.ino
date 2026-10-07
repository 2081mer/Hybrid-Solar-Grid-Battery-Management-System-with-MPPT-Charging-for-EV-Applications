/*
  Hybrid Solar-Grid BMS - Stage G + Arduino IoT Cloud
  Cloud variables (declared in thingProperties.h):
    float  vpack        READ   pack voltage (V)
    float  iout         READ   output / battery current (A)
    float  temp         READ   temperature (C)
    float  soc          READ   state of charge (%)
    bool   autoCharge   READ/WRITE  -> onAutoChargeChange()
*/

#include "thingProperties.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include "driver/mcpwm.h"

// ---------- Pin map ----------
const int PIN_HIN_A = 18, PIN_LIN_A = 19;
const int PIN_HIN_B = 21, PIN_LIN_B = 22;
const int PIN_SD    = 23;
const int PIN_LCD_SDA = 16, PIN_LCD_SCL = 17;
const int PIN_BUZZ  = 4;
const int PIN_RLY_SOLAR = 25;
const int PIN_RLY_GRID  = 26;
const int PIN_RLY_LOAD  = 27;
const int PIN_RLY_EMERG = 14;

const int PIN_V_PANEL = 34;
const int PIN_V_PACK  = 35;
const int PIN_I_PANEL = 32;
const int PIN_I_BATT  = 33;

// ---------- Settings ----------
const uint32_t PWM_FREQ_HZ = 50000;
const float DUTY_MAX   = 0.95f;
const float DUTY_MIN_B = 0.05f;
const float M_MAX = 3.0f;
const float M_MIN = 0.05f;

float I_CC   = 1.50f;    // bulk charge current (A)
float V_CV   = 12.60f;   // absorb voltage (V)
float I_TERM = 0.15f;    // termination current (A)
const float V_PACK_ABS_MAX = 12.80f;

const float V_IN_MIN   = 3.5f;
const float V_IN_TRIP  = 28.0f;
const float I_IN_TRIP  = 4.0f;
const float I_OUT_TRIP = 4.0f;

const float ACS_DIV  = 1.5f;     // divider in front of ACS sensor output
const float ACS_SENS = 0.100f;   // V per A

float poStep = 0.015f;
const uint32_t CTRL_PERIOD_MS = 180;

// Source selection
enum ActiveSource { ACT_NONE, ACT_SOLAR, ACT_GRID };
ActiveSource activeSource = ACT_NONE;

const float    SOLAR_VIN_MIN = 8.0f;
const uint32_t SOLAR_HOLD_MS = 15000;   // solar must be good this long before switching to it
const uint32_t GRID_HOLD_MS  = 8000;    // solar must be bad this long before falling back to grid

// ---------- Objects ----------
LiquidCrystal_I2C lcd(0x27, 16, 2);
Preferences prefs;

// ---------- State ----------
float kPanel = 8.75f, kPack = 5.0f;
float zeroPanelV = 2.5f, zeroBattV = 2.5f;

float vIn = 0, iIn = 0, vOut = 0, iOut = 0, pIn = 0, pInPrev = 0;
float ratioM = 0.25f, dutyA = 0, dutyB = 0;
int   poDir = +1;

bool enabled = false, autoRun = false, tripped = false;
String tripReason = "";
enum ChargeState { ST_IDLE, ST_BULK, ST_ABSORB, ST_FULL, ST_FAULT };
ChargeState chgState = ST_IDLE;

uint32_t dtUnits = 5;
int buzzMode = 0;

uint32_t solarGoodSince = 0, solarBadSince = 0;
uint32_t tCtrl = 0, tLcd = 0, tCloud = 0;

// ========== Helpers ==========
float readVolts(int adcPin, int n = 28) {
  uint32_t sum = 0;
  for (int i = 0; i < n; i++) sum += analogReadMilliVolts(adcPin);
  return (sum / (float)n) / 1000.0f;
}
float readAcsVolts(int adcPin, int n = 40) { return readVolts(adcPin, n) * ACS_DIV; }
float readAmps(int adcPin, float z, int n = 40) { return (readAcsVolts(adcPin, n) - z) / ACS_SENS; }

void zeroCurrentSensors() {
  zeroPanelV = readAcsVolts(PIN_I_PANEL);
  zeroBattV  = readAcsVolts(PIN_I_BATT);
}

// ---------- PWM ----------
void applyDeadTime() {
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, dtUnits, dtUnits);
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, dtUnits, dtUnits);
}

void pwmInit() {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_HIN_A);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_LIN_A);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_HIN_B);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PIN_LIN_B);
  mcpwm_config_t cfg = {};
  cfg.frequency = PWM_FREQ_HZ;
  cfg.cmpr_a = 0; cfg.cmpr_b = 0;
  cfg.counter_mode = MCPWM_UP_COUNTER;
  cfg.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &cfg);
  applyDeadTime();
}

void setLegDuty(mcpwm_timer_t t, float frac) {
  float pct = constrain(frac, 0.0f, 1.0f) * 100.0f;
  mcpwm_set_duty(MCPWM_UNIT_0, t, MCPWM_OPR_A, pct);
  mcpwm_set_duty_type(MCPWM_UNIT_0, t, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

void applyRatio(float M) {
  M = constrain(M, M_MIN, M_MAX);
  if (M <= 1.0f) { dutyB = DUTY_MAX; dutyA = DUTY_MAX * M; }
  else           { dutyA = DUTY_MAX; dutyB = DUTY_MAX / M; }
  dutyA = constrain(dutyA, 0.0f, DUTY_MAX);
  dutyB = constrain(dutyB, DUTY_MIN_B, DUTY_MAX);
  setLegDuty(MCPWM_TIMER_0, dutyA);
  setLegDuty(MCPWM_TIMER_1, dutyB);
  ratioM = M;
}

// ---------- Relays ----------
void setSolar(bool on) {
  if (on && digitalRead(PIN_RLY_GRID)) { digitalWrite(PIN_RLY_GRID, LOW); delay(30); }
  digitalWrite(PIN_RLY_SOLAR, on ? HIGH : LOW);
}
void setGrid(bool on) {
  if (on && digitalRead(PIN_RLY_SOLAR)) { digitalWrite(PIN_RLY_SOLAR, LOW); delay(30); }
  digitalWrite(PIN_RLY_GRID, on ? HIGH : LOW);
}
void setEmerg(bool on) { digitalWrite(PIN_RLY_EMERG, on ? HIGH : LOW); }
void allSourcesOff()   { setSolar(false); setGrid(false); setEmerg(false); activeSource = ACT_NONE; }

// ---------- Driver / fault handling ----------
void driversDisable() {
  digitalWrite(PIN_SD, HIGH);
  enabled = false;
  autoRun = false;
  applyRatio(0);
  if (chgState != ST_FAULT && chgState != ST_FULL) chgState = ST_IDLE;
}

void driversEnable() {
  digitalWrite(PIN_SD, LOW);
  enabled = true;
}

void trip(const char* reason) {
  driversDisable();
  allSourcesOff();
  tripped = true;
  chgState = ST_FAULT;
  tripReason = reason;
  buzzMode = 1;
  Serial.print("TRIP: "); Serial.println(reason);
}

// ---------- Source manager ----------
// Switches relays only with the converter stopped (never under load).
void switchSource(ActiveSource target) {
  if (target == activeSource) return;
  digitalWrite(PIN_SD, HIGH);   // stop switching
  applyRatio(0);
  enabled = false;
  delay(20);
  if (target == ACT_SOLAR)      setSolar(true);
  else if (target == ACT_GRID)  setGrid(true);
  else { setSolar(false); setGrid(false); }
  activeSource = target;
  poDir = +1;
  pInPrev = 0;
}

void sourceManager() {
  uint32_t now = millis();
  bool solarOk = (vIn >= SOLAR_VIN_MIN);

  if (solarOk) { solarBadSince = 0; if (!solarGoodSince) solarGoodSince = now; }
  else         { solarGoodSince = 0; if (!solarBadSince)  solarBadSince  = now; }

  if (activeSource != ACT_SOLAR) {
    if (solarOk && now - solarGoodSince >= SOLAR_HOLD_MS) {
      switchSource(ACT_SOLAR);
    } else if (!solarOk && activeSource == ACT_NONE && now - solarBadSince >= GRID_HOLD_MS) {
      switchSource(ACT_GRID);
    }
  } else {
    if (!solarOk && now - solarBadSince >= GRID_HOLD_MS) {
      switchSource(ACT_GRID);
    }
  }
}

// ---------- Charge control ----------
void controlTick() {
  if (!autoRun) return;

  // Protections
  if (vOut > V_PACK_ABS_MAX) { trip("Pack overvoltage"); return; }
  if (vIn  > V_IN_TRIP)      { trip("Input overvoltage"); return; }
  if (fabsf(iIn)  > I_IN_TRIP)  { trip("Input overcurrent"); return; }
  if (fabsf(iOut) > I_OUT_TRIP) { trip("Output overcurrent"); return; }

  if (activeSource == ACT_NONE) { driversDisable(); return; }
  if (vIn < V_IN_MIN)           { driversDisable(); return; }

  if (!enabled) driversEnable();

  // State machine
  if (chgState == ST_BULK && vOut >= V_CV) chgState = ST_ABSORB;
  if (chgState == ST_ABSORB && iOut < I_TERM) {
    chgState = ST_FULL;
    driversDisable();     // keeps ST_FULL
    return;
  }

  // Regulation limit
  bool overLimit = false;
  if (chgState == ST_BULK)   overLimit = (iOut > I_CC) || (vOut > V_CV);
  if (chgState == ST_ABSORB) overLimit = (vOut > V_CV);

  float M = ratioM;
  if (overLimit) {
    M -= poStep;
  } else if (activeSource == ACT_SOLAR) {
    // Perturb & Observe MPPT
    if (pIn < pInPrev) poDir = -poDir;
    M += poDir * poStep;
  } else {
    // Grid: just push toward the limit
    M += poStep;
  }
  pInPrev = pIn;
  applyRatio(M);
}

// ========== Cloud update ==========
// 3S Li-ion open-circuit voltage -> SoC (%) lookup (per cell)
float estimateSoc(float packV) {
  static const float cv[] = {3.00f, 3.30f, 3.50f, 3.60f, 3.70f, 3.80f, 3.90f, 4.00f, 4.10f, 4.20f};
  static const float cs[] = {0,     5,     10,    20,    35,    50,    65,    80,    92,    100};
  const int N = 10;
  float cell = packV / 3.0f;
  if (cell <= cv[0])   return 0.0f;
  if (cell >= cv[N-1]) return 100.0f;
  for (int i = 1; i < N; i++) {
    if (cell <= cv[i]) {
      float f = (cell - cv[i-1]) / (cv[i] - cv[i-1]);
      return cs[i-1] + f * (cs[i] - cs[i-1]);
    }
  }
  return 0.0f;
}

const char* chargeStateName() {
  switch (chgState) {
    case ST_BULK:   return "BULK";
    case ST_ABSORB: return "ABS";
    case ST_FULL:   return "FULL";
    case ST_FAULT:  return "FAULT";
    default:        return "IDLE";
  }
}

void updateCloudVariables() {
  vpack = vOut;                 // pack voltage
  iout  = iOut;                 // battery current (cloud "iout" vs local "iOut")
  soc   = estimateSoc(vOut);    // estimated from voltage
  temp  = temperatureRead();    // ESP32 chip temp - replace with a battery sensor if you have one
}

// ========== LCD ==========
void updateLcd() {
  char l1[17], l2[17];
  snprintf(l1, sizeof(l1), "V%5.2f I%5.2fA   ", vOut, iOut);
  snprintf(l2, sizeof(l2), "%-5s %-5s %3.0f%%  ",
           (activeSource == ACT_SOLAR) ? "SOLAR" : (activeSource == ACT_GRID) ? "GRID" : "NONE",
           chargeStateName(), soc);
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
}

// ========== Setup ==========
void setup() {
  digitalWrite(PIN_SD, HIGH);
  pinMode(PIN_SD, OUTPUT);
  digitalWrite(PIN_SD, HIGH);

  const int outs[] = {PIN_RLY_SOLAR, PIN_RLY_GRID, PIN_RLY_LOAD, PIN_RLY_EMERG, PIN_BUZZ};
  for (int p : outs) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  Serial.begin(115200);
  delay(500);

  // Arduino IoT Cloud
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  prefs.begin("bms", false);
  kPanel = prefs.getFloat("kPanel", kPanel);
  kPack  = prefs.getFloat("kPack",  kPack);

  Wire.begin(PIN_LCD_SDA, PIN_LCD_SCL);
  lcd.init();
  lcd.backlight();
  lcd.print("Stage G + Cloud");

  pwmInit();
  applyRatio(0);
  zeroCurrentSensors();

  Serial.println("BMS Stage G + IoT Cloud ready");
}

// ========== Loop ==========
void loop() {
  ArduinoCloud.update();   // keep cloud connection alive

  // Sensors
  vIn  = readVolts(PIN_V_PANEL) * kPanel;
  iIn  = readAmps(PIN_I_PANEL, zeroPanelV);
  vOut = readVolts(PIN_V_PACK) * kPack;
  iOut = readAmps(PIN_I_BATT, zeroBattV);
  pIn  = vIn * iIn;

  // Source selection + charge control
  if (!tripped) sourceManager();

  if (millis() - tCtrl >= CTRL_PERIOD_MS) {
    tCtrl = millis();
    controlTick();
  }

  // autoCharge switch from the dashboard
  if (autoCharge && !autoRun && !tripped && activeSource != ACT_NONE &&
      chgState != ST_FULL) {
    autoRun = true;
    chgState = ST_BULK;
  }
  if (!autoCharge && (autoRun || tripped || chgState == ST_FULL)) {
    // switching OFF also clears a fault / full state so you can restart
    driversDisable();
    tripped = false;
    chgState = ST_IDLE;
    buzzMode = 0;
  }
  // restart after FULL if the pack has dropped
  if (chgState == ST_FULL && autoCharge && vOut < (V_CV - 0.5f)) {
    chgState = ST_IDLE;
  }

  // Buzzer on fault
  digitalWrite(PIN_BUZZ, (buzzMode == 1 && (millis() / 300) % 2) ? HIGH : LOW);

  // Cloud + LCD
  if (millis() - tCloud > 2000) { tCloud = millis(); updateCloudVariables(); }
  if (millis() - tLcd   > 500)  { tLcd   = millis(); updateLcd(); }

  delay(10);
}

// Called automatically when autoCharge changes from the dashboard
void onAutoChargeChange() {
  Serial.print("autoCharge changed to: ");
  Serial.println(autoCharge);
}
