#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <SoftwareSerial.h>

// ==================== ESP32 LINK (UART via SoftwareSerial) ====================
SoftwareSerial Link(12, 11);   // UNO RX=D12, UNO TX=D11
const uint32_t LINK_BAUD = 9600;

// -------------------- STATE --------------------
enum State { STOP_FWD, RUN_FWD, STOP_BWD, RUN_BWD };

State state = STOP_FWD;
State lastShownState = (State)(-1);
State lastReportedStateToESP32 = (State)(-1);

// -------------------- PAUSE CONTROL --------------------
static bool isPaused = false;
static State pausedState = STOP_FWD;

// -------------------- BLE STATUS FLAG (from ESP32) --------------------
static bool bleConnectedFlag = false;
static int lastShownBle = -1;
static int lastShownPaused = -1;

// -------------------- LCD --------------------
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// -------------------- PINS --------------------
const int dirPin   = 2;
const int stepPin  = 3;
const int dirPin2  = 6;
const int stepPin2 = 7;
const int buttonPin = 8;

#define TRIG_PIN 4
#define ECHO_PIN 5
const int ledPin = 13;

// -------------------- RATIO / TIMING --------------------
const int ratio_num = 5;
long ratio_acc = 0;

// ======= SPEED CONTROL (FASTER BUT STABLE) =======
// Lower = faster. Ramp prevents stalls at start.
static const unsigned long STEP_RATE_SLOW = 1500; // us/step when starting to run
static const unsigned long STEP_RATE_FAST = 1000; // us/step target (try 950, then 900 if stable)
static const unsigned long RAMP_MS        = 600;  // ramp duration

static const unsigned int  stepPulseUs = 2;       // HIGH pulse width (>=2us is safe for most drivers)
unsigned long lastStepTime = 0;

static unsigned long runStartMs = 0;
static bool wasRunning = false;

const unsigned long sensorInterval = 1000; // ms
unsigned long previousSensorMillis = 0;

// -------------------- DISTANCE --------------------
float dist_cm = 0.0;
long  dist_mm = 0;
long  lastShownDistMm = -1;

// For your 4–9cm thresholds, echo is ~250–600us.
// Smaller timeout = less blocking = smoother + faster stepping.
const unsigned long echoTimeoutUs = 3000;

// -------------------- RESET DISPLAY/LOGIC LOCKOUT --------------------
const unsigned long resetLockoutMs = 500;
unsigned long resetLockoutUntil = 0;

// -------------------- BUTTON (polled debounce) --------------------
bool buttonEvent = false;
static bool lastButtonLevel = HIGH;
static unsigned long lastButtonEdgeMs = 0;
static const unsigned long buttonDebounceMs = 200;

// -------------------- COMM BUFFERS --------------------
static char usbBuf[80];
static uint8_t usbLen = 0;

static char linkBuf[80];
static uint8_t linkLen = 0;

// -------------------- HELPERS --------------------
const char* stateName(State s) {
  switch (s) {
    case STOP_FWD: return "STOP_FWD";
    case RUN_FWD:  return "RUN_FWD";
    case STOP_BWD: return "STOP_BWD";
    case RUN_BWD:  return "RUN_BWD";
    default:       return "UNKNOWN";
  }
}

int numDigitsLong(long v) {
  if (v == 0) return 1;
  int d = 0;
  if (v < 0) { d++; v = -v; }
  while (v > 0) { v /= 10; d++; }
  return d;
}

static void trimInPlace(char* s) {
  uint16_t i = 0;
  while (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n') i++;
  if (i > 0) memmove(s, s + i, strlen(s + i) + 1);

  int16_t end = (int16_t)strlen(s) - 1;
  while (end >= 0 && (s[end] == ' ' || s[end] == '\t' || s[end] == '\r' || s[end] == '\n')) {
    s[end] = '\0';
    end--;
  }
}

static void toUpperInPlace(char* s) {
  for (uint16_t i = 0; s[i]; i++) {
    if (s[i] >= 'a' && s[i] <= 'z') s[i] = (char)(s[i] - 32);
  }
}

// Compute ramped step interval while RUNning
static inline unsigned long currentStepIntervalUs() {
  bool running = (state == RUN_FWD || state == RUN_BWD);

  if (running && !wasRunning) {
    runStartMs = millis();
  }
  if (!running) {
    wasRunning = false;
    return STEP_RATE_SLOW;
  }
  wasRunning = true;

  unsigned long elapsed = millis() - runStartMs;
  if (elapsed >= RAMP_MS) return STEP_RATE_FAST;

  unsigned long diff = STEP_RATE_SLOW - STEP_RATE_FAST;
  return STEP_RATE_SLOW - (diff * elapsed) / RAMP_MS;
}

void lcdUpdateIfChanged() {
  // Line 1: State
  if (state != lastShownState) {
    const char* name = stateName(state);

    lcd.setCursor(0, 0);
    lcd.print("State: ");
    lcd.print(name);

    int used = 7 + (int)strlen(name);
    for (int i = used; i < 16; i++) lcd.print(' ');

    lastShownState = state;
  }

  // Line 2: Distance + flags (B/P)
  int bNow = bleConnectedFlag ? 1 : 0;
  int pNow = isPaused ? 1 : 0;

  if (dist_mm != lastShownDistMm || bNow != lastShownBle || pNow != lastShownPaused) {
    lcd.setCursor(0, 1);

    lcd.print("D:");
    lcd.print(dist_mm);
    lcd.print("mm ");

    lcd.print("B");
    lcd.print(bNow);
    lcd.print(" ");

    lcd.print("P");
    lcd.print(pNow);

    int used = 2 + numDigitsLong(dist_mm) + 2 + 1 + 2 + 1 + 2;
    for (int i = used; i < 16; i++) lcd.print(' ');

    lastShownDistMm = dist_mm;
    lastShownBle = bNow;
    lastShownPaused = pNow;
  }
}

void forceResetToBackward() {
  digitalWrite(dirPin, LOW);
  digitalWrite(dirPin2, LOW);
  ratio_acc = 0;
  lastStepTime = micros();
  runStartMs = millis();
  state = RUN_BWD;
}

float getDistanceCm() {
  static float lastGood = 0.0f;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, echoTimeoutUs);
  if (duration == 0) return lastGood;

  float cm = (duration * 0.0343f) / 2.0f;
  lastGood = cm;
  return cm;
}

static inline void pulseStepPin(int pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(stepPulseUs);
  digitalWrite(pin, LOW);
}

void stepOnceAandRatio() {
  pulseStepPin(stepPin);

  ratio_acc += 1;
  while (ratio_acc >= ratio_num) {
    pulseStepPin(stepPin2);
    ratio_acc -= ratio_num;
  }
}

void stepOnceAandRatio_linear() {
  pulseStepPin(stepPin);
}

// ==================== SEND TO ESP32 ====================
static void sendToESP32Line(const char* line) {
  Link.println(line);
  Serial.print("[UNO->ESP32] ");
  Serial.println(line);
}

static void reportStateToESP32IfChanged() {
  if (state == lastReportedStateToESP32) return;
  lastReportedStateToESP32 = state;

  char msg[32];
  snprintf(msg, sizeof(msg), "STATE:%s", stateName(state));
  sendToESP32Line(msg);
}

static void send01ToESP32(char v01) {
  if (v01 == '1') digitalWrite(ledPin, HIGH);
  if (v01 == '0') digitalWrite(ledPin, LOW);

  char msg[2] = { v01, '\0' };
  sendToESP32Line(msg);
}

// ==================== PAUSE/RESUME ====================
static void doPause() {
  if (isPaused) return;
  isPaused = true;
  pausedState = state;
  sendToESP32Line("PAUSED");
  lcdUpdateIfChanged();
}

static void doResume() {
  if (!isPaused) return;
  isPaused = false;
  state = pausedState;
  lastStepTime = micros();
  runStartMs = millis();   // ramp again on resume
  sendToESP32Line("RESUMED");
  reportStateToESP32IfChanged();
  lcdUpdateIfChanged();
}

// ==================== COMMANDS FROM ESP32 ====================
static void handleCommandFromESP32(const char* lineRaw) {
  char line[80];
  strncpy(line, lineRaw, sizeof(line) - 1);
  line[sizeof(line) - 1] = '\0';
  trimInPlace(line);
  if (line[0] == '\0') return;

  // BLE status messages (BLE:1 / BLE:0)
  if (strncmp(line, "BLE:", 4) == 0) {
    char v = line[4];
    bleConnectedFlag = (v == '1');
    lcdUpdateIfChanged();
    return;
  }

  char cmd[80];
  strncpy(cmd, line, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';
  toUpperInPlace(cmd);

  Serial.print("[ESP32->UNO] ");
  Serial.println(line);

  if (strcmp(cmd, "1") == 0) { digitalWrite(ledPin, HIGH); return; }
  if (strcmp(cmd, "0") == 0) { digitalWrite(ledPin, LOW);  return; }

  if (strcmp(cmd, "PAUSE") == 0)  { doPause();  return; }
  if (strcmp(cmd, "RESUME") == 0) { doResume(); return; }

  if (isPaused) return;

  if (strcmp(cmd, "START_FORWARD") == 0) {
    digitalWrite(dirPin, HIGH);
    digitalWrite(dirPin2, HIGH);
    state = RUN_FWD;
    resetLockoutUntil = 0;
    lastStepTime = micros();
    runStartMs = millis();

    lastShownState = (State)(-1);
    lcdUpdateIfChanged();
    reportStateToESP32IfChanged();
    return;
  }

  if (strcmp(cmd, "START_BACK") == 0 || strcmp(cmd, "RESET") == 0) {
    forceResetToBackward();
    resetLockoutUntil = millis() + resetLockoutMs;

    lastShownState = (State)(-1);
    lcdUpdateIfChanged();
    reportStateToESP32IfChanged();
    return;
  }

  if (strcmp(cmd, "US_STOP_FOR") == 0) {
    state = STOP_FWD;
    lastShownState = (State)(-1);
    lcdUpdateIfChanged();
    reportStateToESP32IfChanged();
    return;
  }

  if (strcmp(cmd, "US_STOP_BACK") == 0) {
    state = STOP_BWD;
    lastShownState = (State)(-1);
    lcdUpdateIfChanged();
    reportStateToESP32IfChanged();
    return;
  }

  if (strcmp(cmd, "EMERGENCY_STOP") == 0) {
    if (state == RUN_FWD) state = STOP_FWD;
    else if (state == RUN_BWD) state = STOP_BWD;

    lastShownState = (State)(-1);
    lcdUpdateIfChanged();
    reportStateToESP32IfChanged();
    return;
  }
}

// ==================== IO POLLS ====================
static void pollUsbToEsp32() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbLen > 0) {
        usbBuf[usbLen] = '\0';
        trimInPlace(usbBuf);

        if (usbBuf[0] != '\0') {
          if (strcmp(usbBuf, "1") == 0) send01ToESP32('1');
          else if (strcmp(usbBuf, "0") == 0) send01ToESP32('0');
          else sendToESP32Line(usbBuf);
        }
        usbLen = 0;
      }
    } else if (c >= 32 && c <= 126) {
      if (usbLen < sizeof(usbBuf) - 1) usbBuf[usbLen++] = c;
    }
  }
}

// ✅ FIXED: only treat '1'/'0' as single-byte when buffer empty
static void pollEsp32ToUno() {
  while (Link.available()) {
    char c = (char)Link.read();

    if ((c == '1' || c == '0') && linkLen == 0) {
      char tmp[2] = { c, '\0' };
      handleCommandFromESP32(tmp);
      continue;
    }

    if (c == '\n' || c == '\r') {
      if (linkLen > 0) {
        linkBuf[linkLen] = '\0';
        handleCommandFromESP32(linkBuf);
        linkLen = 0;
      }
    } else if (c >= 32 && c <= 126) {
      if (linkLen < sizeof(linkBuf) - 1) linkBuf[linkLen++] = c;
    }
  }
}

static void pollButtonEvent() {
  bool level = (digitalRead(buttonPin) == HIGH);
  unsigned long nowMs = millis();

  if (lastButtonLevel == HIGH && level == LOW) {
    if (nowMs - lastButtonEdgeMs > buttonDebounceMs) {
      buttonEvent = true;
      lastButtonEdgeMs = nowMs;
    }
  }
  lastButtonLevel = level;
}

// -------------------- SETUP --------------------
void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  digitalWrite(stepPin, LOW);
  digitalWrite(stepPin2, LOW);

  Serial.begin(115200);
  Link.begin(LINK_BAUD);

  digitalWrite(dirPin, HIGH);
  digitalWrite(dirPin2, HIGH);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lastShownState = (State)(-1);
  lastShownDistMm = -1;
  lastShownBle = -1;
  lastShownPaused = -1;

  lcdUpdateIfChanged();

  sendToESP32Line("UNO_READY");
  reportStateToESP32IfChanged();

  lastStepTime = micros();
  runStartMs = millis();
}

// -------------------- LOOP --------------------
void loop() {
  pollUsbToEsp32();
  pollEsp32ToUno();
  pollButtonEvent();

  if (isPaused) {
    if (buttonEvent) {
      buttonEvent = false;

      isPaused = false;
      pausedState = STOP_FWD;
      state = STOP_FWD;

      digitalWrite(dirPin, HIGH);
      digitalWrite(dirPin2, HIGH);
      resetLockoutUntil = 0;

      lastShownState = (State)(-1);
      lcdUpdateIfChanged();

      sendToESP32Line("UNPAUSE_BTN");
      reportStateToESP32IfChanged();
    }
    return;
  }

  if (buttonEvent) {
    buttonEvent = false;
    unsigned long nowMs = millis();

    if (state != STOP_FWD) {
      forceResetToBackward();
      resetLockoutUntil = nowMs + resetLockoutMs;
    } else {
      digitalWrite(dirPin, HIGH);
      digitalWrite(dirPin2, HIGH);
      state = RUN_FWD;
      resetLockoutUntil = 0;
      lastStepTime = micros();
      runStartMs = millis();
    }

    lastShownState = (State)(-1);
    lcdUpdateIfChanged();
    reportStateToESP32IfChanged();
    return;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousSensorMillis >= sensorInterval) {
    previousSensorMillis = currentMillis;

    dist_cm = getDistanceCm();
    dist_mm = (long)(dist_cm * 10.0f + 0.5f);
    lcdUpdateIfChanged();
  }

  State prevState = state;

  switch (state) {
    case STOP_FWD:
      break;
    case RUN_FWD:
      if (dist_cm >= 9.0f) state = STOP_BWD;
      break;
    case STOP_BWD:
      break;
    case RUN_BWD:
      if (millis() >= resetLockoutUntil && dist_cm <= 4.0f) state = STOP_FWD;
      break;
  }

  if (state != prevState) {
    lastShownState = (State)(-1);
    lcdUpdateIfChanged();
    reportStateToESP32IfChanged();

    if (state == RUN_FWD || state == RUN_BWD) {
      runStartMs = millis();
      lastStepTime = micros();
    }
  }

  // Stepping (faster + ramped)
  if (state == RUN_FWD || state == RUN_BWD) {
    unsigned long interval = currentStepIntervalUs();
    unsigned long now = micros();

    // Catch-up a little if something blocks briefly (prevents slowdowns)
    // Limit to avoid huge bursts
    uint8_t maxSteps = 4;
    while ((long)(now - lastStepTime) >= (long)interval && maxSteps--) {
      lastStepTime += interval;

      if (state == RUN_FWD) stepOnceAandRatio();
      else                 stepOnceAandRatio_linear();

      now = micros();
      interval = currentStepIntervalUs();
    }
  }
}
