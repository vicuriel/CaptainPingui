#include <Arduino.h>
#include <math.h>

// =======================================================
// ==================== CONFIG GENERAL ====================
// =======================================================
const uint32_t SERIAL_BAUD = 115200;

// Configuración de inversión de motores
const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = true;

// =======================================================
// ======================= PINES ==========================
// =======================================================

// -------- Motor Izquierdo --------
const int LEFT_DIR_PIN = 25;
const int LEFT_PWM_PIN = 26;
const int LEFT_ENC_A   = 34;
const int LEFT_ENC_B   = 35;

// -------- Motor Derecho --------
const int RIGHT_DIR_PIN = 18;
const int RIGHT_PWM_PIN = 19;
const int RIGHT_ENC_A   = 5;
const int RIGHT_ENC_B   = 17;

// -------- Sensor Ultrasónico --------
const int TRIG_PIN = 4;
const int ECHO_PIN = 16;

// -------- Bumper --------
// Cambiar este pin por el que uses realmente
const int BUMPER_PIN = 27;

// PWM ESP32
const int LEFT_PWM_CH  = 0;
const int RIGHT_PWM_CH = 1;
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

// Encoders
const float PPR_MOTOR  = 11.0f;
const float GEAR_RATIO = 110.0f;
const float CPR = PPR_MOTOR * GEAR_RATIO * 4.0f;

int LEFT_ENC_SIGN  = INVERT_LEFT_MOTOR  ? -1 : 1;
int RIGHT_ENC_SIGN = INVERT_RIGHT_MOTOR ? -1 : 1;

// =======================================================
// ===================== CONTROL PID ======================
// =======================================================
struct PIDState {
  float e_prev    = 0.0f;
  float integral  = 0.0f;
  float d_filt    = 0.0f;
  uint32_t t_prev_us = 0;
};

PIDState leftPID, rightPID;

// Ganancias PID
float Kp = 0.8f;
float Ki = 0.4f;
float Kd = 0.02f;

const float INTEGRAL_LIMIT = 60.0f;
const float ALPHA_D = 0.25f;
const float ALPHA_PWM = 0.15f;
const float ALPHA_RPM = 0.4f;
const int PWM_STATIC = 28;
const float PWM_STATIC_THRESHOLD = 5.0f;
const int PWM_MAX = 255;
const float RPM_DEADBAND = 2.0f;

// Timeout real de comandos
const uint32_t CMD_TIMEOUT_MS = 99999;

// =======================================================
// ==================== ULTRASÓNICO =======================
// =======================================================
const float US_STOP_DISTANCE_CM = 8.0f;
const uint32_t US_PERIOD_MS = 100;
bool ultrasonicBlocked = false;

// =======================================================
// ====================== BUMPER ==========================
// =======================================================
const uint32_t BUMPER_PERIOD_MS = 20;
bool bumperBlocked = false;

// =======================================================
// ====================== ESTADO ==========================
// =======================================================
volatile long leftEncCount = 0, rightEncCount = 0;
long leftEncPrev = 0, rightEncPrev = 0;

float leftRPM = 0, rightRPM = 0;
float leftRPMFiltered = 0, rightRPMFiltered = 0;
float leftRefRPM = 0, rightRefRPM = 0;
float leftPWMFiltered = 0, rightPWMFiltered = 0;

bool motorsEnabled = true;
uint32_t lastCmdMs = 0, lastFeedbackMs = 0, lastUSMs = 0, lastBumperMs = 0;
const uint32_t FEEDBACK_PERIOD_MS = 100;

String serialBuffer = "";

// =======================================================
// ================= FUNCIONES TÉCNICAS ===================
// =======================================================

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999.0f;

  return (duration * 0.0343f) / 2.0f;
}

void IRAM_ATTR isrLeftA() {
  bool a = digitalRead(LEFT_ENC_A), b = digitalRead(LEFT_ENC_B);
  leftEncCount += (a == b) ? LEFT_ENC_SIGN : -LEFT_ENC_SIGN;
}

void IRAM_ATTR isrLeftB() {
  bool a = digitalRead(LEFT_ENC_A), b = digitalRead(LEFT_ENC_B);
  leftEncCount += (a != b) ? LEFT_ENC_SIGN : -LEFT_ENC_SIGN;
}

void IRAM_ATTR isrRightA() {
  bool a = digitalRead(RIGHT_ENC_A), b = digitalRead(RIGHT_ENC_B);
  rightEncCount += (a == b) ? RIGHT_ENC_SIGN : -RIGHT_ENC_SIGN;
}

void IRAM_ATTR isrRightB() {
  bool a = digitalRead(RIGHT_ENC_A), b = digitalRead(RIGHT_ENC_B);
  rightEncCount += (a != b) ? RIGHT_ENC_SIGN : -RIGHT_ENC_SIGN;
}

bool updateRPM() {
  static uint32_t prevRPMus = micros();
  uint32_t nowUs = micros();
  float dt = (nowUs - prevRPMus) * 1e-6f;
  if (dt < 0.01f) return false;
  prevRPMus = nowUs;

  long leftNow, rightNow;
  noInterrupts();
  leftNow = leftEncCount;
  rightNow = rightEncCount;
  interrupts();

  leftRPM  = ((float)(leftNow - leftEncPrev) / CPR) * (60.0f / dt);
  rightRPM = ((float)(rightNow - rightEncPrev) / CPR) * (60.0f / dt);
  leftEncPrev = leftNow;
  rightEncPrev = rightNow;

  leftRPMFiltered  = ALPHA_RPM * leftRPM  + (1.0f - ALPHA_RPM) * leftRPMFiltered;
  rightRPMFiltered = ALPHA_RPM * rightRPM + (1.0f - ALPHA_RPM) * rightRPMFiltered;
  return true;
}

float runPID(PIDState &pid, float refRPM, float measRPM) {
  uint32_t nowUs = micros();
  float Ts = (nowUs - pid.t_prev_us) * 1e-6f;
  pid.t_prev_us = nowUs;
  if (Ts <= 0.0f) Ts = 0.001f;

  float e = refRPM - measRPM;

  if (fabs(refRPM) < RPM_DEADBAND && fabs(measRPM) < RPM_DEADBAND) {
    pid.integral = 0;
    pid.e_prev = e;
    return 0;
  }

  pid.integral += Ki * Ts * e;
  pid.integral = constrain(pid.integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float d_raw = (e - pid.e_prev) / Ts;
  pid.d_filt = ALPHA_D * d_raw + (1.0f - ALPHA_D) * pid.d_filt;
  pid.e_prev = e;

  return (Kp * e) + pid.integral + (Kd * pid.d_filt);
}

void setMotorPWM(int dirPin, int pwmCh, float controlSignal, float &pwmFiltered, int pwmMax, bool inverted) {
  if (fabs(controlSignal) < 1e-3f) {
    pwmFiltered = 0;
    ledcWrite(pwmCh, 0);
    return;
  }

  float mag = fabs(controlSignal);
  if (mag > PWM_STATIC_THRESHOLD) mag += PWM_STATIC;
  mag = constrain(mag, 0, pwmMax);
  pwmFiltered = ALPHA_PWM * mag + (1.0f - ALPHA_PWM) * pwmFiltered;

  bool dir = (controlSignal >= 0) ? !inverted : inverted;
  digitalWrite(dirPin, dir ? HIGH : LOW);
  ledcWrite(pwmCh, (int)pwmFiltered);
}

void resetPID() {
  leftPID.integral = 0.0f;
  rightPID.integral = 0.0f;
  leftPID.e_prev = 0.0f;
  rightPID.e_prev = 0.0f;
  leftPID.d_filt = 0.0f;
  rightPID.d_filt = 0.0f;
}

void stopMotorsHard() {
  leftRefRPM = 0;
  rightRefRPM = 0;
  leftPWMFiltered = 0;
  rightPWMFiltered = 0;

  ledcWrite(LEFT_PWM_CH, 0);
  ledcWrite(RIGHT_PWM_CH, 0);

  resetPID();
}

void pauseMotorsBySafety() {
  // Frenado temporal por ultrasónico o bumper, sin borrar referencias
  leftPWMFiltered = 0;
  rightPWMFiltered = 0;

  ledcWrite(LEFT_PWM_CH, 0);
  ledcWrite(RIGHT_PWM_CH, 0);

  resetPID();
}

void updateUltrasonicBlock() {
  if (millis() - lastUSMs < US_PERIOD_MS) return;
  lastUSMs = millis();

  float distanceCm = getDistance();

  Serial.print("US ");
  Serial.println(distanceCm, 1);

  bool newBlocked = distanceCm < US_STOP_DISTANCE_CM;

  if (newBlocked != ultrasonicBlocked) {
    ultrasonicBlocked = newBlocked;

    if (ultrasonicBlocked) {
      Serial.print("OBS_BLOCK ");
      Serial.println(distanceCm, 1);
    } else {
      Serial.print("OBS_CLEAR ");
      Serial.println(distanceCm, 1);
    }
  }
}

void updateBumperBlock() {
  if (millis() - lastBumperMs < BUMPER_PERIOD_MS) return;
  lastBumperMs = millis();

  // Asumido: INPUT_PULLUP -> LOW cuando se aprieta
  bool newBumperBlocked = (digitalRead(BUMPER_PIN) == HIGH);

  if (newBumperBlocked != bumperBlocked) {
    bumperBlocked = newBumperBlocked;

    Serial.print("BUMP ");
    Serial.println(bumperBlocked ? 1 : 0);

    if (bumperBlocked) {
      Serial.println("BUMP_BLOCK");
    } else {
      Serial.println("BUMP_CLEAR");
    }
  }
}

void processCommand(String line) {
  line.trim();

  if (line.startsWith("CMD_RPM ")) {
    int p1 = line.indexOf(' ');
    String rest = line.substring(p1 + 1);
    int p2 = rest.indexOf(' ');
    if (p2 > 0) {
      leftRefRPM = rest.substring(0, p2).toFloat();
      rightRefRPM = rest.substring(p2 + 1).toFloat();
      lastCmdMs = millis();
      Serial.println("ACK");
    }
  } else if (line == "STOP") {
    stopMotorsHard();
    Serial.println("ACK");
  } else if (line == "ENABLE") {
    motorsEnabled = true;
    Serial.println("ACK");
  } else if (line == "DISABLE") {
    motorsEnabled = false;
    stopMotorsHard();
    Serial.println("ACK");
  }
}

// =======================================================
// ======================== MAIN ==========================
// =======================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Bumper con pull-up interno
  pinMode(BUMPER_PIN, INPUT_PULLUP);

  ledcSetup(LEFT_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_PWM_PIN, LEFT_PWM_CH);

  ledcSetup(RIGHT_PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_PWM_CH);

  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);

  attachInterrupt(LEFT_ENC_A, isrLeftA, CHANGE);
  attachInterrupt(LEFT_ENC_B, isrLeftB, CHANGE);
  attachInterrupt(RIGHT_ENC_A, isrRightA, CHANGE);
  attachInterrupt(RIGHT_ENC_B, isrRightB, CHANGE);

  leftPID.t_prev_us = micros();
  rightPID.t_prev_us = micros();

  Serial.println("ACK ESP32_READY");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(serialBuffer);
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }

  updateUltrasonicBlock();
  updateBumperBlock();

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopMotorsHard();
  }

  if (updateRPM()) {
    if (!motorsEnabled) {
      stopMotorsHard();
    } else if (ultrasonicBlocked || bumperBlocked) {
      pauseMotorsBySafety();
    } else {
      float leftU = runPID(leftPID, leftRefRPM, leftRPMFiltered);
      float rightU = runPID(rightPID, rightRefRPM, rightRPMFiltered);

      setMotorPWM(LEFT_DIR_PIN, LEFT_PWM_CH, leftU, leftPWMFiltered, PWM_MAX, INVERT_LEFT_MOTOR);
      setMotorPWM(RIGHT_DIR_PIN, RIGHT_PWM_CH, rightU, rightPWMFiltered, PWM_MAX, INVERT_RIGHT_MOTOR);
    }
  }

  if (millis() - lastFeedbackMs >= FEEDBACK_PERIOD_MS) {
    lastFeedbackMs = millis();
    Serial.print("RPM ");
    Serial.print(leftRPMFiltered);
    Serial.print(" ");
    Serial.println(rightRPMFiltered);
  }
}