// aim.ino
/*******************************************************
 * Arduino：雙軸步進控制 (AIM 完整版 + Servo)
 * [功能]
 * - M1 (Open-Loop): 左右旋轉 (Pin 13,12,11)
 * - M2 (Closed-Loop): 上下俯仰 (Pin 10,9,8, AS5600 I2C)
 * - Servo: 額外機構控制 (Pin 7) ★新增
 *******************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// === AS5600 設定 ===
const uint8_t AS5600_ADDR = 0x36;
const uint8_t ANGLE_MSB_REG = 0x0E;
uint16_t as5600_last_raw = 0;     
long long as5600_total_steps = 0; 
const float AS5600_TOLERANCE = 0.4;

// === Servo 設定 (新增) ===
Servo myServo;
const int PIN_SERVO = 7; // ★ 定義 Servo 腳位

// === 馬達結構 ===
struct StepperAxis {
  int PUL, DIR, ENA;
  bool DIR_INVERT;        
  bool ENA_ACTIVE_HIGH;   
  double currentAngle;    
  long stepsPerRev;       
  int speed_us;           
};

// === 腳位定義 ===
const int M1_PUL = 13, M1_DIR = 12, M1_ENA = 11;
const int M2_PUL = 10, M2_DIR = 9,  M2_ENA = 8;

// === 參數設定 ===
const long M1_STEPS_PER_REV = 42667L; 
const long M2_STEPS_PER_REV = 57600L; 
const float GEAR_RATIO = 9.0;        

StepperAxis m1 = { M1_PUL, M1_DIR, M1_ENA, false,  true, 0.0, M1_STEPS_PER_REV, 50 };
StepperAxis m2 = { M2_PUL, M2_DIR, M2_ENA, true,   true, 0.0, M2_STEPS_PER_REV, 50 };

unsigned long lastPrintTime = 0; 

// === 函式宣告 ===
void initAS5600();
void updateAS5600();
uint16_t readRawAngleAS5600();
double getAS5600Angle();
void setupAxis(const StepperAxis& ax);
static inline void holdEnable(const StepperAxis& ax, bool on);
void stepOnePulse(StepperAxis& ax, int currentSpeedUs);
void reportStatus(bool force = false);
void moveToAngleM1_OpenLoop(float targetAngle);
void moveToAngleM2_ClosedLoop(float targetAngle);
void runContinuousM1(int direction, int speedVal);

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // 提升 I2C 速度

  setupAxis(m1);
  setupAxis(m2);

  // ★ 初始化 Servo
  myServo.attach(PIN_SERVO);
  myServo.write(90); // 預設 90 度
  
  initAS5600(); 

  Serial.println(F("AIM System Ready (With Servo Pin7)"));
}

// ==========================================
// Loop
// ==========================================
void loop() {
  updateAS5600();
  m2.currentAngle = getAS5600Angle();

  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  // 1. 顯示狀態
  if (cmd.equalsIgnoreCase("p")) {
    reportStatus(true);
    return;
  }

  // 2. 停止
  if (cmd.equalsIgnoreCase("stop")) {
    Serial.println(F("Stopped."));
    return;
  }

  // ★ 3. Servo 指令 (新增)
  // S1=60, S2=90, S3=120
  if (cmd.equalsIgnoreCase("S1")) {
      myServo.write(60);
      Serial.println(F("Servo=60"));
      return;
  }
  if (cmd.equalsIgnoreCase("S2")) {
      myServo.write(90);
      Serial.println(F("Servo=90"));
      return;
  }
  if (cmd.equalsIgnoreCase("S3")) {
      myServo.write(120);
      Serial.println(F("Servo=120"));
      return;
  }
  // 格式: servo,角度
  if (cmd.startsWith("servo,")) {
      int val = cmd.substring(6).toInt();
      // 保護範圍 0-180
      if (val < 0) val = 0;
      if (val > 180) val = 180;
      myServo.write(val);
      Serial.print(F("Servo=")); Serial.println(val);
      return;
  }

  // 4. M1 連續旋轉模式
  if (cmd.startsWith("left") || cmd.startsWith("right")) {
    int comma = cmd.indexOf(',');
    int spd = 50; 
    if (comma > 0) spd = cmd.substring(comma + 1).toInt();
    int dir = cmd.startsWith("right") ? 1 : -1;
    runContinuousM1(dir, spd);
    return;
  }

  // 5. 絕對定位模式 (1,90 或 2,90)
  int comma = cmd.indexOf(',');
  if (comma > 0) {
    int axisId = cmd.substring(0, comma).toInt();
    float targetAngle = cmd.substring(comma + 1).toFloat();

    if (axisId == 1) {
      moveToAngleM1_OpenLoop(targetAngle);
      Serial.println(F("M1 Done"));
    } else if (axisId == 2) {
      moveToAngleM2_ClosedLoop(targetAngle);
      Serial.println(F("M2 Done"));
    }
  }
}

// ==========================================
// M2 閉環控制 (含減速比優化)
// ==========================================
void moveToAngleM2_ClosedLoop(float targetAngle) {
  Serial.print(F("M2 Moving to: ")); Serial.println(targetAngle);
  holdEnable(m2, true);

  const double stepsPerDeg = (double)M2_STEPS_PER_REV / 360.0; 

  while (true) {
    updateAS5600(); 
    double currentDeg = getAS5600Angle();
    m2.currentAngle = currentDeg;

    double error = targetAngle - currentDeg;
    if (abs(error) <= AS5600_TOLERANCE) break; 

    // 中斷檢查
    if (Serial.available()) {
      String s = Serial.peek() != -1 ? Serial.readStringUntil('\n') : "";
      if (s.indexOf("stop") >= 0) { Serial.println(F("M2 Interrupted")); break; }
    }

    bool dirLogical = (error > 0);
    bool dirPinState = dirLogical;
    if (m2.DIR_INVERT) dirPinState = !dirPinState;
    digitalWrite(m2.DIR, dirPinState ? HIGH : LOW);

    // 動態步數
    long stepsNeeded = (long)(abs(error) * stepsPerDeg);
    int batchSize = 1;
    if (stepsNeeded > 1600) batchSize = 400;      
    else if (stepsNeeded > 320) batchSize = 100;  
    else if (stepsNeeded > 80) batchSize = 20;    
    else batchSize = 2;                           

    for(int i=0; i<batchSize; i++) stepOnePulse(m2, m2.speed_us);
    
    reportStatus();
  }
  reportStatus(true);
}

// ==========================================
// M1 開環定位
// ==========================================
void moveToAngleM1_OpenLoop(float targetAngle) {
  float diff = targetAngle - m1.currentAngle;
  long stepsToMove = (long)(fabs(diff) * ((double)m1.stepsPerRev / 360.0) + 0.5);
  if (stepsToMove <= 0) { m1.currentAngle = targetAngle; return; }

  bool dirLogical = (diff >= 0);
  int direction = dirLogical ? 1 : -1;
  if (m1.DIR_INVERT) dirLogical = !dirLogical;
  digitalWrite(m1.DIR, dirLogical ? HIGH : LOW);
  holdEnable(m1, true);

  double anglePerStep = 360.0 / (double)m1.stepsPerRev;
  for (long i = 0; i < stepsToMove; i++) {
    stepOnePulse(m1, m1.speed_us);
    m1.currentAngle += (direction * anglePerStep);
    
    if (Serial.available()) {
      String s = Serial.peek() != -1 ? Serial.readStringUntil('\n') : "";
      if (s.indexOf("stop") >= 0) break;
    }
    reportStatus();
  }
  m1.currentAngle = targetAngle; 
  reportStatus(true);
}

// ==========================================
// M1 連續旋轉
// ==========================================
void runContinuousM1(int direction, int speedVal) {
    Serial.println(F("M1 Continuous... (Type 'stop' to end)"));
    
    bool dirPinState = (direction > 0);
    if (m1.DIR_INVERT) dirPinState = !dirPinState;
    digitalWrite(m1.DIR, dirPinState ? HIGH : LOW);
    holdEnable(m1, true);
    
    double anglePerStep = 360.0 / (double)m1.stepsPerRev;

    while(true) {
        if (Serial.available()) {
            String s = Serial.readStringUntil('\n'); 
            s.trim();
            if (s.equalsIgnoreCase("stop")) {
                Serial.println(F("M1 Stopped"));
                break;
            }
        }
        stepOnePulse(m1, speedVal);
        m1.currentAngle += (direction * anglePerStep);
        
        updateAS5600();
        m2.currentAngle = getAS5600Angle();
        reportStatus();
    }
}

// ==========================================
// AS5600 核心邏輯
// ==========================================
void initAS5600() {
  as5600_last_raw = readRawAngleAS5600();
  as5600_total_steps = 0; 
  m2.currentAngle = 0.0;
}

double getAS5600Angle() {
    double sensor_angle = (double)as5600_total_steps * (360.0 / 4096.0);
    return sensor_angle / GEAR_RATIO;
}

void updateAS5600() {
  uint16_t raw = readRawAngleAS5600();
  int delta = int(raw) - int(as5600_last_raw);
  if (delta > 2048) delta -= 4096;
  else if (delta < -2048) delta += 4096;
  as5600_total_steps += (long long)delta;
  as5600_last_raw = raw;
}

uint16_t readRawAngleAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(ANGLE_MSB_REG);
  if (Wire.endTransmission(false) != 0) return as5600_last_raw; 
  
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  unsigned long tstart = micros();
  while (Wire.available() < 2) { if (micros() - tstart > 500) break; }
  
  if (Wire.available() >= 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return ((uint16_t)msb << 8) | (uint16_t)lsb;
  }
  return as5600_last_raw;
}

// ==========================================
// 輔助函式
// ==========================================
void reportStatus(bool force) {
  if (force || (millis() - lastPrintTime > 200)) {
    Serial.print("degree:");
    Serial.print(m1.currentAngle, 2);
    Serial.print(",");
    Serial.println(m2.currentAngle, 2); 
    lastPrintTime = millis();
  }
}

void setupAxis(const StepperAxis& ax) {
  pinMode(ax.PUL, OUTPUT);
  pinMode(ax.DIR, OUTPUT);
  pinMode(ax.ENA, OUTPUT);
  holdEnable(ax, true); 
}

static inline void holdEnable(const StepperAxis& ax, bool on) {
  digitalWrite(ax.ENA, on ? (ax.ENA_ACTIVE_HIGH ? HIGH : LOW)
                          : (ax.ENA_ACTIVE_HIGH ? LOW  : HIGH));
}

void stepOnePulse(StepperAxis& ax, int currentSpeedUs) {
  digitalWrite(ax.PUL, HIGH);
  delayMicroseconds(currentSpeedUs);
  digitalWrite(ax.PUL, LOW);
  delayMicroseconds(currentSpeedUs);
}