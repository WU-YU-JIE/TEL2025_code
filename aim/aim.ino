/*******************************************************
 * Arduino：雙軸步進控制 (M2 閉環絕對定位 - 高減速比優化版)
 * * 參數更新：
 * - 馬達細分: 6400
 * - 減速比: 9
 * - 總步數/圈: 57600 (解析度 160步/度)
 * * [功能]
 * - M2 採用 AS5600 做閉環控制，自動修正誤差。
 * - 針對高減速比優化了步進批次處理，避免讀取 I2C 造成頓挫。
 *******************************************************/

#include <Arduino.h>
#include <Wire.h>

// === AS5600 設定 ===
const uint8_t AS5600_ADDR = 0x36;
const uint8_t ANGLE_MSB_REG = 0x0E;
uint16_t as5600_last_raw = 0;     
long long as5600_total_steps = 0; 
const float AS5600_TOLERANCE = 0.4; // 到位容許誤差 (度) - 9:1減速比可以設精準一點

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

// === 參數設定 (已更新) ===
const long M1_STEPS_PER_REV = 42667L; // M1 暫時維持原樣，若相同請改 57600
const long M2_STEPS_PER_REV = 57600L; // M2: 6400 * 9 = 57600
const float GEAR_RATIO = 9.0; // 新增：減速比 9:1

// 限制範圍
const float LIMIT_MAX = 360.0;
const float LIMIT_MIN = -360.0;

// 物件初始化
StepperAxis m1 = { M1_PUL, M1_DIR, M1_ENA, false,  true, 0.0, M1_STEPS_PER_REV, 50 };
StepperAxis m2 = { M2_PUL, M2_DIR, M2_ENA, true,   true, 0.0, M2_STEPS_PER_REV, 75 };
// speed_us 設為 50us (高細分需要快一點的脈衝，不然會轉很慢)

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
  Wire.setClock(400000); // 使用 400kHz I2C

  setupAxis(m1);
  setupAxis(m2);

  initAS5600(); 

  Serial.println(F("系統就緒 (9:1 減速比優化版)"));
  Serial.print(F("M2 解析度: ")); Serial.print(M2_STEPS_PER_REV / 360.0); Serial.println(F(" steps/deg"));
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

  if (cmd.equalsIgnoreCase("p")) {
    reportStatus(true);
    return;
  }

  if (cmd.equalsIgnoreCase("stop")) {
    Serial.println(F("Stopped."));
    return;
  }

  // 1,90 或 2,90
  int comma = cmd.indexOf(',');
  if (comma > 0 && !cmd.startsWith("left") && !cmd.startsWith("right")) {
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
// M2 閉環控制 (針對 57600 steps/rev 優化)
// ==========================================
void moveToAngleM2_ClosedLoop(float targetAngle) {
  Serial.print(F("M2 Moving to: ")); Serial.println(targetAngle);
  holdEnable(m2, true);

  // 1度 = 160步
  const double stepsPerDeg = (double)M2_STEPS_PER_REV / 360.0; 

  while (true) {
    // 1. 讀取位置
    updateAS5600(); 
    double currentDeg = getAS5600Angle();
    m2.currentAngle = currentDeg;

    // 2. 計算誤差
    double error = targetAngle - currentDeg;

    // 3. 判斷到達
    if (abs(error) <= AS5600_TOLERANCE) {
      break; 
    }

    // 4. 中斷檢查
    if (Serial.available()) {
      String s = Serial.peek() != -1 ? Serial.readStringUntil('\n') : "";
      if (s.indexOf("stop") >= 0) { Serial.println(F("M2 Interrupted")); break; }
    }

    // 5. 設定方向
    bool dirLogical = (error > 0);
    int direction = dirLogical ? 1 : -1;
    bool dirPinState = dirLogical;
    if (m2.DIR_INVERT) dirPinState = !dirPinState;
    digitalWrite(m2.DIR, dirPinState ? HIGH : LOW);

    // 6. 動態步數調配 (關鍵優化)
    // 根據還有多遠，決定這次迴圈跑幾步才讀 I2C
    long stepsNeeded = (long)(abs(error) * stepsPerDeg);
    int batchSize = 1;

    if (stepsNeeded > 1600) batchSize = 400;      // 誤差 > 10度: 一次跑 400 步 (約2.5度)
    else if (stepsNeeded > 320) batchSize = 100;  // 誤差 > 2度:  一次跑 100 步
    else if (stepsNeeded > 80) batchSize = 20;    // 誤差 > 0.5度: 一次跑 20 步
    else batchSize = 2;                           // 微調: 一次跑 2 步

    // 執行批次步進
    for(int i=0; i<batchSize; i++) {
        stepOnePulse(m2, m2.speed_us);
    }
    
    reportStatus();
  }
  reportStatus(true);
}

// ==========================================
// M1 開環
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
// AS5600 核心
// ==========================================
void initAS5600() {
  as5600_last_raw = readRawAngleAS5600();
  as5600_total_steps = 0; 
  m2.currentAngle = 0.0;
}

double getAS5600Angle() {
    // 原始 AS5600 角度
    double sensor_angle = (double)as5600_total_steps * (360.0 / 4096.0);
    
    // 修正：因為 Sensor 在馬達端，轉 9 圈，輸出才轉 1 圈
    // 所以末端角度 = Sensor角度 / 減速比
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

void reportStatus(bool force) {
  if (force || (millis() - lastPrintTime > 200)) {
    Serial.print("Output_Deg:");
    Serial.print(m1.currentAngle, 2);
    Serial.print(",");
    Serial.print(m2.currentAngle, 2); // 這是除以9之後的末端角度
    
    // (選用) 如果你想看馬達實際上轉了幾圈，可以把下面這行取消註解
    // double rawSensorDeg = (double)as5600_total_steps * (360.0 / 4096.0);
    // Serial.print(" (Motor_Deg:"); Serial.print(rawSensorDeg, 1); Serial.print(")");
    
    Serial.println();
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