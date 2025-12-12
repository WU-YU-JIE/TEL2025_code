/*******************************************************
 * Arduino：雙軸步進控制 (M2 選擇性角度 + 視覺訊號打斷版)
 * [2025/12/06 修改說明]
 * 1. readHoleSensors 修改：
 * - 偵測到原 A2 訊號 (v9 在 800~1300) 時，不再印出文字，
 * 而是直接驅動 M1 移動到絕對角度 -14.5 度。
 * - 移除了 A2 的優先權邏輯，現在它是獨立的觸發條件。
 *******************************************************/

#include <Arduino.h>
#include <Wire.h>

// === RC 遙控器設定 ===
const int PIN_RC_CH8  = A8;   // PK0 -> Index 0
const int PIN_RC_CH9  = A9;   // PK1 -> Index 1
const int PIN_RC_CH10 = A10;  // PK2 -> Index 2
const int PIN_RC_CH1  = A12;  // PK3 -> Index 3 (註: 這裡實際上對應 ISR 讀取順序)
const int PIN_RC_CH2  = A13;  // PK4 -> Index 4 
const int PIN_RC_CH7  = A11;  // PK5 -> Index 5 

// RC 參數
const int RC_CENTER = 1488;
const int RC_DEADZONE = 200;
const int RC_HIGH_THRES = RC_CENTER + RC_DEADZONE; // 1688
const int RC_LOW_THRES  = RC_CENTER - RC_DEADZONE; // 1288
const int MANUAL_SPEED_US = 800; 

// === 中斷讀取變數 ===
volatile uint16_t rc_shared[6] = {0}; 
volatile unsigned long rc_start[6] = {0}; 

// 讀取用的變數
int valDetect = 0; // 用於判斷視覺訊號

String lastHoleState = "";   
String lastDetectState = ""; 
unsigned long lastPrintTime = 0; 

// 手動控制旗標
bool m1_manual_active = false;
int  m1_manual_dir = 0; 
bool m2_manual_active = false;
int  m2_manual_dir = 0; 
bool m2_was_manual_active = false; 

// M2 目標角度
float m2_target_angle = 0.0;

// === AS5600 ===
const uint8_t AS5600_ADDR = 0x36;
const uint8_t ANGLE_MSB_REG = 0x0E;
uint16_t as5600_last_raw = 0;     
long long as5600_total_steps = 0; 
const float AS5600_TOLERANCE = 0.4; 

// === 馬達結構 ===
struct StepperAxis {
  int PUL, DIR, ENA;
  bool DIR_INVERT;        
  bool ENA_ACTIVE_HIGH;   
  double currentAngle;    
  long stepsPerRev;       
  int speed_us;           
};

const int M1_PUL = 13, M1_DIR = 12, M1_ENA = 11;
const int M2_PUL = 7, M2_DIR = 6,  M2_ENA = 5;
const long M1_STEPS_PER_REV = 42667L; 
const long M2_STEPS_PER_REV = 57600L; 
const float GEAR_RATIO = 9.0;         

StepperAxis m1 = { M1_PUL, M1_DIR, M1_ENA, false,  true, 0.0, M1_STEPS_PER_REV, 50 };
StepperAxis m2 = { M2_PUL, M2_DIR, M2_ENA, true,   true, 0.0, M2_STEPS_PER_REV, 50 };

// === 函式宣告 ===
void initAS5600();
void updateAS5600();
void syncAS5600Raw(); 
uint16_t readRawAngleAS5600();
double getAS5600Angle();

void setupAxis(const StepperAxis& ax);
static inline void holdEnable(const StepperAxis& ax, bool on);
void stepOnePulse(StepperAxis& ax, int currentSpeedUs);
void reportStatus(bool force = false);

void moveToAngleM1_OpenLoop(float targetAngle);
void moveToAngleM2_ClosedLoop(float targetAngle);
void runContinuousM1(int direction, int speedVal);
void runContinuousM2(int direction, int speedVal);

void readHoleSensors();   
void readManualSensors(); 
void readDetectSensor();  
int  getDetectState(); // 新增：取得簡化的 1/0/-1 狀態
uint16_t getRCValue(uint8_t index); 

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 

  setupAxis(m1);
  setupAxis(m2);

  pinMode(A8, INPUT_PULLUP); pinMode(A9, INPUT_PULLUP); pinMode(A10, INPUT_PULLUP);
  pinMode(A11, INPUT_PULLUP); pinMode(A12, INPUT_PULLUP); pinMode(A13, INPUT_PULLUP);

  PCICR |= (1 << PCIE2);
  PCMSK2 |= 0x3F; 

  initAS5600(); 
  m2_target_angle = 0.0; 

  Serial.println(F("系統就緒 (視覺訊號打斷 + M1定點版)"));
}

// ==========================================
// ISR
// ==========================================
ISR(PCINT2_vect) {
  static uint8_t last_pink = 0;
  uint8_t current_pink = PINK; 
  uint8_t changed = current_pink ^ last_pink; 
  unsigned long now = micros();

  for (int i = 0; i < 6; i++) {
    if (changed & (1 << i)) { 
      if (current_pink & (1 << i)) {
        rc_start[i] = now;
      } else {
        unsigned long duration = now - rc_start[i];
        if (duration > 800 && duration < 2200) {
           rc_shared[i] = (uint16_t)duration;
        }
      }
    }
  }
  last_pink = current_pink;
}

uint16_t getRCValue(uint8_t index) {
  uint16_t val;
  noInterrupts(); 
  val = rc_shared[index];
  interrupts();   
  return val;
}

// ==========================================
// 視覺訊號狀態判斷
// ==========================================
int getDetectState() {
  int val = getRCValue(3); // A11 / Index 3
  if (val > 1600) return 1;
  if (val < 1400) return 0;
  return -1;
}

// ==========================================
// Loop
// ==========================================
void loop() {
  // Loop 中不主動打斷手動，因為手動是你在控
  // 主要負責接收 Serial 指令和手動搖桿

  readManualSensors();

  if (m1_manual_active || m2_manual_active) {
    // Mode 1: 手動模式 (保持原樣)
    if (m1_manual_active) {
      bool dirPinState = (m1_manual_dir > 0); 
      if (m1.DIR_INVERT) dirPinState = !dirPinState;
      digitalWrite(m1.DIR, dirPinState ? HIGH : LOW);
      stepOnePulse(m1, MANUAL_SPEED_US);
      double anglePerStep = 360.0 / (double)m1.stepsPerRev;
      m1.currentAngle += (m1_manual_dir * anglePerStep);
    }
    if (m2_manual_active) {
      if (!m2_was_manual_active) {
        syncAS5600Raw();
        m2_was_manual_active = true;
      }
      bool dirPinState = (m2_manual_dir > 0); 
      if (m2.DIR_INVERT) dirPinState = !dirPinState;
      digitalWrite(m2.DIR, dirPinState ? HIGH : LOW);
      stepOnePulse(m2, MANUAL_SPEED_US);
      updateAS5600();
      m2.currentAngle = getAS5600Angle();
      m2_target_angle = m2.currentAngle; 
    } else {
      m2_was_manual_active = false;
    }
    readDetectSensor(); // 更新顯示
    reportStatus();

  } else {
    // Mode 2: 序列/待機
    m2_was_manual_active = false;
    readHoleSensors(); // 這裡包含新的 M1 -14.5度 邏輯
    readDetectSensor();

    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.length() > 0) {
        if (cmd.equalsIgnoreCase("p")) reportStatus(true);
        else if (cmd.equalsIgnoreCase("stop")) {
          Serial.println(F("Stopped."));
          m2_target_angle = m2.currentAngle;
        }
        else if (cmd.startsWith("left") || cmd.startsWith("right")) {
           int comma = cmd.indexOf(',');
           int spd = 50; if (comma > 0) spd = cmd.substring(comma+1).toInt();
           int dir = cmd.startsWith("right") ? 1 : -1;
           runContinuousM1(dir, spd); 
        }
        else if (cmd.startsWith("increase") || cmd.startsWith("decrease")) {
           int comma = cmd.indexOf(',');
           int spd = 1500; if (comma > 0) spd = cmd.substring(comma+1).toInt();
           int dir = cmd.startsWith("increase") ? 1 : -1;
           runContinuousM2(dir, spd); 
        }
        else {
           int comma = cmd.indexOf(',');
           if (comma > 0) {
             int axisId = cmd.substring(0, comma).toInt();
             float target = cmd.substring(comma+1).toFloat();
             if (axisId == 1) moveToAngleM1_OpenLoop(target); 
             else if (axisId == 2) moveToAngleM2_ClosedLoop(target); 
           }
        }
      }
    }

    // M2 定位修正 (如果靜止時有誤差)
    float diff = m2_target_angle - m2.currentAngle;
    if (abs(diff) > AS5600_TOLERANCE) {
        bool dirLogical = (diff > 0);
        bool dirPinState = dirLogical;
        if (m2.DIR_INVERT) dirPinState = !dirPinState;
        digitalWrite(m2.DIR, dirPinState ? HIGH : LOW);
        
        syncAS5600Raw();
        stepOnePulse(m2, m2.speed_us);
        updateAS5600();
        m2.currentAngle = getAS5600Angle();
    }
  }
}

// ==========================================
// 運動控制
// ==========================================

void moveToAngleM2_ClosedLoop(float targetAngle) {
  Serial.print(F("M2 Moving to: ")); Serial.println(targetAngle);
  
  int startDetectState = getDetectState();

  syncAS5600Raw();
  holdEnable(m2, true);
  m2_target_angle = targetAngle;
  const double stepsPerDeg = (double)M2_STEPS_PER_REV / 360.0; 

  while (true) {
    int currDetectState = getDetectState();
    if (currDetectState != -1 && currDetectState != startDetectState) {
        Serial.println(F("Detect Change: STOP"));
        m2_target_angle = m2.currentAngle; 
        break;
    }

    readManualSensors(); 
    if (m1_manual_active || m2_manual_active) {
        Serial.println(F("RC Override"));
        m2_target_angle = m2.currentAngle;
        break;
    }

    updateAS5600(); 
    m2.currentAngle = getAS5600Angle();
    
    double error = targetAngle - m2.currentAngle;
    if (abs(error) <= AS5600_TOLERANCE) break; 

    if (Serial.available()) {
      String s = Serial.peek() != -1 ? Serial.readStringUntil('\n') : "";
      if (s.indexOf("stop") >= 0) { Serial.println(F("M2 Interrupted")); break; }
    }

    bool dirLogical = (error > 0);
    bool dirPinState = dirLogical;
    if (m2.DIR_INVERT) dirPinState = !dirPinState;
    digitalWrite(m2.DIR, dirPinState ? HIGH : LOW);

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

void moveToAngleM1_OpenLoop(float targetAngle) {
  int startDetectState = getDetectState();
  
  float diff = targetAngle - m1.currentAngle;
  long stepsToMove = (long)(fabs(diff) * ((double)m1.stepsPerRev / 360.0) + 0.5);
  // 如果已經在目標位置附近（步數為0），直接更新狀態並返回
  if (stepsToMove <= 0) { 
      m1.currentAngle = targetAngle; 
      // 這裡不需特別 print，除非想要確認收到指令
      return; 
  }
  
  Serial.print(F("M1 Moving to: ")); Serial.println(targetAngle);

  bool dirLogical = (diff >= 0);
  int direction = dirLogical ? 1 : -1;
  if (m1.DIR_INVERT) dirLogical = !dirLogical;
  digitalWrite(m1.DIR, dirLogical ? HIGH : LOW);
  holdEnable(m1, true);

  double anglePerStep = 360.0 / (double)m1.stepsPerRev;
  for (long i = 0; i < stepsToMove; i++) {
    
    if (i % 50 == 0) {
       int currDetectState = getDetectState();
       if (currDetectState != -1 && currDetectState != startDetectState) {
           Serial.println(F("Detect Change: STOP"));
           m1.currentAngle += (i * direction * anglePerStep); 
           return; 
       }
       readManualSensors();
    }

    if (m1_manual_active || m2_manual_active) { Serial.println(F("RC Override")); break; }

    stepOnePulse(m1, m1.speed_us);
    
    if (Serial.available()) {
      String s = Serial.peek() != -1 ? Serial.readStringUntil('\n') : "";
      if (s.indexOf("stop") >= 0) break;
    }
    if (i % 200 == 0) reportStatus();
  }
  
  if (!m1_manual_active && !m2_manual_active) m1.currentAngle = targetAngle; 
  reportStatus(true);
}

void runContinuousM1(int direction, int speedVal) {
    Serial.println(F("M1 Continuous..."));
    
    int startDetectState = getDetectState(); 

    bool dirPinState = (direction > 0);
    if (m1.DIR_INVERT) dirPinState = !dirPinState;
    digitalWrite(m1.DIR, dirPinState ? HIGH : LOW);
    holdEnable(m1, true);
    
    double anglePerStep = 360.0 / (double)m1.stepsPerRev;
    long loopCount = 0;
    while(true) {
        loopCount++;
        
        if (loopCount % 100 == 0) {
            int currDetectState = getDetectState();
            if (currDetectState != -1 && currDetectState != startDetectState) {
               Serial.println(F("Detect Change: STOP"));
               break;
            }
            readManualSensors();
        }

        if (Serial.available()) {
             String s = Serial.readStringUntil('\n'); s.trim();
             if (s.equalsIgnoreCase("stop")) { Serial.println(F("M1 Stopped")); break; }
        }
        
        if (m1_manual_active || m2_manual_active) { Serial.println(F("RC Override")); break; }

        stepOnePulse(m1, speedVal);
        m1.currentAngle += (direction * anglePerStep);
        if (loopCount % 500 == 0) reportStatus();
    }
}

void runContinuousM2(int direction, int speedVal) {
    Serial.println(F("M2 Continuous..."));
    int startDetectState = getDetectState(); 
    syncAS5600Raw();
    bool dirPinState = (direction > 0);
    if (m2.DIR_INVERT) dirPinState = !dirPinState;
    digitalWrite(m2.DIR, dirPinState ? HIGH : LOW);
    holdEnable(m2, true);
    
    double anglePerStep = 360.0 / (double)m2.stepsPerRev;
    long loopCount = 0;
    while(true) {
        loopCount++;
        if (loopCount % 100 == 0) {
            int currDetectState = getDetectState();
            if (currDetectState != -1 && currDetectState != startDetectState) {
               Serial.println(F("Detect Change: STOP"));
               break;
            }
            readManualSensors();
        }
        if (Serial.available()) {
             String s = Serial.readStringUntil('\n'); s.trim();
             if (s.equalsIgnoreCase("stop")) { Serial.println(F("M2 Stopped")); break; }
        }
        if (m1_manual_active || m2_manual_active) { Serial.println(F("RC Override")); break; }

        stepOnePulse(m2, speedVal);
        m2.currentAngle += (direction * anglePerStep);
        updateAS5600();
        m2.currentAngle = getAS5600Angle();
        if (loopCount % 500 == 0) reportStatus();
    }
    updateAS5600();
    m2.currentAngle = getAS5600Angle();
    m2_target_angle = m2.currentAngle;
    reportStatus(true);
}

// ==========================================
// RC 感測 (主要修改處)
// ==========================================
void readHoleSensors() {
  int v8  = getRCValue(0); 
  int v9  = getRCValue(1); 
  int v10 = getRCValue(2); 
  
  // 1. 檢查是否為「原A2指令區間」(v9: 800~1300)
  //    如果成立，直接執行 M1 移動至 -14.5 度，不再進行後續判斷
  if (v9 < 1300 && v9 > 800) {
      // 呼叫移動指令 (內部邏輯：若已在目標位置則直接返回，不會卡住)
      moveToAngleM1_OpenLoop(-14.5);
      return; 
  }

  // 2. 如果不是上述指令，則進行一般的 B/C/D 孔位偵測
  //    (已移除 A2 的 if-else 優先權綁定，這裡只負責字串輸出)
  if (v8 != 0 || v9 != 0 || v10 != 0) {
      String rowChar = "";
      String colChar = "";
      
      // 判斷 Row (由 v8 控制)
      if (v8 > 1700)      rowChar = "B";
      else if (v8 > 1300) rowChar = "C";
      else if (v8 > 800)  rowChar = "D";
      else rowChar = "?";

      // 判斷 Col (由 v10 控制)
      if (v10 > 1700)      colChar = "1";
      else if (v10 > 1300) colChar = "2";
      else if (v10 > 800)  colChar = "3";
      else colChar = "?";

      // 只有當行列都有效時才輸出
      if (rowChar != "?" && colChar != "?") {
        String currentHole = rowChar + colChar;
        if (!currentHole.equals(lastHoleState)) {
          Serial.print("hole:"); Serial.println(currentHole);
          lastHoleState = currentHole;
        }
      }
  }
}

void readManualSensors() {
  int vm1 = getRCValue(4); 
  int vm2 = getRCValue(5); 

  if (vm1 > RC_HIGH_THRES) { m1_manual_active = true; m1_manual_dir = -1; }
  else if (vm1 < RC_LOW_THRES && vm1 > 0) { m1_manual_active = true; m1_manual_dir = 1; }
  else { m1_manual_active = false; }

  if (vm2 > RC_HIGH_THRES) { m2_manual_active = true; m2_manual_dir = 1; }
  else if (vm2 < RC_LOW_THRES && vm2 > 0) { m2_manual_active = true; m2_manual_dir = -1; }
  else { m2_manual_active = false; }
}

void readDetectSensor() {
  valDetect = getRCValue(3); 

  if (valDetect > 1700) {
    if (lastDetectState != "on") {
      Serial.println("detect:on");
      lastDetectState = "on";
    }
  } else if (valDetect < 1300 && valDetect > 800) {
    if (lastDetectState != "off") {
      Serial.println("detect:off");
      lastDetectState = "off";
    }
  }
}

// ==========================================
// AS5600 & 輔助
// ==========================================
void initAS5600() {
  as5600_last_raw = readRawAngleAS5600();
  as5600_total_steps = 0; 
  m2.currentAngle = 0.0;
}
void syncAS5600Raw() {
  as5600_last_raw = readRawAngleAS5600();
}
double getAS5600Angle() {
    return ((double)as5600_total_steps * (360.0 / 4096.0)) / GEAR_RATIO;
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
  Wire.beginTransmission(AS5600_ADDR); Wire.write(ANGLE_MSB_REG);
  if (Wire.endTransmission(false) != 0) return as5600_last_raw; 
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  unsigned long tstart = micros();
  while (Wire.available() < 2) { if (micros() - tstart > 500) break; }
  if (Wire.available() >= 2) {
    uint8_t msb = Wire.read(); uint8_t lsb = Wire.read();
    return ((uint16_t)msb << 8) | (uint16_t)lsb;
  }
  return as5600_last_raw;
}
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
  pinMode(ax.PUL, OUTPUT); pinMode(ax.DIR, OUTPUT); pinMode(ax.ENA, OUTPUT);
  holdEnable(ax, true); 
}
static inline void holdEnable(const StepperAxis& ax, bool on) {
  digitalWrite(ax.ENA, on ? (ax.ENA_ACTIVE_HIGH ? HIGH : LOW) : (ax.ENA_ACTIVE_HIGH ? LOW  : HIGH));
}
void stepOnePulse(StepperAxis& ax, int currentSpeedUs) {
  digitalWrite(ax.PUL, HIGH); delayMicroseconds(currentSpeedUs);
  digitalWrite(ax.PUL, LOW); delayMicroseconds(currentSpeedUs);
}