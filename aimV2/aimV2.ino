/*******************************************************
 * Arduino：雙軸步進控制 (嚴格分流版 + 中斷防卡頓)
 * * [解決卡頓方案]
 * - 捨棄阻塞式的 pulseIn()，改用 Pin Change Interrupt (PCINT)
 * - 適用於 Arduino Mega 2560 (A8-A15 為 Port K)
 * - 訊號讀取在背景執行，馬達運轉不再受讀取訊號影響
 * * * [核心邏輯]
 * 1. 先判斷是否手動 (CH1/CH2)
 * 2. 模式分流:
 * [Mode 1: 手動] 
 * - 執行馬達移動, 讀取 Detect (CH7)
 * - **忽略** 孔位感測 (CH8/9/10)
 * [Mode 2: 序列/待機]
 * - 讀取 孔位感測 (CH8/9/10)
 * - 讀取 Detect (CH7)
 * - 檢查 Serial 指令
 * - 若執行指令 -> 進入阻塞直到完成 (期間忽略孔位)
 * - M2 主動鎖定
 * * [M1] 開環
 * * [M2] 閉環 (手動時開環)
 *******************************************************/

#include <Arduino.h>
#include <Wire.h>

// === RC 遙控器設定 ===
// 注意：以下腳位對應 Arduino Mega 的 Port K (PCINT2)
const int PIN_RC_CH8  = A8;   // PK0 -> Index 0
const int PIN_RC_CH9  = A9;   // PK1 -> Index 1
const int PIN_RC_CH10 = A10;  // PK2 -> Index 2
const int PIN_RC_CH1  = A11;  // PK3 -> Index 3
const int PIN_RC_CH2  = A12;  // PK4 -> Index 4
const int PIN_RC_CH7  = A13;  // PK5 -> Index 5

// RC 參數
const int RC_CENTER = 1488;
const int RC_DEADZONE = 200;
const int RC_HIGH_THRES = RC_CENTER + RC_DEADZONE; // 1688
const int RC_LOW_THRES  = RC_CENTER - RC_DEADZONE; // 1288

// 手動模式速度設定
const int M1_MANUAL_SPEED_US = 1500; // M1 手動模式速度
const int M2_MANUAL_SPEED_US = 50;   // M2 手動模式速度 (改成 50)

// === 中斷讀取變數 (Volatile) ===
volatile uint16_t rc_shared[6] = {0}; // 儲存最新的脈衝寬度
volatile unsigned long rc_start[6] = {0}; // 記錄脈衝開始時間

// 讀取用的安全變數
int valA8 = 0, valA9 = 0, valA10 = 0;
int valA11 = 0, valA12 = 0, valA13 = 0;

String lastHoleState = "";   
String lastDetectState = ""; 
unsigned long lastPrintTime = 0; 

// 手動控制旗標
bool m1_manual_active = false;
int  m1_manual_dir = 0; // 1: Right, -1: Left

bool m2_manual_active = false;
int  m2_manual_dir = 0; // 1: Increase, -1: Decrease

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
const int M2_PUL = 10, M2_DIR = 9,  M2_ENA = 8;
const long M1_STEPS_PER_REV = 42667L; 
const long M2_STEPS_PER_REV = 57600L; 
const float GEAR_RATIO = 9.0;         

StepperAxis m1 = { M1_PUL, M1_DIR, M1_ENA, false,  true, 0.0, M1_STEPS_PER_REV, 50 };
StepperAxis m2 = { M2_PUL, M2_DIR, M2_ENA, true,   true, 0.0, M2_STEPS_PER_REV, 50 };

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
void runContinuousM2(int direction, int speedVal);

// Sensor Functions (Now Non-Blocking)
void readHoleSensors();   // A8, A9, A10
void readManualSensors(); // A11, A12
void readDetectSensor();  // A13
uint16_t getRCValue(uint8_t index); // Helper to read volatile safely

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 

  setupAxis(m1);
  setupAxis(m2);

  // 設定腳位為輸入 (加上 Pullup 防止雜訊)
  pinMode(PIN_RC_CH8, INPUT_PULLUP); 
  pinMode(PIN_RC_CH9, INPUT_PULLUP); 
  pinMode(PIN_RC_CH10, INPUT_PULLUP);
  pinMode(PIN_RC_CH1, INPUT_PULLUP); 
  pinMode(PIN_RC_CH2, INPUT_PULLUP); 
  pinMode(PIN_RC_CH7, INPUT_PULLUP);

  // === 啟動 Pin Change Interrupt (Port K) ===
  // PCIE2 開啟 Port K (A8-A15) 的中斷功能
  PCICR |= (1 << PCIE2);
  // PCMSK2 設定哪些腳位要觸發中斷 (Bit 0~5 對應 A8~A13)
  // 00111111 (Binary) = 0x3F
  PCMSK2 |= 0x3F;

  initAS5600(); 
  m2_target_angle = 0.0; 

  Serial.println(F("系統就緒 (中斷抗卡頓版)"));
}

// ==========================================
// 中斷服務程式 (ISR) - 背景讀取 RC 訊號
// ==========================================
ISR(PCINT2_vect) {
  static uint8_t last_pink = 0;
  uint8_t current_pink = PINK; // 讀取 Port K 當前狀態
  uint8_t changed = current_pink ^ last_pink; // 找出哪些 Bit 改變了
  unsigned long now = micros();

  // 掃描 A8 (Bit 0) 到 A13 (Bit 5)
  for (int i = 0; i < 6; i++) {
    if (changed & (1 << i)) { // 如果這隻腳位有變化
      if (current_pink & (1 << i)) {
        // 上升緣 (Rising Edge): 記錄開始時間
        rc_start[i] = now;
      } else {
        // 下降緣 (Falling Edge): 計算寬度
        // 簡單濾波: 只有合理的 RC 範圍 (800~2200) 才更新
        unsigned long duration = now - rc_start[i];
        if (duration > 800 && duration < 2200) {
           rc_shared[i] = (uint16_t)duration;
        }
      }
    }
  }
  last_pink = current_pink;
}

// 安全讀取全域變數的 Helper
uint16_t getRCValue(uint8_t index) {
  uint16_t val;
  noInterrupts(); // 暫停中斷以確保讀取完整 (Atomic read)
  val = rc_shared[index];
  interrupts();   // 恢復中斷
  return val;
}

// ==========================================
// Loop
// ==========================================
void loop() {
  // 0. 基礎更新
  updateAS5600();
  m2.currentAngle = getAS5600Angle();

  // 1. 先讀取手動狀態，決定走哪條路
  // 注意：現在 readManualSensors 是極速的，不消耗時間
  readManualSensors();

  if (m1_manual_active || m2_manual_active) {
    // ==========================================
    // Mode 1: 手動模式
    // ==========================================
    // 規則: 執行 M1/M2 Control & Detect Switch
    // 規則: **不理會** CH8/9/10
    
    // M1 動作
    if (m1_manual_active) {
      bool dirPinState = (m1_manual_dir > 0); 
      if (m1.DIR_INVERT) dirPinState = !dirPinState;
      digitalWrite(m1.DIR, dirPinState ? HIGH : LOW);
      
      stepOnePulse(m1, M1_MANUAL_SPEED_US);
      double anglePerStep = 360.0 / (double)m1.stepsPerRev;
      m1.currentAngle += (m1_manual_dir * anglePerStep);
    }

    // M2 動作
    if (m2_manual_active) {
      bool dirPinState = (m2_manual_dir > 0); 
      if (m2.DIR_INVERT) dirPinState = !dirPinState;
      digitalWrite(m2.DIR, dirPinState ? HIGH : LOW);
      
      stepOnePulse(m2, M2_MANUAL_SPEED_US);
      m2_target_angle = m2.currentAngle; // 同步目標
    }

    // Detect 感測
    readDetectSensor();
    
    reportStatus();

  } else {
    // ==========================================
    // Mode 2: 序列/待機模式
    // ==========================================
    // 規則: 理一遍 CH8/9/10
    // 規則: 執行 Serial Read & Detect Switch
    
    // 理一遍孔位
    readHoleSensors();
    
    // Detect 感測
    readDetectSensor();

    // 處理 Serial 指令
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      
      if (cmd.length() > 0) {
        if (cmd.equalsIgnoreCase("p")) {
          reportStatus(true);
        }
        else if (cmd.equalsIgnoreCase("stop")) {
          Serial.println(F("Stopped."));
          m2_target_angle = m2.currentAngle;
        }
        else if (cmd.startsWith("left") || cmd.startsWith("right")) {
           int comma = cmd.indexOf(',');
           int spd = 50; if (comma > 0) spd = cmd.substring(comma+1).toInt();
           int dir = cmd.startsWith("right") ? 1 : -1;
           runContinuousM1(dir, spd); // 進入阻塞執行，直到 stop
        }
        else if (cmd.startsWith("increase") || cmd.startsWith("decrease")) {
           int comma = cmd.indexOf(',');
           // M2 連續預設速度同步為 50
           int spd = 50; if (comma > 0) spd = cmd.substring(comma+1).toInt();
           int dir = cmd.startsWith("increase") ? 1 : -1;
           runContinuousM2(dir, spd); // 進入阻塞執行，直到 stop
        }
        else {
           // 絕對定位
           int comma = cmd.indexOf(',');
           if (comma > 0) {
              int axisId = cmd.substring(0, comma).toInt();
              float target = cmd.substring(comma+1).toFloat();
              if (axisId == 1) moveToAngleM1_OpenLoop(target); // 進入阻塞執行，直到到位
              else if (axisId == 2) moveToAngleM2_ClosedLoop(target); // 進入阻塞執行，直到到位
           }
        }
      }
    }

    // M2 主動鎖定 (僅在沒指令且沒手動時執行)
    float diff = m2_target_angle - m2.currentAngle;
    if (abs(diff) > AS5600_TOLERANCE) {
      bool dirLogical = (diff > 0);
      bool dirPinState = dirLogical;
      if (m2.DIR_INVERT) dirPinState = !dirPinState;
      digitalWrite(m2.DIR, dirPinState ? HIGH : LOW);
      stepOnePulse(m2, m2.speed_us);
    }
  }
}

// ==========================================
// RC 感測 (改用 getRCValue 讀取中斷值)
// ==========================================

void readHoleSensors() {
  // 直接讀取，無需 pulseIn 等待
  // A8=Index 0, A9=Index 1, A10=Index 2
  valA8  = getRCValue(0); 
  valA9  = getRCValue(1); 
  valA10 = getRCValue(2); 

  if (valA8 != 0 || valA9 != 0 || valA10 != 0) {
      String rowChar = "";
      String colChar = "";
      
      if (valA9 < 1300 && valA9 > 800) {
        rowChar = "A"; colChar = "2"; 
      } else {
        if (valA8 > 1700)      rowChar = "B";
        else if (valA8 > 1300) rowChar = "C";
        else if (valA8 > 800)  rowChar = "D";
        else rowChar = "?";
      }

      if (colChar == "") {
        if (valA10 > 1700)      colChar = "1";
        else if (valA10 > 1300) colChar = "2";
        else if (valA10 > 800)  colChar = "3";
        else colChar = "?";
      }

      if (rowChar != "?" && colChar != "?") {
        String currentHole = rowChar + colChar;
        if (!currentHole.equals(lastHoleState)) {
          Serial.print("hole:");
          Serial.println(currentHole);
          lastHoleState = currentHole;
        }
      }
  }
}

void readManualSensors() {
  // A11=Index 3, A12=Index 4
  valA11 = getRCValue(3);
  valA12 = getRCValue(4);

  // M1
  if (valA11 > RC_HIGH_THRES) { m1_manual_active = true; m1_manual_dir = -1; }
  else if (valA11 < RC_LOW_THRES && valA11 > 0) { m1_manual_active = true; m1_manual_dir = 1; }
  else { m1_manual_active = false; }

  // M2
  if (valA12 > RC_HIGH_THRES) { m2_manual_active = true; m2_manual_dir = 1; }
  else if (valA12 < RC_LOW_THRES && valA12 > 0) { m2_manual_active = true; m2_manual_dir = -1; }
  else { m2_manual_active = false; }
}

void readDetectSensor() {
  // A13=Index 5
  valA13 = getRCValue(5);

  if (valA13 > 1700) {
    if (lastDetectState != "on") {
      Serial.println("detect:on");
      lastDetectState = "on";
    }
  } else if (valA13 < 1200 && valA13 > 800) {
    if (lastDetectState != "off") {
      Serial.println("detect:off");
      lastDetectState = "off";
    }
  }
}

// 已經不需要 readPulseSafe 了，全部由中斷處理

// ==========================================
// 運動控制 (阻塞型，執行中不理會孔位)
// ==========================================

// M2 閉環定位
void moveToAngleM2_ClosedLoop(float targetAngle) {
  Serial.print(F("M2 Moving to: ")); Serial.println(targetAngle);
  holdEnable(m2, true);
  m2_target_angle = targetAngle;
  
  const double stepsPerDeg = (double)M2_STEPS_PER_REV / 360.0; 

  while (true) {
    // 執行中只讀取 Override 相關與 Detect
    // 因為改用中斷讀取，這些函式現在執行速度極快 (微秒級)，不會造成卡頓
    readManualSensors(); 
    readDetectSensor();
    
    // 手動介入 -> 退出
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

// M1 開環定位
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
    // 雖然現在是中斷讀取很快，但也不需要每步都檢查，每 200 步檢查一次即可
    if (i % 200 == 0) {
       readManualSensors();
       readDetectSensor();
    }

    if (m1_manual_active || m2_manual_active) {
        Serial.println(F("RC Override"));
        break;
    }

    stepOnePulse(m1, m1.speed_us);
    m1.currentAngle += (direction * anglePerStep);
    
    if (Serial.available()) {
      String s = Serial.peek() != -1 ? Serial.readStringUntil('\n') : "";
      if (s.indexOf("stop") >= 0) break;
    }
    reportStatus();
  }
  if (!m1_manual_active && !m2_manual_active) m1.currentAngle = targetAngle; 
  reportStatus(true);
}

// M1 連續
void runContinuousM1(int direction, int speedVal) {
    Serial.println(F("M1 Continuous..."));
    bool dirPinState = (direction > 0);
    if (m1.DIR_INVERT) dirPinState = !dirPinState;
    digitalWrite(m1.DIR, dirPinState ? HIGH : LOW);
    holdEnable(m1, true);
    
    double anglePerStep = 360.0 / (double)m1.stepsPerRev;
    long loopCount = 0;
    while(true) {
        loopCount++;
        if (loopCount % 500 == 0) {
           readManualSensors();
           readDetectSensor();
        }

        if (Serial.available()) {
            String s = Serial.readStringUntil('\n'); s.trim();
            if (s.equalsIgnoreCase("stop")) { Serial.println(F("M1 Stopped")); break; }
        }
        
        if (m1_manual_active || m2_manual_active) { Serial.println(F("RC Override")); break; }

        stepOnePulse(m1, speedVal);
        m1.currentAngle += (direction * anglePerStep);
        updateAS5600(); m2.currentAngle = getAS5600Angle();
        reportStatus();
    }
}

// M2 連續 (開環)
void runContinuousM2(int direction, int speedVal) {
    Serial.println(F("M2 Continuous..."));
    bool dirPinState = (direction > 0);
    if (m2.DIR_INVERT) dirPinState = !dirPinState;
    digitalWrite(m2.DIR, dirPinState ? HIGH : LOW);
    holdEnable(m2, true);
    
    double anglePerStep = 360.0 / (double)m2.stepsPerRev;
    long loopCount = 0;
    while(true) {
        loopCount++;
        if (loopCount % 500 == 0) {
           readManualSensors();
           readDetectSensor();
        }

        if (Serial.available()) {
            String s = Serial.readStringUntil('\n'); s.trim();
            if (s.equalsIgnoreCase("stop")) { 
              Serial.println(F("M2 Stopped")); 
              break; 
            }
        }
        
        if (m1_manual_active || m2_manual_active) { Serial.println(F("RC Override")); break; }

        stepOnePulse(m2, speedVal);
        m2.currentAngle += (direction * anglePerStep);
        reportStatus();
    }
    // 結束後同步
    updateAS5600();
    m2.currentAngle = getAS5600Angle();
    m2_target_angle = m2.currentAngle;
    reportStatus(true);
}

// ==========================================
// AS5600 & 輔助
// ==========================================
void initAS5600() {
  as5600_last_raw = readRawAngleAS5600();
  as5600_total_steps = 0; 
  m2.currentAngle = 0.0;
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