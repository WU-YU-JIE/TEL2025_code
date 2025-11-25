/*******************************************************
 * Arduino：雙軸步進控制 (修正版)
 * 格式1：1,角度 或 2,角度 (絕對座標定位)
 * 格式2：left,速度 或 right,速度 (M1 連續旋轉，直到 stop 或 +-80度)
 * 指令 stop：停止連續旋轉
 * 輸出：轉動時持續回傳 degree:M1角度,M2角度
 *******************************************************/
#include <Arduino.h>

struct StepperAxis {
  int PUL, DIR, ENA;
  bool DIR_INVERT;        // true=反向
  bool ENA_ACTIVE_HIGH;   // true=HIGH為使能
  double currentAngle;    // 目前角度
  long stepsPerRev;       // 每圈步數
  int speed_us;           // 預設速度
};

// === 腳位 ===
const int M1_PUL = 13, M1_DIR = 12, M1_ENA = 11;
const int M2_PUL = 10, M2_DIR = 9,  M2_ENA = 8;

// === 參數 ===
const long M1_STEPS_PER_REV = 42667L;
const long M2_STEPS_PER_REV = 57600L;
const float LIMIT_MAX = 80.0;
const float LIMIT_MIN = -80.0;

StepperAxis m1 = { M1_PUL, M1_DIR, M1_ENA, false,  true, 0.0, M1_STEPS_PER_REV, 100 };
StepperAxis m2 = { M2_PUL, M2_DIR, M2_ENA, true,  true, 0.0, M2_STEPS_PER_REV, 100 };

unsigned long lastPrintTime = 0; 

// === 函式宣告 (這裡修正了) ===
// C++ 規則：預設參數只能在宣告時寫一次
void reportStatus(bool force = false); 
static inline void holdEnable(const StepperAxis& ax, bool on);
void setupAxis(const StepperAxis& ax);
void stepOnePulse(StepperAxis& ax, int currentSpeedUs);
void runContinuous(StepperAxis& ax, int direction, int speedVal);
void moveToAngle(StepperAxis& ax, float targetAngle);

void setup() {
  Serial.begin(115200);
  setupAxis(m1);
  setupAxis(m2);
  Serial.println(F("就緒："));
  Serial.println(F("1. 定位模式: 1,角度 或 2,角度"));
  Serial.println(F("2. 連續模式: left,50 或 right,50 (M1移動)"));
  Serial.println(F("3. 停止: stop"));
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  // 1. 處理 stop
  if (cmd.equalsIgnoreCase("stop")) {
    Serial.println(F("Stopped."));
    return;
  }

  // 2. 處理 left/right (連續移動模式)
  if (cmd.startsWith("left") || cmd.startsWith("right")) {
    int comma = cmd.indexOf(',');
    int spd = 50; 
    if (comma > 0) {
      spd = cmd.substring(comma + 1).toInt();
    }
    
    bool goRight = cmd.startsWith("right");
    int direction = goRight ? 1 : -1;
    
    runContinuous(m1, direction, spd);
    return;
  }

  // 3. 處理 1,角度 或 2,角度 (絕對定位)
  int comma = cmd.indexOf(',');
  if (comma > 0) {
    int axisId = cmd.substring(0, comma).toInt();
    float targetAngle = cmd.substring(comma + 1).toFloat();

    if (axisId == 1) {
      moveToAngle(m1, targetAngle);
      Serial.println(F("M1 OK"));
    } else if (axisId == 2) {
      moveToAngle(m2, targetAngle);
      Serial.println(F("M2 OK"));
    }
  } else {
    Serial.println(F("❌ 格式錯誤"));
  }
}

// === 輔助功能 ===

// 回報角度 (修正：這裡拿掉 = false)
void reportStatus(bool force) {
  if (force || (millis() - lastPrintTime > 200)) {
    Serial.print("degree:");
    Serial.print(m1.currentAngle, 2);
    Serial.print(",");
    Serial.println(m2.currentAngle, 2);
    lastPrintTime = millis();
  }
}

static inline void holdEnable(const StepperAxis& ax, bool on) {
  digitalWrite(ax.ENA, on ? (ax.ENA_ACTIVE_HIGH ? HIGH : LOW)
                          : (ax.ENA_ACTIVE_HIGH ? LOW  : HIGH));
}

void setupAxis(const StepperAxis& ax) {
  pinMode(ax.PUL, OUTPUT);
  pinMode(ax.DIR, OUTPUT);
  pinMode(ax.ENA, OUTPUT);
  holdEnable(ax, true); 
}

void stepOnePulse(StepperAxis& ax, int currentSpeedUs) {
  digitalWrite(ax.PUL, HIGH);
  delayMicroseconds(currentSpeedUs);
  digitalWrite(ax.PUL, LOW);
  delayMicroseconds(currentSpeedUs);
}

// === 核心運動邏輯 ===

// 模式 A: 連續旋轉
void runContinuous(StepperAxis& ax, int direction, int speedVal) {
  Serial.println(F("Start Continuous Move..."));
  
  bool dirPinState = (direction > 0); 
  if (ax.DIR_INVERT) dirPinState = !dirPinState;
  digitalWrite(ax.DIR, dirPinState ? HIGH : LOW);
  
  holdEnable(ax, true);

  double anglePerStep = 360.0 / (double)ax.stepsPerRev;
  
  while (true) {
    if (Serial.available()) {
      String stopCmd = Serial.readStringUntil('\n');
      stopCmd.trim();
      if (stopCmd.equalsIgnoreCase("stop")) {
        Serial.println(F("Command Stop received."));
        break;
      }
    }

    if (ax.currentAngle >= LIMIT_MAX && direction > 0) {
      Serial.println(F("Limit +80 reached."));
      break;
    }
    if (ax.currentAngle <= LIMIT_MIN && direction < 0) {
      Serial.println(F("Limit -80 reached."));
      break;
    }

    stepOnePulse(ax, speedVal);
    ax.currentAngle += (direction * anglePerStep);

    reportStatus(); // 這裡使用預設參數
  }
}

// 模式 B: 絕對定位
void moveToAngle(StepperAxis& ax, float targetAngle) {
  float diff = targetAngle - ax.currentAngle;
  long stepsToMove = (long)(fabs(diff) * ((double)ax.stepsPerRev / 360.0) + 0.5);
  
  if (stepsToMove <= 0) { 
    ax.currentAngle = targetAngle; 
    return; 
  }

  bool dirLogical = (diff >= 0);
  int direction = dirLogical ? 1 : -1;

  if (ax.DIR_INVERT) dirLogical = !dirLogical;
  digitalWrite(ax.DIR, dirLogical ? HIGH : LOW);

  holdEnable(ax, true);

  double anglePerStep = 360.0 / (double)ax.stepsPerRev;

  for (long i = 0; i < stepsToMove; i++) {
    stepOnePulse(ax, ax.speed_us);
    ax.currentAngle += (direction * anglePerStep);
    
    if (Serial.available()) {
       String s = Serial.peek() != -1 ? Serial.readStringUntil('\n') : "";
       if (s.indexOf("stop") >= 0) {
         Serial.println(F("Interrupted by stop"));
         break;
       }
    }

    reportStatus();
  }
  ax.currentAngle = targetAngle;
  reportStatus(true);
}