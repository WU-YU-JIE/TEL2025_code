/*******************************************************
 * Arduino：雙軸步進（只支援 1,角度 / 2,角度）
 * M1: 42667 步/圈
 * M2: 57600 步/圈
 * 範例：1,45.0   或   2,90
 *******************************************************/
#include <Arduino.h>

struct StepperAxis {
  int PUL, DIR, ENA;
  bool DIR_INVERT;        // true=反向（若方向顛倒，改這個）
  bool ENA_ACTIVE_HIGH;   // true=HIGH為使能
  float currentAngle;     // 目前角度(度)
  long stepsPerRev;       // 每圈步數(含細分/減速)
  int speed_us;           // 半脈衝延遲(μs)，越小越快
};

// === 腳位（依你前兩份程式） ===
const int M1_PUL = 13, M1_DIR = 12, M1_ENA = 11;  // 軸1
const int M2_PUL = 10, M2_DIR = 9,  M2_ENA = 8;   // 軸2

// === 每圈步數（已依你提供設定） ===
const long M1_STEPS_PER_REV = 42667L;
const long M2_STEPS_PER_REV = 57600L;

// === 參數 ===
StepperAxis m1 = { M1_PUL, M1_DIR, M1_ENA, true,  true, 0.0, M1_STEPS_PER_REV, 25 };
StepperAxis m2 = { M2_PUL, M2_DIR, M2_ENA, true,  true, 0.0, M2_STEPS_PER_REV, 25 };

static inline void holdEnable(const StepperAxis& ax, bool on) {
  digitalWrite(ax.ENA, on ? (ax.ENA_ACTIVE_HIGH ? HIGH : LOW)
                          : (ax.ENA_ACTIVE_HIGH ? LOW  : HIGH));
}

void setupAxis(const StepperAxis& ax) {
  pinMode(ax.PUL, OUTPUT);
  pinMode(ax.DIR, OUTPUT);
  pinMode(ax.ENA, OUTPUT);
  holdEnable(ax, true); // 使能保持鎖住
}

void setup() {
  Serial.begin(115200);
  setupAxis(m1);
  setupAxis(m2);
  Serial.println(F("就緒：只接受 1,角度 或 2,角度"));
  Serial.println(F("M1=42667 步/圈, M2=57600 步/圈"));
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (!cmd.length()) return;

  int comma = cmd.indexOf(',');
  if (comma <= 0) {
    Serial.println(F("❌ 格式錯誤：請輸入 1,角度 或 2,角度"));
    return;
  }

  int axisId = cmd.substring(0, comma).toInt();
  float targetAngle = cmd.substring(comma + 1).toFloat();

  if (axisId == 1) {
    moveToAngle(m1, targetAngle);
    Serial.println(F("M1 OK"));
  } else if (axisId == 2) {
    moveToAngle(m2, targetAngle);
    Serial.println(F("M2 OK"));
  } else {
    Serial.println(F("❌ 馬達編號只接受 1 或 2"));
  }
}

void moveToAngle(StepperAxis& ax, float targetAngle) {
  float diff = targetAngle - ax.currentAngle;
  // 自行四捨五入以避免 lround 帶來的額外體積
  long stepsToMove = (long)(fabs(diff) * ((double)ax.stepsPerRev / 360.0) + 0.5);
  if (stepsToMove <= 0) { ax.currentAngle = targetAngle; return; }

  bool dirLogical = (diff >= 0);
  if (ax.DIR_INVERT) dirLogical = !dirLogical;
  digitalWrite(ax.DIR, dirLogical ? HIGH : LOW);

  holdEnable(ax, true); // 確保保持

  for (long i = 0; i < stepsToMove; i++) {
    digitalWrite(ax.PUL, HIGH);
    delayMicroseconds(ax.speed_us);
    digitalWrite(ax.PUL, LOW);
    delayMicroseconds(ax.speed_us);
  }
  ax.currentAngle = targetAngle;
}
