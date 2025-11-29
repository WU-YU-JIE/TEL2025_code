  /*******************************************************
  * Arduino：雙軸步進控制 (完整版)
  * * [M1 配置]
  * - 模式: 開環 (Open-Loop)
  * - 功能: 絕對定位 + 連續旋轉 (left/right)
  * * [M2 配置]
  * - 模式: 閉環 (Closed-Loop via AS5600)
  * - 硬體: 馬達細分 6400, 減速比 9:1
  * - 邏輯: 讀取馬達端 Encoder，除以 9 換算末端角度
  * - 優化: 動態步數批次處理 (Adaptive Step Batching)
  * * [指令格式]
  * 1. M1定位: "1,90"
  * 2. M2定位: "2,90"
  * 3. M1連續: "left,50" 或 "right,50"
  * 4. 停止: "stop"
  * 5. 狀態: "p"
  *******************************************************/

  #include <Arduino.h>
  #include <Wire.h>

  // === AS5600 設定 ===
  const uint8_t AS5600_ADDR = 0x36;
  const uint8_t ANGLE_MSB_REG = 0x0E;
  uint16_t as5600_last_raw = 0;     
  long long as5600_total_steps = 0; 
  const float AS5600_TOLERANCE = 0.4; // 到位容許誤差 (度)

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
  const long M2_STEPS_PER_REV = 57600L; // 6400 * 9 = 57600
  const float GEAR_RATIO = 9.0;         // M2 減速比 9:1

  // 限制範圍 (僅供參考，主要由使用者控制)
  const float LIMIT_MAX = 360.0;
  const float LIMIT_MIN = -360.0;

  // 物件初始化
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

    initAS5600(); 

    Serial.println(F("系統就緒 (完整版)"));
    Serial.print(F("M2 Gear Ratio: ")); Serial.println(GEAR_RATIO);
  }

  // ==========================================
  // Loop
  // ==========================================
  void loop() {
    // 持續更新 M2 角度 (即使沒動也要讀，確保 p 指令準確)
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

    // 3. M1 連續旋轉模式
    if (cmd.startsWith("left") || cmd.startsWith("right")) {
      int comma = cmd.indexOf(',');
      int spd = 50; 
      if (comma > 0) spd = cmd.substring(comma + 1).toInt();
      
      // left = -1 (反轉), right = 1 (正轉)
      int dir = cmd.startsWith("right") ? 1 : -1;
      
      runContinuousM1(dir, spd);
      return;
    }

    // 4. 絕對定位模式
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

    // 計算每度對應多少步 (用於動態調速估算)
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

      // 6. 動態步數調配 (Adaptive Step Batching)
      // 根據誤差大小決定一次跑幾步才讀 I2C
      long stepsNeeded = (long)(abs(error) * stepsPerDeg);
      int batchSize = 1;

      if (stepsNeeded > 1600) batchSize = 400;      // 誤差大: 快速衝
      else if (stepsNeeded > 320) batchSize = 100;  
      else if (stepsNeeded > 80) batchSize = 20;    
      else batchSize = 2;                           // 接近時: 精細調

      // 執行批次步進
      for(int i=0; i<batchSize; i++) {
          stepOnePulse(m2, m2.speed_us);
      }
      
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
      
      // 檢查停止指令
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
          
          // 重要：轉 M1 時也要監控 M2 角度
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
      // 1. 計算 Sensor 累積角度
      double sensor_angle = (double)as5600_total_steps * (360.0 / 4096.0);
      // 2. 除以減速比，得到末端角度
      return sensor_angle / GEAR_RATIO;
  }

  void updateAS5600() {
    uint16_t raw = readRawAngleAS5600();
    int delta = int(raw) - int(as5600_last_raw);
    
    // 多圈過零處理
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
    // 快速 timeout (500us)
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
    // === 修改這裡 ===
    // 原本是: Serial.print("Output_Deg:"); 
    Serial.print("degree:");  // 改成這樣以匹配 Python 端
    // =================
    
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