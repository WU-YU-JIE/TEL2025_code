// chassis_shooter.ino
// 單一韌體，全時運作模式 (無 Mode 區分)
// 改良版：使用 Pin Change Interrupt 讀取 RC，解決步進馬達卡頓問題
// 
// 功能：
//   1. 麥輪底盤：全時接收遙控器 A8/A9/A10 (CH3/4/5) 控制 vx,vy,wz
//   2. 發射機構：全時接收遙控器 A12 (CH6) 控制 F/S/R 流程，A12 (CH11) 控制 Servo
//
// 遙控器映射 (R12DS)：
//   A8 (CH3)  -> vx (1920->1, 1056->-1, deadband 0.1)
//   A9 (CH4)  -> vy (1920->1, 1056->-1, deadband 0.1)
//   A10 (CH5) -> wz (1920->1, 1056->-1, deadband 0.3)
//   A12 (CH11)-> Servo S2/S3 (1900->S3, 1056->S2) CH7跟CH11交換了
//   A11 (CH6) -> Function F/S/R (1920->R, 1488->S, 1056->F)

#include <Arduino.h>
#include <math.h>
#include <Servo.h>

// ================== 遙控器定義 & 中斷變數 ==================
// 使用 Pin Change Interrupt (PCINT2) 讀取 A8-A12 (Port K)
const int PIN_RC_VX   = A8;   // PK0 (PCINT16)
const int PIN_RC_VY   = A9;   // PK1 (PCINT17)
const int PIN_RC_WZ   = A10;  // PK2 (PCINT18)
const int PIN_RC_SERVO= A12;  // PK3 (PCINT19)
const int PIN_RC_FUNC = A11;  // PK4 (PCINT20)

// 中斷用全域變數
volatile uint16_t rc_raw_values[5] = {0}; // 0:VX, 1:VY, 2:WZ, 3:Servo, 4:Func
unsigned long rc_rise_times[5] = {0};
volatile uint8_t last_port_k = 0;

const int RC_MAX = 1920;
const int RC_MIN = 1056;
const int RC_MID = 1488;

// 誤差範圍定義
const float DB_VX = 0.1;
const float DB_VY = 0.1;
const float DB_WZ = 0.3;

// ================== 角度 Servo（pin10） ==================
Servo servoAngle;   // 專門控制 pin10 的 Servo（S2/S3）

// ================== SHOOTER 狀態機定義 ==================
enum ShooterLogicState {
  ST_IDLE,        // 停止狀態 (S)
  ST_FIRE_PREP,   // F指令階段1: NEO -50, 等待 0.5s
  ST_FIRING,      // F指令階段2: NEO -100, 步進前進
  ST_STOP_PREP,   // S指令階段1: NEO -50, 等待 0.5s (從 -100 降下來時)
  ST_REV_SLOWDOWN,// R指令階段1: NEO -50, 等待 0.5s (從 Firing 降下來時)
  ST_REVERSING    // R指令階段2: NEO 20, 步進反轉
};
ShooterLogicState currentShooterState = ST_IDLE;
unsigned long stateTimerMs = 0; // 用於計時 0.5 秒

// ================== CHASSIS（麥輪） ==================
// Pins (MEGA)
const uint8_t M1_PWM_A_PIN = 2;  // 前左 index 0
const uint8_t M1_PWM_B_PIN = 3;
const uint8_t M2_PWM_A_PIN = 4;  // 左後 index 1
const uint8_t M2_PWM_B_PIN = 5;
const uint8_t M3_PWM_A_PIN = 7;  // 右前 index 2
const uint8_t M3_PWM_B_PIN = 6;
const uint8_t M4_PWM_A_PIN = 8;  // 右後 index 3
const uint8_t M4_PWM_B_PIN = 9;
const uint8_t servo = 10;

const int   MIN_PWM   = 80;
const int   MAX_PWM   = 255;
const float DEAD_BAND = 0.02;
const float K_OMEGA   = 0.5;

float currentPWMvals[4] = {0,0,0,0};
float targetPWMvals[4]  = {0,0,0,0};

// ================== SHOOTER（NEO + 步進） ==================
Servo sparkMax;
const int SPARK_PWM_PIN = 11;     // NEO (Spark MAX) PWM
int  neoSpeed = 0;                // -100..100（僅作狀態記錄）

const int PUL = 53;
const int DIR = 52;
const int ENA = 51;
const unsigned int PULSE_US = 25;

enum StepState { STEP_STOP=0, STEP_FWD, STEP_REV };
volatile StepState stepState = STEP_STOP;
unsigned long lastToggleMicros = 0;
bool pulseLevel = false;

// ================== 共用 ==================
String serialBuf = "";
const unsigned long TELE_INTERVAL_MS = 250UL; 
const unsigned long LOGIC_INTERVAL_MS = 20UL; // 邏輯更新頻率 (50Hz)
unsigned long lastLogicMs = 0;

bool equalsIgnoreCase(const String& a, const char* b) {
  String t=a; t.toUpperCase(); String u=String(b); u.toUpperCase(); return t==u;
}

// ------------------ 中斷服務常式 (ISR) ------------------
// 當 Port K (A8-A15) 的腳位狀態改變時觸發
ISR(PCINT2_vect) {
  unsigned long now = micros();
  uint8_t curr = PINK; // 讀取 Port K 輸入暫存器
  uint8_t change = curr ^ last_port_k;
  last_port_k = curr;

  // 檢查 A8(bit0) ~ A12(bit4)
  for(int i=0; i<5; i++) {
     if (change & (1<<i)) { // 若該腳位發生變化
        if (curr & (1<<i)) {
           // 上升緣 (Rise)
           rc_rise_times[i] = now;
        } else {
           // 下降緣 (Fall)，計算脈寬
           // 簡單過濾異常短或長的脈衝 (例如 <500 或 >2500) 可在此處做，或後續處理
           rc_raw_values[i] = (uint16_t)(now - rc_rise_times[i]);
        }
     }
  }
}

// 輔助：原子性讀取 RC 數值
uint16_t getRawRC(int ch_idx) {
  noInterrupts();
  uint16_t val = rc_raw_values[ch_idx];
  interrupts();
  return val;
}

// 輔助：數值映射
float mapRC(uint16_t raw, float deadband) {
  if (raw < 800 || raw > 2200) return 0.0; // 濾除無效訊號
  long val = constrain(raw, RC_MIN, RC_MAX);
  float mapped = map(val, RC_MIN, RC_MAX, -1000, 1000) / 1000.0;
  if (fabs(mapped) < deadband) return 0.0;
  return mapped;
}

// ------------------ CHASSIS 函式 ------------------
void applyMotorDirect(int idx, int pwm_abs, bool forward) {
  switch(idx){
    case 0: if(forward){ analogWrite(M1_PWM_A_PIN,pwm_abs); analogWrite(M1_PWM_B_PIN,0);} else { analogWrite(M1_PWM_A_PIN,0); analogWrite(M1_PWM_B_PIN,pwm_abs);} break;
    case 1: if(forward){ analogWrite(M2_PWM_A_PIN,pwm_abs); analogWrite(M2_PWM_B_PIN,0);} else { analogWrite(M2_PWM_A_PIN,0); analogWrite(M2_PWM_B_PIN,pwm_abs);} break;
    case 2: if(forward){ analogWrite(M3_PWM_A_PIN,pwm_abs); analogWrite(M3_PWM_B_PIN,0);} else { analogWrite(M3_PWM_A_PIN,0); analogWrite(M3_PWM_B_PIN,pwm_abs);} break;
    case 3: if(forward){ analogWrite(M4_PWM_A_PIN,pwm_abs); analogWrite(M4_PWM_B_PIN,0);} else { analogWrite(M4_PWM_A_PIN,0); analogWrite(M4_PWM_B_PIN,pwm_abs);} break;
  }
}

void applyWheelCommand(float w0,float w1,float w2,float w3){
  float a[4]={w0,w1,w2,w3}; float maxv=0.0001f;
  // 固定反向（依你的接線）：前左、右前反向
  a[0]=-a[0]; a[2]=-a[2];
  for(int i=0;i<4;i++){ if(fabs(a[i])<DEAD_BAND) a[i]=0; if(fabs(a[i])>maxv) maxv=fabs(a[i]); }
  if(maxv>1.0f) for(int i=0;i<4;i++) a[i]/=maxv;
  for(int i=0;i<4;i++){
    float mag=fabs(a[i]);
    if(mag<DEAD_BAND) targetPWMvals[i]=0;
    else {
      float pwm=MIN_PWM+(MAX_PWM-MIN_PWM)*mag;
      targetPWMvals[i]=(a[i]>=0)?pwm:-pwm;
    }
  }
}

void rampUpdateAndApply(){
  for(int i=0;i<4;i++){
    // 直接套用目標值 (無加減速)
    currentPWMvals[i] = targetPWMvals[i];
    float cur = currentPWMvals[i];

    if (fabs(cur)<1.0){
      switch(i){
        case 0: analogWrite(M1_PWM_A_PIN,0); analogWrite(M1_PWM_B_PIN,0); break;
        case 1: analogWrite(M2_PWM_A_PIN,0); analogWrite(M2_PWM_B_PIN,0); break;
        case 2: analogWrite(M3_PWM_A_PIN,0); analogWrite(M3_PWM_B_PIN,0); break;
        case 3: analogWrite(M4_PWM_A_PIN,0); analogWrite(M4_PWM_B_PIN,0); break;
      }
    }else{
      applyMotorDirect(i,(int)fabs(cur),(cur>0));
    }
  }
}

void mecanum_from_cmd(float vx,float vy,float wz,float out[4]){
  // 機體朝向補償：等效順時針 +90° → (vx_r,vy_r)=(-vy, vx)
  float vx_r=-vy, vy_r=vx;
  float wFL=vx_r - vy_r - K_OMEGA*wz;
  float wFR=vx_r + vy_r + K_OMEGA*wz;
  float wRL=vx_r + vy_r - K_OMEGA*wz;
  float wRR=vx_r - vy_r + K_OMEGA*wz;
  float m=max(max(fabs(wFL),fabs(wFR)),max(fabs(wRL),fabs(wRR)));
  if(m>1.0f){ wFL/=m; wFR/=m; wRL/=m; wRR/=m; }
  out[0]=wFL; out[1]=wFR; out[2]=wRL; out[3]=wRR;
}

// ------------------ SHOOTER 函式 ------------------
void setSparkSpeed(int speed){
  neoSpeed = speed;
  int us = map(speed, -100, 100, 1000, 2000);
  sparkMax.writeMicroseconds(us);
}

// 步進方向反轉 (F=LOW, R=HIGH)
void startForward(){ digitalWrite(DIR,LOW); stepState=STEP_FWD; }
void startReverse(){ digitalWrite(DIR,HIGH);  stepState=STEP_REV; }
void stopStepper(){ stepState=STEP_STOP; pulseLevel=false; digitalWrite(PUL,LOW); digitalWrite(ENA,HIGH); }

void serviceStepper(){
  if (stepState == STEP_STOP) return;           // 停止狀態不占用 CPU
  unsigned long now=micros();
  if ((unsigned long)(now - lastToggleMicros) >= PULSE_US){
    pulseLevel=!pulseLevel;
    digitalWrite(PUL, pulseLevel?HIGH:LOW);
    lastToggleMicros=now;
  }
}

// ------------------ 邏輯控制 ------------------

// 更新 Shooter 狀態機
void updateShooterLogic(int cmdVal) {
  // cmdVal: 0=Stop, 1=Forward, -1=Reverse
  unsigned long now = millis();

  switch (cmdVal) {
    // === 指令 F (Forward/Fire) ===
    case 1: 
      if (currentShooterState != ST_FIRE_PREP && currentShooterState != ST_FIRING) {
        setSparkSpeed(-50);
        stateTimerMs = now;
        currentShooterState = ST_FIRE_PREP;
        stopStepper(); 
      }
      else if (currentShooterState == ST_FIRE_PREP) {
        if (now - stateTimerMs >= 500) {
          setSparkSpeed(-100);
          startForward();
          currentShooterState = ST_FIRING;
          Serial.println("STATE: FIRING");
        }
      }
      break;

    // === 指令 S (Stop) ===
    case 0:
      if (currentShooterState == ST_FIRING) {
         setSparkSpeed(-50);
         stateTimerMs = now;
         currentShooterState = ST_STOP_PREP;
      }
      else if (currentShooterState == ST_STOP_PREP) {
        if (now - stateTimerMs >= 500) {
          setSparkSpeed(0);
          stopStepper();
          currentShooterState = ST_IDLE;
          Serial.println("STATE: STOPPED");
        }
      }
      else if (currentShooterState != ST_IDLE) {
          setSparkSpeed(0);
          stopStepper();
          currentShooterState = ST_IDLE;
          Serial.println("STATE: STOP (Direct)");
      }
      break;

    // === 指令 R (Reverse/Clear) ===
    case -1:
      // 條件1: 若正在全速開火 (-100)，先降速
      if (currentShooterState == ST_FIRING) {
          setSparkSpeed(-50);
          stopStepper(); 
          stateTimerMs = now;
          currentShooterState = ST_REV_SLOWDOWN; 
          Serial.println("STATE: REV SLOWDOWN");
      }
      // 檢查降速等待時間
      else if (currentShooterState == ST_REV_SLOWDOWN) {
         if (now - stateTimerMs >= 500) {
            // 條件2: 降速完成，執行 NEO 20 並反轉
            setSparkSpeed(20);
            startReverse();
            currentShooterState = ST_REVERSING;
            Serial.println("STATE: REV (NEO 20)");
         }
      }
      // 條件3: 否則 (非開火狀態)，直接設 NEO 20 並反轉
      else if (currentShooterState != ST_REVERSING) {
          setSparkSpeed(20);
          startReverse();
          currentShooterState = ST_REVERSING;
          Serial.println("STATE: REV (Direct)");
      }
      break;
  }
}

void process_control_logic() {
  // 這裡不再使用 pulseIn，直接讀取中斷捕捉到的值，速度極快
  
  // --- 底盤控制 (Map index: 0=VX, 1=VY, 2=WZ) ---
  float vx = mapRC(getRawRC(0), DB_VX);
  float vy = mapRC(getRawRC(1), DB_VY);
  float wz = mapRC(getRawRC(2), DB_WZ);
  
  float out[4]; 
  mecanum_from_cmd(vx, vy, wz, out); 
  applyWheelCommand(out[0], out[1], out[2], out[3]);

  // --- 射擊控制 ---
  // A. Servo (Map index: 3)
  uint16_t sVal = getRawRC(4);
  if (sVal > 1500) { 
      if(servoAngle.read() != 125) { servoAngle.write(125); Serial.println("SERVO=125 (S3)"); }
  } else if (sVal > 900) { 
      if(servoAngle.read() != 90) { servoAngle.write(90); Serial.println("SERVO=90 (S2)"); }
  }

  // B. Function F/S/R (Map index: 4) - 1920->R, 1056->F
  uint16_t fVal = getRawRC(3);
  int cmd = 0; 
  if (fVal > 1700) cmd = -1;      // > 1700 -> R
  else if (fVal < 1200 && fVal > 500) cmd = 1; // < 1200 -> F
  else cmd = 0;                   // -> S
  
  // 更新狀態機
  updateShooterLogic(cmd);
}

// ------------------ Telemetry ------------------
void print_telemetry(){
  Serial.print("TELE: PWM[");
  Serial.print(currentPWMvals[0],0); Serial.print(",");
  Serial.print(currentPWMvals[1],0); Serial.print(",");
  Serial.print(currentPWMvals[2],0); Serial.print(",");
  Serial.print(currentPWMvals[3],0); Serial.print("]");
  
  Serial.print(" SH: NEO="); Serial.print(neoSpeed);
  Serial.print(" STEP=");
  Serial.print(stepState==STEP_STOP?"S":(stepState==STEP_FWD?"F":"R"));
  Serial.print(" ST="); Serial.println(currentShooterState);
  
  // 測試用：印出 RC 原始值，檢查中斷是否正常
  // Serial.print(" RC: "); Serial.print(getRawRC(0)); Serial.print(" "); Serial.println(getRawRC(4));
}

// ------------------ 指令處理 (僅保留手動除錯) ------------------
void handle_line(String line){
  line.trim();
  if (!line.length()) return;

  const String LEGACY = "<11323310>";
  if (line.startsWith(LEGACY)) { line = line.substring(LEGACY.length()); line.trim(); }

  // 僅保留射擊測試指令
  if (equalsIgnoreCase(line,"F")) { updateShooterLogic(1); return; }
  if (equalsIgnoreCase(line,"S")) { updateShooterLogic(0); return; }
  if (equalsIgnoreCase(line,"R")) { updateShooterLogic(-1); return; }

  long spd = line.toInt();
  if (spd>100) spd=100; if (spd<-100) spd=-100;
  if (spd != 0 || line.equals("0")) { 
      setSparkSpeed((int)spd);
  }
}

// ================== setup / loop ==================
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== MEGA Integrated Chassis & Shooter (ISR RC) ===");

  // 設定 RC 輸入 (宣告為輸入，雖然中斷不需要 pullup，但習慣上設定)
  pinMode(PIN_RC_VX, INPUT);
  pinMode(PIN_RC_VY, INPUT);
  pinMode(PIN_RC_WZ, INPUT);
  pinMode(PIN_RC_SERVO, INPUT);
  pinMode(PIN_RC_FUNC, INPUT);

  // 啟用 Pin Change Interrupt (Port K / A8-A15)
  // PCIE2 對應 Port K
  PCICR |= (1 << PCIE2);
  // PCMSK2 設定哪些 pin 觸發 (PK0~PK4 -> A8~A12 -> PCINT16~PCINT20)
  PCMSK2 |= (1 << PCINT16) | (1 << PCINT17) | (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20);

  // 底盤腳位
  pinMode(M1_PWM_A_PIN, OUTPUT); pinMode(M1_PWM_B_PIN, OUTPUT);
  pinMode(M2_PWM_A_PIN, OUTPUT); pinMode(M2_PWM_B_PIN, OUTPUT);
  pinMode(M3_PWM_A_PIN, OUTPUT); pinMode(M3_PWM_B_PIN, OUTPUT);
  pinMode(M4_PWM_A_PIN, OUTPUT); pinMode(M4_PWM_B_PIN, OUTPUT);
  pinMode(servo, OUTPUT);

  // 步進腳位
  pinMode(PUL,OUTPUT); pinMode(DIR,OUTPUT); pinMode(ENA,OUTPUT);
  digitalWrite(PUL,LOW); digitalWrite(ENA,HIGH); // 先抱死

  // 啟動發射系統 (SparkMAX)
  sparkMax.attach(SPARK_PWM_PIN);
  setSparkSpeed(0);

  // 啟動 Servo
  servoAngle.attach(servo);   
  servoAngle.write(90);       

  Serial.println("System Ready. RC via Interrupts.");
}

void loop(){
  // 1. 服務步進馬達 (最高優先級，無條件執行)
  // 由於移除了 blocking pulseIn，這裡的執行頻率會非常高
  serviceStepper();

  // 2. 處理 RC 邏輯 (使用 Timer 控制更新率，避免過度計算)
  if (millis() - lastLogicMs >= LOGIC_INTERVAL_MS) {
    lastLogicMs = millis();
    process_control_logic();
    
    // 底盤 PWM 更新也放在這裡，不需要每微秒更新
    rampUpdateAndApply();
  }

  // 3. 收 Serial 指令 (除錯用)
  while (Serial.available()){
    char c=(char)Serial.read(); if (c=='\r') continue;
    if (c=='\n'){ String line=serialBuf; serialBuf=""; handle_line(line); }
    else {
      serialBuf += c;
      if (serialBuf.length()>200) serialBuf=serialBuf.substring(serialBuf.length()-200);
    }
  }

  // 5. Telemetry
  static unsigned long lastTele = 0;
  if (millis() - lastTele >= TELE_INTERVAL_MS){ 
    lastTele = millis(); 
    print_telemetry(); 
  }
}