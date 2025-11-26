// chassis_shooter.ino
// 單一韌體，板上分流兩種模式以降低負荷：
//   模式 CHASSIS：只跑麥輪底盤（vx,vy,wz）
//   模式 SHOOTER：只跑 NEO + 步進（整數 -100..100, F/R/S）
//
// 串列指令 (無 TOKEN；相容舊有 <11323310> 前綴會自動剝除) ：
//   MODE CHASSIS          切換到底盤模式（並關閉發射模組）
//   MODE SHOOTER          切換到發射模式（並關閉底盤）
//   STATUS                顯示目前模式與該模式的狀態
//   STOP                  依模式停當前子系統：CHASSIS 停車；SHOOTER 停步進+NEO=0
//   vx,vy,wz              僅 CHASSIS 模式有效 (範圍建議 -1..1)
//   F / R / S             僅 SHOOTER 模式有效（步進 前/反/停(抱死)）
//   -100..100             僅 SHOOTER 模式有效（NEO 速度）
//
// 新增：
//   S1 / S2 / S3          僅 SHOOTER 模式有效（pin10 Servo 轉到 60 / 90 / 120 度）
//
// 降負荷重點：
//  - 僅在 CHASSIS 模式執行 rampUpdateAndApply() 與底盤 telemetry
//  - 僅在 SHOOTER 模式執行 serviceStepper()（步進脈衝）與 NEO Servo（attach）
//  - 切到 CHASSIS 會 detach Servo（釋放計時器中斷），步進 ENA=HIGH（抱死、停脈衝）
//  - 切到 SHOOTER 會 stopAll() 關閉所有底盤 PWM
//  - Telemetry 僅輸出當前模式的必要資訊

#include <Arduino.h>
#include <math.h>
#include <Servo.h>

// ================== 角度 Servo（pin10，新增） ==================
Servo servoAngle;   // 專門控制 pin10 的 Servo（S1/S2/S3）

// ================== 模式定義 ==================
enum Mode { MODE_CHASSIS = 0, MODE_SHOOTER = 1 };
volatile Mode g_mode = MODE_CHASSIS;

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
const float RAMP_STEP = 6.0;
const float DEAD_BAND = 0.02;
const float K_OMEGA   = 0.5;

float currentPWMvals[4] = {0,0,0,0};
float targetPWMvals[4]  = {0,0,0,0};
bool  motorActive       = false;

// ================== SHOOTER（NEO + 步進） ==================
Servo sparkMax;
const int SPARK_PWM_PIN = 13;     // NEO (Spark MAX) PWM
bool servoAttached = false;
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
unsigned long lastTelemetryMs = 0;
const unsigned long TELE_CHASSIS_MS = 200UL;
const unsigned long TELE_SHOOTER_MS = 400UL; // 發射端資訊較少，頻率可放慢
unsigned long lastTeleBudgetMs = 0;

bool equalsIgnoreCase(const String& a, const char* b) {
  String t=a; t.toUpperCase(); String u=String(b); u.toUpperCase(); return t==u;
}

// ------------------ CHASSIS 函式 ------------------
void stopAllChassis() {
  analogWrite(M1_PWM_A_PIN,0); analogWrite(M1_PWM_B_PIN,0);
  analogWrite(M2_PWM_A_PIN,0); analogWrite(M2_PWM_B_PIN,0);
  analogWrite(M3_PWM_A_PIN,0); analogWrite(M3_PWM_B_PIN,0);
  analogWrite(M4_PWM_A_PIN,0); analogWrite(M4_PWM_B_PIN,0);
  for (int i=0;i<4;i++){ currentPWMvals[i]=0; targetPWMvals[i]=0; }
  motorActive=false;
  Serial.println("CHASSIS stop");
}

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
  motorActive=true;
}

void rampUpdateAndApply(){
  if (g_mode != MODE_CHASSIS) return; // 非底盤模式，不做任何 PWM 更新
  for(int i=0;i<4;i++){
    float cur=currentPWMvals[i], tgt=targetPWMvals[i];
    if (fabs(tgt-cur)<=RAMP_STEP) cur=tgt; else cur += (tgt>cur)?RAMP_STEP:-RAMP_STEP;
    currentPWMvals[i]=cur;
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
void ensureServo(bool wantAttach){
  if (wantAttach && !servoAttached){
    sparkMax.attach(SPARK_PWM_PIN);
    servoAttached = true;
  } else if (!wantAttach && servoAttached){
    sparkMax.detach();
    servoAttached = false;
  }
}

void setSparkSpeed(int speed){
  neoSpeed = speed;
  if (!servoAttached) return; // 非 SHOOTER 模式時不送脈衝
  int us = map(speed, -100, 100, 1000, 2000);
  sparkMax.writeMicroseconds(us);
  Serial.print("NEO="); Serial.print(speed);
  Serial.print(" pwm(us)="); Serial.println(us);
}

void startForward(){ digitalWrite(DIR,HIGH); stepState=STEP_FWD; Serial.println("STEP=F"); }
void startReverse(){ digitalWrite(DIR,LOW);  stepState=STEP_REV; Serial.println("STEP=R"); }
void stopStepper(){ stepState=STEP_STOP; pulseLevel=false; digitalWrite(PUL,LOW); digitalWrite(ENA,HIGH); Serial.println("STEP=S (hold)"); }

void serviceStepper(){
  if (g_mode != MODE_SHOOTER) return;           // 非發射模式，不產生脈衝
  if (stepState == STEP_STOP) return;           // 停止狀態不占用 CPU
  unsigned long now=micros();
  if ((unsigned long)(now - lastToggleMicros) >= PULSE_US){
    pulseLevel=!pulseLevel;
    digitalWrite(PUL, pulseLevel?HIGH:LOW);
    lastToggleMicros=now;
  }
}

// ------------------ Telemetry ------------------
void tele_chassis(){
  Serial.print("TELE-CH: cur[");
  for(int i=0;i<4;i++){ if(i) Serial.print(","); Serial.print(currentPWMvals[i],1); }
  Serial.print("] tgt[");
  for(int i=0;i<4;i++){ if(i) Serial.print(","); Serial.print(targetPWMvals[i],1); }
  Serial.println("]");
}
void tele_shooter(){
  Serial.print("TELE-SH: NEO="); Serial.print(neoSpeed);
  Serial.print(" STEP=");
  Serial.println(stepState==STEP_STOP?"STOP":(stepState==STEP_FWD?"FWD":"REV"));
}

// ------------------ 模式切換 ------------------
void enterChassisMode(){
  // 關閉發射子系統
  stopStepper();                 // ENA=HIGH 抱死
  ensureServo(false);            // detach Servo，釋放計時器中斷
  setSparkSpeed(0);              // 更新邏輯值，實際輸出因已 detach 不會送
  // 底盤安全歸零
  stopAllChassis();
  g_mode = MODE_CHASSIS;
  lastTelemetryMs = millis();
  Serial.println(">> MODE=CHASSIS");
}

void enterShooterMode(){
  // 關閉底盤輸出
  stopAllChassis();
  // 啟動發射子系統
  ensureServo(true);             // attach Servo
  digitalWrite(ENA,HIGH);        // 上電預設抱死
  g_mode = MODE_SHOOTER;
  lastTelemetryMs = millis();
  Serial.println(">> MODE=SHOOTER");
}

// ------------------ 狀態印出 ------------------
void print_status(){
  Serial.println("=== STATUS ===");
  Serial.print("MODE: "); Serial.println(g_mode==MODE_CHASSIS?"CHASSIS":"SHOOTER");
  if (g_mode==MODE_CHASSIS){
    Serial.print("motorActive: "); Serial.println(motorActive?"YES":"NO");
    for (int i=0;i<4;i++){
      Serial.print("cur[");Serial.print(i);Serial.print("]=");Serial.print(currentPWMvals[i],1);
      Serial.print(" tgt[");Serial.print(i);Serial.print("]=");Serial.println(targetPWMvals[i],1);
    }
  } else {
    Serial.print("NEO speed: "); Serial.println(neoSpeed);
    Serial.print("STEP state: "); Serial.println(stepState==STEP_STOP?"STOP":(stepState==STEP_FWD?"FWD":"REV"));
  }
  Serial.println("=============");
}

// ------------------ 指令處理 ------------------
void process_cmd_vel(String &msg){
  // 僅 CHASSIS 模式有效
  if (g_mode != MODE_CHASSIS) { Serial.println("IGNORED: vx,vy,wz (MODE!=CHASSIS)"); return; }
  char buf[128]; msg.toCharArray(buf,sizeof(buf));
  char *p=strtok(buf,","); float vx=0,vy=0,wz=0;
  if(p) vx=atof(p);
  p=strtok(NULL,","); if(p) vy=atof(p);
  p=strtok(NULL,","); if(p) wz=atof(p);
  float out[4]; mecanum_from_cmd(vx,vy,wz,out); applyWheelCommand(out[0],out[1],out[2],out[3]);
}

void handle_line(String line){
  line.trim();
  if (!line.length()) return;

  // 相容舊 TOKEN：若前綴 <11323310>，剝掉
  const String LEGACY = "<11323310>";
  if (line.startsWith(LEGACY)) { line = line.substring(LEGACY.length()); line.trim(); }

  // 模式切換
  if (line.equalsIgnoreCase("MODE CHASSIS")) { enterChassisMode(); return; }
  if (line.equalsIgnoreCase("MODE SHOOTER")) { enterShooterMode(); return; }

  // 通用
  if (line.equalsIgnoreCase("STATUS")) { print_status(); return; }
  if (line.equalsIgnoreCase("STOP")) {
    if (g_mode==MODE_CHASSIS) stopAllChassis();
    else { stopStepper(); setSparkSpeed(0); if (servoAttached) sparkMax.writeMicroseconds(1500); }
    Serial.println("STOP (current mode)");
    return;
  }

  // 依模式解譯
  if (g_mode==MODE_CHASSIS){
    // 只處理 vx,vy,wz
    if (line.indexOf(',')>=0) { process_cmd_vel(line); return; }
    Serial.println("IGNORED (CHASSIS): not vx,vy,wz");
    return;
  } else {
    // SHOOTER：F/R/S / S1/S2/S3 / 整數
    if (equalsIgnoreCase(line,"F")) { startForward(); return; }
    if (equalsIgnoreCase(line,"R")) { startReverse(); return; }
    if (equalsIgnoreCase(line,"S")) { stopStepper();  return; }

    // ★ 新增：S1 / S2 / S3 控制 pin10 的 Servo 角度
    if (equalsIgnoreCase(line,"S1")) { servoAngle.write(60);  Serial.println("SERVO=60");  return; }
    if (equalsIgnoreCase(line,"S2")) { servoAngle.write(90);  Serial.println("SERVO=90");  return; }
    if (equalsIgnoreCase(line,"S3")) { servoAngle.write(120); Serial.println("SERVO=120"); return; }

    // 其餘視為 NEO 速度
    long spd = line.toInt();
    if (spd>100) spd=100; if (spd<-100) spd=-100;
    setSparkSpeed((int)spd);
    return;
  }
}

// ================== setup / loop ==================
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== MEGA Multiplex (CHASSIS / SHOOTER) NoToken ===");

  // 底盤腳位
  pinMode(M1_PWM_A_PIN, OUTPUT); pinMode(M1_PWM_B_PIN, OUTPUT);
  pinMode(M2_PWM_A_PIN, OUTPUT); pinMode(M2_PWM_B_PIN, OUTPUT);
  pinMode(M3_PWM_A_PIN, OUTPUT); pinMode(M3_PWM_B_PIN, OUTPUT);
  pinMode(M4_PWM_A_PIN, OUTPUT); pinMode(M4_PWM_B_PIN, OUTPUT);
  pinMode(servo, OUTPUT);

  // 步進腳位
  pinMode(PUL,OUTPUT); pinMode(DIR,OUTPUT); pinMode(ENA,OUTPUT);
  digitalWrite(PUL,LOW); digitalWrite(ENA,HIGH); // 先抱死

  // 初始進 CHASSIS，並確保發射子系統關閉
  ensureServo(false);
  enterChassisMode();

  // ★ 新增：pin10 Servo 初始化
  servoAngle.attach(servo);   // 或直接寫 10
  servoAngle.write(90);       // 預設 90 度

  Serial.println("usage：MODE CHASSIS / MODE SHOOTER / STATUS / STOP / vx,vy,wz / F/R/S / S1/S2/S3 / -100..100");
}

void loop(){
  // 只跑當前模式的服務
  if (g_mode==MODE_CHASSIS){
    // 無需處理步進；NEO Servo 已 detach
  } else {
    // 發射模式：處理步進脈衝
    serviceStepper();
  }

  // 收指令（\n 為一行）
  while (Serial.available()){
    char c=(char)Serial.read(); if (c=='\r') continue;
    if (c=='\n'){ String line=serialBuf; serialBuf=""; handle_line(line); }
    else {
      serialBuf += c;
      if (serialBuf.length()>200) serialBuf=serialBuf.substring(serialBuf.length()-200);
    }
  }

  // 僅在 CHASSIS 模式更新 PWM
  if (g_mode==MODE_CHASSIS) rampUpdateAndApply();

  // Telemetry（各模式各自頻率）
  unsigned long now = millis();
  if (g_mode==MODE_CHASSIS){
    if (now - lastTelemetryMs >= TELE_CHASSIS_MS){ lastTelemetryMs = now; tele_chassis(); }
  } else {
    if (now - lastTelemetryMs >= TELE_SHOOTER_MS){ lastTelemetryMs = now; tele_shooter(); }
  }
}
