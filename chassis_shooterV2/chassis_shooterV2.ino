// chassis_shooter.ino
// 單一韌體：麥輪底盤 + 遙控器混控 (RC Mix) / 發射機構
//
// [RC 連線配置] Arduino MEGA 腳位：
//   - Pin 10 <--> 接收機 CH3 (控制 vx 前後)
//   - Pin 11 <--> 接收機 CH4 (控制 vy 左右)
//   - Pin 12 <--> 接收機 CH5 (控制 wz 旋轉)
//
// [RC 數值校正]
//   - 範圍: 1056 ~ 1916
//   - 中點: 1486 (實測 1484)
//   - 邏輯: 1916=+1.0, 1056=-1.0

#include <Arduino.h>
#include <math.h>
#include <Servo.h>

// ================== 模式定義 ==================
enum Mode { MODE_CHASSIS = 0, MODE_SHOOTER = 1 };
volatile Mode g_mode = MODE_CHASSIS;

// ================== RC 遙控器設定 (更新) ==================
const int PIN_RC_CH3 = 10; // 對應 vx
const int PIN_RC_CH4 = 11; // 對應 vy
const int PIN_RC_CH5 = 12; // 對應 wz

// 儲存來自 Serial 指令的速度
float serial_vx = 0;
float serial_vy = 0;
float serial_wz = 0;

// ★ 校正後的參數
const int RC_MIN = 1056;
const int RC_MAX = 1916;
const int RC_MID = 1486;    // 數學中點
const int RC_SWING = 430;   // 單邊擺幅
const int RC_DEADBAND = 40; // 死區 (容許 1446~1526 為 0)

// ================== CHASSIS（麥輪） ==================
const uint8_t M1_PWM_A_PIN = 2;  // 前左
const uint8_t M1_PWM_B_PIN = 3;
const uint8_t M2_PWM_A_PIN = 4;  // 左後
const uint8_t M2_PWM_B_PIN = 5;
const uint8_t M3_PWM_A_PIN = 7;  // 右前
const uint8_t M3_PWM_B_PIN = 6;
const uint8_t M4_PWM_A_PIN = 8;  // 右後
const uint8_t M4_PWM_B_PIN = 9;

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
const int SPARK_PWM_PIN = 13;
bool servoAttached = false;
int  neoSpeed = 0;

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
const unsigned long TELE_SHOOTER_MS = 400UL;

bool equalsIgnoreCase(const String& a, const char* b) {
  String t=a; t.toUpperCase(); String u=String(b); u.toUpperCase(); return t==u;
}

// ------------------ RC 讀取函式 ------------------
float readRCChannel(int pin) {
  unsigned long val = pulseIn(pin, HIGH, 25000); 
  
  if (val == 0) return 0.0;

  // 1. 死區處理
  if (val > (RC_MID - RC_DEADBAND) && val < (RC_MID + RC_DEADBAND)) {
    return 0.0;
  }

  // 2. 對應到 -1.0 ~ 1.0 (使用校正後的 RC_SWING)
  float result = (float)((long)val - RC_MID) / (float)RC_SWING; 
  
  // 3. 限制範圍
  if (result > 1.0) result = 1.0;
  if (result < -1.0) result = -1.0;
  
  return result;
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
  serial_vx = 0; serial_vy = 0; serial_wz = 0;
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
  if (g_mode != MODE_CHASSIS) return;
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
  if (!servoAttached) return;
  int us = map(speed, -100, 100, 1000, 2000);
  sparkMax.writeMicroseconds(us);
  Serial.print("NEO="); Serial.print(speed);
  Serial.print(" pwm(us)="); Serial.println(us);
}

void startForward(){ digitalWrite(DIR,HIGH); stepState=STEP_FWD; Serial.println("STEP=F"); }
void startReverse(){ digitalWrite(DIR,LOW);  stepState=STEP_REV; Serial.println("STEP=R"); }
void stopStepper(){ stepState=STEP_STOP; pulseLevel=false; digitalWrite(PUL,LOW); digitalWrite(ENA,HIGH); Serial.println("STEP=S (hold)"); }

void serviceStepper(){
  if (g_mode != MODE_SHOOTER) return;
  if (stepState == STEP_STOP) return;
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
  stopStepper();
  ensureServo(false);
  setSparkSpeed(0);
  stopAllChassis(); 
  g_mode = MODE_CHASSIS;
  lastTelemetryMs = millis();
  Serial.println(">> MODE=CHASSIS");
}

void enterShooterMode(){
  stopAllChassis();
  ensureServo(true);
  digitalWrite(ENA,HIGH);
  g_mode = MODE_SHOOTER;
  lastTelemetryMs = millis();
  Serial.println(">> MODE=SHOOTER");
}

// ------------------ 狀態印出 ------------------
void print_status(){
  Serial.println("=== STATUS ===");
  Serial.print("MODE: "); Serial.println(g_mode==MODE_CHASSIS?"CHASSIS":"SHOOTER");
  if (g_mode==MODE_CHASSIS){
    Serial.print("Serial Cmd: "); 
    Serial.print(serial_vx); Serial.print(",");
    Serial.print(serial_vy); Serial.print(",");
    Serial.println(serial_wz);
  }
  Serial.println("=============");
}

// ------------------ 指令處理 ------------------
void process_cmd_vel(String &msg){
  if (g_mode != MODE_CHASSIS) { Serial.println("IGNORED: vx,vy,wz (MODE!=CHASSIS)"); return; }
  char buf[128]; msg.toCharArray(buf,sizeof(buf));
  char *p=strtok(buf,","); float vx=0,vy=0,wz=0;
  if(p) vx=atof(p);
  p=strtok(NULL,","); if(p) vy=atof(p);
  p=strtok(NULL,","); if(p) wz=atof(p);

  serial_vx = vx;
  serial_vy = vy;
  serial_wz = wz;
}

void handle_line(String line){
  line.trim();
  if (!line.length()) return;

  const String LEGACY = "<11323310>";
  if (line.startsWith(LEGACY)) { line = line.substring(LEGACY.length()); line.trim(); }

  if (line.equalsIgnoreCase("MODE CHASSIS")) { enterChassisMode(); return; }
  if (line.equalsIgnoreCase("MODE SHOOTER")) { enterShooterMode(); return; }

  if (line.equalsIgnoreCase("STATUS")) { print_status(); return; }
  if (line.equalsIgnoreCase("STOP")) {
    if (g_mode==MODE_CHASSIS) stopAllChassis();
    else { stopStepper(); setSparkSpeed(0); if (servoAttached) sparkMax.writeMicroseconds(1500); }
    Serial.println("STOP (current mode)");
    return;
  }

  if (g_mode==MODE_CHASSIS){
    if (line.indexOf(',')>=0) { process_cmd_vel(line); return; }
    Serial.println("IGNORED (CHASSIS): not vx,vy,wz");
    return;
  } else {
    // SHOOTER
    if (equalsIgnoreCase(line,"F")) { startForward(); return; }
    if (equalsIgnoreCase(line,"R")) { startReverse(); return; }
    if (equalsIgnoreCase(line,"S")) { stopStepper();  return; }

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
  Serial.println("\n=== MEGA Multiplex (CHASSIS + RC / SHOOTER) ===");

  // 底盤 PWM
  pinMode(M1_PWM_A_PIN, OUTPUT); pinMode(M1_PWM_B_PIN, OUTPUT);
  pinMode(M2_PWM_A_PIN, OUTPUT); pinMode(M2_PWM_B_PIN, OUTPUT);
  pinMode(M3_PWM_A_PIN, OUTPUT); pinMode(M3_PWM_B_PIN, OUTPUT);
  pinMode(M4_PWM_A_PIN, OUTPUT); pinMode(M4_PWM_B_PIN, OUTPUT);

  // RC 輸入
  pinMode(PIN_RC_CH3, INPUT); // Pin 10 -> vx
  pinMode(PIN_RC_CH4, INPUT); // Pin 11 -> vy
  pinMode(PIN_RC_CH5, INPUT); // Pin 12 -> wz

  // 步進
  pinMode(PUL,OUTPUT); pinMode(DIR,OUTPUT); pinMode(ENA,OUTPUT);
  digitalWrite(PUL,LOW); digitalWrite(ENA,HIGH);

  ensureServo(false);
  enterChassisMode();

  Serial.println("Ready. Use AT10II: CH3=vx, CH4=vy, CH5=wz");
}

void loop(){
  // SHOOTER 模式
  if (g_mode == MODE_SHOOTER) {
    serviceStepper();
  }

  // Serial 指令
  while (Serial.available()){
    char c=(char)Serial.read(); if (c=='\r') continue;
    if (c=='\n'){ String line=serialBuf; serialBuf=""; handle_line(line); }
    else {
      serialBuf += c;
      if (serialBuf.length()>200) serialBuf=serialBuf.substring(serialBuf.length()-200);
    }
  }

  // CHASSIS 模式：Serial + RC 混控
  if (g_mode == MODE_CHASSIS) {
    // 1. 讀取 RC (CH3=vx, CH4=vy, CH5=wz)
    float rc_vx = readRCChannel(PIN_RC_CH3); // 前後
    float rc_vy = readRCChannel(PIN_RC_CH4); // 左右
    float rc_wz = readRCChannel(PIN_RC_CH5); // 旋轉

    // 2. 混合
    float total_vx = serial_vx + rc_vx;
    float total_vy = serial_vy + rc_vy;
    float total_wz = serial_wz + rc_wz;

    // 3. 限制 -1 ~ 1
    if(total_vx > 1.0) total_vx = 1.0; if(total_vx < -1.0) total_vx = -1.0;
    if(total_vy > 1.0) total_vy = 1.0; if(total_vy < -1.0) total_vy = -1.0;
    if(total_wz > 1.0) total_wz = 1.0; if(total_wz < -1.0) total_wz = -1.0;

    // 4. 計算與應用
    float out[4]; 
    mecanum_from_cmd(total_vx, total_vy, total_wz, out); 
    applyWheelCommand(out[0], out[1], out[2], out[3]);

    // 5. Ramp 更新
    rampUpdateAndApply();
  }

  // Telemetry
  unsigned long now = millis();
  if (g_mode==MODE_CHASSIS){
    if (now - lastTelemetryMs >= TELE_CHASSIS_MS){ lastTelemetryMs = now; tele_chassis(); }
  } else {
    if (now - lastTelemetryMs >= TELE_SHOOTER_MS){ lastTelemetryMs = now; tele_shooter(); }
  }
}