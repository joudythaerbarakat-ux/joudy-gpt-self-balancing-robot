/*
 * AI_ROBOT — ESP32 BLE/Serial RC + Self-Balance + MPU6050/MPU9250 + LCD Faces
 * ESP32 Arduino Core 3.x compatible
 *
 * Joudy GPT — Self-Balancing Robot
 * Presented at NICE 2026, Tishk International University
 *
 * Runtime tuning via BLE or Serial (no re-upload needed):
 * - MODE:RC / MODE:BALANCE
 * - CAL
 * - AXIS:X / AXIS:Y
 * - GYRO:X / GYRO:Y / GYRO:Z
 * - INV:0 / INV:1
 * - KP:12.0 / KI:0.0 / KD:0.8
 * - SP:0.0
 * - MINPWM:80
 * - DIAG:1 / DIAG:0
 *
 * RC commands: F, B, L, R, S
 * Face commands: FACE:IDLE/HAPPY/SAD/ANGRY/SURPRISED/TALKING/THINKING
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ─────────────────────────────────────────────────────────────
// BLE UUIDs
// ─────────────────────────────────────────────────────────────
#define SERVICE_UUID  "12345678-1234-1234-1234-123456789abc"
#define CHAR_UUID     "abcdefab-cdef-abcd-efab-cdefabcdefab"

// ─────────────────────────────────────────────────────────────
// MOTOR PINS — L298N
// ─────────────────────────────────────────────────────────────
#define MOTOR_A_IN1  14
#define MOTOR_A_IN2  27
#define MOTOR_A_ENA  12

#define MOTOR_B_IN3  13
#define MOTOR_B_IN4  15
#define MOTOR_B_ENB  33

// ─────────────────────────────────────────────────────────────
// PWM
// ─────────────────────────────────────────────────────────────
#define PWM_FREQ  1000
#define PWM_BITS  8

const uint8_t SPEED_TABLE[3] = {120, 185, 230};
uint8_t currentSpeed = SPEED_TABLE[1];
int MIN_MOTOR_PWM = 80;

// ─────────────────────────────────────────────────────────────
// LCD
// ─────────────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ─────────────────────────────────────────────────────────────
// MPU
// ─────────────────────────────────────────────────────────────
#define MPU_ADDR 0x68

// ─────────────────────────────────────────────────────────────
// ROBOT MODE
// ─────────────────────────────────────────────────────────────
enum RobotMode { MODE_RC, MODE_BALANCE };
RobotMode robotMode = MODE_RC;

enum TiltAxis { TILT_X, TILT_Y };
enum GyroAxis { GYRO_X, GYRO_Y, GYRO_Z };

TiltAxis tiltAxis = TILT_X;
GyroAxis gyroAxis = GYRO_X;

bool invertAngle   = false;
bool diagnosticMode = true;

// ─────────────────────────────────────────────────────────────
// KALMAN FILTER
// ─────────────────────────────────────────────────────────────
struct Kalman {
  float angle = 0.0f;
  float bias  = 0.0f;
  float P[2][2] = {{1,0},{0,1}};

  const float Q_angle = 0.001f;
  const float Q_bias  = 0.003f;
  const float R_meas  = 0.03f;

  float update(float newAngle, float newRate, float dt) {
    angle += dt * (newRate - bias);
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S  = P[0][0] + R_meas;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    float y  = newAngle - angle;

    angle += K0 * y;
    bias  += K1 * y;

    float P00t = P[0][0], P01t = P[0][1];
    P[0][0] -= K0 * P00t;
    P[0][1] -= K0 * P01t;
    P[1][0] -= K1 * P00t;
    P[1][1] -= K1 * P01t;

    return angle;
  }

  void reset(float startAngle = 0.0f) {
    angle = startAngle; bias = 0.0f;
    P[0][0]=1; P[0][1]=0; P[1][0]=0; P[1][1]=1;
  }
};

Kalman kalman;

// ─────────────────────────────────────────────────────────────
// PID
// ─────────────────────────────────────────────────────────────
float Kp = 12.0f, Ki = 0.0f, Kd = 0.8f;
float angle_setpoint = 0.0f, base_setpoint = 0.0f;
float integral = 0.0f, prev_error = 0.0f;
float accelAngleOffset = 0.0f, gyroOffset = 0.0f;

const float INTEGRAL_LIMIT = 80.0f;
const float TILT_CUTOUT    = 35.0f;

bool calibrated = false;

float balanceDriveTrim = 0.0f;
int   balanceTurnTrim  = 0;

const float BALANCE_FORWARD_SP  =  2.0f;
const float BALANCE_BACKWARD_SP = -2.0f;
const int   BALANCE_TURN_PWM    =  35;

// ─────────────────────────────────────────────────────────────
// DEBUG
// ─────────────────────────────────────────────────────────────
unsigned long lastDebugMs = 0;
const int DEBUG_INTERVAL  = 150;

// ─────────────────────────────────────────────────────────────
// BLE
// ─────────────────────────────────────────────────────────────
bool deviceConnected = false;
BLECharacteristic* pCharacteristic = nullptr;

// ─────────────────────────────────────────────────────────────
// LCD FACE
// ─────────────────────────────────────────────────────────────
enum FaceState { IDLE, HAPPY, SAD, ANGRY, SURPRISED, TALKING, THINKING };
FaceState currentFace = IDLE, pendingFace = IDLE;
bool faceChanged = false;

#define C_EYE_OPEN   0
#define C_EYE_HALF   1
#define C_EYE_SHUT   2
#define C_M_HAPPY    3
#define C_M_SAD      4
#define C_M_TALK     5
#define C_M_FLAT     6
#define C_EYEBROW_UP 7

byte EYES_OPEN[8]    = {0b00000,0b01110,0b11111,0b11011,0b11111,0b01110,0b00000,0b00000};
byte EYES_HALF[8]    = {0b00000,0b00000,0b01110,0b11111,0b11111,0b01110,0b00000,0b00000};
byte EYES_SHUT[8]    = {0b00000,0b00000,0b00000,0b11111,0b00000,0b00000,0b00000,0b00000};
byte MOUTH_HAPPY[8]  = {0b00000,0b00000,0b10001,0b01110,0b00000,0b00000,0b00000,0b00000};
byte MOUTH_SAD[8]    = {0b00000,0b00000,0b01110,0b10001,0b00000,0b00000,0b00000,0b00000};
byte MOUTH_TALK[8]   = {0b00000,0b01110,0b11111,0b01110,0b00000,0b00000,0b00000,0b00000};
byte MOUTH_FLAT[8]   = {0b00000,0b00000,0b11111,0b00000,0b00000,0b00000,0b00000,0b00000};
byte EYEBROW_UP[8]   = {0b11111,0b00000,0b01110,0b11111,0b11011,0b11111,0b01110,0b00000};

struct FaceBuffer { byte left,right,mouth; bool browLeft,browRight; };
FaceBuffer displayedFace = {255,255,255,false,false};

// ─────────────────────────────────────────────────────────────
// FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────
void handleCommand(String cmd);
void handleSerialCommands();
void motorsStop();
void motorsDriveRaw(int l, int r);
void motorsDrive(int pwm, bool lf, bool rf);
void motorsTurn(bool spinRight);
bool mpuInit();
void mpuRead(float&,float&,float&,float&,float&,float&);
float getRawAccelAngle(float,float,float);
float getRawGyroRate(float,float,float);
void calibrateMPU();
void balanceUpdate(float dt);
void setupLCDChars();
void lcdClearFace();
void updateFace();
void renderFace(byte,byte,byte,bool,bool);
void notifyText(const String&);

// ─────────────────────────────────────────────────────────────
// BLE CALLBACKS
// ─────────────────────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*)    override { deviceConnected = true;  Serial.println("[BLE] Connected"); }
  void onDisconnect(BLEServer*) override {
    deviceConnected = false; motorsStop();
    robotMode = MODE_RC; balanceDriveTrim = 0; balanceTurnTrim = 0;
    Serial.println("[BLE] Disconnected");
    BLEDevice::startAdvertising();
  }
};

class CharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String cmd = c->getValue().c_str();
    cmd.trim(); handleCommand(cmd);
  }
};

// ─────────────────────────────────────────────────────────────
// NOTIFY
// ─────────────────────────────────────────────────────────────
void notifyText(const String &msg) {
  Serial.println(msg);
  if (deviceConnected && pCharacteristic) {
    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
  }
}

// ─────────────────────────────────────────────────────────────
// MPU
// ─────────────────────────────────────────────────────────────
bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x75); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)1);
  if (!Wire.available()) { notifyText("[MPU] Not found."); return false; }
  uint8_t w = Wire.read();
  Serial.print("[MPU] WHO_AM_I=0x"); Serial.println(w,HEX);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true); delay(100);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(true);
  notifyText("[MPU] Configured."); return true;
}

void mpuRead(float &ax,float &ay,float &az,float &gx,float &gy,float &gz) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)14,(uint8_t)true);
  if (Wire.available()<14){ax=ay=az=gx=gy=gz=0;return;}
  int16_t AX=(Wire.read()<<8)|Wire.read(), AY=(Wire.read()<<8)|Wire.read(), AZ=(Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read();
  int16_t GX=(Wire.read()<<8)|Wire.read(), GY=(Wire.read()<<8)|Wire.read(), GZ=(Wire.read()<<8)|Wire.read();
  ax=AX/16384.0f; ay=AY/16384.0f; az=AZ/16384.0f;
  gx=GX/131.0f;   gy=GY/131.0f;   gz=GZ/131.0f;
}

float getRawAccelAngle(float ax,float ay,float az) {
  float a = (tiltAxis==TILT_X) ? atan2f(ax,az)*(180/PI) : atan2f(ay,az)*(180/PI);
  return invertAngle ? -a : a;
}

float getRawGyroRate(float gx,float gy,float gz) {
  float r = (gyroAxis==GYRO_X)?gx:(gyroAxis==GYRO_Y)?gy:gz;
  return invertAngle ? -r : r;
}

// ─────────────────────────────────────────────────────────────
// CALIBRATION
// ─────────────────────────────────────────────────────────────
void calibrateMPU() {
  notifyText("[CAL] Hold upright. Starting in 2s...");
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Hold upright");
  lcd.setCursor(0,1); lcd.print("Cal in 2 sec");
  motorsStop(); delay(2000);
  float angleSum=0, gyroSum=0;
  for(int i=0;i<300;i++){
    float ax,ay,az,gx,gy,gz;
    mpuRead(ax,ay,az,gx,gy,gz);
    angleSum += getRawAccelAngle(ax,ay,az);
    gyroSum  += getRawGyroRate(gx,gy,gz);
    delay(5);
  }
  accelAngleOffset = angleSum/300; gyroOffset = gyroSum/300;
  calibrated = true;
  kalman.reset(0); integral=0; prev_error=0;
  Serial.printf("[CAL] angle_offset=%.3f gyro_offset=%.4f\n",accelAngleOffset,gyroOffset);
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Angle offset:");
  lcd.setCursor(0,1); lcd.print(accelAngleOffset,2); lcd.print(" deg");
  delay(1500); lcdClearFace();
  notifyText("[CAL] Done.");
}

// ─────────────────────────────────────────────────────────────
// MOTORS
// ─────────────────────────────────────────────────────────────
void motorsStop() {
  digitalWrite(MOTOR_A_IN1,HIGH); digitalWrite(MOTOR_A_IN2,HIGH);
  digitalWrite(MOTOR_B_IN3,HIGH); digitalWrite(MOTOR_B_IN4,HIGH);
  ledcWrite(MOTOR_A_ENA,0); ledcWrite(MOTOR_B_ENB,0);
}

void setMotorA(int pwm) {
  pwm=constrain(pwm,-255,255);
  if(!pwm){digitalWrite(MOTOR_A_IN1,HIGH);digitalWrite(MOTOR_A_IN2,HIGH);ledcWrite(MOTOR_A_ENA,0);return;}
  digitalWrite(MOTOR_A_IN1,pwm>0); digitalWrite(MOTOR_A_IN2,pwm<0);
  ledcWrite(MOTOR_A_ENA,abs(pwm));
}

void setMotorB(int pwm) {
  pwm=constrain(pwm,-255,255);
  if(!pwm){digitalWrite(MOTOR_B_IN3,HIGH);digitalWrite(MOTOR_B_IN4,HIGH);ledcWrite(MOTOR_B_ENB,0);return;}
  digitalWrite(MOTOR_B_IN3,pwm>0); digitalWrite(MOTOR_B_IN4,pwm<0);
  ledcWrite(MOTOR_B_ENB,abs(pwm));
}

void motorsDriveRaw(int l,int r) { setMotorA(l); setMotorB(r); }
void motorsDrive(int pwm,bool lf,bool rf) { motorsDriveRaw(lf?pwm:-pwm, rf?pwm:-pwm); }
void motorsTurn(bool right) { motorsDriveRaw(right?currentSpeed:-currentSpeed, right?-currentSpeed:currentSpeed); }

// ─────────────────────────────────────────────────────────────
// BALANCE UPDATE
// ─────────────────────────────────────────────────────────────
void balanceUpdate(float dt) {
  if(robotMode!=MODE_BALANCE||!calibrated) { if(!calibrated) motorsStop(); return; }
  if(dt<=0.001f||dt>0.1f) return;

  float ax,ay,az,gx,gy,gz;
  mpuRead(ax,ay,az,gx,gy,gz);
  float accelAngle = getRawAccelAngle(ax,ay,az)-accelAngleOffset;
  float gyroRate   = getRawGyroRate(gx,gy,gz)-gyroOffset;
  float angle      = kalman.update(accelAngle,gyroRate,dt);

  if(diagnosticMode && millis()-lastDebugMs>=DEBUG_INTERVAL) {
    Serial.printf("angle:%.2f sp:%.2f\n",angle,angle_setpoint);
    lastDebugMs=millis();
  }

  if(fabsf(angle)>TILT_CUTOUT) { motorsStop(); integral=0; prev_error=0; return; }

  angle_setpoint = base_setpoint + balanceDriveTrim;
  float error    = angle_setpoint - angle;
  integral      += error*dt;
  integral       = constrain(integral,-INTEGRAL_LIMIT,INTEGRAL_LIMIT);
  float deriv    = (error-prev_error)/dt;
  prev_error     = error;

  int output = constrain((int)(Kp*error+Ki*integral+Kd*deriv),-255,255);
  int spd    = abs(output);
  if(spd<5) spd=0; else spd=constrain(spd+MIN_MOTOR_PWM,MIN_MOTOR_PWM,255);

  int balPwm = (spd==0)?0:(output>0?spd:-spd);
  motorsDriveRaw(constrain(balPwm+balanceTurnTrim,-255,255),
                 constrain(balPwm-balanceTurnTrim,-255,255));
}

// ─────────────────────────────────────────────────────────────
// COMMAND HANDLER
// ─────────────────────────────────────────────────────────────
void handleCommand(String cmd) {
  cmd.trim(); cmd.toUpperCase(); cmd.replace(" ","");
  if(!cmd.length()) return;
  Serial.print("[CMD] "); Serial.println(cmd);

  if(cmd=="CAL")           { calibrateMPU(); return; }
  if(cmd=="MODE:RC")       { robotMode=MODE_RC; balanceDriveTrim=0; balanceTurnTrim=0; motorsStop(); notifyText("[MODE] RC"); return; }
  if(cmd=="MODE:BALANCE"||cmd=="MODE:BAL") {
    if(!calibrated){notifyText("[MODE] Send CAL first.");return;}
    robotMode=MODE_BALANCE; balanceDriveTrim=0; balanceTurnTrim=0;
    integral=0; prev_error=0; kalman.reset(0); notifyText("[MODE] BALANCE"); return;
  }
  if(cmd=="AXIS:X")  { tiltAxis=TILT_X; calibrated=false; notifyText("[AXIS] X. Send CAL."); return; }
  if(cmd=="AXIS:Y")  { tiltAxis=TILT_Y; calibrated=false; notifyText("[AXIS] Y. Send CAL."); return; }
  if(cmd=="GYRO:X")  { gyroAxis=GYRO_X; calibrated=false; notifyText("[GYRO] X. Send CAL."); return; }
  if(cmd=="GYRO:Y")  { gyroAxis=GYRO_Y; calibrated=false; notifyText("[GYRO] Y. Send CAL."); return; }
  if(cmd=="GYRO:Z")  { gyroAxis=GYRO_Z; calibrated=false; notifyText("[GYRO] Z. Send CAL."); return; }
  if(cmd=="INV:0")   { invertAngle=false; calibrated=false; notifyText("[INV] OFF. Send CAL."); return; }
  if(cmd=="INV:1")   { invertAngle=true;  calibrated=false; notifyText("[INV] ON. Send CAL."); return; }
  if(cmd=="DIAG:0")  { diagnosticMode=false; notifyText("[DIAG] OFF"); return; }
  if(cmd=="DIAG:1")  { diagnosticMode=true;  notifyText("[DIAG] ON");  return; }

  if(cmd.startsWith("KP:"))     { Kp=cmd.substring(3).toFloat(); Serial.print("[PID] Kp="); Serial.println(Kp); return; }
  if(cmd.startsWith("KI:"))     { Ki=cmd.substring(3).toFloat(); integral=0; Serial.print("[PID] Ki="); Serial.println(Ki); return; }
  if(cmd.startsWith("KD:"))     { Kd=cmd.substring(3).toFloat(); Serial.print("[PID] Kd="); Serial.println(Kd); return; }
  if(cmd.startsWith("SP:"))     { base_setpoint=cmd.substring(3).toFloat(); integral=0; prev_error=0; return; }
  if(cmd.startsWith("MINPWM:")) { MIN_MOTOR_PWM=constrain(cmd.substring(7).toInt(),0,180); return; }
  if(cmd=="SPD:1"){ currentSpeed=SPEED_TABLE[0]; return; }
  if(cmd=="SPD:2"){ currentSpeed=SPEED_TABLE[1]; return; }
  if(cmd=="SPD:3"){ currentSpeed=SPEED_TABLE[2]; return; }

  if(cmd=="FACE:IDLE")     { pendingFace=IDLE;      faceChanged=true; return; }
  if(cmd=="FACE:HAPPY")    { pendingFace=HAPPY;     faceChanged=true; return; }
  if(cmd=="FACE:SAD")      { pendingFace=SAD;       faceChanged=true; return; }
  if(cmd=="FACE:ANGRY")    { pendingFace=ANGRY;     faceChanged=true; return; }
  if(cmd=="FACE:SURPRISED"){ pendingFace=SURPRISED; faceChanged=true; return; }
  if(cmd=="FACE:TALKING")  { pendingFace=TALKING;   faceChanged=true; return; }
  if(cmd=="FACE:THINKING") { pendingFace=THINKING;  faceChanged=true; return; }

  // Movement
  if(robotMode==MODE_RC) {
    if(cmd=="F") { motorsDrive(currentSpeed,true,true);   return; }
    if(cmd=="B") { motorsDrive(currentSpeed,false,false); return; }
    if(cmd=="L") { motorsTurn(false); return; }
    if(cmd=="R") { motorsTurn(true);  return; }
    if(cmd=="S") { motorsStop();      return; }
  }
  if(robotMode==MODE_BALANCE) {
    if(cmd=="F") { balanceDriveTrim=BALANCE_FORWARD_SP;  return; }
    if(cmd=="B") { balanceDriveTrim=BALANCE_BACKWARD_SP; return; }
    if(cmd=="L") { balanceTurnTrim=-BALANCE_TURN_PWM;    return; }
    if(cmd=="R") { balanceTurnTrim=BALANCE_TURN_PWM;     return; }
    if(cmd=="S") { balanceDriveTrim=0; balanceTurnTrim=0; return; }
  }
  Serial.println("[CMD] Unknown.");
}

void handleSerialCommands() {
  static String input="";
  while(Serial.available()) {
    char c=Serial.read();
    if(c=='\n'||c=='\r') { if(input.length()) { handleCommand(input); input=""; } }
    else { input+=c; if(input.length()>80) input=""; }
  }
}

// ─────────────────────────────────────────────────────────────
// LCD FACES
// ─────────────────────────────────────────────────────────────
void setupLCDChars() {
  lcd.createChar(C_EYE_OPEN,EYES_OPEN); lcd.createChar(C_EYE_HALF,EYES_HALF);
  lcd.createChar(C_EYE_SHUT,EYES_SHUT); lcd.createChar(C_M_HAPPY,MOUTH_HAPPY);
  lcd.createChar(C_M_SAD,MOUTH_SAD);    lcd.createChar(C_M_TALK,MOUTH_TALK);
  lcd.createChar(C_M_FLAT,MOUTH_FLAT);  lcd.createChar(C_EYEBROW_UP,EYEBROW_UP);
}

void lcdClearFace() { lcd.clear(); displayedFace={255,255,255,false,false}; }

void renderFace(byte l,byte r,byte m,bool bl,bool br) {
  byte fl=bl?C_EYEBROW_UP:l, fr=br?C_EYEBROW_UP:r;
  if(fl!=displayedFace.left||bl!=displayedFace.browLeft)  { lcd.setCursor(3,0);  lcd.write(fl); }
  if(fr!=displayedFace.right||br!=displayedFace.browRight){ lcd.setCursor(12,0); lcd.write(fr); }
  if(m!=displayedFace.mouth)                               { lcd.setCursor(7,1);  lcd.write(m);  }
  displayedFace={fl,fr,m,bl,br};
}

void updateFace() {
  if(faceChanged) { currentFace=pendingFace; faceChanged=false; lcdClearFace(); }
  unsigned long t=millis();
  switch(currentFace) {
    case IDLE:      ((t%3000)<2800)?renderFace(C_EYE_OPEN,C_EYE_OPEN,C_M_FLAT,0,0):renderFace(C_EYE_SHUT,C_EYE_SHUT,C_M_FLAT,0,0); break;
    case HAPPY:     renderFace(C_EYE_OPEN,C_EYE_OPEN,C_M_HAPPY,0,0); break;
    case SAD:       renderFace(C_EYE_HALF,C_EYE_HALF,C_M_SAD,0,0); break;
    case ANGRY:     renderFace(C_EYE_SHUT,C_EYE_SHUT,C_M_SAD,1,1); break;
    case SURPRISED: renderFace(C_EYEBROW_UP,C_EYEBROW_UP,C_M_TALK,1,1); break;
    case TALKING:   ((t/120)%2)?renderFace(C_EYE_OPEN,C_EYE_OPEN,C_M_TALK,0,0):renderFace(C_EYE_OPEN,C_EYE_OPEN,C_M_FLAT,0,0); break;
    case THINKING:  { int p=(t/600)%3;
      if(p==0) renderFace(C_EYE_HALF,C_EYE_OPEN,C_M_FLAT,0,0);
      else if(p==1) renderFace(C_EYE_SHUT,C_EYE_OPEN,C_M_FLAT,0,0);
      else renderFace(C_EYE_HALF,C_EYE_OPEN,C_M_FLAT,1,0); break; }
  }
}

// ─────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200); delay(300);
  Serial.println("[BOOT] AI_ROBOT — Joudy GPT");

  pinMode(MOTOR_A_IN1,OUTPUT); pinMode(MOTOR_A_IN2,OUTPUT);
  pinMode(MOTOR_B_IN3,OUTPUT); pinMode(MOTOR_B_IN4,OUTPUT);
  ledcAttach(MOTOR_A_ENA,PWM_FREQ,PWM_BITS);
  ledcAttach(MOTOR_B_ENB,PWM_FREQ,PWM_BITS);
  motorsStop();

  Wire.begin(21,22); Wire.setClock(400000);
  lcd.init(); lcd.backlight(); setupLCDChars(); lcdClearFace();
  lcd.setCursor(0,0); lcd.print("JOUDY GPT"); lcd.setCursor(0,1); lcd.print("Booting...");

  bool mpuOk = mpuInit();
  lcd.clear();
  if(!mpuOk) { lcd.print("MPU ERROR"); lcd.setCursor(0,1); lcd.print("RC only"); }
  else        { lcd.print("MPU Ready"); lcd.setCursor(0,1); lcd.print("Send CAL"); }
  delay(1000); lcdClearFace();

  BLEDevice::init("AI_ROBOT");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHAR_UUID,
    BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_WRITE|BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new CharCallbacks());
  pCharacteristic->setValue("READY");
  pService->start();
  BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
  BLEDevice::getAdvertising()->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("[BOOT] Advertising as AI_ROBOT. Send HELP.");
}

// ─────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  static unsigned long lastUs=micros(), lastFaceMs=0;
  unsigned long nowUs=micros();
  float dt=(nowUs-lastUs)*1e-6f;
  lastUs=nowUs;

  handleSerialCommands();
  balanceUpdate(dt);

  unsigned long nowMs=millis();
  if(nowMs-lastFaceMs>=100) { lastFaceMs=nowMs; updateFace(); }

  long el=(long)(micros()-nowUs);
  if(el<10000) delayMicroseconds(10000-el);
}
