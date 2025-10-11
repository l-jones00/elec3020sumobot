#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23008.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#define I2C_SDA   11
#define I2C_SCL   12
#define MCP_ADDR  0x20

#define IR_FL 0   
#define IR_RL 1    
#define IR_RR 2   
#define IR_FR 3 
#define IR_FC 4   

#define PIN_STBY 13
#define PIN_AIN1 1    // LEFT  - forward PWM
#define PIN_AIN2 2    // LEFT  - reverse PWM
#define PIN_BIN1 10   // RIGHT - forward PWM
#define PIN_BIN2 3    // RIGHT - reverse PWM

#define BUTTON_PIN 0  

// =================== DISPLAY ===================
TFT_eSPI tft;

static void drawCenteredText(const String& s, int textSize) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(textSize);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(s, tft.width()/2, tft.height()/2);
}

static void showCountdown() {
  drawCenteredText("3", 5); delay(1000);
  drawCenteredText("2", 5); delay(1000);
  drawCenteredText("1", 5); delay(1000);
  drawCenteredText("GO!", 4); delay(250);
  tft.fillScreen(TFT_BLACK);
}

// =================== IR INPUTS =================
Adafruit_MCP23008 mcp;

// Debounce (0/1) -> latched state with simple consecutive-sample threshold
struct Debounced { uint8_t lo=0, hi=0; bool state=false; }; // state=true => BLACK
static const uint8_t  TRIP      = 4;   // consecutive samples to flip
static const uint8_t  SAMPLE_MS = 2;   // IR sample period

Debounced irFL, irFR, irRL, irRR, irFC;

static inline void updateDebounced(Debounced &d, int v){
  if (v==0){ d.lo = d.lo<255? d.lo+1:255; d.hi=0; if(!d.state && d.lo>=TRIP) d.state=true; }
  else     { d.hi = d.hi<255? d.hi+1:255; d.lo=0; if( d.state && d.hi>=TRIP) d.state=false; }
}
static inline void sampleIR(){
  updateDebounced(irFL, mcp.digitalRead(IR_FL));
  updateDebounced(irFR, mcp.digitalRead(IR_FR));
  updateDebounced(irRL, mcp.digitalRead(IR_RL));
  updateDebounced(irRR, mcp.digitalRead(IR_RR));
  updateDebounced(irFC, mcp.digitalRead(IR_FC));
}

#define PWM_FREQ 20000   
#define PWM_RES  8       
#define CH_A_IN1 0
#define CH_A_IN2 1
#define CH_B_IN1 2
#define CH_B_IN2 3

// TUNE
uint8_t BASE_FWD_L = 200;   // left forward base
uint8_t BASE_FWD_R = 200;   // right forward base
uint8_t BASE_REV_L = 200;   // left reverse base
uint8_t BASE_REV_R = 200;   // right reverse base

// Spin speeds (pivot in place)
uint8_t SPIN_SPEED_L = 220;
uint8_t SPIN_SPEED_R = 220;

// Helper: drive per wheel with signed speed -255..+255
static inline void motorLeft(int spd){
  spd = constrain(spd, -255, 255);
  if (spd > 0) { // forward -> PWM on AIN1, AIN2=0
    ledcWrite(CH_A_IN2, 0);
    ledcWrite(CH_A_IN1, spd);
  } else if (spd < 0) { // reverse -> PWM on AIN2, AIN1=0
    ledcWrite(CH_A_IN1, 0);
    ledcWrite(CH_A_IN2, -spd);
  } else {
    ledcWrite(CH_A_IN1, 0);
    ledcWrite(CH_A_IN2, 0); // coast
  }
}
static inline void motorRight(int spd){
  spd = constrain(spd, -255, 255);
  if (spd > 0) { // forward
    ledcWrite(CH_B_IN2, 0);
    ledcWrite(CH_B_IN1, spd);
  } else if (spd < 0) { // reverse
    ledcWrite(CH_B_IN1, 0);
    ledcWrite(CH_B_IN2, -spd);
  } else {
    ledcWrite(CH_B_IN1, 0);
    ledcWrite(CH_B_IN2, 0);
  }
}
static inline void setMotors(int left, int right){
  motorLeft(left);
  motorRight(right);
}
static inline void hardStop(){ setMotors(0,0); }

// Convenience motions using your trimmed bases
static inline void forwardBoth(){
  motorLeft( BASE_FWD_L);
  motorRight(BASE_FWD_R);
}
static inline void reverseBoth(){
  motorLeft(-BASE_REV_L);
  motorRight(-BASE_REV_R);
}
static inline void spinLeft(){   // CCW: left back, right fwd
  motorLeft(-SPIN_SPEED_L);
  motorRight( SPIN_SPEED_R);
}
static inline void spinRight(){  // CW: left fwd, right back
  motorLeft( SPIN_SPEED_L);
  motorRight(-SPIN_SPEED_R);
}

// =================== MOTION STATE MACHINE ===================
enum MotionState : uint8_t {
  MS_FORWARD,
  MS_REVERSE,      // timed
  MS_SPIN_LEFT,    // timed
  MS_SPIN_RIGHT,   // timed
  MS_SETTLE        // timed stop
};

MotionState motion = MS_FORWARD;
uint32_t    state_until = 0;
MotionState next_after_settle = MS_FORWARD;

static const uint16_t REVERSE_MS = 1000; // back away from edge
static const uint16_t SETTLE_MS  = 60;   // short coast/stop after moves
static const uint16_t SPIN_90_MS = 230;  // calibrate this when we figure out bot movements
static inline uint32_t timeForDeg(uint16_t deg){ return (uint32_t)SPIN_90_MS * deg / 90; }

static inline void startState(MotionState st, uint32_t duration_ms, MotionState after=MS_SETTLE){
  motion = st;
  state_until = millis() + duration_ms;
  next_after_settle = after;
}

static inline void runStateMachine(){
  uint32_t now = millis();
  switch (motion){
    case MS_FORWARD:
      forwardBoth();
      break;

    case MS_REVERSE:
      reverseBoth();
      if ((int32_t)(now - state_until) >= 0){
        motion = MS_SETTLE; state_until = now + SETTLE_MS; next_after_settle = MS_FORWARD;
      }
      break;

    case MS_SPIN_LEFT:
      spinLeft();
      if ((int32_t)(now - state_until) >= 0){
        motion = MS_SETTLE; state_until = now + SETTLE_MS; next_after_settle = MS_FORWARD;
      }
      break;

    case MS_SPIN_RIGHT:
      spinRight();
      if ((int32_t)(now - state_until) >= 0){
        motion = MS_SETTLE; state_until = now + SETTLE_MS; next_after_settle = MS_FORWARD;
      }
      break;

    case MS_SETTLE:
      hardStop();
      if ((int32_t)(now - state_until) >= 0){
        motion = next_after_settle;
      }
      break;
  }
}

// Triggers (IR events)
static inline void onFrontEdge(){            startState(MS_REVERSE,    REVERSE_MS, MS_FORWARD); }
static inline void onRearLeftEdge(){         startState(MS_SPIN_RIGHT, timeForDeg(270), MS_FORWARD); }
static inline void onRearRightEdge(){        startState(MS_SPIN_LEFT,  timeForDeg(270), MS_FORWARD); }
static inline void onRearBothEdge(){         startState(MS_SPIN_LEFT,  timeForDeg(270), MS_FORWARD); }

bool started    = false;
bool lastButton = HIGH;

void setup(){
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Press button to start", tft.width()/2, 10);

  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  if (!mcp.begin(MCP_ADDR)) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("MCP23008 FAILED", tft.width()/2, tft.height()/2);
    while(true) { delay(1000); }
  }
  mcp.pinMode(IR_FL, INPUT);
  mcp.pinMode(IR_FR, INPUT);
  mcp.pinMode(IR_FC, INPUT);
  mcp.pinMode(IR_RL, INPUT);
  mcp.pinMode(IR_RR, INPUT);

  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH); // enable driver

  ledcSetup(CH_A_IN1, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_AIN1, CH_A_IN1);
  ledcSetup(CH_A_IN2, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_AIN2, CH_A_IN2);
  ledcSetup(CH_B_IN1, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_BIN1, CH_B_IN1);
  ledcSetup(CH_B_IN2, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_BIN2, CH_B_IN2);
  hardStop();

  Serial.println("Ready: IR + PWM motors + countdown start");
}

void loop(){
  // Wait for button falling edge, then countdown
  if (!started){
    bool cur = digitalRead(BUTTON_PIN);
    if (lastButton == HIGH && cur == LOW){
      Serial.println("Button pressed -> countdown");
      showCountdown();
      started = true;
      Serial.println("GO!");
    }
    lastButton = cur;
    return;
  }

  // Periodic IR sampling and edge handling
  static uint32_t lastSample = 0;
  uint32_t now = millis();
  if (now - lastSample >= SAMPLE_MS){
    lastSample = now;
    sampleIR();

    // Front (FL/FC/FR) has top priority
    if (irFL.state || irFC.state || irFR.state){
      onFrontEdge();
    } else {
      // Rear reacts mainly while cruising forward
      if (motion == MS_FORWARD){
        bool rl = irRL.state, rr = irRR.state;
        if      (rl && !rr) onRearLeftEdge();
        else if (rr && !rl) onRearRightEdge();
        else if (rl && rr)  onRearBothEdge();
      }
    }
  }

  runStateMachine();

//for debugging while we're testing
  static uint32_t dbgT = 0;
  if (now - dbgT >= 100){
    dbgT = now;
    Serial.printf("IR FL=%d FC=%d FR=%d | RL=%d RR=%d | state=%d\n",
      irFL.state, irFC.state, irFR.state, irRL.state, irRR.state, motion);
  }
}

///TO INCLUDE
//- PATHFINDING
//- TFT LIGHTING UP
//- DEBUGGIN THE HEK OUT OF THIS PLS