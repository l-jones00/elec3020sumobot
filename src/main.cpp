#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23008.h>
#include <TFT_eSPI.h>

#include "ultrasonic.h"

TFT_eSPI tft;

Adafruit_MCP23008 mcp;
#define I2C_SDA 11
#define I2C_SCL 12
#define MCP_ADDR 0x20

// IR pins on MCP23008 expander
// NEED TO DETERMINE WHICH OF THESE ARE FRONT/RIGHT/LEFT/BACKL/BACKR
#define IR_FL 1
#define IR_RL 3
#define IR_RR 4
#define IR_FR 2
#define IR_FC 0

#define TRIG_F 16
#define ECHO_F 17

#define PIN_STBY 13
#define PIN_AIN1 1  // PWM channel CH_A_IN1 (forward PWM)
#define PIN_AIN2 2  // PWM channel CH_A_IN2 (reverse PWM)
#define PIN_BIN1 10 // PWM channel CH_B_IN1 (forward PWM)
#define PIN_BIN2 3  // PWM channel CH_B_IN2 (reverse PWM)

#define PWM_FREQ 20000
#define PWM_RES 8
#define CH_A_IN1 0
#define CH_A_IN2 1
#define CH_B_IN1 2
#define CH_B_IN2 3

// Direction logic per side — NO rewiring needed
#define LEFT_FORWARD_USES_IN1 1  // 1: forward uses IN1 PWM, 0: forward uses IN2 PWM
#define RIGHT_FORWARD_USES_IN1 1 // try 0 if your right side is acting reversed
#define LEFT_DIR_SIGN (+1)       // flip to -1 if needed
#define RIGHT_DIR_SIGN (+1)      // flip to -1 if needed
#define SPEED 255

#define US_THRESHOLD 50 // threshold before it stops scanning and drives forwards

// TUNING
int LEFT_SPEED_BASE = 250;
int RIGHT_SPEED_BASE = 250;
int TURNL = 200;
int TURNR = 244;

static inline int clamp255(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }

void active_brake()
{
  ledcWrite(CH_B_IN1, 255);
  ledcWrite(CH_B_IN2, 255);

  ledcWrite(CH_A_IN1, 255);
  ledcWrite(CH_A_IN2, 255);
  delay(250);
}

// Motor control: speed in -255..+255 (forward positive, reverse negative)
void motorLeft(int spd)
{
  spd = constrain(spd * LEFT_DIR_SIGN, -255, 255);
  if (spd == 0)
  {
    ledcWrite(CH_A_IN1, 0);
    ledcWrite(CH_A_IN2, 0);
    return;
  }
  bool fwd = spd > 0;
  int duty = clamp255(abs(spd));
  if (LEFT_FORWARD_USES_IN1)
  {
    if (fwd)
    {
      ledcWrite(CH_A_IN2, 0);
      ledcWrite(CH_A_IN1, duty);
    }
    else
    {
      ledcWrite(CH_A_IN1, 0);
      ledcWrite(CH_A_IN2, duty);
    }
  }
  else
  {
    if (fwd)
    {
      ledcWrite(CH_A_IN1, 0);
      ledcWrite(CH_A_IN2, duty);
    }
    else
    {
      ledcWrite(CH_A_IN2, 0);
      ledcWrite(CH_A_IN1, duty);
    }
  }
}

void motorRight(int spd)
{
  spd = constrain(spd * RIGHT_DIR_SIGN, -255, 255);
  if (spd == 0)
  {
    ledcWrite(CH_B_IN1, 0);
    ledcWrite(CH_B_IN2, 0);
    return;
  }
  bool fwd = spd > 0;
  int duty = clamp255(abs(spd));
  if (RIGHT_FORWARD_USES_IN1)
  {
    if (fwd)
    {
      ledcWrite(CH_B_IN2, 0);
      ledcWrite(CH_B_IN1, duty);
    }
    else
    {
      ledcWrite(CH_B_IN1, 0);
      ledcWrite(CH_B_IN2, duty);
    }
  }
  else
  {
    if (fwd)
    {
      ledcWrite(CH_B_IN1, 0);
      ledcWrite(CH_B_IN2, duty);
    }
    else
    {
      ledcWrite(CH_B_IN2, 0);
      ledcWrite(CH_B_IN1, duty);
    }
  }
}

/*
void slowRightStraight(int leftCmd, int rightCmd){
  motorLeft(-leftCmd);
  motorRight(-rightCmd);
}
*/

/*
void fastLeftStraight(int leftCmd, int rightCmd){
  motorLeft(-leftCmd);
  motorRight(rightCmd);
}
*/

/*
void setMotors(int leftCmd, int rightCmd){
  motorLeft(-leftCmd);
  motorRight(-rightCmd);
}
void fastTurn(int leftCmd, int rightCmd){
  setMotors(leftCmd, rightCmd);
}
*/

/*
void slowTurn(int leftCmd, int rightCmd){
  motorLeft(leftCmd);
  motorRight(-rightCmd);
}
*/
void reverseBack()
{
  // TODO
}

// ---------- IR logic (non-blocking) ----------
#define IR_POLL_MS 5
#define TURN_MS 800      // how long to run your turn (tune)
#define STBY_PAUSE_MS 50 // how long STBY stays LOW before re-enable
#define IR_ACTIVE_LOW 0  // normally at 0

inline bool irTriggered(uint8_t pin)
{

  int v = mcp.digitalRead(pin);
#if IR_ACTIVE_LOW
  return (v == 0);
#else
  return (v != 0);
#endif
}

// States for simple non-blocking behavior
// States for simple non-blocking behavior
enum Action : uint8_t
{
  ACT_IDLE = 0,
  ACT_FRONT_PAUSE,      // STBY low briefly
  ACT_FRONT_SPIN_180,   // timed spin in place
  ACT_FRONT_ESCAPE_FWD, // timed forward to get off black
  ACT_ATTACK
};

static Action g_action = ACT_IDLE;
static uint32_t g_actionUntil = 0;
static uint32_t g_lastIRPoll = 0;

// Timings
#define STBY_PAUSE_MS 50
#define FRONT_SPIN_MS 700      // you measured ~200 ms for 180°
#define FRONT_ESCAPE_MS 700    // forward to clear the line
#define US_SCANNING_MODE_MS 70 // basically controls how much it turns in scanning mode

static inline bool anyFront()
{
  // tft.fillScreen(TFT_RED);
  return irTriggered(IR_FL) || irTriggered(IR_FC) || irTriggered(IR_FR);
}
static inline bool anyRear()
{
  return irTriggered(IR_RL) || irTriggered(IR_RR);
}
void ir_update_decision()
{
  const uint32_t now = millis();
  if (now - g_lastIRPoll < IR_POLL_MS)
    return;
  g_lastIRPoll = now;

  if (g_action != ACT_IDLE)
   return;

  if (anyFront())
  {
    // FRONT: stop & pause driver, then we'll spin 180 and escape forward
    // setMotors(0,0);
    digitalWrite(PIN_STBY, HIGH);
    active_brake();

    digitalWrite(PIN_STBY, LOW);
    g_action = ACT_FRONT_PAUSE;
    g_actionUntil = now + STBY_PAUSE_MS;
    return;
  }

  if (anyRear())
  {
    // If rear touches, you can keep your old behavior,
    // or do the same pause->spin->escape. For now, reuse the same sequence:
    // setMotors(0,0);
    digitalWrite(PIN_STBY, LOW);
    g_action = ACT_FRONT_PAUSE;
    g_actionUntil = now + STBY_PAUSE_MS;
    return;
  }
}

void straight()
{
  digitalWrite(PIN_STBY, HIGH);
  ledcWrite(CH_B_IN2, 0);
  ledcWrite(CH_B_IN1, 0);

  // digitalWrite(PIN_BIN2, LOW);
  // digitalWrite(PIN_BIN1, LOW);

  ledcWrite(CH_A_IN2, SPEED);
  ledcWrite(CH_A_IN1, 0);

  // digitalWrite(PIN_AIN1, LOW);
  // digitalWrite(PIN_AIN2, HIGH);
}

void turn()
{
  digitalWrite(PIN_STBY, HIGH);
  ledcWrite(CH_B_IN2, 0);
  ledcWrite(CH_B_IN1, 0);

  ledcWrite(CH_A_IN2, 0);
  ledcWrite(CH_A_IN1, SPEED);
}

// void scan_us()
// {
//   // Only adjust high-level mode from IDLE/SCAN; leave timed actions alone
//   if (g_action != ACT_IDLE)
//     return;

//   if (us_f < US_THRESHOLD)
//   {
//     // target acquired -> attack
//     g_action = ACT_ATTACK;
//     g_actionUntil = 0;
//   }
//   else
//   {
//     // target lost -> scan (turn)
//     if (g_action != ACT_IDLE)
//     {
//       g_action = ACT_IDLE;
//       g_actionUntil = 0; // continuous until target or IR event
//     }
//   }
// }


void ir_run_action_or_cruise()
{
  const uint32_t now = millis();

  switch (g_action)
  {
  case ACT_FRONT_PAUSE:
    // Hold driver off for a brief moment
    if ((int32_t)(g_actionUntil - now) <= 0)
    {
      digitalWrite(PIN_STBY, HIGH); // re-enable driver
      // Start 180° spin: spin in place (left fwd, right rev). Direction doesn't matter.
      // Uses your base trims so both sides apply equally.

      g_action = ACT_FRONT_SPIN_180;
      g_actionUntil = now + FRONT_SPIN_MS;
    }
    else
    {
      // setMotors(0,0);
    }
    return;

  case ACT_FRONT_SPIN_180:
    // Spin in place: left forward, right reverse (or swap if you prefer the other way)
    // motorLeft(  LEFT_SPEED_BASE);
    // motorRight(RIGHT_SPEED_BASE);
    
    turn();
    // scan_us();
    if (us_f < US_THRESHOLD) {
        tft.fillScreen(TFT_GREEN);
        g_action = ACT_ATTACK;
      }

    if ((int32_t)(g_actionUntil - now) <= 0)
    {
      // After spin, drive forward to get off the black
      g_action = ACT_FRONT_ESCAPE_FWD;
      g_actionUntil = now + FRONT_ESCAPE_MS;
      
    }
    

    
    return;

  case ACT_FRONT_ESCAPE_FWD:
    // Go forward using your base trims
    straight();
    if ((int32_t)(g_actionUntil - now) <= 0)
    {
      // Done escaping → back to idle cruise
      g_action = ACT_IDLE;
    }
    return;

    // case ACT_SCAN: // scanning mode - just turn
    // turn();
    //     if ((int32_t)(g_actionUntil - now) <= 0)
    // {
    //   // Done escaping → back to idle cruise
    //   g_action = ACT_IDLE;
    // }

case ACT_ATTACK:
    straight();
    //scan_us();
    if (us_f > US_THRESHOLD) {
      g_action = ACT_IDLE;
    }
    return;

  case ACT_IDLE: // aka, attacking mode
  default:
    // Normal cruise: go straight using base trims
    straight();
    tft.fillScreen(TFT_WHITE);
    if (us_f > US_THRESHOLD) {
//       tft.fillScreen(TFT_WHITE);
   }
    
    
    // active_brake();
    // delay(200);
    // turn();
    // scan_us();
    // setMotors(clamp255(-LEFT_SPEED_BASE), clamp255(-RIGHT_SPEED_BASE));
    return;
  }
}
#define BUTTON 0          // TFT button GPIO
#define COUNTDOWN_MS 3000 // 3 seconds
#define BTN_ACTIVE_LOW 1  // T-Display button is typically active-LOW
enum StartState : uint8_t
{
  START_WAIT = 0,
  START_COUNTDOWN,
  START_RUN
};
static StartState g_start = START_WAIT;
static uint32_t g_tCountdown = 0;

void start_begin()
{
  pinMode(BUTTON, INPUT_PULLUP);
  digitalWrite(PIN_STBY, LOW); // motors OFF until countdown finishes
  // setMotors(0,0);
}

// returns true once the robot is allowed to move
bool start_update()
{
  const uint32_t now = millis();

  switch (g_start)
  {
  case START_WAIT:
    if (digitalRead(BUTTON) == LOW)
    {
      g_start = START_COUNTDOWN;
      g_tCountdown = now + COUNTDOWN_MS;
      digitalWrite(PIN_STBY, LOW); // ensure driver off
      // setMotors(0,0);
    }
    break;

  case START_COUNTDOWN:
    digitalWrite(PIN_STBY, LOW);
    // setMotors(0,0);
    if ((int32_t)(g_tCountdown - now) <= 0)
    {
      digitalWrite(PIN_STBY, HIGH);
      g_start = START_RUN;
      tft.fillScreen(TFT_WHITE);
      tft.setTextColor(TFT_BLACK);
      tft.setRotation(3);
      tft.setCursor(60, 60);
      tft.setTextSize(4);
      tft.print("STARTING");
    }
    break;

  case START_RUN:

    break;
  }

  return (g_start == START_RUN);
}

void setup()
{
  // Serial.begin(115200);

  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);

  tft.init();
  tft.fillScreen(TFT_WHITE);
  tft.setTextColor(TFT_BLACK);
  tft.setRotation(3);
  tft.setCursor(60, 60);
  tft.setTextSize(4);
  tft.print("PRESS BUTTON\nTO START!!");

  start_begin();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  if (!mcp.begin(MCP_ADDR))
  {
    // Serial.println("MCP23008 init FAILED");
    while (true)
      delay(1000);
  }
  mcp.pinMode(IR_FL, INPUT);
  mcp.pinMode(IR_FR, INPUT);
  mcp.pinMode(IR_FC, INPUT);
  mcp.pinMode(IR_RL, INPUT);
  mcp.pinMode(IR_RR, INPUT);

  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);

  ledcSetup(CH_A_IN1, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_AIN1, CH_A_IN1);
  ledcSetup(CH_A_IN2, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_AIN2, CH_A_IN2);
  ledcSetup(CH_B_IN1, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_BIN1, CH_B_IN1);
  ledcSetup(CH_B_IN2, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_BIN2, CH_B_IN2);

  // ultrasonics
  pinMode(ECHO_F, INPUT);
  pinMode(ECHO_FL, INPUT);
  pinMode(ECHO_FR, INPUT);

  pinMode(TRIG_F, OUTPUT);
  pinMode(TRIG_FL, OUTPUT);
  pinMode(TRIG_FR, OUTPUT);

  xTaskCreatePinnedToCore(
      ultrasonic_task,   // Task function
      "ultrasonic_task", // Task name
      4096,              // Stack size (bytes)
      NULL,              // Parameters
      1,                 // Priority
      NULL,              // Task handle
      1                  // Core 1
  );

  // setMotors(0, 0);

  // Serial.println("IR+Motor test ready. Tuning LEFT_SPEED_BASE / RIGHT_SPEED_BASE.");
}

void loop()
{
  /*static uint32_t tIR = 0;
  if (millis() - tIR >= 100){
    tIR = millis();
    int fl = mcp.digitalRead(IR_FL);
    if (fl == '0') reverseBack;
    int fc = mcp.digitalRead(IR_FC);
    if (fc == '0') reverseBack;
    int fr = mcp.digitalRead(IR_FR);
    if (fr == '0') reverseBack;
    int rl = mcp.digitalRead(IR_RL);
    if (rl == '0') fastTurn(TURNL, TURNR);
    int rr = mcp.digitalRead(IR_RR);
    if (rr == '0') fastTurn(TURNL, TURNR);
    //Serial.printf("IR FL=%d FC=%d FR=%d | RL=%d RR=%d\n", fl, fc, fr, rl, rr);
  }*/

  // Increase/decrease LEFT_SPEED_BASE / RIGHT_SPEED_BASE until it drives straight.

  if (!start_update())
  {
    return;
  }
  ir_update_decision();
  ir_run_action_or_cruise(); // execute action (or cruise) without delays

  // no delay() here — everything is time-driven

  // test reverse symmetry:
  /*
  setMotors(-clamp255(LEFT_SPEED_BASE), -clamp255(RIGHT_SPEED_BASE));
  delay(1000);
  setMotors(0, 0);
  delay(500);
  */
}
