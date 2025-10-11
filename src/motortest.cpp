#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23008.h>

Adafruit_MCP23008 mcp;
#define I2C_SDA   11
#define I2C_SCL   12
#define MCP_ADDR  0x20

// IR pins on MCP23008 expander 
// NEED TO DETERMINE WHICH OF THESE ARE FRONT/RIGHT/LEFT/BACKL/BACKR 
#define IR_FL 0  
#define IR_RL 1  
#define IR_RR 2  
#define IR_FR 3 
#define IR_FC 4  

#define PIN_STBY 13
#define PIN_AIN1 1   //  PWM channel CH_A_IN1 (forward PWM)
#define PIN_AIN2 2   // PWM channel CH_A_IN2 (reverse PWM)
#define PIN_BIN1 10  // PWM channel CH_B_IN1 (forward PWM)
#define PIN_BIN2 3   // PWM channel CH_B_IN2 (reverse PWM)

#define PWM_FREQ   20000   
#define PWM_RES    8       
#define CH_A_IN1   0
#define CH_A_IN2   1
#define CH_B_IN1   2
#define CH_B_IN2   3

// TUNING 
int LEFT_SPEED_BASE  = 200;  
int RIGHT_SPEED_BASE = 200;  

static inline int clamp255(int v){ return v < 0 ? 0 : (v > 255 ? 255 : v); }

// Motor control: speed in -255..+255 (forward positive, reverse negative)
void motorLeft(int spd){
  spd = constrain(spd, -255, 255);
  if (spd > 0){
    // Forward: PWM on AIN1, AIN2 LOW
    ledcWrite(CH_A_IN2, 0);
    ledcWrite(CH_A_IN1, spd);
  } else if (spd < 0){
    // Reverse: PWM on AIN2, AIN1 LOW  (required by TB6612 truth table)
    ledcWrite(CH_A_IN1, 0);
    ledcWrite(CH_A_IN2, -spd);
  } else {
    // Coast
    ledcWrite(CH_A_IN1, 0);
    ledcWrite(CH_A_IN2, 0);
  }
}

void motorRight(int spd){
  spd = constrain(spd, -255, 255);
  if (spd > 0){
    ledcWrite(CH_B_IN2, 0);
    ledcWrite(CH_B_IN1, spd);
  } else if (spd < 0){
    ledcWrite(CH_B_IN1, 0);
    ledcWrite(CH_B_IN2, -spd);
  } else {
    ledcWrite(CH_B_IN1, 0);
    ledcWrite(CH_B_IN2, 0);
  }
}

void setMotors(int leftCmd, int rightCmd){
  motorLeft(leftCmd);
  motorRight(rightCmd);
}

void setup(){
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  if (!mcp.begin(MCP_ADDR)){
    Serial.println("MCP23008 init FAILED");
    while(true) delay(1000);
  }
  mcp.pinMode(IR_FL, INPUT);
  mcp.pinMode(IR_FR, INPUT);
  mcp.pinMode(IR_FC, INPUT);
  mcp.pinMode(IR_RL, INPUT);
  mcp.pinMode(IR_RR, INPUT);


  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  ledcSetup(CH_A_IN1, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_AIN1, CH_A_IN1);
  ledcSetup(CH_A_IN2, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_AIN2, CH_A_IN2);
  ledcSetup(CH_B_IN1, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_BIN1, CH_B_IN1);
  ledcSetup(CH_B_IN2, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_BIN2, CH_B_IN2);

  setMotors(0, 0);

  Serial.println("IR+Motor test ready. Tuning LEFT_SPEED_BASE / RIGHT_SPEED_BASE.");
}

void loop(){
  static uint32_t tIR = 0;
  if (millis() - tIR >= 100){
    tIR = millis();
    int fl = mcp.digitalRead(IR_FL);
    int fc = mcp.digitalRead(IR_FC);
    int fr = mcp.digitalRead(IR_FR);
    int rl = mcp.digitalRead(IR_RL);
    int rr = mcp.digitalRead(IR_RR);
    Serial.printf("IR FL=%d FC=%d FR=%d | RL=%d RR=%d\n", fl, fc, fr, rl, rr);
  }

  // Increase/decrease LEFT_SPEED_BASE / RIGHT_SPEED_BASE until it drives straight.
  setMotors(clamp255(LEFT_SPEED_BASE), clamp255(RIGHT_SPEED_BASE));
  delay(20);

  // test reverse symmetry:
  /*
  setMotors(-clamp255(LEFT_SPEED_BASE), -clamp255(RIGHT_SPEED_BASE));
  delay(1000);
  setMotors(0, 0);
  delay(500);
  */
}
