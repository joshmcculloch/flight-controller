#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Simple_AHRS.h>

#define PWM_THROTTLE_PIN 8
#define PWM_ROLL_PIN     6
#define PWM_PITCH_PIN    7
#define PWM_YAW_PIN      9
#define PWM_MODE_PIN     10

#define FAILSAFE_TIMEOUT 100

#define IMU_MERGE_FACTOR 0.1

struct PlaneState {
  float roll;
  float pitch;
  float heading;
};

PlaneState planeState = {0,0,0};
PlaneState targetState = {0,0,0};

enum Mode{FAILSAFE, MANUAL, STABILISED, AUTO};
Mode mode = FAILSAFE;


class PIDController {
  public:
  float p, i, d;
  float previous_error = 0;
  float intergral = 0;
  float iRange;
  unsigned long last_time;
  
  public:
  PIDController(float p, float i, float d) {
    this->p = p;
    this->i = i;
    this->d = d;
    this->last_time = millis();
    this->iRange = i > 0 ? 30/i : 1;
  }
  void update(float current, float target) {
    float error = current - target;
    float delta = (float)(millis()-last_time) / 1000;
    float ui = intergral + error*delta;
    float ud = (error-previous_error) / delta;
    
    previous_error = error;
    intergral = constrain(ui, -iRange, iRange);
    last_time = millis();
    
    output = p*error + i*ui + d*ud;
  }

  float output;
  
};

void pinChangeHandler();


class PWMInput {
  
  uint8_t pinState = 0;
  uint8_t pin;
  unsigned long riseTime = 0;

  void checkPin() {
    uint8_t currentState = digitalRead(pin);
    if (pinState != currentState) {
      
      if (currentState == 0) {
        duty = micros() - riseTime;
        //duty = ((duration -1000) * 256) / 1000;
      } else {
        riseTime = micros();
      }
      pinState = currentState;
      lastupdate = millis();
    } 
  }
  
  public:
  unsigned long lastupdate = 0;
  uint16_t duty = 0;
  
  PWMInput(uint8_t pin) {
    digitalWrite(pin,HIGH); 
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group

    this->pin = pin;
    pinState = digitalRead(pin);
  }
  
  void pinChange() {
    checkPin();
  }
  
};


ISR (PCINT0_vect) {
  pinChangeHandler();
}

ISR (PCINT1_vect) {
  pinChangeHandler();
}


ISR (PCINT2_vect) {
  pinChangeHandler();
}


PWMInput pwmYaw(PWM_YAW_PIN);
PWMInput pwmRoll(PWM_ROLL_PIN);
PWMInput pwmPitch(PWM_PITCH_PIN);
PWMInput pwmThrottle(PWM_THROTTLE_PIN);
PWMInput pwmMode(PWM_MODE_PIN);

void pinChangeHandler() {
  pwmYaw.pinChange();
  pwmRoll.pinChange();
  pwmPitch.pinChange();
  pwmThrottle.pinChange();
  pwmMode.pinChange();
}

Servo throttle;
Servo rudder;
Servo elevator;
Servo aileron;

Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);

Adafruit_Simple_AHRS          ahrs(&accel, &mag);
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

PIDController elevatorPID = PIDController(2,0,0);
PIDController aileronPID = PIDController(2,0,0);

void setup() {

  Serial.begin(115200);
  
  Serial.println("Setting up sensors...");
  accel.begin();
  mag.begin();
  bmp.begin();
  
  Serial.println("Setting up servos...");
  throttle.attach(15);
  aileron.attach(17);
  rudder.attach(14);
  elevator.attach(16);
  Serial.println("READY");
}

uint16_t loop_count = 0;
unsigned long log_time = 0;
void loop() {
  

  if(pwmPitch.lastupdate < (millis() - FAILSAFE_TIMEOUT) || pwmPitch.lastupdate < 1000) {
    mode = FAILSAFE;
  } else {
    if (pwmMode.duty > 1600) {
      mode = STABILISED;
    } else {
      mode = MANUAL;
    }
    
  }

  sensors_vec_t   orientation;
  if (ahrs.getOrientation(&orientation) && false)
  {
    planeState.roll = (1-IMU_MERGE_FACTOR)*planeState.roll + IMU_MERGE_FACTOR*orientation.roll;
    planeState.pitch = (1-IMU_MERGE_FACTOR)*planeState.pitch  + IMU_MERGE_FACTOR*orientation.pitch;
    planeState.heading = (1-IMU_MERGE_FACTOR)*planeState.heading + IMU_MERGE_FACTOR*orientation.heading;
  }

  elevatorPID.update(planeState.pitch, targetState.pitch);
  aileronPID.update(planeState.roll, targetState.roll);

  switch (mode) {
    case(FAILSAFE):
      digitalWrite(13,HIGH); 
      throttle.writeMicroseconds(1000);
      aileron.writeMicroseconds(1500);
      rudder.writeMicroseconds(1500);
      elevator.writeMicroseconds(1500);
      break;
    case(MANUAL):
      digitalWrite(13,LOW); 
      throttle.writeMicroseconds(pwmThrottle.duty);
      aileron.writeMicroseconds(pwmRoll.duty);
      rudder.writeMicroseconds(pwmYaw.duty);
      elevator.writeMicroseconds(pwmPitch.duty);
      break;
    case(STABILISED):
      digitalWrite(13,LOW);
      
      throttle.writeMicroseconds(pwmThrottle.duty);
      aileron.write(90 + aileronPID.output);
      rudder.writeMicroseconds(pwmYaw.duty);
      elevator.write(90 + elevatorPID.output);
      break;
  }

  if (log_time < millis()) {
    Serial.print(" ");
    Serial.println(loop_count);
    loop_count = 0;
    

    Serial.print("Elevator:");
    Serial.print(pwmPitch.duty);
    Serial.print("    Aileron:");
    Serial.print(pwmRoll.duty);
    Serial.print("    Throttle:");
    Serial.print(pwmThrottle.duty);
    Serial.print("    Rudder:");
    Serial.print(pwmYaw.duty);
    Serial.print("    MODE:");
    switch (mode) {
      case(FAILSAFE):
        Serial.print(" FAILSAFE");
        break;
      case(MANUAL):
        Serial.print(" MANUAL");
        break;
      case(STABILISED):
        Serial.print(" STABILISED");
        break;
      case(AUTO):
        Serial.print(" AUTO");
        break;
    }
    Serial.print(" - ");
    Serial.println(millis() - pwmPitch.lastupdate);
    
    Serial.print(F("Roll: "));
    Serial.print(planeState.roll);
    Serial.print(F(" Pitch: "));
    Serial.print(planeState.pitch);
    Serial.print(F(" Heading: "));
    Serial.print(planeState.heading);
    Serial.println(F(""));
    
    log_time = millis() + 1000;
  }

  loop_count++;
}
