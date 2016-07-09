#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Simple_AHRS.h>


#define PWM_ROLL_PIN     6
#define PWM_ROLL_INDEX   0

#define PWM_PITCH_PIN    7
#define PWM_PITCH_INDEX  1

#define PWM_THROTTLE_PIN    8
#define PWM_THROTTLE_INDEX  2

#define PWM_YAW_PIN      9
#define PWM_YAW_INDEX    3

#define PWM_MODE_PIN     10
#define PWM_MODE_INDEX   4

#define PWM_AUX_PIN     11
#define PWM_AUX_INDEX   5

#define FAILSAFE_TIMEOUT 100

#define IMU_MERGE_FACTOR 0.3

#define IMU_UPDATE_RATE 100
#define PID_UPDATE_RATE 100
#define SERVO_UPDATE_RATE 100
#define SERIAL_UPDATE_RATE 3
#define MODE_UPDATE_RATE 100

unsigned long int next_imu_update = 0;
unsigned long int next_pid_update = 0;
unsigned long int next_servo_update = 0;
unsigned long int next_serial_update = 0;
unsigned long int next_mode_update = 0;

Servo throttle;
Servo rudder;
Servo elevator;
Servo aileron;

Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);

Adafruit_Simple_AHRS          ahrs(&accel, &mag);
float seaLevelPressure;


struct PlaneState {
  float roll;
  float pitch;
  float heading;
  float altitude;
};

PlaneState planeState = {0,0,0,0};
PlaneState targetState = {0,0,0,0};

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


PIDController elevatorPID = PIDController(2,0,0);
PIDController aileronPID = PIDController(2,0,0);
PIDController altitudePID = PIDController(1,0.1,0);

struct PWMInputPin {
  unsigned long riseTime;
  uint16_t highTime;
};

PWMInputPin PWMPins[6] = {0};

void setupPin(uint8_t pin) {
  digitalWrite(pin,HIGH); 
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

// Pins 8 - 13
uint8_t pinMaskHigh = B00001111;
uint8_t previousStateHigh = 0;
ISR (PCINT0_vect) {
  uint8_t changed = (PINB & pinMaskHigh) ^ previousStateHigh;
  previousStateHigh = PINB & pinMaskHigh;
  if (changed & B00000001) { // Pin 8
    if (PINB & B00000001) { // High
      PWMPins[PWM_THROTTLE_INDEX].riseTime = micros();
    } else { // Low
      PWMPins[PWM_THROTTLE_INDEX].highTime = micros() - PWMPins[PWM_THROTTLE_INDEX].riseTime;
    }
  }
  if (changed & B00000010) { // Pin 9
    if (PINB & B00000010) { // High
      PWMPins[PWM_YAW_INDEX].riseTime = micros();
    } else { // Low
      PWMPins[PWM_YAW_INDEX].highTime = micros() - PWMPins[PWM_YAW_INDEX].riseTime;
    }
  }
  if (changed & B00000100) { // Pin 10
    if (PINB & B00000100) { // High
      PWMPins[PWM_MODE_INDEX].riseTime = micros();
    } else { // Low
      PWMPins[PWM_MODE_INDEX].highTime = micros() - PWMPins[PWM_MODE_INDEX].riseTime;
    }
  }
  if (changed & B00001000) { // Pin 11
    if (PINB & B00001000) { // High
      PWMPins[PWM_AUX_INDEX].riseTime = micros();
    } else { // Low
      PWMPins[PWM_AUX_INDEX].highTime = micros() - PWMPins[PWM_AUX_INDEX].riseTime;
    }
  }
}


// Pins 0 - 7
uint8_t pinMaskLow = B11000000;
uint8_t previousStateLow = 0;
ISR (PCINT2_vect) {

  uint8_t changed = (PIND & pinMaskLow) ^ previousStateLow;
  previousStateLow = PIND & pinMaskLow;
  
  if (changed & B01000000) { // Pin 6
    if (PIND & B01000000) { // High
      PWMPins[PWM_ROLL_INDEX].riseTime = micros();
    } else { // Low
      PWMPins[PWM_ROLL_INDEX].highTime = micros() - PWMPins[PWM_ROLL_INDEX].riseTime;
    }
  }

  if (changed & B10000000) { // Pin 7
    if (PIND & B10000000) { // High
      PWMPins[PWM_PITCH_INDEX].riseTime = micros();
    } else { // Low
      PWMPins[PWM_PITCH_INDEX].highTime = micros() - PWMPins[PWM_PITCH_INDEX].riseTime;
    }
  }
}

float getAirPressure(uint16_t samples) {
  float pressure = 0;
  sensors_event_t bmp_event;
  uint16_t i=0;
  while(i<samples){
    bmp.getEvent(&bmp_event);
    if (bmp_event.pressure)
    {
      i++;
      pressure += bmp_event.pressure;
    }
  }
  return pressure / samples;
}

void setup() {

  Serial.begin(115200);

  Serial.println("Setting up PWM input");
  setupPin(PWM_YAW_PIN);
  setupPin(PWM_ROLL_PIN);
  setupPin(PWM_PITCH_PIN);
  setupPin(PWM_THROTTLE_PIN);
  setupPin(PWM_MODE_PIN);
  setupPin(PWM_AUX_PIN);
  
  Serial.println("Setting up sensors...");
  accel.begin();
  mag.begin();
  bmp.begin();
  
  Serial.println("Setting up servos...");
  rudder.attach(14);
  throttle.attach(15);
  elevator.attach(16);
  aileron.attach(17);

  Serial.println("Sampling air pressure");
  seaLevelPressure = getAirPressure(100);
  Serial.println("READY");
}

uint16_t loop_count = 0;
unsigned long log_time = 0;
void loop() {

  if (next_mode_update < millis()) {
    next_mode_update += 1000/MODE_UPDATE_RATE;
    if((PWMPins[PWM_ROLL_INDEX].riseTime/1000) < (millis() - (FAILSAFE_TIMEOUT)) || (PWMPins[PWM_ROLL_INDEX].riseTime/1000)  < 1000) {
      mode = FAILSAFE;
    } else {
      if (PWMPins[PWM_MODE_INDEX].highTime  > 1600) {
        if (mode != STABILISED) {
          targetState.altitude = planeState.altitude;
          targetState.pitch = 0;
        }
        mode = STABILISED;
      } else {
        mode = MANUAL;
      } 
    }
  }

  if (next_imu_update < millis()) {
    next_imu_update += 1000/IMU_UPDATE_RATE;
    sensors_vec_t   orientation;
    if (ahrs.getOrientation(&orientation))
    {
      planeState.roll = (1-IMU_MERGE_FACTOR)*planeState.roll + IMU_MERGE_FACTOR*orientation.roll;
      planeState.pitch = (1-IMU_MERGE_FACTOR)*planeState.pitch  + IMU_MERGE_FACTOR*orientation.pitch;
      planeState.heading = (1-IMU_MERGE_FACTOR)*planeState.heading + IMU_MERGE_FACTOR*orientation.heading;
    }
    sensors_event_t bmp_event;
    bmp.getEvent(&bmp_event);
    if (bmp_event.pressure)
    {
      float temperature;
      bmp.getTemperature(&temperature);
      planeState.altitude = (1-IMU_MERGE_FACTOR)*planeState.altitude +
        IMU_MERGE_FACTOR*bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
    }
  }

  if (next_pid_update < millis()) {
    next_pid_update += 1000/PID_UPDATE_RATE;
    if( mode == STABILISED) {
      elevatorPID.update(planeState.pitch, targetState.pitch);
      aileronPID.update(planeState.roll, targetState.roll);
      altitudePID.update(planeState.altitude, targetState.altitude);
      targetState.pitch = altitudePID.output;
    }
  }

  if (next_servo_update < millis()) {
    next_servo_update += 1000/SERVO_UPDATE_RATE;
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
        //unsigned long start = millis();
        throttle.writeMicroseconds(PWMPins[PWM_THROTTLE_INDEX].highTime);
        aileron.writeMicroseconds(PWMPins[PWM_ROLL_INDEX].highTime);
        rudder.writeMicroseconds(PWMPins[PWM_YAW_INDEX].highTime);
        elevator.writeMicroseconds(PWMPins[PWM_PITCH_INDEX].highTime);
        //Serial.println(millis() - start);
        break;
      case(STABILISED):
        digitalWrite(13,LOW);
        
        throttle.writeMicroseconds(PWMPins[PWM_THROTTLE_INDEX].highTime);
        aileron.write(90 + aileronPID.output);
        rudder.writeMicroseconds(PWMPins[PWM_YAW_INDEX].highTime);
        elevator.write(90 + elevatorPID.output);
        break;
    }
  }

  if (next_serial_update < millis()) {
    next_serial_update += 1000/SERIAL_UPDATE_RATE;  
    Serial.println("");
    Serial.print("Elevator:");
    Serial.print(PWMPins[PWM_PITCH_INDEX].highTime);
    Serial.print("    Aileron:");
    Serial.print(PWMPins[PWM_ROLL_INDEX].highTime);
    Serial.print("    Throttle:");
    Serial.print(PWMPins[PWM_THROTTLE_INDEX].highTime);
    Serial.print("    Rudder:");
    Serial.print(PWMPins[PWM_YAW_INDEX].highTime);
    Serial.print("    MODE:");
    switch (mode) {
      case(FAILSAFE):
        Serial.println(" FAILSAFE");
        break;
      case(MANUAL):
        Serial.println(" MANUAL");
        break;
      case(STABILISED):
        Serial.println(" STABILISED");
        break;
      case(AUTO):
        Serial.println(" AUTO");
        break;
    }
    
    Serial.print(F("Current Roll: "));
    Serial.print(planeState.roll);
    Serial.print(F(" Pitch: "));
    Serial.print(planeState.pitch);
    Serial.print(F(" Heading: "));
    Serial.print(planeState.heading);
    Serial.print(F(" Altitude: "));
    Serial.print(planeState.altitude);
    Serial.println(F(""));

    Serial.print(F("Target Roll: "));
    Serial.print(targetState.roll);
    Serial.print(F(" Pitch: "));
    Serial.print(targetState.pitch);
    Serial.print(F(" Heading: "));
    Serial.print(targetState.heading);
    Serial.print(F(" Altitude: "));
    Serial.print(targetState.altitude);
    Serial.println(F(""));
  }
}
