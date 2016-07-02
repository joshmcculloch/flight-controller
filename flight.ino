
#define PWM_THROTTLE_PIN 8
#define PWM_ROLL_PIN     6
#define PWM_PITCH_PIN    7
#define PWM_YAW_PIN      9
#define PWM_MODE_PIN     5;

#define FAILSAFE_TIMEOUT 100


enum Mode{FAILSAFE, MANUAL, STABILISED, AUTO};
Mode mode = FAILSAFE;


void pinChangeHandler();


class PWMInput {
  
  uint8_t pinState = 0;
  uint8_t pin;
  unsigned long riseTime = 0;

  void checkPin() {
    uint8_t currentState = digitalRead(pin);
    if (pinState != currentState) {
      
      if (currentState == 0) {
        unsigned long duration = micros() - riseTime;
        duty = ((duration -1000) * 256) / 1000;
      } else {
        riseTime = micros();
      }
      pinState = currentState;
      lastupdate = millis();
    } 
  }
  
  public:
  unsigned long lastupdate = 0;
  uint8_t duty = 0;
  
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

void pinChangeHandler() {
  pwmYaw.pinChange();
  pwmRoll.pinChange();
  pwmPitch.pinChange();
  pwmThrottle.pinChange();
}

void setup() {

  Serial.begin(115200);

  Serial.println("READY");
}

void loop() {
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

  if(pwmPitch.lastupdate < (millis() - FAILSAFE_TIMEOUT) || pwmPitch.lastupdate < 1000) {
    mode = FAILSAFE;
  } else {
    mode = MANUAL;
  }
  delay(10);
}
