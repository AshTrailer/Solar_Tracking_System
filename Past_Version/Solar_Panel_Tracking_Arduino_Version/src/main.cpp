#include <Arduino.h>
#include <RtcDS1302.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <Adafruit_MMC56x3.h>
#include <Wire.h>

#define MAG_DECLINATION 11.0  // Magnetic declination in degrees

#define countof(a) (sizeof(a) / sizeof(a[0]))
#define MAX_MOTOR_SPEED_DEG_PER_SEC  15.0
#define MAX_MOTOR_ACCEL_DEG_PER_SEC2 10.0
#define MAX_MOTOR_ANGLE_DEG          360
#define ANALOG_RESOLUTION            1023

const int BUTTON1_PIN  = 2;
const int BUTTON2_PIN  = 3;
const int BUTTON3_PIN  = 4;
const int BUTTON4_PIN  = 5;
//const unsigned long BUTTON4_DEBOUNCE_MS = 50;
//const unsigned long BUTTON4_REPEAT_MS   = 200;

const int DS1302_RST  = 6;
const int DS1302_DAT  = 7;
const int DS1302_CLK  = 8;

const int SERVO_PIN1  = 9;
#define MAX_SERVO_SPEED_DEG_PER_SEC  10.0
#define SERVO_UPDATE_MS              20
#define SERVO_MAX_ANGLE              180

const int STEPPER_PIN1 = 10;
const int STEPPER_PIN2 = 11;
const int STEPPER_PIN3 = 12;
const int STEPPER_PIN4 = 13;
const int STEPS_PER_REV = 2048;

#define VRX_PIN A0
#define VRY_PIN A1
const int JOY_CENTER_X = 521; 
const int JOY_CENTER_Y = 521; 
const int JOY_DEADZONE = 10;

unsigned long lastPrintTime = 0;

enum ButtonMode {
  BUTTON_TOGGLE,
  BUTTON_HOLD,
  BUTTON_PULSE,
  BUTTON_REPEAT
};

class Button {
  public:
    Button(int pin, ButtonMode mode, unsigned long debounce = 20, unsigned long repeatInterval = 500)
      : _pin(pin), _mode(mode), _debounce(debounce), _repeatInterval(repeatInterval),
        _lastState(HIGH), _buttonState(HIGH), _lastDebounceTime(0),
        _output(false), _lastOutputTime(0) {
      pinMode(pin, INPUT_PULLUP);
    }

    bool update() {
      bool reading = digitalRead(_pin);

      debounceCheck(reading);

      if ((millis() - _lastDebounceTime) > _debounce) {
        bool lastLogic = _buttonState;
        _buttonState = reading;

        switch (_mode) {
          case BUTTON_TOGGLE:
            handleToggle(lastLogic);
            break;

          case BUTTON_HOLD:
            handleHold();
            break;

          case BUTTON_PULSE:
            handlePulse(lastLogic);
            break;

          case BUTTON_REPEAT:
            handleRepeat();
            break;
        }
      }

      _lastState = reading;
      return _output;
    }

  private:
    int _pin;
    ButtonMode _mode;
    unsigned long _debounce;
    unsigned long _repeatInterval;

    bool _lastState;
    bool _buttonState;
    unsigned long _lastDebounceTime;

    bool _output;
    unsigned long _lastOutputTime;

    void debounceCheck(bool reading) {
      if (reading != _lastState) {
        _lastDebounceTime = millis();
      }
    }

    void handleToggle(bool lastLogic) {
      if (lastLogic == HIGH && _buttonState == LOW) {
        _output = !_output;
      }
    }

    void handleHold() {
      _output = (_buttonState == LOW);
    }

    void handlePulse(bool lastLogic) {
      _output = (lastLogic == HIGH && _buttonState == LOW);
    }

    void handleRepeat() {
      if (_buttonState == LOW) {
        if (millis() - _lastOutputTime >= _repeatInterval) {
          _output = true;
          _lastOutputTime = millis();
        } else {
          _output = false;
        }
      } else {
        _output = false;
      }
    }
};

Button btn1(BUTTON1_PIN, BUTTON_PULSE);
Button btn2(BUTTON2_PIN, BUTTON_PULSE);
Button btn3(BUTTON3_PIN, BUTTON_PULSE);
Button btn4(BUTTON4_PIN, BUTTON_PULSE);
//Button btn1(BUTTON1_PIN, BUTTON_TOGGLE);
//Button btn2(BUTTON2_PIN, BUTTON_HOLD);
//Button btn3(BUTTON3_PIN, BUTTON_PULSE);
//Button btn4(BUTTON4_PIN, BUTTON_REPEAT, BUTTON4_DEBOUNCE_MS, BUTTON4_REPEAT_MS);

void printDateTime(const RtcDateTime& Xdt)
{
  char datestring[26];

  snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            Xdt.Month(),
            Xdt.Day(),
            Xdt.Year(),
            Xdt.Hour(),
            Xdt.Minute(),
            Xdt.Second() );
  Serial.print(datestring);
}

class MyServo {
  public:
    MyServo(int pin, float maxSpeedDegPerSec = 20.0) 
      : servoPin(pin), Servo_currentAngle(0), targetAngle(0),
        MAX_SPEED(maxSpeedDegPerSec), lastUpdateMillis(0) {}

    void begin() {
      myservo.attach(servoPin);
      myservo.write((int)Servo_currentAngle);
      lastUpdateMillis = millis();
    }

    void moveTo(float angle) {
      targetAngle = constrain(angle, 0, SERVO_MAX_ANGLE);
    }

    void update() {
      unsigned long now = millis();
      float dt = (now - lastUpdateMillis) / 1000.0f;
      lastUpdateMillis = now;

      float diff = targetAngle - Servo_currentAngle;
      float maxStep = MAX_SPEED * dt;

      if (abs(diff) > maxStep) {
        Servo_currentAngle += (diff > 0 ? maxStep : -maxStep);
      } 
      else {
        Servo_currentAngle = targetAngle;
      }
      myservo.write((int)Servo_currentAngle);
    }

    float getCurrentAngle() {
      return Servo_currentAngle;
    }

  private:
    Servo myservo;
    int servoPin;
    float Servo_currentAngle;
    float targetAngle;
    float MAX_SPEED;  // degrees per second
    unsigned long lastUpdateMillis;
};

int readTargetAngleFromSerial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); 
    int angle = input.toInt(); 
    if (angle >= 0 && angle <= SERVO_MAX_ANGLE) {
      return angle;
    } 
    else {
      Serial.println("Invalid angle! Enter 0 ~ 180.");
    }
  }
  return -1;
}

class StepperControl {
  public:
    StepperControl(uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4,
                   float maxSpeedDegPerSec = MAX_MOTOR_SPEED_DEG_PER_SEC,
                   float maxAccelDegPerSec2 = MAX_MOTOR_ACCEL_DEG_PER_SEC2)
      : stepper(AccelStepper::FULL4WIRE, pin1, pin3, pin2, pin4)
    {
      this->maxSpeedDeg = maxSpeedDegPerSec;
      this->maxAccelDeg = maxAccelDegPerSec2;
      Motor_currentAngle = 0;
      defaultAngle = 0;
    }

    void begin() {
      stepper.setMaxSpeed(degToSteps(maxSpeedDeg));
      stepper.setAcceleration(degToSteps(maxAccelDeg));
      stepper.setCurrentPosition(0);
      Motor_currentAngle = 0;
      defaultAngle = 0;
    }

    void moveTo(float angle) {
      angle = constrain(angle, 0, MAX_MOTOR_ANGLE_DEG); 
      targetAngle = angle;
      long targetSteps = degToSteps(targetAngle);
      stepper.moveTo(targetSteps);
    }

    void update() {
      stepper.run();
      Motor_currentAngle = stepsToDeg(stepper.currentPosition());
    }

    float getCurrentAngle() {
      return Motor_currentAngle;
    }

    void setCurrentAngle(float angle) {
      angle = constrain(angle, 0, MAX_MOTOR_ANGLE_DEG);
      Motor_currentAngle = angle;
      targetAngle = angle;
      long posSteps = degToSteps(angle);
      stepper.setCurrentPosition(posSteps);
    }

    void returnToDefault() {
      moveTo(defaultAngle);
    }

    void setDefaultAngle(float angle) {
      defaultAngle = constrain(angle, 0, MAX_MOTOR_ANGLE_DEG);
    }

  private:
    AccelStepper stepper;
    float maxSpeedDeg;
    float maxAccelDeg;
    float Motor_currentAngle;
    float targetAngle;
    float defaultAngle;

    long degToSteps(float deg) {
      return (long)((deg / 360.0) * STEPS_PER_REV);
    }

    float stepsToDeg(long steps) {
      return (steps * 360.0) / STEPS_PER_REV;
    }
};

class MagnetometerNorth {
  public:
    MagnetometerNorth(Adafruit_MMC5603* sensor, StepperControl* stepper)
      : mag(sensor), stepper(stepper),
        offsetX(0), offsetY(0),
        calibrated(false) {}

    void begin() {
      if (!mag->begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {
        Serial.println("No MMC5603 detected.");
        while(1) delay(10);
      }
      Serial.println("MMC5603 initialized.");
      mag->setContinuousMode(true);
      mag->setDataRate(100); // 100 Hz
    }

  void calibrateHardIron(unsigned long duration_ms = 15000) {
    Serial.println("Rotate sensor horizontally and tilt slightly for calibration...");
    unsigned long startTime = millis();

    float minX =  1e6, maxX = -1e6;
    float minY =  1e6, maxY = -1e6;
    float minZ =  1e6, maxZ = -1e6;

    while (millis() - startTime < duration_ms) {
      sensors_event_t event;
      mag->getEvent(&event);

      if (abs(event.magnetic.x) > 200 || abs(event.magnetic.y) > 200 || 
          abs(event.magnetic.z) > 200) continue;

      if (event.magnetic.x < minX) minX = event.magnetic.x;
      if (event.magnetic.x > maxX) maxX = event.magnetic.x;
      if (event.magnetic.y < minY) minY = event.magnetic.y;
      if (event.magnetic.y > maxY) maxY = event.magnetic.y;
      if (event.magnetic.z < minZ) minZ = event.magnetic.z;
      if (event.magnetic.z > maxZ) maxZ = event.magnetic.z;

      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 200) {
        lastPrint = millis();
        Serial.print("X: "); Serial.print(event.magnetic.x,3);
        Serial.print(" | Y: "); Serial.print(event.magnetic.y,3);
        Serial.print(" | Z: "); Serial.print(event.magnetic.z,3);
        Serial.print(" | minX: "); Serial.print(minX,3);
        Serial.print(" maxX: "); Serial.print(maxX,3);
        Serial.print(" | minY: "); Serial.print(minY,3);
        Serial.print(" maxY: "); Serial.print(maxY,3);
        Serial.println();
      }
    }

    offsetX = (maxX + minX) / 2.0;
    offsetY = (maxY + minY) / 2.0;

    calibrated = true;

    Serial.println("=== Calibration done ===");
    Serial.print("OffsetX: "); Serial.print(offsetX,3);
    Serial.print(" | OffsetY: "); Serial.print(offsetY,3);
}


  void calibrateToNorth() {
    sensors_event_t event;
    mag->getEvent(&event);

    float x = event.magnetic.x - offsetX;
    float y = event.magnetic.y - offsetY;

    float heading = atan2(y, x) * 180.0 / PI;
    if (heading < 0) heading += 360.0;

    heading += MAG_DECLINATION;
    if (heading >= 360) heading -= 360;

    stepperOffset = heading;
    stepper->setDefaultAngle(0); 
    calibrated = true;

    Serial.print("Geographic north calibrated. Stepper offset: ");
    Serial.println(stepperOffset, 2);
  }

  float getHeading() {
    sensors_event_t event;
    mag->getEvent(&event);

    float x = event.magnetic.x - offsetX;
    float y = event.magnetic.y - offsetY;

    float heading = atan2(y, x) * 180.0 / PI;
    if (heading < 0) heading += 360.0;

    heading += MAG_DECLINATION;
    if (heading >= 360) heading -= 360;

    return heading;
  }

  void autoCorrectStepper() {
    if (!calibrated) return;

    float currentHeading = getHeading(); 
    float deltaHeading = currentHeading - stepperOffset;

    if (deltaHeading > 180) deltaHeading -= 360;
    if (deltaHeading < -180) deltaHeading += 360;

    if (abs(deltaHeading) < 2.0) return;

    float targetAngle = -deltaHeading;
    if (targetAngle < 0) targetAngle += 360;
    if (targetAngle >= 360) targetAngle -= 360;

    stepper->moveTo(targetAngle);
}

  float getStepperOffset() {
    return stepperOffset;
  }

  private:
    Adafruit_MMC5603* mag;
    StepperControl* stepper;
    float offsetX, offsetY;
    bool calibrated;
    float stepperOffset;
};

struct TwoAxisTarget {
    double horizontal;
    int    vertical;
};
TwoAxisTarget readTwoAxisFromSerial() {
  static String buffer = "";
  TwoAxisTarget target = {-1, -1};

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      int commaIndex = buffer.indexOf(',');
      if (commaIndex != -1) {
        target.horizontal = buffer.substring(0, commaIndex).toDouble();
        target.vertical   = buffer.substring(commaIndex + 1).toInt();
      }
      buffer = "";
      break;
      }
      else {
        buffer += c;
      }
  }

  return target;
}

ThreeWire myWire(DS1302_DAT,DS1302_CLK,DS1302_RST); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

MyServo servo1(SERVO_PIN1);
StepperControl stepper1(STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4);

Adafruit_MMC5603 mmc = Adafruit_MMC5603(0x30);
MagnetometerNorth north(&mmc, &stepper1);

void setup() {
/*
  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  if (!Rtc.IsDateTimeValid()) Rtc.SetDateTime(compiled);
  if (!Rtc.GetIsRunning()) Rtc.SetIsRunning(true);
  if (Rtc.GetIsWriteProtected()) Rtc.SetIsWriteProtected(false);
*/
  
  Serial.begin(115200);
  Wire.begin();

  servo1.begin();

  stepper1.begin();
 
  north.begin();
  north.calibrateHardIron(20000);
  north.calibrateToNorth();
}

void loop() {
  if (btn1.update()) {

  }
  if (btn2.update()) {
    
  }

  if (btn3.update()) {

  }
  if (btn4.update()) {


  }
  
  static TwoAxisTarget lastTarget = {-1, -1};
  unsigned long now = millis();
  if (lastTarget.horizontal != -1 && lastTarget.vertical != -1) {
    stepper1.moveTo(lastTarget.horizontal);
    servo1.moveTo(lastTarget.vertical);
  } 
  else {
    north.autoCorrectStepper();
  }

  stepper1.update();
  servo1.update();

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 5000) {
    lastPrint = now;


    Serial.println();
  }




  
/*
  int x = analogRead(VRX_PIN);
  int y = analogRead(VRY_PIN);

  float speedX = 0.0;
  float speedY = 0.0;
  int dx = x - JOY_CENTER_X;
  int dy = y - JOY_CENTER_Y;

  if (abs(dx) > JOY_DEADZONE) {
    if (dx < 0) {
      speedX = map(dx, -JOY_CENTER_X, -JOY_DEADZONE, 
                  -MAX_MOTOR_SPEED_DEG_PER_SEC, 0);
    }
    else {
      speedX = map(dx, JOY_DEADZONE, ANALOG_RESOLUTION - JOY_CENTER_X, 
                  0, MAX_MOTOR_SPEED_DEG_PER_SEC);
    }
  }
  if (abs(dy) > JOY_DEADZONE) {
    if (dy < 0) {
      speedY = map(dy, -JOY_CENTER_Y, -JOY_DEADZONE, 
                  MAX_SERVO_SPEED_DEG_PER_SEC, 0);
    }
    else {
      speedY = map(dy, JOY_DEADZONE, ANALOG_RESOLUTION - JOY_CENTER_Y,
                  0, -MAX_SERVO_SPEED_DEG_PER_SEC);
    }
  }

  static float Motor_currentAngle = 0.0;
  static unsigned long XlastUpdate = millis();
  static float Servo_currentAngle = 90.0;
  static unsigned long YlastUpdate = millis();

  unsigned long now = millis();
  float Xdt = (now - XlastUpdate) / 1000.0;
  XlastUpdate = now;
  Motor_currentAngle += speedX * Xdt;
  if (Motor_currentAngle < 0) Motor_currentAngle = 0;
  if (Motor_currentAngle > MAX_MOTOR_ANGLE_DEG) Motor_currentAngle = MAX_MOTOR_ANGLE_DEG;
  stepper1.moveTo(Motor_currentAngle);
  stepper1.update();

  if (now - YlastUpdate >= SERVO_UPDATE_MS){
    float Ydt = (now - YlastUpdate) / 1000.0;
    Servo_currentAngle += speedY * Ydt;
    Servo_currentAngle = constrain(Servo_currentAngle, 0, SERVO_MAX_ANGLE);
    servo1.moveTo(Servo_currentAngle);
    servo1.update();
    YlastUpdate = now;
  }

  float Xangle = stepper1.getCurrentAngle();
  float Yangle = servo1.getCurrentAngle();
  float diffX = Motor_currentAngle - Xangle;
  float diffY = Servo_currentAngle - Yangle;
  if (now - lastPrintTime >= 200) {
    lastPrintTime = now;
    Serial.print("X=");
    Serial.print(x);
    Serial.print("; Cal Angle =");
    Serial.print(Motor_currentAngle, 1);
    Serial.print("; Angle =");
    Serial.print(Xangle, 1);
    Serial.print("; Angle Different=");
    Serial.print(diffX, 1);
    Serial.print("; X_Speed=");
    Serial.println(speedX, 1);

    Serial.print(" Y=");
    Serial.print(y);
    Serial.print("; Cal Angle =");
    Serial.print(Servo_currentAngle, 1);
    Serial.print(" Angle =");
    Serial.print(Yangle, 1);
    Serial.print("; Angle Different=");
    Serial.print(diffY, 1);
    Serial.print("; Y_Speed=");
    Serial.println(speedY, 1);
    Serial.println("------------------------");
  }
*/
}



