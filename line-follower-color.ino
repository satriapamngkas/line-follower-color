#include <AFMotor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

AF_DCMotor motorRight(2);
AF_DCMotor motorLeft(3);

// Bluetooth module pins
#define BT_RX 2
#define BT_TX 13

// PCA9685 Servo Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channels on PCA9685
#define SERVO0 0
#define SERVO1 1
#define SERVO2 2
#define SERVO3 3

// Servo min and max pulse lengths
#define SERVOMIN 150
#define SERVOMAX 600

// Servo positions
uint16_t capit = 90;
uint16_t bukaCapit = 40;
uint16_t servo_positions[4] = { 50, 60, 30, 65 };
uint16_t initialServo[4] = { 155, 135, 90, 90 };

// Color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);

SoftwareSerial bluetooth(BT_RX, BT_TX);

bool isManual = false;
String current_color = "Merah";
int motor_speed = 150;
int sorted_red = 0;
int sorted_green = 0;
int sorted_blue = 0;
String color_data = "";
int bufferIndex = 0;
bool isHold = false;
bool colorDetection = true;
bool delayDetection = false;
unsigned long startDelayDetection = 0;
unsigned long lastColorDetection = 0;
unsigned long actionStartTime = 0;
bool isActionRunning = false;
char currentAction;

void setup() {

  // Initialize Bluetooth
  bluetooth.begin(9600);
  Serial.begin(9600);

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);

  // Set default servo positions
  setServoAngle(SERVO0, initialServo[0]);  //kecil kanan, serong kiri 175
  delay(500);
  setServoAngle(SERVO3, initialServo[3]);  //90 capit, 40 buka
  delay(500);
  setServoAngle(SERVO2, initialServo[2]);  //kecil naik, 90 aman tertinggi, 150 terendah, posisi awal 90
  delay(500);
  setServoAngle(SERVO1, initialServo[1]);  //kecil maju, 150 aman mundur, 50 maju, posisi awal 100
  delay(500);

  motorRight.setSpeed(150);
  motorLeft.setSpeed(150);

  motorRight.run(RELEASE);
  motorLeft.run(RELEASE);

  if (!tcs.begin()) {
    Serial.println("No TCS34725 found. Check connections.");
  } else {
    Serial.println("TCS34725 Initialized");
  }

  Serial.println("Robot initialized.");
}

void loop() {
  if (bluetooth.available()) {
    char command = bluetooth.read();
    Serial.print("Received command: ");
    Serial.println(command);
    handleBluetoothCommand(command);
  }
  if (isActionRunning && millis() - actionStartTime >= 1000) {
    stopMotors();
    isActionRunning = false;
  }
}

void handleBluetoothCommand(char command) {
  if (command == 'S') {
    stopMotors();
    isActionRunning = false;
    return;
  }

  if (command != 'A' && !isManual) {
    isManual = true;
    stopMotors();
  }

  switch (command) {
    case 'F':
    case 'B':
    case 'L':
    case 'R':
      executeMotorCommand(command);
      break;
    case 'M':
      isManual = true;
      stopMotors();
      Serial.println("Manual mode activated.");
      break;
    case '1':
      adjustServo(SERVO1, -10);
      adjustServo(SERVO2, -10);
      break;
    case '2': adjustServo(SERVO0, 10); break;
    case '3': adjustServo(SERVO0, -10); break;
    case '4':
      adjustServo(SERVO1, 10);
      adjustServo(SERVO2, 10);
      break;
    case '5':
      detectColor();
      setServoAngle(SERVO3, 90);
      color_data = current_color + "#" + String(sorted_red) + "#" + String(sorted_green) + "#" + String(sorted_blue);
      Serial.println("Sent color_data: " + color_data);
      bluetooth.println(color_data);
      break;
    case '6':
      setServoAngle(SERVO3, 40);
      if (current_color == "Merah") {
        sorted_red++;
      } else if (current_color == "Hijau") {
        sorted_green++;
      } else if (current_color == "Biru") {
        sorted_blue++;
      }
      color_data = current_color + "#" + String(sorted_red) + "#" + String(sorted_green) + "#" + String(sorted_blue);
      Serial.println("Sent color_data: " + color_data);
      bluetooth.println(color_data);
      break;
    case '7': adjustServo(SERVO2, -10); break;
    case '8': adjustServo(SERVO2, 10); break;
    default: Serial.println("Unknown command."); break;
  }
}


void detectColor() {
  float red, green, blue;
  unsigned long timeNow = millis();
  if (timeNow - lastColorDetection >= 60) {
    tcs.getRGB(&red, &green, &blue);
    tcs.setInterrupt(true);

    Serial.print("R: ");
    Serial.print(int(red));
    Serial.print(" G: ");
    Serial.print(int(green));
    Serial.print(" B: ");
    Serial.println(int(blue));

    if (red > green && red > blue && red > 125 /* && distance <= 15*/) {
      // stopMotors();
      Serial.println("Red detected.");
      current_color = "Merah";

    } else if (green > red && green > blue && green > 120 /* && distance <= 15*/) {
      // stopMotors();
      Serial.println("Green detected.");
      current_color = "Hijau";
    }
    lastColorDetection = timeNow;
  }
}


void executeMotorCommand(char command) {
  switch (command) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
  }
  actionStartTime = millis();
  isActionRunning = true;
}

void adjustServo(uint8_t servo, int16_t delta) {
  uint16_t newAngle = constrain(servo_positions[servo] + delta, 0, 270);
  // uint16_t newAngle = servo_positions[servo] + delta;
  setServoAngle(servo, newAngle);
  servo_positions[servo] = newAngle;

  Serial.print("Servo ");
  Serial.print(servo);
  Serial.print(" adjusted to ");
  Serial.println(newAngle);
}

void moveForward() {
  Serial.println("Moving forward.");
  motorRight.setSpeed(motor_speed);
  motorLeft.setSpeed(motor_speed);
  motorRight.run(FORWARD);
  motorLeft.run(FORWARD);
}

void moveBackward() {
  Serial.println("Moving backward.");
  motorRight.setSpeed(motor_speed);
  motorLeft.setSpeed(motor_speed);
  motorRight.run(BACKWARD);
  motorLeft.run(BACKWARD);
}

void turnLeft() {
  Serial.println("Turning left.");
  motorRight.setSpeed(motor_speed);
  motorLeft.setSpeed(motor_speed);
  motorRight.run(FORWARD);
  motorLeft.run(BACKWARD);
}

void turnRight() {
  Serial.println("Turning right.");
  motorRight.setSpeed(motor_speed);
  motorLeft.setSpeed(motor_speed);
  motorRight.run(BACKWARD);
  motorLeft.run(FORWARD);
}

void stopMotors() {
  Serial.println("Stopping motors.");
  motorRight.run(RELEASE);
  motorLeft.run(RELEASE);
}

void setServoAngle(uint8_t servo, uint16_t angle) {
  uint16_t pulse = map(angle, 0, 270, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo, 0, pulse);
  Serial.print("Set servo ");
  Serial.print(servo);
  Serial.print(" to angle: ");
  Serial.println(angle);
}
