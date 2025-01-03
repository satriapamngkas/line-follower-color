#include <AFMotor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

// Motor definitions
AF_DCMotor motorFrontLeft(0);   // Motor 1
AF_DCMotor motorBackLeft(1);    // Motor 2
AF_DCMotor motorBackRight(2);   // Motor 3
AF_DCMotor motorFrontRight(3);  // Motor 4

// IR Sensor pins
#define IR_LEFT A0
#define IR_RIGHT A1

// Ultrasonic sensor pins
#define TRIG_PIN A2
#define ECHO_PIN A3

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
#define SERVOMIN 150  // Min pulse length
#define SERVOMAX 600  // Max pulse length

// Servo positions tracker
uint16_t servoPositions[4] = { 50, 60, 30, 65 };

// Color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);

// Bluetooth Serial
SoftwareSerial bluetooth(BT_RX, BT_TX);

// Robot mode
bool isManual = false;

void setup() {
  delay(3000);
  // Initialize IR Sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Initialize Ultrasonic Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize Bluetooth
  bluetooth.begin(9600);
  Serial.begin(9600);

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);

  // Set default servo positions
  setServoAngle(SERVO0, 50);
  setServoAngle(SERVO3, 65);
  delay(500);
  setServoAngle(SERVO1, 60);
  delay(500);
  // setServoAngle(SERVO1, -15);
  // delay(500);
  setServoAngle(SERVO2, 30);
  delay(500);
  // setServoAngle(SERVO3, 60);
  // delay(500);
  // setServoAngle(SERVO1, 60);
  // delay(500);
  // setServoAngle(SERVO2, 30);
  // moveForward();

  // Initialize color sensor
  if (!tcs.begin()) {
    Serial.println("No TCS34725 found. Check connections.");
    while (1)
      ;
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

  // detectColor();

  // performPickup("Test pickup");
  // delay(2000);
  // performDrop("Test drop");

  // getUltrasonicDistance();
  delay(100);

  if (!isManual) {
    lineFollowWithObstacleDetectionAndColor();
  }
}

void handleBluetoothCommand(char command) {
  Serial.print("Handling Bluetooth command: ");
  Serial.println(command);

  switch (command) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopMotors(); break;
    case 'M':
      isManual = true;
      Serial.println("Manual mode activated.");
      break;
    case 'A':
      isManual = false;
      Serial.println("Automatic mode activated.");
      break;
    case '1': adjustServo(SERVO0, -10); break;
    case '2': adjustServo(SERVO0, 10); break;
    case '3': adjustServo(SERVO1, -10); break;
    case '4': adjustServo(SERVO1, 10); break;
    case '5': adjustServo(SERVO3, 65); break;
    case '6': adjustServo(SERVO2, 10); break;
    case '7': adjustServo(SERVO3, -10); break;
    case '8': adjustServo(SERVO3, 10); break;
    default: Serial.println("Unknown command."); break;
  }
}

void lineFollowWithObstacleDetectionAndColor() {
  bool left = digitalRead(IR_LEFT);
  bool right = digitalRead(IR_RIGHT);
  float distance = getUltrasonicDistance();

  Serial.print("IR Left: ");
  Serial.print(left);
  Serial.print(", IR Right: ");
  Serial.print(right);
  Serial.print(", Distance: ");
  Serial.println(distance);

  if (distance < 15.0) {
    Serial.println("Obstacle detected! Stopping.");
    stopMotors();
    return;
  }

  detectColor(distance);

  if (left && right) {
    moveForward();
  } else if (!left && right) {
    turnRight();
  } else if (left && !right) {
    turnLeft();
  } else {

    moveBackward();
  }
}

void detectColor(float distance) {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  Serial.print("Color readings - R: ");
  Serial.print(r);
  Serial.print(", G: ");
  Serial.print(g);
  Serial.print(", B: ");
  Serial.print(b);
  Serial.print(", C: ");
  Serial.println(c);

  if (r > g && r > b && r > 2000 && distance <= 15.5) {
    Serial.println("Red detected.");
    performPickup("Red");
  } else if (g > r && g > b && g > 2000 && distance <= 15.5) {
    Serial.println("Green detected.");
    performPickup("Green");
  } else if (b > r && b > g && b > 2000 && distance <= 15.5) {
    Serial.println("Blue detected.");
    performPickup("Blue");
  }
}

float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration * 0.034) / 2;

  Serial.print("Ultrasonic distance measured: ");
  Serial.println(distance);

  return distance;
}

void performPickup(String color) {
  Serial.print("Starting pickup process for color: ");
  Serial.println(color);

  setServoAngle(SERVO3, 40);
  delay(500);
  setServoAngle(SERVO1, -15);
  delay(500);
  setServoAngle(SERVO3, 65);
  delay(500);

  Serial.println("Pickup complete. Returning servos to default positions.");

  setServoAngle(SERVO1, 60);
  // delay(500);
  setServoAngle(SERVO2, 30);
  delay(500);
}

void performDrop(String color) {
  Serial.print("Starting drop process for color: ");
  Serial.println(color);

  setServoAngle(SERVO1, -15);
  delay(500);
  setServoAngle(SERVO3, 40);
  delay(500);

  Serial.println("Drop complete. Returning servos to default positions.");

  setServoAngle(SERVO1, 60);
  // delay(500);
  setServoAngle(SERVO2, 30);
  delay(500);
  setServoAngle(SERVO3, 65);
  delay(500);
}

void adjustServo(uint8_t servo, int16_t delta) {
  uint16_t newAngle = constrain(servoPositions[servo] + delta, 0, 180);
  setServoAngle(servo, newAngle);
  servoPositions[servo] = newAngle;

  Serial.print("Servo ");
  Serial.print(servo);
  Serial.print(" adjusted to ");
  Serial.println(newAngle);
}

void setServoAngle(uint8_t servo, uint16_t angle) {
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo, 0, pulse);

  Serial.print("Set servo ");
  Serial.print(servo);
  Serial.print(" to angle: ");
  Serial.println(angle);
}

void moveForward() {
  Serial.println("Moving forward.");
  motorFrontLeft.setSpeed(150);
  motorFrontRight.setSpeed(150);
  motorBackLeft.setSpeed(150);
  motorBackRight.setSpeed(150);
  motorFrontLeft.run(FORWARD);
  motorFrontRight.run(FORWARD);
  motorBackLeft.run(FORWARD);
  motorBackRight.run(FORWARD);
}

void moveBackward() {
  Serial.println("Moving backward.");
  motorFrontLeft.setSpeed(150);
  motorFrontRight.setSpeed(150);
  motorBackLeft.setSpeed(150);
  motorBackRight.setSpeed(150);
  motorFrontLeft.run(BACKWARD);
  motorFrontRight.run(BACKWARD);
  motorBackLeft.run(BACKWARD);
  motorBackRight.run(BACKWARD);
}

void turnLeft() {
  Serial.println("Turning left.");
  motorFrontLeft.setSpeed(150);
  motorFrontRight.setSpeed(150);
  motorBackLeft.setSpeed(150);
  motorBackRight.setSpeed(150);
  motorFrontLeft.run(BACKWARD);
  motorFrontRight.run(FORWARD);
  motorBackLeft.run(BACKWARD);
  motorBackRight.run(FORWARD);
}

void turnRight() {
  Serial.println("Turning right.");
  motorFrontLeft.setSpeed(150);
  motorFrontRight.setSpeed(150);
  motorBackLeft.setSpeed(150);
  motorBackRight.setSpeed(150);
  motorFrontLeft.run(FORWARD);
  motorFrontRight.run(BACKWARD);
  motorBackLeft.run(FORWARD);
  motorBackRight.run(BACKWARD);
}

void stopMotors() {
  Serial.println("Stopping motors.");
  motorFrontLeft.run(RELEASE);
  motorFrontRight.run(RELEASE);
  motorBackLeft.run(RELEASE);
  motorBackRight.run(RELEASE);
}
