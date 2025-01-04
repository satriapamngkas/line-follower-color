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
uint16_t servo_positions[4] = { 50, 60, 30, 65 };

// Color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);

SoftwareSerial bluetooth(BT_RX, BT_TX);

bool isManual = false;
String currentColor = "";
int motor_speed = 150;

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
  setServoAngle(SERVO1, 80);
  delay(500);
  // setServoAngle(SERVO1, -15);
  // delay(500);
  setServoAngle(SERVO2, 120);
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
  // delay(100);

  // if (!isManual) {
  //   lineFollowWithObstacleAndColorDetection();
  // }
}

void handleBluetoothCommand(char command) {
  if (command != 'A' && !isManual) {
    isManual = true;
    stopMotors();
  }
  Serial.print("Handling Bluetooth command: ");
  Serial.println(command);

  switch (command) {
    case 'F':
      moveForward(motor_speed);
      delay(1000);
      stopMotors();
      break;
    case 'B':
      moveBackward(motor_speed);
      delay(1000);
      stopMotors();
      break;
    case 'L':
      turnLeft();
      delay(1000);
      stopMotors();
      break;
    case 'R':
      turnRight();
      delay(1000);
      stopMotors();
      break;
    case 'S': stopMotors(); break;
    case 'M':
      isManual = true;
      stopMotors();
      Serial.println("Manual mode activated.");
      break;
    case 'A':
      isManual = false;
      Serial.println("Automatic mode activated.");
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
    case '5': setServoAngle(SERVO3, 65); break;
    case '6': setServoAngle(SERVO3, 40); break;
    default: Serial.println("Unknown command."); break;
  }
}


void lineFollowWithObstacleAndColorDetection() {
  bool left = digitalRead(IR_LEFT);
  bool right = digitalRead(IR_RIGHT);
  float distance = getUltrasonicDistance();

  Serial.print("IR Left: ");
  Serial.print(left);
  Serial.print(", IR Right: ");
  Serial.print(right);
  Serial.print(", Distance: ");
  Serial.println(distance);

  if (distance < 10.0) {
    Serial.println("Obstacle detected! Stopping.");
    stopMotors();
    // return;
  }

  detectColor(distance);

  if (left && right) {
    moveForward(motor_speed);
  } else if (!left && right) {
    turnRight();
  } else if (left && !right) {
    turnLeft();
  } else {

    moveBackward(abs(motor_speed));
  }
}

void detectColor(float distance) {
  float red, green, blue;

  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);

  tcs.setInterrupt(true);  // turn off LED

  Serial.print("R: ");
  Serial.print(int(red));
  Serial.print(" G: ");
  Serial.print(int(green));
  Serial.print(" B: ");
  Serial.println(int(blue));


  if (red > green && red > blue && red > 140 && distance <= 10.5) {
    Serial.println("Red detected.");
    performPickup("Red");
  } else if (green > red && green > blue && green > 100 && distance <= 10.5) {
    Serial.println("Green detected.");
    performPickup("Green");
  // } else if (blue > red && blue > green && blue > 2000 && distance <= 10.5) {
  } else if (blue < red && blue < green && blue > 90 && distance <= 10.5) {
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
  currentColor = color;

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
  currentColor = "";

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
  uint16_t newAngle = constrain(servo_positions[servo] + delta, 0, 180);
  setServoAngle(servo, newAngle);
  servo_positions[servo] = newAngle;

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

void moveForward(float current_motor_speed) {
  Serial.println("Moving forward.");
  motorFrontLeft.setSpeed(current_motor_speed);
  motorFrontRight.setSpeed(current_motor_speed);
  motorBackLeft.setSpeed(current_motor_speed);
  motorBackRight.setSpeed(current_motor_speed);
  motorFrontLeft.run(FORWARD);
  motorFrontRight.run(FORWARD);
  motorBackLeft.run(FORWARD);
  motorBackRight.run(FORWARD);
}

void moveBackward(float current_motor_speed) {
  Serial.println("Moving backward.");
  motorFrontLeft.setSpeed(current_motor_speed);
  motorFrontRight.setSpeed(current_motor_speed);
  motorBackLeft.setSpeed(current_motor_speed);
  motorBackRight.setSpeed(current_motor_speed);
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
