// Include necessary libraries
#include <AFMotor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_PWMServoDriver.h>

// Define motors using AFMotor library
AF_DCMotor motorFrontLeft(1); // Motor 1
AF_DCMotor motorFrontRight(2); // Motor 2
AF_DCMotor motorBackLeft(3); // Motor 3
AF_DCMotor motorBackRight(4); // Motor 4

// Define IR Sensor pins
#define IR_LEFT A0
#define IR_RIGHT A1

// Ultrasonic sensor pins
#define TRIG_PIN 6
#define ECHO_PIN 7

// PCA9685 Servo Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channels on PCA9685
#define SERVO_BASE 0
#define SERVO_SHOULDER 1
#define SERVO_ELBOW 2
#define SERVO_WRIST 3

// Servo min and max pulse lengths
#define SERVOMIN 150  // Min pulse length
#define SERVOMAX 600  // Max pulse length

// Color sensor object
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);

// Variables for robot mode
bool isManual = false;

void setup() {
  // Initialize IR Sensor pins as input
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50 Hz

  // Default servo positions
  setServoAngle(SERVO_BASE, 90);
  setServoAngle(SERVO_SHOULDER, 90);
  setServoAngle(SERVO_ELBOW, 90);
  setServoAngle(SERVO_WRIST, 90);

  // Initialize color sensor
  if (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  // Start serial communication for debugging and Bluetooth
  Serial.begin(9600);
  Serial.println("Robot initialized.");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    handleBluetoothCommand(command);
  }

  if (!isManual) {
    lineFollowWithObstacleDetection();
  }
}

void handleBluetoothCommand(char command) {
  switch (command) {
    case 'F': // Move forward
      moveForward();
      break;
    case 'B': // Move backward
      moveBackward();
      break;
    case 'L': // Turn left
      turnLeft();
      break;
    case 'R': // Turn right
      turnRight();
      break;
    case 'S': // Stop
      stopMotors();
      break;
    case 'M': // Switch to manual mode
      isManual = true;
      break;
    case 'A': // Switch to automatic mode
      isManual = false;
      break;
    case '1': // Base servo left
      setServoAngle(SERVO_BASE, getServoAngle(SERVO_BASE) - 10);
      break;
    case '2': // Base servo right
      setServoAngle(SERVO_BASE, getServoAngle(SERVO_BASE) + 10);
      break;
    case '3': // Shoulder up
      setServoAngle(SERVO_SHOULDER, getServoAngle(SERVO_SHOULDER) - 10);
      break;
    case '4': // Shoulder down
      setServoAngle(SERVO_SHOULDER, getServoAngle(SERVO_SHOULDER) + 10);
      break;
    case '5': // Elbow up
      setServoAngle(SERVO_ELBOW, getServoAngle(SERVO_ELBOW) - 10);
      break;
    case '6': // Elbow down
      setServoAngle(SERVO_ELBOW, getServoAngle(SERVO_ELBOW) + 10);
      break;
    case '7': // Wrist up
      setServoAngle(SERVO_WRIST, getServoAngle(SERVO_WRIST) - 10);
      break;
    case '8': // Wrist down
      setServoAngle(SERVO_WRIST, getServoAngle(SERVO_WRIST) + 10);
      break;
    default:
      Serial.println("Unknown command.");
  }
}

void lineFollowWithObstacleDetection() {
  int left = analogRead(IR_LEFT);
  int right = analogRead(IR_RIGHT);
  float distance = getUltrasonicDistance();

  if (distance < 10.0) {
    Serial.println("Obstacle detected!");
    stopMotors();
    return;
  }

  if (left < 500 && right < 500) {
    moveForward();
  } else if (left < 500) {
    turnLeft();
  } else if (right < 500) {
    turnRight();
  } else {
    stopMotors();
  }

  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  if (r > g && r > b && r > 2000) {
    Serial.println("Red detected");
    performPickup();
  } else if (g > r && g > b && g > 2000) {
    Serial.println("Green detected");
    performPickup();
  } else if (b > r && b > g && b > 2000) {
    Serial.println("Blue detected");
    performPickup();
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

  return distance;
}

void performPickup() {
  static unsigned long lastActionTime = 0;
  static int step = 0;
  unsigned long currentTime = millis();

  switch (step) {
    case 0:
      stopMotors();
      setServoAngle(SERVO_SHOULDER, 120);
      lastActionTime = currentTime;
      step++;
      break;
    case 1:
      if (currentTime - lastActionTime >= 500) {
        setServoAngle(SERVO_ELBOW, 60);
        lastActionTime = currentTime;
        step++;
      }
      break;
    case 2:
      if (currentTime - lastActionTime >= 500) {
        setServoAngle(SERVO_WRIST, 30);
        lastActionTime = currentTime;
        step++;
      }
      break;
    case 3:
      if (currentTime - lastActionTime >= 500) {
        setServoAngle(SERVO_SHOULDER, 90);
        setServoAngle(SERVO_ELBOW, 90);
        setServoAngle(SERVO_WRIST, 90);
        step = 0; // Reset for next use
      }
      break;
  }
}

void moveForward() {
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
  motorFrontLeft.setSpeed(100);
  motorFrontRight.setSpeed(100);
  motorBackLeft.setSpeed(100);
  motorBackRight.setSpeed(100);
  motorFrontLeft.run(BACKWARD);
  motorFrontRight.run(FORWARD);
  motorBackLeft.run(BACKWARD);
  motorBackRight.run(FORWARD);
}

void turnRight() {
  motorFrontLeft.setSpeed(100);
  motorFrontRight.setSpeed(100);
  motorBackLeft.setSpeed(100);
  motorBackRight.setSpeed(100);
  motorFrontLeft.run(FORWARD);
  motorFrontRight.run(BACKWARD);
  motorBackLeft.run(FORWARD);
  motorBackRight.run(BACKWARD);
}

void stopMotors() {
  motorFrontLeft.run(RELEASE);
  motorFrontRight.run(RELEASE);
  motorBackLeft.run(RELEASE);
  motorBackRight.run(RELEASE);
}

void setServoAngle(uint8_t servo, uint16_t angle) {
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo, 0, pulse);
}

uint16_t getServoAngle(uint8_t servo) {
  // Stub function: Replace with actual tracking if necessary
  return 90; // Default to 90 degrees for simplicity
}
