#include <AFMotor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

// Motor definitions
// AF_DCMotor motorFrontLeft(0);   // Motor 1
// AF_DCMotor motorBackLeft(1);   // Motor 2
AF_DCMotor motorLeft(2);   // Motor 3
AF_DCMotor motorRight(3);  // Motor 4

// IR Sensor pins
#define IR_LEFT A0
#define IR_RIGHT A1
#define IR_MID 9

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

void setup() {
  delay(2000);
  // Initialize IR Sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_MID, INPUT);

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
  setServoAngle(SERVO0, initialServo[0]);  //kecil kanan, serong kiri 175
  delay(500);
  setServoAngle(SERVO3, initialServo[3]);  //90 capit, 40 buka
  delay(500);
  setServoAngle(SERVO2, initialServo[2]);  //kecil naik, 90 aman tertinggi, 150 terendah, posisi awal 90
  delay(500);
  setServoAngle(SERVO1, initialServo[1]);  //kecil maju, 150 aman mundur, 50 maju, posisi awal 100
  delay(500);

  // motorFrontLeft.setSpeed(150);
  motorRight.setSpeed(150);
  // motorBackLeft.setSpeed(150);
  motorLeft.setSpeed(150);

  // motorFrontLeft.run(RELEASE);
  motorRight.run(RELEASE);
  // motorBackLeft.run(RELEASE);
  motorLeft.run(RELEASE);

  if (!tcs.begin()) {
    Serial.println("No TCS34725 found. Check connections.");
    // while (1)
    //   ;
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

  unsigned long timeNow = millis();

  if (delayDetection) {
    colorDetection = false;
    if (timeNow - startDelayDetection >= 5000) {
      Serial.println("Start color detection");
      colorDetection = true;
      delayDetection = false;
    }
  }

  // color_data = current_color + "#" + String(sorted_red) + "#" + String(sorted_green) + "#" + String(sorted_blue);
  // // current_color = color;
  // bluetooth.println(color_data);
  // Serial.println("Sent color_data: " + color_data);
  // delay(500);

  // irTest();
  // colorTest();
  // detectColor();

  if (!isManual) {
    lineFollowWithObstacleAndColorDetection();
  }
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
      moveForward();
      delay(1000);
      stopMotors();
      break;
    case 'B':
      moveBackward();
      delay(1000);
      stopMotors();
      break;
    case 'L':
      turnLeft();
      delay(3000);
      stopMotors();
      break;
    case 'R':
      turnRight();
      delay(3000);
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

void lineFollowWithObstacleAndColorDetection() {
  bool left = digitalRead(IR_LEFT);
  bool right = digitalRead(IR_RIGHT);
  bool mid = digitalRead(IR_MID);

  float distance = getUltrasonicDistance();

  Serial.print("IR Left: ");
  Serial.print(left);
  Serial.print(", IR Right: ");
  Serial.print(right);
  Serial.print(", IR Mid: ");
  Serial.print(mid);
  Serial.print(", Distance: ");
  Serial.print(distance);
  Serial.print("\t");

  if (distance < 8.0 && distance != 0.0) {
    Serial.println("Obstacle detected! Stopping.");
    stopMotors();
    // return;
  } else if (left) {
    // Prioritas kiri: belok kiri jika sensor kiri mendeteksi garis hitam
    turnLeft();
  } else if (right) {
    // Prioritas kanan: belok kanan jika sensor kanan mendeteksi garis hitam
    turnRight();
  } else if (mid) {
    // Jika hanya sensor tengah mendeteksi garis hitam, terus maju
    moveForward();
  } else {
    // Jika semua sensor mendeteksi putih, mundur
    // moveBackward();
    stopMotors();
  }
  if (colorDetection)
    detectColor();
}


void detectColor() {
  // Serial.println("Start color detection");
  float red, green, blue;
  unsigned long timeNow = millis();
  // tcs.setInterrupt(false);  // turn on LED
  // delay(60);  // takes 50ms to read
  if (timeNow - lastColorDetection >= 60) {
    tcs.getRGB(&red, &green, &blue);
    tcs.setInterrupt(true);  // turn off LED

    Serial.print("R: ");
    Serial.print(int(red));
    Serial.print(" G: ");
    Serial.print(int(green));
    Serial.print(" B: ");
    Serial.println(int(blue));


    if (red > green && red > blue && red > 125 /* && distance <= 15*/) {
      stopMotors();
      Serial.println("Red detected.");
      if (!isManual) {
        !isHold ? performPickup("Merah") : performDrop("Merah");
      } else {
        current_color = "Merah";
      }
    } else if (green > red && green > blue && green > 120 /* && distance <= 15*/) {
      stopMotors();
      Serial.println("Green detected.");
      if (!isManual) {
        !isHold ? performPickup("Hijau") : performDrop("Hijau");
      } else {
        current_color = "Hijau";
      }
      // } else if (blue > red && blue > green && blue > 2000 /* && distance <= 15*/) {
    }
    // else if (/*blue > red && blue > green &&*/ blue > 127 /* && distance <= 15*/) {
    //   stopMotors();
    //   Serial.println("Blue detected.");
    //   !isHold ? performPickup("Biru") : performDrop("Biru");
    // }
    lastColorDetection = timeNow;
  }
}

void irTest() {
  bool left = digitalRead(IR_LEFT);
  bool right = digitalRead(IR_RIGHT);
  bool mid = digitalRead(IR_MID);
  // float distance = getUltrasonicDistance();

  Serial.print("IR Left: ");
  Serial.print(left);
  Serial.print(", IR Mid: ");
  Serial.print(mid);
  Serial.print(", IR Right: ");
  Serial.println(right);
  // Serial.print(", Distance: ");
  // Serial.println(distance);
}

void colorTest() {
  float red, green, blue;
  unsigned long timeNow = millis();
  // tcs.setInterrupt(false);  // turn on LED
  // delay(60);  // takes 50ms to read
  if (timeNow - lastColorDetection >= 60) {
    tcs.getRGB(&red, &green, &blue);
    // tcs.setInterrupt(true);  // turn off LED

    Serial.print("R: ");
    Serial.print(int(red));
    Serial.print(" G: ");
    Serial.print(int(green));
    Serial.print(" B: ");
    Serial.println(int(blue));


    if (red > green && red > blue && red > 140 /* && distance <= 15*/) {
      Serial.println("Red detected.");
      !isHold ? performPickup("Merah") : performDrop("Merah");
    } else if (green > red && green > blue && green > 136 /* && distance <= 15*/) {

      Serial.println("Green detected.");
      !isHold ? performPickup("Hijau") : performDrop("Hijau");
    } else if (/*blue > red && blue > green &&*/ blue > 127 /* && distance <= 15*/) {
      Serial.println("Blue detected.");
      !isHold ? performPickup("Biru") : performDrop("Biru");
    }
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

  // Serial.print("Ultrasonic distance measured: ");
  // Serial.println(distance);

  return distance;
}

void performPickup(String color) {
  Serial.print("Starting pickup process for color: ");

  Serial.println(color);

  setServoAngle(SERVO3, 40);
  delay(1000);
  setServoAngle(SERVO2, 120);
  delay(1000);
  setServoAngle(SERVO1, 30);
  delay(1000);
  setServoAngle(SERVO3, 90);
  delay(1000);

  Serial.println("Pickup complete. Returning servos to default positions.");
  current_color = color;
  color_data = current_color + "#" + String(sorted_red) + "#" + String(sorted_green) + "#" + String(sorted_blue);
  Serial.println("Sent color_data: " + color_data);
  bluetooth.println(color_data);

  setServoAngle(SERVO2, initialServo[2]);
  delay(500);
  setServoAngle(SERVO1, initialServo[1]);
  delay(2000);
  isHold = true;
  Serial.println("Start delay color detection");
  delayDetection = true;
  startDelayDetection = millis();
}

void performDrop(String color) {
  Serial.print("Starting drop process for color: ");
  Serial.println(color);
  if (color == "Merah") {
    sorted_red++;
  } else if (color == "Hijau") {
    sorted_green++;
  } else if (color == "Biru") {
    sorted_blue++;
  }

  setServoAngle(SERVO1, 30);
  delay(1000);
  setServoAngle(SERVO2, 120);
  delay(1000);
  setServoAngle(SERVO3, 40);
  delay(1000);

  Serial.println("Drop complete. Returning servos to default positions.");
  current_color = "Tidak ada";
  color_data = current_color + "#" + String(sorted_red) + "#" + String(sorted_green) + "#" + String(sorted_blue);
  bluetooth.println(color_data);

  setServoAngle(SERVO2, initialServo[2]);
  delay(500);
  setServoAngle(SERVO1, initialServo[1]);
  delay(1000);
  setServoAngle(SERVO3, 90);
  delay(500);
  isHold = false;
  Serial.println("Start delay color detection");
  delayDetection = true;
  startDelayDetection = millis();
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

void setServoAngle(uint8_t servo, uint16_t angle) {
  uint16_t pulse = map(angle, 0, 270, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo, 0, pulse);

  Serial.print("Set servo ");
  Serial.print(servo);
  Serial.print(" to angle: ");
  Serial.println(angle);
}

void turnLeft() {
  Serial.println("Turning left.");
  // motorFrontLeft.setSpeed(motor_speed);
  motorRight.setSpeed(motor_speed);
  // motorBackLeft.setSpeed(motor_speed);
  motorLeft.setSpeed(motor_speed);

  // motorFrontLeft.run(BACKWARD);
  motorRight.run(FORWARD);
  // motorBackLeft.run(BACKWARD);
  motorLeft.run(BACKWARD);
}

void turnRight() {
  Serial.println("Turning right.");
  // motorFrontLeft.setSpeed(motor_speed);
  motorRight.setSpeed(motor_speed);
  // motorBackLeft.setSpeed(motor_speed);
  motorLeft.setSpeed(motor_speed);

  // motorFrontLeft.run(FORWARD);
  motorRight.run(BACKWARD);
  // motorBackLeft.run(FORWARD);
  motorLeft.run(FORWARD);
}

void moveForward() {
  Serial.println("Moving forward.");
  // motorFrontLeft.setSpeed(motor_speed);
  motorRight.setSpeed(motor_speed);
  // motorBackLeft.setSpeed(motor_speed);
  motorLeft.setSpeed(motor_speed);

  // motorFrontLeft.run(FORWARD);
  motorRight.run(FORWARD);
  // motorBackLeft.run(FORWARD);
  motorLeft.run(FORWARD);
}

void moveBackward() {
  Serial.println("Moving backward.");
  // motorFrontLeft.setSpeed(motor_speed);
  motorRight.setSpeed(motor_speed);
  // motorBackLeft.setSpeed(motor_speed);
  motorLeft.setSpeed(motor_speed);

  // motorFrontLeft.run(BACKWARD);
  motorRight.run(BACKWARD);
  // motorBackLeft.run(BACKWARD);
  motorLeft.run(BACKWARD);
}
void stopMotors() {
  Serial.println("Stopping motors.");
  // motorFrontLeft.run(RELEASE);
  motorRight.run(RELEASE);
  // motorBackLeft.run(RELEASE);
  motorLeft.run(RELEASE);
}
