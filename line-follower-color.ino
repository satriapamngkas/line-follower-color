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

// Servo positions tracker
uint16_t servo_positions[4] = { 50, 60, 30, 65 };

// Color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);

SoftwareSerial bluetooth(BT_RX, BT_TX);

bool isManual = false;
String current_color = "Merah";
int motor_speed = 150;
int sorted_red = 2;
int sorted_green = 3;
int sorted_blue = 4;
String color_data = "";
int bufferIndex = 0;
bool isHold = false;
bool stopDetection =false;

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
  setServoAngle(SERVO0, 175);  //kecil kiri, hadap kiri 220
  delay(500);
  setServoAngle(SERVO3, 90);  //90 capit, 40 buka
  delay(500);
  setServoAngle(SERVO2, 90);  //kecil naik, 90 aman tertinggi, 150 terendah, posisi awal 90
  delay(500);
  setServoAngle(SERVO1, 180);  //kecil maju, 150 aman mundur, 50 maju, posisi awal 100
  delay(500);

  motorFrontLeft.setSpeed(150);
  motorFrontRight.setSpeed(150);
  motorBackLeft.setSpeed(150);
  motorBackRight.setSpeed(150);

  motorFrontLeft.run(RELEASE);
  motorFrontRight.run(RELEASE);
  motorBackLeft.run(RELEASE);
  motorBackRight.run(RELEASE);

  if (!tcs.begin()) {
    Serial.println("No TCS34725 found. Check connections.");
    while (1)
      ;
  } else {
    Serial.println("TCS34725 Initialized");
  }

  Serial.println("Robot initialized.");
}

void loop() {
  // if (bluetooth.available()) {
  //   String receivedData = bluetooth.readStringUntil('\n');
  //   Serial.print("Received command: ");
  //   Serial.println(command);
  //   handleBluetoothCommand(receivedData);
  // }

  // while (bluetooth.available()) {
  //   char receivedChar = bluetooth.read();

  //   if (receivedChar == '\n') {           // Akhir perintah
  //     commandBuffer[bufferIndex] = '\0';  // Tambahkan terminator string
  //     handleBluetoothCommand(commandBuffer);
  //     bufferIndex = 0;  // Reset indeks buffer
  //   } else if (bufferIndex < BUFFER_SIZE - 1) {
  //     commandBuffer[bufferIndex++] = receivedChar;
  //   }
  // }

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

// void handleBluetoothCommand(String command) {
//   Serial.print("Handling Bluetooth command: ");
//   Serial.println(command);

//   if (command == "AF") {
//     moveForward();
//     delay(1000);
//     stopMotors();
//   } else if (command == "AB") {
//     moveBackward();
//     delay(1000);
//     stopMotors();
//   } else if (command == "AL") {
//     turnLeft();
//     delay(1000);
//     stopMotors();
//   } else if (command == "AR") {
//     turnRight();
//     delay(1000);
//     stopMotors();
//   } else if (command == "AS") {
//     stopMotors();
//   } else if (command == "AM") {
//     isManual = true;
//     stopMotors();
//     Serial.println("Manual mode activated.");
//   } else if (command == "AA") {
//     isManual = false;
//     Serial.println("Automatic mode activated.");
//   } else if (command == "A1") {
//     adjustServo(SERVO1, -10);
//     adjustServo(SERVO2, -10);
//   } else if (command == "A2") {
//     adjustServo(SERVO0, 10);
//   } else if (command == "A3") {
//     adjustServo(SERVO0, -10);
//   } else if (command == "A4") {
//     adjustServo(SERVO1, 10);
//     adjustServo(SERVO2, 10);
//   } else if (command == "A5") {
//     setServoAngle(SERVO3, 65);
//   } else if (command == "A6") {
//     setServoAngle(SERVO3, 40);
//   } else if (command == "A7") {
//     adjustServo(SERVO2, -10);
//   } else if (command == "A8") {
//     adjustServo(SERVO2, 10);
//   } else {
//     Serial.println("Unknown command.");
//   }
// }

void handleBluetoothCommand(const char* command) {
  Serial.print("Handling Bluetooth command: ");
  Serial.println(command);

  if (strcmp(command, "AF") == 0) {
    moveForward();
    delay(1000);
    stopMotors();
  } else if (strcmp(command, "AB") == 0) {
    moveBackward();
    delay(1000);
    stopMotors();
  } else if (strcmp(command, "AL") == 0) {
    turnLeft();
    delay(1000);
    stopMotors();
  } else if (strcmp(command, "AR") == 0) {
    turnRight();
    delay(1000);
    stopMotors();
  } else if (strcmp(command, "AS") == 0) {
    stopMotors();
  } else if (strcmp(command, "AM") == 0) {
    isManual = true;
    stopMotors();
    Serial.println("Manual mode activated.");
  } else if (strcmp(command, "AA") == 0) {
    isManual = false;
    Serial.println("Automatic mode activated.");
  } else if (strcmp(command, "A1") == 0) {
    adjustServo(SERVO1, -10);
    adjustServo(SERVO2, -10);
  } else if (strcmp(command, "A2") == 0) {
    adjustServo(SERVO0, 10);
  } else if (strcmp(command, "A3") == 0) {
    adjustServo(SERVO0, -10);
  } else if (strcmp(command, "A4") == 0) {
    adjustServo(SERVO1, 10);
    adjustServo(SERVO2, 10);
  } else if (strcmp(command, "A5") == 0) {
    setServoAngle(SERVO3, 65);
  } else if (strcmp(command, "A6") == 0) {
    setServoAngle(SERVO3, 40);
  } else if (strcmp(command, "A7") == 0) {
    adjustServo(SERVO2, -10);
  } else if (strcmp(command, "A8") == 0) {
    adjustServo(SERVO2, 10);
  } else {
    Serial.println("Unknown command.");
  }
}



void lineFollowWithObstacleAndColorDetection() {
  bool left = digitalRead(IR_LEFT);
  bool right = digitalRead(IR_RIGHT);
  bool mid = digitalRead(IR_MID);
  // float distance = getUltrasonicDistance();

  Serial.print("IR Left: ");
  Serial.print(left);
  Serial.print(", IR Right: ");
  Serial.print(right);
  Serial.print(", IR Mid: ");
  Serial.print(mid);
  Serial.print("\t");

  // Serial.print(", Distance: ");
  // Serial.println(distance);

  // if (distance < 8.0) {
  //   Serial.println("Obstacle detected! Stopping.");
  //   stopMotors();
  //   // return;
  // } else 
  if (left) {
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
  // detectColor();
}


void detectColor() {
  float red, green, blue;
  // tcs.setInterrupt(false);  // turn on LED
  delay(60);  // takes 50ms to read
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // turn off LED

  Serial.print("R: ");
  Serial.print(int(red));
  Serial.print(" G: ");
  Serial.print(int(green));
  Serial.print(" B: ");
  Serial.println(int(blue));


  if (red > green && red > blue && red > 140 /* && distance <= 15*/) {
    stopMotors();
    Serial.println("Red detected.");
    !isHold ? performPickup("Merah") : performDrop("Merah");
  } else if (green > red && green > blue && green > 136 /* && distance <= 15*/) {
    // stopMotors();
    Serial.println("Green detected.");
    !isHold ? performPickup("Hijau") : performDrop("Hijau");
    // } else if (blue > red && blue > green && blue > 2000 /* && distance <= 15*/) {
  } else if (/*blue > red && blue > green &&*/ blue > 127 /* && distance <= 15*/) {
    stopMotors();
    Serial.println("Blue detected.");
    !isHold ? performPickup("Biru") : performDrop("Biru");
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
  // tcs.setInterrupt(false);  // turn on LED
  delay(60);  // takes 50ms to read
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

  setServoAngle(SERVO3, 20);
  delay(1000);
  setServoAngle(SERVO2, 120);
  delay(1000);
  setServoAngle(SERVO1, 0);
  delay(1000);
  setServoAngle(SERVO3, 90);
  delay(1000);

  Serial.println("Pickup complete. Returning servos to default positions.");
  color_data = current_color + "#" + String(sorted_red) + "#" + String(sorted_green) + "#" + String(sorted_blue);
  Serial.println("Sent color_data: " + color_data);
  current_color = color;
  bluetooth.println(color_data);

  setServoAngle(SERVO2, 90);
  delay(500);
  setServoAngle(SERVO1, 100);
  delay(2000);
  isHold = true;
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

  setServoAngle(SERVO1, 0);
  delay(1000);
  setServoAngle(SERVO2, 120);
  delay(1000);
  setServoAngle(SERVO3, 40);
  delay(1000);

  Serial.println("Drop complete. Returning servos to default positions.");
  current_color = "Tidak ada";
  color_data = current_color + "#" + String(sorted_red) + "#" + String(sorted_green) + "#" + String(sorted_blue);
  bluetooth.println(color_data);

  setServoAngle(SERVO2, 90);
  delay(500);
  setServoAngle(SERVO1, 100);
  delay(1000);
  setServoAngle(SERVO3, 90);
  delay(500);
  isHold = false;
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
  motorFrontLeft.setSpeed(100);
  motorFrontRight.setSpeed(150);
  motorBackLeft.setSpeed(100);
  motorBackRight.setSpeed(150);

  motorFrontLeft.run(BACKWARD);
  motorFrontRight.run(FORWARD);
  motorBackLeft.run(BACKWARD);
  motorBackRight.run(FORWARD);
}

void turnRight() {
  Serial.println("Turning right.");
  motorFrontLeft.setSpeed(150);
  motorFrontRight.setSpeed(100);
  motorBackLeft.setSpeed(150);
  motorBackRight.setSpeed(100);

  motorFrontLeft.run(FORWARD);
  motorFrontRight.run(BACKWARD);
  motorBackLeft.run(FORWARD);
  motorBackRight.run(BACKWARD);
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
void stopMotors() {
  Serial.println("Stopping motors.");
  motorFrontLeft.run(RELEASE);
  motorFrontRight.run(RELEASE);
  motorBackLeft.run(RELEASE);
  motorBackRight.run(RELEASE);
}
