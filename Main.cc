#include <Wire.h>
#define motor1Pin1 2
#define motor1Pin2 4
#define motor1EnablePin 3
#define motor2Pin1 5
#define motor2Pin2 7
#define motor2EnablePin 6

#define trigPin 9
#define echoPin 8

#define distanceThreshold 20
#define pixyI2CAddress 0x54

// Pixy camera variables
int pixyXCenter = 0;
int pixyYCenter = 0;
int pixyWidth = 0;
int pixyHeight = 0;

void setup() {
  // Configure motor pins as output
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor1EnablePin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor2EnablePin, OUTPUT);

  // Configure ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Configure Pixy camera
  Wire.begin();
  Wire.beginTransmission(pixyI2CAddress);
  Wire.write(0x00);  // Set Pixy to color code mode
  Wire.write(0x00);  // Set color code to red
  Wire.endTransmission();

  // Set initial motor direction
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() {
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.println(distance);

  // Read Pixy camera data
  Wire.requestFrom(pixyI2CAddress, 14);
  if (Wire.available() == 14) {
    pixyXCenter = Wire.read() << 8 | Wire.read();
    pixyYCenter = Wire.read() << 8 | Wire.read();
    pixyWidth = Wire.read() << 8 | Wire.read();
    pixyHeight = Wire.read() << 8 | Wire.read();
  }

  // Check if an object is detected
  if (pixyWidth > 0 && pixyHeight > 0) {
    // Object detected, move towards it
    if (pixyXCenter < 160) {
      // Object is to the left, turn left
      moveLeft();
    } else if (pixyXCenter > 320) {
      // Object is to the right, turn right
      moveRight();
    } else {
      // Object is centered, move forward
      moveForward();
    }
  } else {
    // No object detected, move forward
    moveForward();
  }

  // Check if the robot is too close to an obstacle
  if (distance <= distanceThreshold || distance > 2000) {
    // Obstacle detected, move backward
    moveBackward();
  } else if (10 <= distance && distance <= distanceThreshold){
    // Obstacle detected, stop motors
    stopMotors();
  }
}

void moveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  analogWrite(motor1EnablePin, 255);
  analogWrite(motor2EnablePin, 255);
}

void stopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);

  analogWrite(motor1EnablePin, 0);
  analogWrite(motor2EnablePin, 0);
}

void moveBackward(){
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  // Set motor speeds to zero
  analogWrite(motor1EnablePin, 255);
  analogWrite(motor2EnablePin, 255);
}

void moveLeft() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  analogWrite(motor1EnablePin, 255);
  analogWrite(motor2EnablePin, 255);
}

void moveRight() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  analogWrite(motor1EnablePin, 255);
  analogWrite(motor2EnablePin, 255);
}
