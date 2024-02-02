#include <Wire.h>
#include <Pixy2.h>

#define motor1Pin1 2
#define motor1Pin2 4
#define motor1EnablePin 3
#define motor2Pin1 5
#define motor2Pin2 7
#define motor2EnablePin 6

#define trigPin 9
#define echoPin 8

#define distanceThreshold 10
#define pixyI2CAddress 0x54
#define MotorSpeed 50

// Pixy camera and global variables
int pixyXCenter = 0;
int pixyYCenter = 0;
int pixyWidth = 0;
int pixyHeight = 0;
int Distance;
bool BallDetected;
bool GoalDetected;
bool BallCatched;

Pixy2 pixy;

void setup() {

  // initualizing pixy 
  pixy.init();
  
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

  // Set initial motor direction(this is made for knowing the direction of the motors)
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() {
  // calculating distance
  long duration, Distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  Distance = duration * 0.034 / 2;

  int Blocks = pixy.ccc.getBlocks();
  if (Blocks > 0) {
    for (int i = 0; i < Blocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1) { // don't forget to replace 1 with name of the color signature!!!
        pixyXCenter = pixy.ccc.blocks[i].m_x;
        pixyYCenter = pixy.ccc.blocks[i].m_y;
        pixyWidth = pixy.ccc.blocks[i].m_width;
        pixyHeight = pixy.ccc.blocks[i].m_height;
        
        BallDetected = true;
        Follow();
      }
    }
  }
  else{
    SearchBall();
  }

}

void MoveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  analogWrite(motor1EnablePin, MotorSpeed);
  analogWrite(motor2EnablePin, MotorSpeed);

  Serial.println("MoveForward");
}

void StopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);

  analogWrite(motor1EnablePin, 0);
  analogWrite(motor2EnablePin, 0);

  Serial.println("stopmotors");
}

void MoveBackward(){
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  // Set motor speeds to zero
  analogWrite(motor1EnablePin, MotorSpeed);
  analogWrite(motor2EnablePin, MotorSpeed);

  Serial.println("Movebackwards");
}

void MoveLeft() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  analogWrite(motor1EnablePin, MotorSpeed);
  analogWrite(motor2EnablePin, MotorSpeed);

  Serial.println("MoveLeft");
}

void MoveRight() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  analogWrite(motor1EnablePin, MotorSpeed);
  analogWrite(motor2EnablePin, MotorSpeed);

  Serial.println("MoveRight");
}

void SearchBall(){
  MoveLeft();
}
void SearchGoal(){
  Follow();
}

void Follow() {
  // Check if an object is detected
  if (pixyWidth > 0 && pixyHeight > 0) {
    // Object detected, move towards it
    if (pixyXCenter <= 50) {
      // Object is to the left, turn left
      MoveLeft();
    } else if (pixyXCenter > 250) {
      // Object is to the right, turn right
      MoveRight();
    } else {
      //object is centered,go get it!
      MoveForward();
    }
  } else {
    // No object detected, move forward
    MoveForward();
  }
}
