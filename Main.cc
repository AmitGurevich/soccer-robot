#include <Pixy2.h>

// Motor and sensor pin defines
#define motor1Pin1 2
#define motor1Pin2 4
#define motor1EnablePin 3
#define motor2Pin1 5
#define motor2Pin2 7
#define motor2EnablePin 6

#define trigPin 9
#define echoPin 8

// Pixy camera address and motor speed
#define pixyI2CAddress 0x54
#define MotorSpeed 50

// Pixy camera and global variables
int BallXCenter;
int GoalXCenter;
int Distance;
bool BallDetected;
bool GoalDetected;
bool BallCatched;

// Pixy2 object
Pixy2 pixy;

// Motor class
class Motor {
public:
  int pin1;
  int pin2;
  int enablePin;

  Motor(int pin1, int pin2, int enablePin) {
    this->pin1 = pin1;
    this->pin2 = pin2;
    this->enablePin = enablePin;
  }

  void init() {
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(enablePin, OUTPUT);
  }

  void forward() {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(enablePin, MotorSpeed);
  }

  void backward() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(enablePin, MotorSpeed);
  }

  void stop() {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    analogWrite(enablePin, 0);
  }
};

// Ultrasonic sensor class
class UltrasonicSensor {
public:
  int trigPin;
  int echoPin;

  UltrasonicSensor(int trigPin, int echoPin) {
    this->trigPin = trigPin;
    this->echoPin = echoPin;
  }

  void init() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }

  int getDistance() {
    long duration, distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
    return distance;
  }
};

// Main robot class
class Robot {
public:
  Motor motor1;
  Motor motor2;
  UltrasonicSensor ultrasonicSensor;

  Robot(Motor motor1, Motor motor2, UltrasonicSensor ultrasonicSensor) {
    this->motor1 = motor1;
    this->motor2 = motor2;
    this->ultrasonicSensor = ultrasonicSensor;
  }

  void init() {
    motor1.init();
    motor2.init();
    ultrasonicSensor.init();
  }

  void moveForward() {
    motor1.forward();
    motor2.forward();
  }

  void moveBackward() {
    motor1.backward();
    motor2.backward();
  }

  void moveLeft() {
    motor1.forward();
    motor2.backward();
  }

  void moveRight() {
    motor1.backward();
    motor2.forward();
  }

  void stop() {
    motor1.stop();
    motor2.stop();
  }

  int getDistance() {
    return ultrasonicSensor.getDistance();
  }

  void Follow(int pixyXCenter) {

  // Calculate proportional difference for motor speed adjustments
  int difference = pixyXCenter - 150;  // Assuming camera center is at 150
  int motorSpeedDifference = difference * 2;  // Adjust the multiplier as needed
  
  // Constrain motor speeds within valid range
  int motor1Speed = constrain(MotorSpeed - motorSpeedDifference, 0, 255);
  int motor2Speed = constrain(MotorSpeed + motorSpeedDifference, 0, 255);

  // Move both motors forward with adjusted speeds
  analogWrite(motor1EnablePin, motor1Speed);
  analogWrite(motor2EnablePin, motor2Speed);

  // Set both motors to move forward (no turning)
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

  void searchBall() {
    moveLeft();
  }

  void searchGoal() {
    moveLeft();
  }
};

// Robot object
Robot robot(Motor(motor1Pin1, motor1Pin2, motor1EnablePin), Motor(motor2Pin1, motor2Pin2, motor2EnablePin), UltrasonicSensor(trigPin, echoPin));

void setup() {
  pixy.init();
  robot.init();
  Serial.begin(9600);
}

void loop() {
  Distance = robot.getDistance();
  int Blocks = pixy.ccc.getBlocks();
  for (int i = 0; i < Blocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == 1) {
      BallXCenter = pixy.ccc.blocks[i].m_x;
      BallDetected = true;
      if (Distance < 5 || Distance >= 2000) {
        BallCatched = true;
      } else {
        BallCatched = false;
      }
    } else if (pixy.ccc.blocks[i].m_signature == 2) {
      GoalXCenter = pixy.ccc.blocks[i].m_x;
      GoalDetected = true;
    }
  }

  if (BallCatched == true && GoalDetected == true) {
    // You have the ball and found the goal
    robot.follow(GoalXCenter);
  } else if (BallCatched == true) {
    // You got the ball, now find the Goal!
    robot.searchGoal();
  } else if (BallDetected == true) {
    // Ball found, Go for it!
    robot.follow(BallXCenter);
  } else {
    // Ball isn't detected, search for it!
    robot.searchBall();
  }
}
