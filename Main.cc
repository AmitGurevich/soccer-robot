#include <Pixy2.h>

#define motor1Pin1 2
#define motor1Pin2 4
#define motor1EnablePin 3
#define motor2Pin1 5
#define motor2Pin2 7
#define motor2EnablePin 6

#define trigPin 9
#define echoPin 8

#define pixyI2CAddress 0x54
#define MotorSpeed 50

// Pixy camera and global variables
int BallXCenter;
int GoalXCenter;
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
  for (int i = 0; i < Blocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == 1) { // don't forget to replace 1 with name of the color signature!!!
      BallXCenter = pixy.ccc.blocks[i].m_x;
      
      BallDetected = true;
      
      if(Distance < 5 || Distance >=2000){
        BallCatched = true;
      }
      else{
        BallCatched = false;
      }

    }
    else{
      BallDetected = false;
    }

    if (pixy.ccc.blocks[i].m_signature == 2) { // don't forget to replace 2 with name of the color signature!!!
      GoalXCenter = pixy.ccc.blocks[i].m_x;

      GoalDetected = true;
    }
    else{
      GoalDetected = false;
    }
  }

  if(BallCatched == true && GoalDetected == true){
    //You have the ball and found the goal
    Follow(GoalXCenter);
  }
  else if(BallCatched == true){
    //You got the ball,now find the Goal!
    SearchGoal();
  }
  else if(BallDetected == true){
    //Ball found,Go for it!
    Follow(BallXCenter);
  }
  else{
    //Ball isn't detected,search for it!
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

  Serial.println("StopMotors");
}

void MoveBackward(){
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  analogWrite(motor1EnablePin, MotorSpeed);
  analogWrite(motor2EnablePin, MotorSpeed);

  Serial.println("MoveBackwards");
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
  MoveLeft();
}

void Follow(int pixyXCenter) {
  if (pixyXCenter <= 50) {
    // Object is to the left, turn left
    MoveLeft();
  } else if (pixyXCenter > 250) {
    // Object is to the right, turn right
    MoveRight();
  } else {
    // object is centered, go get it!
    MoveForward();
  }
}
