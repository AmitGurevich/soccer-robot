#include <Pixy2.h>

#define motor1Pin1 2
#define motor1Pin2 4
#define motor1EnablePin 3
#define motor2Pin1 5
#define motor2Pin2 7
#define motor2EnablePin 6

#define pixyI2CAddress 0x54
#define MotorSpeed 50

// Pixy camera and global variables
int BallXCenter;
int GoalXCenter;
int BallXWidth;
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

  // Set initial motor direction(this is made for knowing the direction of the motors)
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() { 
  int Blocks = pixy.ccc.getBlocks();

  BallCatched = false;
  GoalDetected = false;

  for (int i = 0; i < Blocks; i++) {
    Serial.println(pixy.ccc.blocks[i].m_signature);
    if (pixy.ccc.blocks[i].m_signature == 1) { // don't forget to replace 1 with name of the color signature!!!

      BallXCenter = pixy.ccc.blocks[i].m_x;
      BallXWidth = pixy.ccc.blocks[i].m_width;

      BallDetected = true;
      
      if(BallXWidth >= 200){
        
        BallCatched = true;

      }
    }

    if (pixy.ccc.blocks[i].m_signature == 2) {

      GoalXCenter = pixy.ccc.blocks[i].m_x;
      GoalDetected = true;

    }
  }

  if(BallCatched == true && GoalDetected == true){
    //You have the ball and found the goal
    Follow(GoalXCenter);
    Serial.println("Goal");
  }
  else if(BallCatched == true){
    //You got the ball,now find the Goal!
    SearchGoal();
    Serial.println("SearchGoal");
  }
  else if(BallDetected == true){
    //Ball found,Go for it!
    Serial.println("FollowBall");
    Follow(BallXCenter);
  }
  else{
    //Ball isn't detected,search for it!
    SearchBall();
    Serial.println(BallDetected);
    Serial.println("SearchBall");
  }
}

void MoveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  analogWrite(motor1EnablePin, MotorSpeed);
  analogWrite(motor2EnablePin, MotorSpeed);
}

void MoveBackward(){
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  analogWrite(motor1EnablePin, MotorSpeed);
  analogWrite(motor2EnablePin, MotorSpeed);
}

void MoveLeft() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  analogWrite(motor1EnablePin, MotorSpeed);
  analogWrite(motor2EnablePin, MotorSpeed);
}

void MoveRight() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  analogWrite(motor1EnablePin, MotorSpeed);
  analogWrite(motor2EnablePin, MotorSpeed);
}

void SearchBall(){
  MoveLeft();
}
void SearchGoal(){
  MoveLeft();
}

void Debug(int ToPrint){
  Serial.println(ToPrint);
  delay(500);
}
void Follow(int pixyXCenter) {

  if (pixyXCenter <= 50) {
    // Object is to the left, turn left
    MoveLeft();
    //Serial.println("MoveLeft");
  } else if (pixyXCenter > 250) {
    // Object is to the right, turn right
    MoveRight();
    //Serial.println("MoveRight");
  } else {
    // object is centered, go get it!
    //Serial.println("MoveForward");
    MoveForward();
  }

}
