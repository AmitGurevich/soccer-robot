// Include the Pixy2 library
#include <Pixy2.h>

// Create a Pixy2 object
Pixy2 pixy;

// Define the pins for the motor controller
#define ENA 9
#define IN1 8
#define IN2 7
#define IN3 6
#define IN4 5
#define ENB 10

// Define the pins for the ultrasonic sensor
#define TRIG 11
#define ECHO 12

// Define the base speed of the motors
#define BASE_SPEED 100

// Define the dead zone for the error
#define DEAD_ZONE 10

// Define the PID constants
#define KP 0.5
#define KI 0.1
#define KD 0.2

// Define the maximum distance to stop the robot
#define MAX_DIST 20

// Define some variables for the PID loop
int error = 0;
int prev_error = 0;
int integral = 0;
int derivative = 0;
int output = 0;

// Define a variable for the distance
int distance = 0;

// Define a function to set the speed and direction of the left motor
void leftMotor(int speed, bool dir) {
  analogWrite(ENA, speed); // Set the speed using PWM
  digitalWrite(IN1, dir); // Set the direction using a boolean value
  digitalWrite(IN2, !dir); // Set the opposite direction using the negation of the boolean value
}

// Define a function to set the speed and direction of the right motor
void rightMotor(int speed, bool dir) {
  analogWrite(ENB, speed); // Set the speed using PWM
  digitalWrite(IN3, dir); // Set the direction using a boolean value
  digitalWrite(IN4, !dir); // Set the opposite direction using the negation of the boolean value
}

// Define a function to get the distance from the ultrasonic sensor
int getDistance() {
  long duration; // Define a variable to store the duration of the pulse
  int distance; // Define a variable to store the distance
  digitalWrite(TRIG, LOW); // Set the trigger pin to low
  delayMicroseconds(2); // Wait for 2 microseconds
  digitalWrite(TRIG, HIGH); // Set the trigger pin to high
  delayMicroseconds(10); // Wait for 10 microseconds
  digitalWrite(TRIG, LOW); // Set the trigger pin to low
  duration = pulseIn(ECHO, HIGH); // Measure the duration of the echo pulse
  distance = duration * 0.034 / 2; // Calculate the distance in centimeters
  return distance; // Return the distance
}

// Define a function to follow the object using the Pixy2 camera
void followObject() {
  int x; // Define a variable to store the x coordinate of the object
  int y; // Define a variable to store the y coordinate of the object
  int w; // Define a variable to store the width of the object
  int h; // Define a variable to store the height of the object
  pixy.ccc.getBlocks(); // Get the blocks detected by the Pixy2 camera
  if (pixy.ccc.numBlocks) { // Check if there are any blocks
    x = pixy.ccc.blocks[0].m_x; // Get the x coordinate of the largest block
    y = pixy.ccc.blocks[0].m_y; // Get the y coordinate of the largest block
    w = pixy.ccc.blocks[0].m_width; // Get the width of the largest block
    h = pixy.ccc.blocks[0].m_height; // Get the height of the largest block
    error = x - pixy.frameWidth / 2; // Calculate the error as the difference between the x coordinate and the center of the frame
    integral = integral + error; // Calculate the integral as the sum of the errors
    derivative = error - prev_error; // Calculate the derivative as the difference between the current error and the previous error
    output = KP * error + KI * integral + KD * derivative; // Calculate the output as the weighted sum of the error, integral and derivative
    prev_error = error; // Update the previous error
    if (abs(error) < DEAD_ZONE) { // Check if the error is within the dead zone
      output = 0; // Set the output to zero
    }
    if (output > 0) { // Check if the output is positive
      leftMotor(BASE_SPEED - output, true); // Decrease the speed of the left motor and move forward
      rightMotor(BASE_SPEED, true); // Keep the speed of the right motor and move forward
    }
    else if (output < 0) { // Check if the output is negative
      leftMotor(BASE_SPEED, true); // Keep the speed of the left motor and move forward
      rightMotor(BASE_SPEED + output, true); // Increase the speed of the right motor and move forward
    }
    else { // Check if the output is zero
      leftMotor(BASE_SPEED, true); // Keep the speed of the left motor and move forward
      rightMotor(BASE_SPEED, true); // Keep the speed of the right motor and move forward
    }
  }
  else { // If there are no blocks
    leftMotor(0, true); // Stop the left motor
    rightMotor(0, true); // Stop the right motor
  }
}

// Define the setup function
void setup() {
  // Initialize the serial communication
  Serial.begin(9600);
  // Initialize the Pixy2 camera
  pixy.init();
  // Set the signature of the object to track
  pixy.setLamp(1, 0); // Turn on the LED
  pixy.setLED(255, 255, 255); // Set the LED color to white
  pixy.changeProg("color_connected_components"); // Change the program to color connected components
  pixy.setServos(500, 500); // Center the servos
  // Set the pins for the motor controller as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  // Set the pins for the ultrasonic sensor as inputs and outputs
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

// Define the loop function
void loop() {
  distance = getDistance(); // Get the distance from the ultrasonic sensor
  Serial.print("Distance: "); // Print the distance
  Serial.println(distance);
  if (distance > MAX_DIST) { // Check if the distance is greater than the maximum distance
    followObject(); // Follow the object using the Pixy2 camera
  }
  else { // If the distance is less than or equal to the maximum distance
    leftMotor(0, true); // Stop the left motor
    rightMotor(0, true); // Stop the right motor
  }
}
