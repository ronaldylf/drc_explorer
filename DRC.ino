#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <My_ultrassonic.h>

DC_motor_controller motorR;
DC_motor_controller motorL;

TwoMotors both(&motorL, &motorR);

void interruptR (){
  motorR.isr();
}

void interruptL (){
  motorL.isr();
}

////////////////////////////
// editable variables
float basespeed = 50;
float turnspeed = 40;
////////////////////////////////
// dynamic variables
float left_distance = 0;
float front_distance = 0;
float right_distance = 0;
bool obstacleAhead = false;
float c = 1.0; // coefficient ( 1 or -1)
////////////////////////////////

void setup (){
  Serial.begin (9600);
  // right motor:
  motorR.hBridge(0,0,0);
  motorR.setEncoderPin(2,5);
  motorR.setRR(21.3);
  motorR.setPIDconstants(1.9, 0.9, 0.1);
  motorR.setPins();
  motorR.walk(0);

  // left motor:
  motorL.hBridge(0, 0, 0);
  motorL.setEncoderPin(3,4);
  motorL.setRR(21.3);
  motorL.setPIDconstants(1.9, 0.9, 0.1);
  motorL.setPins();
  motorL.walk(0);

  attachInterrupt(digitalPinToInterrupt(2), interruptR, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), interruptL, FALLING);

  both.setGyreDegreesRatio(1.28, 180);
  
  ////////////////////// start movements////////////////////////
  adjustFrontDistance(basespeed, 5);
  delay(100);
  both.turnDegree(-turnspeed, -90);
  
  adjustFrontDistance(basespeed, 5);
  delay(100);
  both.turnDegree(-turnspeed, -90);

  both.together(basespeed, 1);
  both.turnDegree(-turnspeed, -90);
  adjustFrontDistance(basespeed, 5);
  both.turnDegree(turnspeed, 90);

  readDistances();
  obstacleAhead = front_distance<=65;
  // outline obstacle and goes to wall
  if (obstacleAhead) dodgeObstacle();
  adjustFrontDistance(basespeed, 5);

  both.turnDegree(turnspeed, 90);
  readDistances();
  obstacleAhead = front_distance<=65;
  // outline obstacle and goes to wall
  if (obstacleAhead) dodgeObstacle();
  adjustFrontDistance(basespeed, 5);

  both.turnDegree(turnspeed, 90);

  while (!isRescueArea()) {
    motorR.walk(basespeed);
    motorL.walk(basespeed);
  }
  both.stop();
  
  ////////////////////// start rescue////////////////////////
  both.together(basespeed, 1);
  both.turnDegree(turnspeed, 90);
  Align();
  adjustFrontDistance(basespeed, 5);
  both.turnDegree(-turnspeed, -90);
  adjustFrontDistance(basespeed, 5);
  both.turnDegree(-turnspeed, -90);
  takeDownWall();
}


void loop () {
  
}
void readDistances(){
  left_distance = 0;
  front_distance = 0;
  right_distance = 0;
}

float getLargerDirectionCoefficient() {
  // return the coefficient of direction with largest distance value
  readDistances();
  if (right_distance>left_distance) return 1.0;
  return -1.0;
}


void adjustFrontDistance(float speed, float desired_distance) {
  both.stop();
  speed = abs(speed);
  desired_distance = abs(speed);
  float c = 1.0;
  readDistances();
  bool go_forward = front_distance>desired_distance;
  bool go_backward = front_distance<desired_distance;
  
  if (!go_forward && !go_backward) {
    // in this case front_distance==desired_distance
  } else if (go_forward) {
    while (front_distance>desired_distance) {
      motorR.walk(speed);
      motorL.walk(speed);
      readDistances();
    }
  } else if (go_backward) {
    while (front_distance<desired_distance) {
      motorR.walk(-speed);
      motorL.walk(-speed);
      readDistances();
    }
  }
  both.stop();
}

void dodgeObstacle() {
  adjustFrontDistance(basespeed, 5);
  /*
    outline obstacle
    (take into consideration that the obstacle can be standing or lying)
    verify if the dimensions 24x13 cm of the robot will be able to pass
    all curves on the track
    */
}

bool isRescueArea() {
  return false;
}

void Align() {
  
}

void takeDownWall(float speed) {
  speed = abs(speed);
  both.together(speed, 0.5);
  both.together(-speed, -0.5);
  both.stop();
}

void debug() {
  both.stop();
  while(true) delay(1000);
}
