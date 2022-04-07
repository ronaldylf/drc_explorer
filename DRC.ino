#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <My_ultrassonic.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox_front = Adafruit_VL53L0X();
Servo servoDistance;
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
// static variables
float basespeed = 50.0;
float turnspeed = 40.0;
////////////////////////////////
// dynamic variables
float front_distance = 0;
float right_distance = 0;
bool objectAhead = false;
float c = 1.0; // coefficient ( 1 or -1)
////////////////////////////////

void setup (){
  Serial.begin (9600);
  // servo to move distance sensor
  servoDistance.attach(0);
  servoDistance.write(90); //0(left) 90(front)
  
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

  debugBlock:
    Serial.println("debug here");
    while(true);
  
  ////////////////////// start movements////////////////////////
  //first running and climb
  adjustFrontDistance(basespeed, 5);

  // going down first stair
  delay(100);
  both.turnDegree(-turnspeed, -90);
  adjustFrontDistance(basespeed, 5);
  delay(100);
  both.turnDegree(-turnspeed, -90);
  goto obRight;
  
  // mid lane blocks
  obRight:
    // without obstacle in mid in right
    both.together(basespeed, 1);
    both.turnDegree(-turnspeed, -90);
    while (rightDistance()<70) {
      motorR.walk(basespeed);
      motorL.walk(basespeed);
    }
    both.together(50, 1);
    both.stop();
    both.turnDegree(turnspeed, 90);
    adjustFrontDistance(basespeed, 5);
    both.turnDegree(turnspeed, 90);
    
  obLeft:
    adjustFrontDistance(basespeed, 5);
    both.turnDegree(turnspeed, 90);
    
  obHall:
    while (rightDistance()<10) {
      motorR.walk(basespeed);
      motorL.walk(basespeed);
    }
    both.stop();
    both.together(basespeed, 1);
    float c = 0.5; // turning coefficient
    float rotations = 1;
    both.together(turnspeed, rotations, turnspeed*c, rotations);
    both.turnDegree(-turnspeed, -90);
    adjustFrontDistance(basespeed, 5);
    both.turnDegree(-turnspeed, -90);
    adjustFrontDistance(basespeed, 5);
    both.turnDegree(turnspeed, 90);
  
  adjustFrontDistance(basespeed, 6);
  both.turnDegree(turnspeed, 90);
  // second running
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
  takeDownWall(40);
}


void loop () {
  
}


float frontDistance(byte angle=0) {
  servoDistance.write(angle);
  return readDistance();
}

float rightDistance()() {
  servoDistance.write(90);
  return (readDistance()-10); // distance from center to right side of robot
}

float readDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  lox_front.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter/10;
  } else {
    //out of range
    return 200; // Max distance (cm)
  }
}

void adjustFrontDistance(float speed, float desired_distance) {
  both.stop();
  speed = abs(speed);
  desired_distance = abs(desired_distance);
  front_distance = frontDistance()();
  bool go_forward = front_distance>desired_distance;
  bool go_backward = front_distance<desired_distance;
  
  if (!go_forward && !go_backward) {
    // in this case front_distance==desired_distance
  } else if (go_forward) {
    while (frontDistance()()>desired_distance) {
      motorR.walk(speed);
      motorL.walk(speed);
    }
  } else if (go_backward) {
    while (frontDistance()()<desired_distance) {
      motorR.walk(-speed);
      motorL.walk(-speed);
    }
  }
  both.stop();
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
