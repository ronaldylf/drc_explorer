#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <My_ultrassonic.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>
#include <LCDScroll.h>

Adafruit_VL53L0X lox_front = Adafruit_VL53L0X();
Servo servoDistance;
DC_motor_controller motorR;
DC_motor_controller motorL;
LCDScroll screen;

TwoMotors both(&motorL, &motorR);

void interruptR (){
  motorR.isr();
}

void interruptL (){
  motorL.isr();
}

////////////////////////////
// pin variables
#define ok_button 0
////////////////////////////
// static variables
float basespeed = 50.0;
float turnspeed = 40.0;
////////////////////////////////
// dynamic variables
float front_distance = 0;
float right_distance = 0;
bool objectAhead = false;
////////////////////////////////

void setup (){
  Serial.begin (9600);
  // servo to move distance sensor
  servoDistance.attach(0);
  servoDistance.write(90); //0(left) 90(front)
  
  // right motor:
  motorL.hBridge(12, 11, 13);
  motorL.setEncoderPin(3,4);
  motorL.setRR(30);
  motorL.setPIDconstants(2.2, 0.9, 0.15);
  motorL.setPins();
  motorL.stop();
  attachInterrupt(digitalPinToInterrupt(3), interruptL, FALLING);

  // left motor:
  motorR.hBridge(9, 10, 8);
  motorR.setEncoderPin(3, 4);
  motorR.setRR(30);
  motorR.setPIDconstants(2.2, 0.9, 0.15);
  motorR.setPins();
  motorR.stop();
  attachInterrupt(digitalPinToInterrupt(2), interruptR, FALLING);
  
  both.setGyreDegreesRatio(1.28, 90);

  while(true) {
    motorR.walk(30);
    motorL.walk(30);
  }

  screen.setButtons(0, 0, INPUT);
  String options[] = {
    "debugBlock",
    "obRight",
    "obLeft",
    "obHall"
  };
  screen.setOptions(options);
  byte chosen_id = 0;
  while(!digitalRead(ok_button)) chosen_id = screen.getCurrentId();
  screen.write("starting...", "id: "+String(chosen_id));
  
  if (chosen_id==0) goto debugBlock;
  
  debugBlock:
    Serial.println("debug here");
    debug();
  
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
    both.together(basespeed, 1);
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
  Align(-130);
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

float rightDistance() {
  servoDistance.write(90);
  float distance = readDistance()-10;
  if (distance<0) return 0;
  return distance;
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
  front_distance = frontDistance();
  bool go_forward = front_distance>desired_distance;
  bool go_backward = front_distance<desired_distance;
  
  if (!go_forward && !go_backward) {
    // in this case front_distance==desired_distance
  } else if (go_forward) {
    while (frontDistance()>desired_distance) {
      motorR.walk(speed);
      motorL.walk(speed);
    }
  } else if (go_backward) {
    while (frontDistance()<desired_distance) {
      motorR.walk(-speed);
      motorL.walk(-speed);
    }
  }
  both.stop();
}


bool isRescueArea() {
  return false;
}

void Align(byte pwm_speed) {
  motorR.run(pwm_speed);
  motorL.run(pwm_speed);
  // add here a: while (not touch the button);
  delay(1000);
  both.stop();
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
