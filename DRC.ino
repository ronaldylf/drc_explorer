#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <My_ultrassonic.h>
#include <LineSensors_Master.h>

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
// variables
float left_distance = 0;
float front_distance = 0;
float right_distance = 0;
float basespeed = 50;
float turnspeed = 40;
float c = 1.0; // coefficient ( 1 or -1)
////////////////////////////////

void setup (){
  Serial.begin (9600);
  // right motor:
  motorR.hBridge(9,10,8);
  motorR.setEncoderPin(2,5);
  motorR.setRR(21.3);
  motorR.setPIDconstants(1.9, 0.9, 0.1);
  motorR.setPins();
  motorR.walk(0);

  // left motor:
  motorL.hBridge(11,12,13);
  motorL.setEncoderPin(3,4);
  motorL.setRR(21.3);
  motorL.setPIDconstants(1.9, 0.9, 0.1);
  motorL.setPins();
  motorL.walk(0);

  attachInterrupt(digitalPinToInterrupt(2), interruptR, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), interruptL, FALLING);

  both.setGyreDegreesRatio(1.28, 180);
  ////////////////////// start ////////////////////////
  forwardWhileFrontDistance(">=", 5);
  delay(100);
  c = getLargerDirectionCoefficient();
  both.turnDegree(turnspeed*c, 90*c);
  both.stop();
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


void forwardWhileFrontDistance(String string_condition, float value) {
  bool condition = true;
  while (condition) {
    motorR.walk(basespeed);
    motorL.walk(basespeed);
    condition = false;
    readDistances();
    if (string_condition==">") condition = (front_distance>value);
    if (string_condition==">=") condition = (front_distance>=value);
    if (string_condition=="<") condition = (front_distance<value);
    if (string_condition=="<=") condition = (front_distance<=value);
    readDistances();
  }
  both.stop();
}

void debug() {
  both.stop();
  while(true) delay(1000);
}
