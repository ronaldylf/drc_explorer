#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <My_ultrassonic.h>
#include <Servo.h>
#include <LCDScroll.h>


Servo servoDistance;
DC_motor_controller motorR;
DC_motor_controller motorL;
LCDScroll screen;
My_ultrassonic ultrassonic(24, 22);

TwoMotors both(&motorL, &motorR);

void interruptR () {
  motorR.isr();
}

void interruptL () {
  motorL.isr();
}

////////////////////////////
// pin variables
#define ok_button 0
////////////////////////////
// static variables
float basespeed = 80.0;
float turnspeed = 40.0;
////////////////////////////////
// dynamic variables
float front_distance = 0;
float right_distance = 0;
bool objectAhead = false;
byte code_id = 1;
////////////////////////////////
void Align(byte speed = 100) {
//  motorR.walk(speed);
//  motorL.walk(speed);
//  // add here a: while (not touch the button);
//  delay(500);
//  both.stop();
}


void setup () {
  Serial.begin (9600);

  // for the distance sensor
  ultrassonic.setPins();


  //  Serial.println("waiting...");
  //  delay(200);
  //  while(true) {
  //    Serial.println(readDistance());
  //    delay(100);
  //  }

  // servo to move distance sensor
  servoDistance.attach(7);
  servoDistance.write(90); //0(right) 90(front) 180 (left)


  // left motor:
  motorL.hBridge(12, 11, 13);
  motorL.setEncoderPin(2, 4);
  motorL.setRR(30);
  motorL.setPIDconstants(2.2, 0.9, 0.15);
  motorL.setPins();
  motorL.stop();
  attachInterrupt(digitalPinToInterrupt(2), interruptL, FALLING);

  // right motor:
  motorR.hBridge(9, 10, 8);
  motorR.setEncoderPin(3, 5);
  motorR.setRR(30);
  motorR.setPIDconstants(2.2, 0.9, 0.15);
  motorR.setPins();
  motorR.stop();
  attachInterrupt(digitalPinToInterrupt(3), interruptR, FALLING);

  both.setGyreDegreesRatio(1.28, 90);

  screen.setButtons(0, 0, INPUT);
  String options[] = {
    "debugBlock",
    "obRight",
    "obLeft",
    "obHall"
  };
  //  screen.setOptions(options);
  //  while(!digitalRead(ok_button)) code_id = screen.getCurrentId();
  //  screen.write("starting...", "id: "+String(code_id));

  if (code_id == 0) {
    Serial.println("debug here, codeid: " + String(code_id));
    while (true) {
      Serial.println(frontDistance());
    }
    while (true) {
      Serial.print("right: ");
      Serial.println(rightDistance());
      
      delay(1000);
      Serial.print("front: ");
      Serial.println(frontDistance());
      
      delay(1000);
      Serial.print("left: ");
      Serial.println(leftDistance());
      delay(1000);
    }
  }
    

  ////////////////////// start movements////////////////////////
//  while(true) {
//    Serial.println(frontDistance());
//  }
  //first running and climb
  Serial.println("first climb");
  adjustFrontDistance(basespeed, 5);
  Align();
  debug();

  // going down first stair
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
  while (rightDistance() < 70) {
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
  while (rightDistance() < 10) {
    motorR.walk(basespeed);
    motorL.walk(basespeed);
  }
  both.stop();
  both.together(basespeed, 1);
  float c = 0.5; // turning coefficient
  float rotations = 1;
  both.together(turnspeed, rotations, turnspeed * c, rotations);
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


float frontDistance() {
  servoDistance.write(90);
  return readDistance();
}

float rightDistance() {
  servoDistance.write(0);
  return readDistance();
}

float leftDistance() {
  servoDistance.write(180);
  return (readDistance()-10.0);
}

float readDistance() {
  float distance = ultrassonic.getDistance_cm();
  Serial.println(distance);
  return distance;
}

void adjustFrontDistance(float speed, float desired_distance) {
  both.stop();
  speed = abs(speed);
  desired_distance = abs(desired_distance);
  front_distance = frontDistance();
  bool go_forward = front_distance > desired_distance;
  bool go_backward = front_distance < desired_distance;

  if (!go_forward && !go_backward) {
    // in this case front_distance==desired_distance
  } else if (go_forward) {
    while (frontDistance() > desired_distance) {
      motorR.walk(speed);
      motorL.walk(speed);
    }
  } else if (go_backward) {
    while (frontDistance() < desired_distance) {
      motorR.walk(-speed);
      motorL.walk(-speed);
    }
  }
  both.stop();
}

bool isRescueArea() {
  return false;
}

void takeDownWall(float speed) {
  speed = abs(speed);
  both.together(speed, 0.5);
  both.together(-speed, -0.5);
  both.stop();
}

void debug() {
  both.stop();
  while (true) delay(1000);
}
