#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <My_ultrassonic.h>
#include <Servo.h>
// #include <LCDScroll.h>


Servo servoDistance;
DC_motor_controller motorR;
DC_motor_controller motorL;
// LCDScroll screen;
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
void Align(byte speed = 70) {
  //  if (speed > 0) {
  //    both.together(speed, 0.4);
  //  } else {
  //    // going backwards
  //  }
  //  both.stop();
}

void adjustFrontDistance(float speed, float desired_distance, bool stop_ = true) {
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

  if (stop_) {
    both.stop();
  }
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

  both.setGyreDegreesRatio(1.5, 90);

  // screen.setButtons(0, 0, INPUT);
  String options[] = {
    "debugBlock",
    "obRight",
    "obLeft",
    "obHall",
    "rescueArena",
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
  //first running and climb
  Serial.println("first climb");
  adjustFrontDistance(basespeed, 3.2);
  Align();

  // going down first stair
  both.turnDegree(-turnspeed, -90);
  adjustFrontDistance(basespeed, 4);
  Align();
  both.together(-basespeed, -0.08);
  both.turnDegree(-turnspeed, -90);

  //  obHall();
  obRight();

  while (!isRescueArena()) {
    motorR.walk(basespeed);
    motorL.walk(basespeed);
  }

  ////////////////////// start rescue////////////////////////
  rescueArena();
}

void obRight() {
  both.together(basespeed, 1.37);
  both.turnDegree(-turnspeed, -90);
  motorL.walk(turnspeed, 2.75);
  both.stop();

  rightDistance();
  delay(350);

  while (rightDistance() < 30) {
    motorR.walk(basespeed);
    motorL.walk(basespeed);
  }
  both.stop();
  motorL.walk(turnspeed, 2.75);
  adjustFrontDistance(basespeed, 3.5);
  both.turnDegree(turnspeed, 90);
}

void obHall() {
  rightDistance();
  delay(200);
  while (rightDistance() < 35) {
    motorR.walk(basespeed);
    motorL.walk(basespeed);
  }

  rightCircumvent();

  both.turnDegree(-turnspeed, -90);
  adjustFrontDistance(basespeed, 4);
  Align();
  both.turnDegree(-turnspeed, -90);
  while (rightDistance() < 35) {
    motorR.walk(basespeed);
    motorL.walk(basespeed);
  }

  rightCircumvent();
}

void rightCircumvent() {
  float rot_right = 1;
  float rot_left = 6.5;
  float right_coefficient = (rot_right / rot_left);
  float spinspeed = basespeed;
  both.together(spinspeed, rot_left, spinspeed * right_coefficient, rot_right);
  both.stop();
}
void rescueArena() {
  both.together(basespeed, 1.85);
  both.turnDegree(turnspeed, 90);
  //Align(-130);
  adjustFrontDistance(basespeed, 5);
  both.turnDegree(-turnspeed, -90);
  adjustFrontDistance(basespeed, 5);
  both.turnDegree(-turnspeed, -90);
  takeDownWall(40);
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
  return (readDistance() - 10.0);
}

float readDistance() {
  float distance = ultrassonic.getDistance_cm();
  Serial.println(distance);
  return distance;
}

bool isRescueArena() {
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


void loop () {

}
