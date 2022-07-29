#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include <Pushbutton.h>
#include "Adafruit_VL53L0X.h"

Servo ServoDistance;
Servo ServoColor;
/////////// to get cube
Servo BaseArm;
Servo MidArm;
Servo Gear;
//////////

SparkFun_APDS9960 apds = SparkFun_APDS9960();

DC_motor_controller motorR;
DC_motor_controller motorL;
TwoMotors both(&motorL, &motorR);
void interruptR () {
  motorR.isr();
}
void interruptL () {
  motorL.isr();
}
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
////////////////////////////
// setup variables
Pushbutton left_button(A0);
Pushbutton mid_button(A1);
Pushbutton right_button(A2);

////////////////////////////
// static variables
float basespeed = 25.0;
float turnspeed = 15.0;
////////////////////////////////
// editable variables
bool have_right = false;
bool have_hall = false;
bool have_left = false;
////////////////////////////////
// dynamic variables
float front_distance = 0;
float left_distance = 0;
float right_distance = 0;
String colors[2] = {"WHITE", "RED"}; // {binary_color, RGB_color}
////////////////////////////////
void adjustFrontDistance(float speed, float desired_distance, bool stop_ = true) {
  speed = abs(speed);
  desired_distance = abs(desired_distance);
  float current_distance = frontDistance();
  if (current_distance>desired_distance) {
    while ((frontDistance() > desired_distance)) {
      motorR.walk(speed);
      motorL.walk(speed);
    }
  } else if (current_distance<desired_distance) {
    while ((frontDistance() < desired_distance)) {
      motorR.walk(-speed);
      motorL.walk(-speed);
    }
  }

  if (stop_) {
    both.stop();
  }
}

void reachWall(bool go_back = true) {
  adjustFrontDistance(100, 2.3);
  if (go_back) {
    both.together(-basespeed, -0.07);
  }
}

void alignBack(byte intertia_time=1000) {
  const byte button = 7;
  const byte speed = 100;
  pinMode(button, INPUT_PULLUP);
  while(digitalRead(button)) {
    motorR.walk(-speed);
    motorL.walk(-speed);
  }
  delay(intertia_time);
  both.stop();
}

void rightCircumvent(float rot_left = 6.5, float rot_right = 1) {
  float right_coefficient = (rot_right / rot_left);
  float spinspeed = basespeed;
  both.together(spinspeed, rot_left, spinspeed * right_coefficient, rot_right);
  both.stop();
}

String readColor(int max_black = 1) {
  uint16_t ambient = 0;
  uint16_t red = 0;
  uint16_t green = 0;
  uint16_t blue = 0;
  apds.readAmbientLight(ambient);
  apds.readRedLight(red);
  apds.readGreenLight(green);
  apds.readBlueLight(blue);
  
  String current_color = "WHITE";
  float red_margin = 0.85; float possible_r;
  float green_margin = 0.72; float possible_g;
  float blue_margin = 0.85; float possible_b;
  
  float a = float(ambient);
  float r = float(red);
  float g = float(green);
  float b = float(blue);

//  Serial.print("a: "+String(a)+" ");
//  Serial.print("r: "+String(r)+" ");
//  Serial.print("g: "+String(g)+" ");
//  Serial.print("b: "+String(b)+"");
//  Serial.println();

  possible_r = r * red_margin;
  possible_g = g * green_margin;
  possible_b = b * blue_margin;
  if (possible_r > g && possible_r > b) {
    current_color = "RED";
  } else if (possible_g > r && possible_g > b) {
    current_color = "GREEN";
  } else if (possible_b > r && possible_b > g) {
    current_color = "BLUE";
  }

  if (r<=max_black && g<=max_black && b<=max_black) {
    current_color = "BLACK";
  }

  return current_color;
}

String frontColor() {
  ServoColor.write(90);
  return readColor();
}

String groundColor() {
  ServoColor.write(180);
  return readColor();
}


bool isCube() {
  frontColor();
  pinMode(A7, INPUT);
  float value = analogRead(A7);
  Serial.println(value <= 100);
  return (value <= 100);
}

bool isCircle() {
  groundColor();
  pinMode(A7, INPUT);
  float value = analogRead(A7);
  Serial.println(value <= 100);
  return (value <= 100);
}

void armDOWN(float timer_speed = 12) {

}

void armUP(float timer_speed = 15) {

}

void armAway() {
  BaseArm.write(0);
  MidArm.write(20);
}

void getCube() {
  Gear.write(180);
  BaseArm.write(95);
  MidArm.write(140);
  delay(1000);
  Gear.write(0);
  while(true) {
    motorR.walk(-basespeed);
    motorL.walk(-basespeed);
  }
}

void setup () {
  //////////////////////////////////// setup part ///////////////////////////////////////////////////
  Serial.begin(9600);
  while (! Serial) {
    delay(1);
  }


  Serial.println("reiniciou");


  // left motor:
  motorL.hBridge(11, 12, 13);
  motorL.setEncoderPin(2, 5);
  motorL.setRR(30);
  motorL.setPIDconstants(4, 0.9, 0.15);
  motorL.setPins();
  motorL.stop();
  attachInterrupt(digitalPinToInterrupt(2), interruptL, FALLING);


  // right motor:
  motorR.hBridge(9, 10, 8);
  motorR.setEncoderPin(3, 4);
  motorR.setRR(30);
  motorR.setPIDconstants(4, 0.9, 0.15);
  motorR.setPins();
  motorR.stop();
  attachInterrupt(digitalPinToInterrupt(3), interruptR, FALLING);

  both.setGyreDegreesRatio(1.4, 90);

  // servo to move color sensor
  ServoColor.attach(42); // 90(front) 180(ground)
  frontColor();

  // servo to move the arm
  BaseArm.attach(48); // 0(super down) , 180(super up)
  MidArm.attach(46); // 0 (extremo abaixada), 90 (extremo recuada)
  Gear.attach(44); // 180 (aberta) 0 (fechada)
  armAway();

  
  // servo to move distance sensor
  ServoDistance.attach(50); //0(right) 90(front) 180 (left)
  /*
    // for test servo
    while(true) {
    ServoDistance.write(0); delay(700); Serial.println("right");
    ServoDistance.write(90); delay(700); Serial.println("front");
    ServoDistance.write(180); delay(700); Serial.println("left");
    } */

  if (!lox.begin()) { // for distance sensor
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }

  //if (!apds.init()) { // for color sensor
  //  Serial.println(F("Failed to boot APDS"));
  //  while (1);
  //}
  apds.enableLightSensor(false);

  //////////////////////////////////// actions part ///////////////////////////////////////////////////
  // cleaning panel

  /*
    //// button wait first obstacle position
    while (true) {
    if (!have_left) {
      have_left = left_button.isPressed();
    }

    if (!have_hall) {
      have_hall = mid_button.isPressed();
    }

    if (!have_right) {
      have_right = right_button.isPressed();
    }

    if (have_left || have_hall || have_right) {
      break;
    }
    }

    // indicate the button worked part1

    if (!have_left && have_hall && !have_right) {
    have_left = true;
    }


    // wait to release buttons
    left_button.waitForRelease();
    mid_button.waitForRelease();
    right_button.waitForRelease();
    ////// button wait second obstacle position
    bool left_pressed = false;
    bool mid_pressed = false;
    bool right_pressed = false;
    while (true) {
    left_pressed = left_button.isPressed();
    mid_pressed = mid_button.isPressed();
    right_pressed = right_button.isPressed();
    if (left_pressed || mid_pressed || right_pressed) {
      break;
    }
    }
    if (have_left == false) {
    have_left = left_pressed;
    }

    if (have_hall == false) {
    have_hall = mid_pressed;
    }

    if (have_right == false) {
    have_right = right_pressed;
    }
  */

  // indicate the button worked part2

  bool debug_mode = true; // debug mode
  if (debug_mode) {
//    Serial.println(frontDistance()); debug();
    RescueProcess();
    debug();
  }

  have_right = true;   // remove all after tests
  have_hall = true;
  have_left = false;  //////////////////////////////////// CHECKPOINT part ///////////////////////////////////////////////////
  front_distance = frontDistance();

  if (front_distance < 140) {
    goto checkpoint_A;
  } else {
    goto checkpoint_C;
  }

  ////////////////////// start robot movements////////////////////////
checkpoint_A:
  //first section
  Serial.println("first section");
  reachWall();

  // going down first stair
  both.turnDegree(-turnspeed, -90);
  reachWall(false);

  both.together(-basespeed, -0.2);
  both.turnDegree(-turnspeed, -90);
  if (have_left) {
    obLeft();
  } else {
    obRight();
  }


  reachWall();
  both.turnDegree(turnspeed, 90);
  Serial.println("last hall");
  both.together(basespeed, 5.8);
  /////////////////

  Serial.println("pre-rescue");
checkpoint_C:
  rightDistance();
  delay(200);
  while (!isRescueArena()) {
    motorR.walk(50);
    motorL.walk(50);
  }
  ////////////////////// start rescue//////////////////// ////
  both.stop();
  RescueProcess();
}

bool isRescueArena() {
  //  return (groundColor() == "BLUE");
  return rightDistance() >= 15;
}

void RescueProcess() {
  basespeed = 60;
  turnspeed = 40;
  goto debugBlock;
  both.together(basespeed, 0.3);
  rightCircumvent(4.2, 0.84);
  debugBlock: Serial.println("starting debugBlock");
  frontDistance();
  delay(100);
  reachWall();
  both.turnDegree(-turnspeed, -105);

  ///////////////
  both.together(basespeed, 0.41);
  both.turnDegree(-turnspeed, -95);
  alignBack();
  both.together(basespeed, 3);
  both.turnDegree(turnspeed, 90);
  reachWall();
  adjustFrontDistance(basespeed, 3.8);
  both.turnDegree(turnspeed, 95);

  // go to cube
  while(frontColor()!="RED") {
    motorR.walk(basespeed);
    motorL.walk(basespeed);
  }

  adjustFrontDistance(basespeed, 28);
  getCube();

  while(true) {
    motorR.walk(-basespeed);
    motorL.walk(-basespeed);
  }
}

void obLeft() {
  if (have_hall) {
    rightDistance();
    delay(200);
    while (rightDistance() < 10) {
      motorR.walk(basespeed);
      motorL.walk(basespeed);
    }

    rightCircumvent();
    both.turnDegree(-turnspeed, -90);
    reachWall();
    both.turnDegree(-turnspeed, -90);
    rightDistance();
    delay(200);

    while (rightDistance() > 10) {
      motorR.walk(-basespeed);
      motorL.walk(-basespeed);
    }
    both.stop();
    while (rightDistance() < 10) {
      motorR.walk(basespeed);
      motorL.walk(basespeed);
    }
    rightCircumvent(3.25, 0.5); // 1/4 turn
  } else {
    frontDistance();
    delay(200);
    reachWall();
    both.turnDegree(turnspeed, 85);
  }
}

void obRight() {
  both.together(basespeed, 1.65);
  both.turnDegree(-turnspeed, -90);
  reachWall();
  both.turnDegree(turnspeed, 90);
  reachWall();
  both.turnDegree(turnspeed, 90);
  if (have_hall) {
    both.together(basespeed, 2); // a lot
    rightCircumvent(3.25, 0.5); // 1/4 turn
    both.together(basespeed, 0.3); // not so much
    both.turnDegree(-turnspeed, -90);
    reachWall();
    both.turnDegree(-turnspeed, -90);

    rightDistance();
    delay(200);

    while (rightDistance() > 10) {
      motorR.walk(-basespeed);
      motorL.walk(-basespeed);
    }
    both.stop();
    while (rightDistance() < 10) {
      motorR.walk(basespeed);
      motorL.walk(basespeed);
    }

    rightCircumvent(3.25, 0.5); // 1/4 turn
  }
}

float customDistance(byte angle = 0) {
  ServoDistance.write(angle);
  return readDistance();
}

float frontDistance() {
  ServoDistance.write(90);
  return readDistance();
}

float rightDistance() {
  ServoDistance.write(0);
  return readDistance();
}

float leftDistance() {
  ServoDistance.write(180);
  return (readDistance() - 10.0);
}

float readDistance() {
  float distance = 0;
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    distance = measure.RangeMilliMeter / 10;
  } else {
    distance = 200;
  }

  return (distance - 2);
}

float degreeToRad(float degrees = 0) {
  return (PI * degrees) / 180;
}

void debug() {
  both.stop();
  while (true) {
    delay(1000);
  }
}


void loop () {
}
