#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include <Pushbutton.h>
#include "Adafruit_VL53L0X.h"


Servo servoDistance;
Servo servoColor;
Servo servoArm;

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
float basespeed = 50.0;
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
  while (frontDistance() > desired_distance) {
    motorR.walk(speed);
    motorL.walk(speed);
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

void alignBack(float timer_going_back=2000, float timer_stop=300) {
  motorR.walk(-basespeed);
  motorL.walk(-basespeed);
  delay(timer_going_back);
  both.stop();
  delay(timer_stop);
}

void rightCircumvent(float rot_left = 6.5, float rot_right = 1) {
  float right_coefficient = (rot_right / rot_left);
  float spinspeed = basespeed;
  both.together(spinspeed, rot_left, spinspeed * right_coefficient, rot_right);
  both.stop();
}

void readColor(uint16_t max_black = 450) {
  String current_color = "BRANCO";
  byte red_margin = 0.8;
  byte green_margin = 0.8;
  byte blue_margin = 0.8;

  uint16_t ambient = 0;
  uint16_t r = 0;
  uint16_t g = 0;
  uint16_t b = 0;

  uint16_t possible_r;
  uint16_t possible_g;
  uint16_t possible_b;

  apds.readAmbientLight(ambient);
  apds.readRedLight(r);
  apds.readGreenLight(g);
  apds.readBlueLight(b);

  possible_r = r*red_margin;
  possible_g = g*green_margin;
  possible_b = b*blue_margin;
  if (r>possible_g && r>possible_b) {
    current_color = "RED";
  }
}

String frontColor(byte mode = 1, uint16_t max_black = 450) {
  servoColor.write(0);
  readColor(max_black);
  return colors[mode];
}

String groundColor(byte mode = 1, uint16_t max_black = 450) {
  servoColor.write(90);
  readColor(max_black);
  return colors[mode];
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
  Serial.println("arm going down");
  for (int i = 0; i <= 90; i++) {
    servoArm.write(i);
    delay(timer_speed);
  }
}

void armUP(float timer_speed = 15) {
  Serial.println("arm going up");
  for (int i = 90; i >= 0; i--) {
    servoArm.write(i);
    delay(timer_speed);
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
  motorL.setPIDconstants(2.2, 0.9, 0.15);
  motorL.setPins();
  motorL.stop();
  attachInterrupt(digitalPinToInterrupt(2), interruptL, FALLING);


  // right motor:
  motorR.hBridge(9, 10, 8);
  motorR.setEncoderPin(3, 4);
  motorR.setRR(30);
  motorR.setPIDconstants(2.2, 0.9, 0.15);
  motorR.setPins();
  motorR.stop();
  attachInterrupt(digitalPinToInterrupt(3), interruptR, FALLING);
  
  both.setGyreDegreesRatio(1.5, 90);

  while(true) {
    motorR.walk(40);
    motorL.walk(40);
  }
  
  // servo to move color sensor
  servoColor.attach(52); // 0(front) 90(ground)
  frontColor();

  // servo to move the arm
  servoArm.attach(48); // 0(up) 90(ground)
  servoArm.write(0);

  // servo to move distance sensor
  servoDistance.attach(50); //0(right) 90(front) 180 (left)
  /*
  // for test servo
  while(true) {
    servoDistance.write(0); delay(700); Serial.println("right");
    servoDistance.write(90); delay(700); Serial.println("front");
    servoDistance.write(180); delay(700); Serial.println("left");
  } */

  if (!lox.begin()) { // for distance sensor
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }

  if (!apds.init()) { // for color sensor
    Serial.println(F("Failed to boot APDS"));
    while (1);
  }
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

  have_right = false;   // remove all after tests
  have_hall = false;
  have_left = true;

  //////////////////////////////////// CHECKPOINT part ///////////////////////////////////////////////////
  servoArm.write(0); // arm up fast
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
    motorR.walk(30);
    motorL.walk(30);
  }
  ////////////////////// start rescue//////////////////// ////         
  both.stop();
  rescueArena();
}

bool isRescueArena() {
  //  return (groundColor() == "BLUE");
  return rightDistance() >= 15;
}

void rescueArena() {
  both.together(basespeed, 0.3);
  rightCircumvent(4, 0.8);
  frontDistance();
  delay(100);
  reachWall();
  both.turnDegree(-turnspeed, -90);

  reachWall();
  
  both.turnDegree(-turnspeed, -95); // more
  alignBack(); // align in the back
  both.together(basespeed, 3);
  both.turnDegree(turnspeed, 90);
  both.together(basespeed, 1.8);
  both.turnDegree(turnspeed, 90);

  float align_cube_distance = 30; // distance before get cube
  float align_cube_rotations = 0.2;
  // rotations before get cube
  float jump = 7;
  float current_distance = frontDistance() - jump;
  float degrees_correction = 0;
  float rotations_correction = 1;
  while (true) {
    adjustFrontDistance(basespeed, align_cube_distance);
    both.together(basespeed, align_cube_rotations);
    both.turnDegree(turnspeed, degrees_correction);
    armDOWN(7);
    both.together(-basespeed, -rotations_correction);
    both.turnDegree(-turnspeed, -degrees_correction);
    both.turnDegree(turnspeed, 180);
    adjustFrontDistance(basespeed, current_distance);
    current_distance -= jump;
    armUP(7);
    rightDistance();
    delay(100);
    right_distance = rightDistance();
    both.turnDegree(turnspeed, 90);
    frontDistance();
    delay(100);
    adjustFrontDistance(basespeed, right_distance-3); // VAI DE 3 EM 3 CENTÍMETROS
    both.turnDegree(turnspeed, 90);
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
  servoDistance.write(angle);
  return readDistance();
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
  float distance = 0;
  VL53L0X_RangingMeasurementData_t measure;
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    distance = measure.RangeMilliMeter / 10;
  } else {
    Serial.println(" out of range ");
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
