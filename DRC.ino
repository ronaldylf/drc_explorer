#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <My_ultrassonic.h>
#include <Servo.h>
#include <Wire.h>
//#include <SparkFun_APDS9960.h>
#include <Pushbutton.h>
#include "Adafruit_VL53L0X.h"


Servo servoDistance;
Servo servoColor;
Servo servoArm;

//SparkFun_APDS9960 apds = SparkFun_APDS9960();

DC_motor_controller motorR;
DC_motor_controller motorL;
My_ultrassonic ultrassonic(24, 22);
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


#define left_led 53 // green
#define mid_led 51 // yellow
#define right_led 49 // red
#define green_led 53
#define yellow_led 51
#define red_led 49
////////////////////////////
// static variables
float basespeed = 80.0;
float turnspeed = 40.0;
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
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;
String current_colors[2] = {"WHITE", "RED"}; // {binary_color, RGB_color}
float small = 0;
float super_small = 0;
float big = 0;
float super_big = 0;
String checkpoint = "";
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
  //  // Read the light levels (ambient, red, green, blue)
  //  if (  !apds.readAmbientLight(ambient_light) ||
  //        !apds.readRedLight(red_light) ||
  //        !apds.readGreenLight(green_light) ||
  //        !apds.readBlueLight(blue_light) ) {
  //    Serial.println("Error reading light values");
  //  } else {
  //    //    Serial.print("Ambient: ");
  //    //    Serial.print(ambient_light);
  //    //    Serial.print(" Red: ");
  //    //    Serial.print(red_light);
  //    //    Serial.print(" Green: ");
  //    //    Serial.print(green_light);
  //    //    Serial.print(" Blue: ");
  //    //    Serial.println(blue_light);
  //  }
  //
  //  if (ambient_light <= max_black) {
  //    current_colors[0] = "BLACK";
  //  } else {
  //    current_colors[0] = "WHITE";
  //  }
  //
  //  // current_colors
  //  if ((red_light > green_light) && (red_light > blue_light)) {
  //    current_colors[1] = "RED";
  //  } else if ((green_light > red_light) && (green_light > blue_light)) {
  //    current_colors[1] = "GREEN";
  //  } else {
  //    current_colors[1] = "BLUE";
  //  }
}

String frontColor(byte mode = 1, uint16_t max_black = 450) {
  servoColor.write(0);
  readColor(max_black);
  return current_colors[mode];
}

String groundColor(byte mode = 1, uint16_t max_black = 450) {
  servoColor.write(90);
  readColor(max_black);
  return current_colors[mode];
}

bool isBig(float test_distance) {
  return (small < test_distance < super_big);
}

bool isSuperBig(float test_distance) {
  return (test_distance > super_big);
}

bool isSmall(float test_distance) {
  return (super_small < test_distance < big);
}

bool isSuperSmall(float test_distance) {
  return (test_distance <= super_small);
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

  // setup indication leds
  pinMode(left_led, OUTPUT);
  pinMode(mid_led, OUTPUT);
  pinMode(right_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(yellow_led, OUTPUT);
  pinMode(red_led, OUTPUT);

  // setting all leds to low
  digitalWrite(left_led, LOW);
  digitalWrite(mid_led, LOW);
  digitalWrite(right_led, LOW);
  digitalWrite(green_led, LOW);
  digitalWrite(yellow_led, LOW);
  digitalWrite(red_led, LOW);


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

  // distance sensor
  ultrassonic.setPins();

  // servo to move color sensor
  servoColor.attach(52); // 0(front) 90(ground)
  frontColor();

  // servo to move the arm
  servoArm.attach(48); // 0(up) 90(ground)
  servoArm.write(0);

  // servo to move distance sensor
  servoDistance.attach(50); //0(right) 90(front) 180 (left)

  // for distance sensor
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }


  //////////////////////////////////// actions part ///////////////////////////////////////////////////
  // setting all leds to low
  digitalWrite(green_led, HIGH);
  digitalWrite(yellow_led, HIGH);
  digitalWrite(red_led, HIGH);
  delay(100);
  digitalWrite(green_led, LOW);
  digitalWrite(yellow_led, LOW);
  digitalWrite(red_led, LOW);
  frontDistance();
  rightDistance();
  delay(300);

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

  // set leds to indicate the button worked part1
  digitalWrite(left_led, have_left);
  digitalWrite(mid_led, have_hall);
  digitalWrite(right_led, have_right);

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

  // set leds to indicate the button worked part1
  digitalWrite(left_led, have_left);
  digitalWrite(mid_led, have_hall);
  digitalWrite(right_led, have_right);

  //////////////////////////////////// CHECKPOINT part ///////////////////////////////////////////////////
  servoArm.write(0); // arm up fast
  // all set to low again to maybe debug with led later
  digitalWrite(left_led, LOW);
  digitalWrite(mid_led, LOW);
  digitalWrite(right_led, LOW);
  right_distance = rightDistance();
  frontDistance();
  delay(200);
  front_distance = frontDistance();

  if (front_distance <= 135) {
    goto checkpoint_A;
  }

  if ((front_distance >= 150)) {
    goto checkpoint_C;
  }


  ////////////////////// start robot movements////////////////////////
checkpoint_A:
  digitalWrite(red_led, HIGH);
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
  digitalWrite(green_led, HIGH);
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
  both.together(basespeed, 2);
  both.turnDegree(turnspeed, 90);

  float align_cube_distance = 30; // distance before get cube
  float align_cube_rotations = 0.55; // rotations before get cube
  float jump = 7;
  float current_distance = frontDistance() - jump;
  float degrees_correction = 20;
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
