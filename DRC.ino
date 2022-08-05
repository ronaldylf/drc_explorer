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
byte pin_base_arm;
byte pin_mid_arm;
////////////////////////////////
void adjustFrontDistance(float speed, float desired_distance, bool stop_ = true) {
  speed = abs(speed);
  desired_distance = abs(desired_distance);
  float current_distance = frontDistance();
  if (current_distance > desired_distance) {
    while ((frontDistance() > desired_distance)) {
      motorR.walk(speed);
      motorL.walk(speed);
    }
  } else if (current_distance < desired_distance) {
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

void alignBack(byte speed=100, byte intertia_time = 1000) {
  const byte button = 7;
  pinMode(button, INPUT_PULLUP);
  while (digitalRead(button)) {
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

  if (r <= max_black && g <= max_black && b <= max_black) {
    current_color = "BLACK";
  }

  return current_color;
}

String frontColor() {
  ServoColor.write(100);
  return readColor();
}

String groundColor() {
  ServoColor.write(180);
  return readColor();
}


void armAway() {
  lockArms();
  BaseArm.write(0);
  MidArm.write(90);
  unlockArms();
}

void lockArms() {
  pin_base_arm = 48;
  pin_mid_arm = 46;
  BaseArm.attach(pin_base_arm); // 0(super back) 180(super front)
  MidArm.attach(pin_mid_arm); // 0 (super back), 180 (super front)
}

void unlockArms() {
  delay(1000);
  BaseArm.detach();
  MidArm.detach();
}

void getCube() {
  lockArms();
  BaseArm.write(150);
  MidArm.write(160);
  //  unlockArms();
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
  ServoColor.attach(44);
  frontColor();

  // servos to move the arm
  lockArms();
  armAway();

  // servo to move distance sensor
  ServoDistance.attach(50); //0(right) 90(front) 180 (left)

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

  bool debug_mode = true; // debug mode
  if (debug_mode) {    
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
  turnspeed = 50;
  float maxspeed = 120;
  goto debugBlock;
  both.together(basespeed, 0.3);
  rightCircumvent(4.2, 0.84);
  frontDistance();
  delay(100);
  reachWall();
  both.turnDegree(-turnspeed, -105);

  ///////////////
  both.together(basespeed, 0.40);
  both.turnDegree(-turnspeed, -95);
  alignBack();
  both.together(basespeed, 3.2);
  both.turnDegree(turnspeed, 90);
debugBlock: Serial.println("starting debugBlock");
  reachWall();

  float total_cube_distance = 3.4; // distancia total do cubo ate a parede
  float increment_cube_distance = 15; // (10) distancia entre cada cubo

  for (int cube_id = 0; cube_id <= 99; cube_id++) {
    adjustFrontDistance(basespeed, total_cube_distance);
    both.turnDegree(turnspeed, 95);

    // go to cube
    while (frontColor() != "RED") {
      motorR.walk(basespeed);
      motorL.walk(basespeed);
    }
    groundColor();

    adjustFrontDistance(basespeed, 20);
    getCube();
    delay(800);

    both.together(-basespeed, -3);
    if (cube_id==0) {
      both.turnDegree(turnspeed, 180); 
    } else {
      both.turnDegree(turnspeed, 90);
      alignBack(maxspeed);
      both.turnDegree(turnspeed, 95);
    }

    float search_distance = 28; // 33
    float ratio = 0.8;
    float current_angle = 0;
    bool deliver_cube = false;
    float max_vertical_distance = 75;
    byte turns = 0;
    float max_search_distance[99]; for (int i=0; i<=99; i++) max_search_distance[i] = search_distance;

    while (true) {
      // start search for circle
      adjustFrontDistance(maxspeed, max_search_distance[turns]); // this distance will change

      both.reset();
      //      while ((motorR.canRun() || motorL.canRun()) && !deliver_cube) {
      while (frontDistance() < max_vertical_distance) {
        //        motorR.gyrate(-basespeed, -3.1);
        //        motorL.gyrate(-basespeed, -3.1);
        motorR.walk(-maxspeed); motorL.walk(-maxspeed);
        deliver_cube = (groundColor() == "RED");
        if (deliver_cube) {
          if (frontDistance()>search_distance) {
            both.together(-basespeed, -0.8);
          }
          armAway();
          break;
        }
      }
      if (deliver_cube) { // deliver cube
        total_cube_distance += increment_cube_distance;
        for (int future=0; future<=3; future++) { // a distancia diminui umas 3 voltas pra frente
          max_search_distance[turns+future] = frontDistance()+5;
        }

        adjustFrontDistance(basespeed, max_vertical_distance);
        both.turnDegree(basespeed, 90); reachWall();
        break;
      }

      adjustFrontDistance(basespeed, max_vertical_distance);
      
      // if not found circle:
      both.turnDegree(-turnspeed, -90);
      both.together(basespeed, 0.5);
      both.turnDegree(turnspeed, 90);
      turns++;
    }
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
