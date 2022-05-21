#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <My_ultrassonic.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFun_APDS9960.h>

Servo servoDistance;
Servo servoColor;
SparkFun_APDS9960 apds = SparkFun_APDS9960();
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

////////////////////////////
// pin variables
#define ok_button 0
#define align_button 0
////////////////////////////
// static variables
float basespeed = 80.0;
float turnspeed = 40.0;
////////////////////////////////
// editable variables
bool have_right = true;
bool have_hall = false;
bool have_left = false;
byte code_id = 1;
////////////////////////////////
// dynamic variables
float front_distance = 0;
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;
String current_colors[2] = {"WHITE", "RED"}; // {binary_color, RGB_color}
////////////////////////////////
void Align(byte speed = 80) {
  while (digitalRead(align_button)) {
    motorR.walk(speed);
    motorL.walk(speed);
  }
  delay(400);
  both.stop();
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

void reachWall() {
  adjustFrontDistance(100, 2.3);
  //Align();
}

void rightCircumvent(float rot_left = 6.5, float rot_right = 1) {
  float right_coefficient = (rot_right / rot_left);
  float spinspeed = basespeed;
  both.together(spinspeed, rot_left, spinspeed * right_coefficient, rot_right);
  both.stop();
}

void readColor(uint16_t max_black=450) {
  // Read the light levels (ambient, red, green, blue)
  if (  !apds.readAmbientLight(ambient_light) ||
        !apds.readRedLight(red_light) ||
        !apds.readGreenLight(green_light) ||
        !apds.readBlueLight(blue_light) ) {
    Serial.println("Error reading light values");
  } else {
//    Serial.print("Ambient: ");
//    Serial.print(ambient_light);
//    Serial.print(" Red: ");
//    Serial.print(red_light);
//    Serial.print(" Green: ");
//    Serial.print(green_light);
//    Serial.print(" Blue: ");
//    Serial.println(blue_light);
  }

  if (ambient_light<=max_black) {
    current_colors[0] = "BLACK";
  } else {
    current_colors[0] = "WHITE";
  }

  // current_colors
  if ((red_light>green_light) && (red_light>blue_light)) {
    current_colors[1] = "RED";
  } else if ((green_light>red_light) && (green_light>blue_light)) {
    current_colors[1] = "GREEN";
  } else {
    current_colors[1] = "BLUE";
  }
}

String frontColor(byte mode=1, uint16_t max_black=450) {
  servoColor.write(0);
  readColor(max_black);
  return current_colors[mode];
}

String groundColor(byte mode=1, uint16_t max_black=450) {
  servoColor.write(90);
  readColor(max_black);
  return current_colors[mode];
}

void setup () {
  Serial.begin (9600);

  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  
  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
//    Serial.println(F("Light sensor is now running"));
  } else {
//    Serial.println(F("Something went wrong during light sensor init!"));
  }
  
  // distance sensor
  ultrassonic.setPins();

  // servo to move distance sensor
  servoDistance.attach(52);
  servoDistance.write(90); //0(right) 90(front) 180 (left)
  
  // servo to move color sensor
  servoColor.attach(50);
  servoColor.write(90); // 0(front) 90(ground)
  
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
  delay(500);

  // screen.setButtons(0, 0, INPUT);
  String options[] = {
    "debugBlock",
    "obRight",
    "obLeft",
    "rescueArena",
  };
  //  screen.setOptions(options);
  //  while(!digitalRead(ok_button)) code_id = screen.getCurrentId();
  //  screen.write("starting...", "id: "+String(code_id));

  if (code_id == 0) {
    Serial.println("debug here, codeid: " + String(code_id));
    while (true) {
      Serial.println(frontDistance());
      delay(300);
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


  goto lastHall;
  ////////////////////// start movements////////////////////////
  //first section
  Serial.println("first section");
  reachWall();
  
  // going down first stair
  both.turnDegree(-turnspeed, -90);
  reachWall();
  both.together(-basespeed, -0.08);
  both.turnDegree(-turnspeed, -90);

  if (have_left || ((have_hall==true) && (have_right==false))) {
    obLeft();
  } else {
    obRight();
  }

  Serial.println("last hall");
  reachWall();
  both.turnDegree(turnspeed, 90);

  lastHall:
    both.together(basespeed, 5.8);
  /////////////////
  groundColor();
  delay(200);
  
  while (!isRescueArena()) {
    motorR.walk(30);
    motorL.walk(30);
  }
  ////////////////////// start rescue////////////////////////
  rescueBlock:
    both.stop();
    rescueArena();
}

bool isRescueArena() {
  readColor();
  return (ambient_light<=1000);
}

void rescueArena() {
  both.together(basespeed, 0.1);
  rightCircumvent(4, 0.8);
  adjustFrontDistance(120, 10);
  motorR.walk(turnspeed, 3);
  debug();
  reachWall();
  both.turnDegree(-turnspeed, -90); // more
  both.together(basespeed, 3);
  debug();
  
  // take down wall
  debug();
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
  both.together(basespeed, 1.37);
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


float customDistance(byte angle=0) {
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
  float distance = ultrassonic.getDistance_cm();
  Serial.println(distance);
  return distance;
}

float degreeToRad(float degrees=0) {
  return (PI*degrees)/180;
}

void debug() {
  both.stop();
  while (true) {
    delay(1000);
  }
}


void loop () {

}
