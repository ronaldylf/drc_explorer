#include <DC_motor_controller.h>
#include <TwoMotors.h>
#include <Servo.h>
#include <SparkFun_APDS9960.h>
#include <Pushbutton.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Servo ServoDistance;
Servo ServoColor;
/////////// to get cube
Servo BaseArm;
Servo MidArm;
//////////
Adafruit_SSD1306 display(128, 64, &Wire, -1);

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
VL53L0X eye_lox, arm_lox;
#define SHUT_ARM 51
#define SHUT_EYE 45
////////////////////////////
// setup variables
Pushbutton left_button(A0);
Pushbutton mid_button(A1);
Pushbutton right_button(A2);

////////////////////////////
// static variables
float basespeed = 50.0;
float maxspeed = 120;
float turnspeed = 30.0;
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

void reachWall(bool go_back = true, int delay_time=500) {
  adjustFrontDistance(100, 2.8);
  int t0 = millis(); int t=0;
  while((t-t0)<delay_time) {
    motorR.walk(100); motorL.walk(100);
    t = millis();
  }
  if (go_back) {
    both.together(-basespeed, -0.07);
  }
}

void alignBack(byte speed = 100, byte intertia_time = 1000) {
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
  float red_margin = 0.80; float possible_r; //0.85
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
  delay(200);
  MidArm.write(140); //180 (max down)
//  unlockArms();
}

float customDistance(byte angle = 0) {
  ServoDistance.write(angle);
  return readDistance();
}

void writeText(String text = " ", byte size_ = 3) {
  display.clearDisplay();
  display.setTextSize(size_);
  display.setTextColor(WHITE);
  display.setCursor(0, 28);
  display.println(text); Serial.println(text);
  display.display();
}

void setup () {
  //////////////////////////////////// setup part ///////////////////////////////////////////////////
  Serial.begin(9600);
  while (! Serial) {
    delay(1);
  }

  Serial.println("reiniciou");
  Wire.begin();

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

  both.setGyreDegreesRatio(1.42, 90);

  // servo to move color sensor
  ServoColor.attach(44);
  frontColor();

  // servos to move the arm
  lockArms();  
  armAway();
  // servo to move distance sensor
  ServoDistance.attach(50); //0(right) 90(front) 180 (left)

  // initialize DISPLAY with the I2C addr 0x3C
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // Clear the buffer.
  display.clearDisplay();

  if (!apds.init()) { // for color sensor
    Serial.println(F("Failed to boot APDS"));
    while (1);
  }
  apds.enableLightSensor(false);

  // for distance sensors:
  // Define o sensor 2 como entrada para fazer o pino SHUT_ARM ficar em nível alto
  pinMode(SHUT_EYE, OUTPUT);
  pinMode(SHUT_ARM, INPUT);

  // "Desliga" o sensor 1
  digitalWrite(SHUT_EYE, LOW);
  delay(2);

  // Altera o endereço do sensor 2
  arm_lox.setAddress(0x32);

  // Religa o sensor 1 definindo ele como entrada
  pinMode(SHUT_EYE, INPUT);
  // É possível alterar o endereço do sensor 1 apenas com o código abaixo
  // Como o sensor 2 já está com endereço diferente, não é necessário desligá-lo,
  // pois ele não interferirá na comunicação
  //eye_lox.setAdress(0x31);

  // Inicializa sensores
  eye_lox.init(); eye_lox.setTimeout(500);
  arm_lox.init(); arm_lox.setTimeout(500);
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

    basespeed = 25.0;
    turnspeed = 15.0;

    both.together(-basespeed, -0.2);
    both.turnDegree(-turnspeed, -90);
    alignBack();
    if (have_left) {
      obLeft();
    } else {
      obRight();
    }


    reachWall();
    both.turnDegree(turnspeed, 90);
    alignBack();
    Serial.println("last hall");
    both.together(maxspeed, 6.1);
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
    both.together(basespeed, 3.3); //3.4
    both.turnDegree(turnspeed, 90);
debugBlock: Serial.println("debugBlock");
    reachWall();

    float total_cube_distance = 3.4; // distancia total do cubo ate a parede (old 3.4);
    float increment_cube_distance = 7; // distance entre os cubos

    float search_distance = 28;
    float max_search_distance[99]; for (int i = 0; i <= 99; i++) max_search_distance[i] = search_distance;
    float max_vertical_distance = 87;

    for (int cube_id = 0; cube_id <= 99; cube_id++) {
      if (cube_id==1) total_cube_distance = 7;
      if (cube_id==2) total_cube_distance = 18;
      if (cube_id==3) total_cube_distance = 25;
      writeText(String(total_cube_distance));
      adjustFrontDistance(basespeed, total_cube_distance);
      both.turnDegree(turnspeed, 90);
      frontColor();
      adjustFrontDistance(basespeed, 24.5); // [24, 25]
  
      motorR.stopCounting(); motorL.stopCounting();
      motorR.startCounting(); motorL.startCounting();
      float left_rotations;
      while (frontColor() == "RED") {
        motorL.walk(-basespeed); motorR.walk(basespeed);
        left_rotations = motorR.getRotations();
        writeText(String(left_rotations));
      }
      motorR.stopCounting(); motorL.stopCounting();
      motorR.startCounting(); motorL.startCounting();
      float right_rotations;
      while (frontColor() != "RED") {
        motorL.walk(basespeed); motorR.walk(-basespeed);
        right_rotations = motorL.getRotations();
        writeText(String(right_rotations));
      }
      motorR.stopCounting(); motorL.stopCounting();
      both.stop();
      float correction_rotations = 0.1;
      both.together(turnspeed, correction_rotations, -turnspeed, -correction_rotations);
      right_rotations += correction_rotations;
      float resultant_rotations = right_rotations - left_rotations;
      float side = resultant_rotations / abs(resultant_rotations);
      writeText(String(resultant_rotations));
      getCube(); delay(800);
      both.together(turnspeed * -side, abs(resultant_rotations) * -side, turnspeed * side, abs(resultant_rotations)*side);
  
      while (true) {
        both.together(-basespeed, -2.7);
        writeText(String(arm_lox.readRangeSingleMillimeters() / 10.0));
        if (hasCube()) {
          break;
        } else {
          armAway();
//          adjustFrontDistance(60, 35);
          both.together(basespeed, 2);
          getCube(); delay(800);
        }
      }

      adjustFrontDistance(basespeed, 65);
      
      if (cube_id == 0) {
        both.turnDegree(turnspeed, 180);
      } else {
        both.turnDegree(turnspeed, 90);
        alignBack(maxspeed); both.together(basespeed, 0.2);
        both.turnDegree(turnspeed, 90);
      }
    

      bool deliver_cube = false;
      byte turns = 0;

      groundColor(); delay(200);
      while (true) { //search for circle
        adjustFrontDistance(maxspeed, max_search_distance[turns]); // this distance will change

        both.reset();

        while (frontDistance() < max_vertical_distance) {
          motorR.walk(-maxspeed); motorL.walk(-maxspeed);
          deliver_cube = (groundColor() == "RED");
          if (deliver_cube) {
            if (frontDistance() > search_distance) {
              both.together(-basespeed, -0.8);
            }
            armAway();
            break;
          }
        }
        if (deliver_cube) { // deliver cube
          total_cube_distance += increment_cube_distance;
          const byte avoid_turns = 1; // a distancia diminui umas x voltas pra frente e pra trás
          for (int future = -avoid_turns; future <= avoid_turns; future++) {
            max_search_distance[turns + future] = frontDistance() + 20;
          }

          adjustFrontDistance(basespeed, max_vertical_distance);
          both.turnDegree(basespeed, 90); reachWall();
          break;
        }

        adjustFrontDistance(basespeed, max_vertical_distance);
        turns++;
      
        // if not found circle:
        both.turnDegree(-turnspeed, -90);
        both.together(basespeed, 0.5);
        both.turnDegree(turnspeed, 90);
      }
    }
  }

  void obLeft() {
    if (have_hall) {
      customDistance(10); delay(200);
      while (customDistance(10) < 10) {
        motorR.walk(basespeed);
        motorL.walk(basespeed);
      }

      both.stop(200);
      //    both.together(basespeed, 0.15);
      rightCircumvent();
      //    motorL.reset();
      //    while(motorL.canRun()) {
      //      motorL.gyrate(basespeed, 4.5);
      //      motorR.stop();
      //    }
      both.together(basespeed, 1);
      both.turnDegree(-turnspeed, -90);
      alignBack();
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
    both.together(basespeed, 1.75);
    both.turnDegree(-turnspeed, -90);
    reachWall();
    both.turnDegree(turnspeed, 90);
    reachWall();
    both.turnDegree(turnspeed, 90);
    if (have_hall) {
      alignBack();
      both.together(basespeed, 1);
      customDistance(45); delay(200);
      while (customDistance(45) < 77) {
        motorR.walk(basespeed);
        motorL.walk(basespeed);
      }
      both.together(basespeed, 0.5);
      rightCircumvent(3.25, 0.5); // 1/4 turn
      both.together(basespeed, 1); // not so much
      both.turnDegree(-turnspeed, -90);
      alignBack();
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
    const float distance = eye_lox.readRangeSingleMillimeters() / 10.0;
    return (distance - 2);
  }

  bool hasCube() {
    const float distance = arm_lox.readRangeSingleMillimeters() / 10.0;
    return (distance <= 6);
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
