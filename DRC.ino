// started change color library
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
Servo arm;
Servo hand;
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
#define SHUT_ARM 0
#define SHUT_EYE 0
////////////////////////////
// setup variables
Pushbutton left_button(7);
Pushbutton right_button(6);

////////////////////////////
// static variables
float basespeed = 70.0;//50
float maxspeed = 60.0;
float turnspeed = 30.0;
////////////////////////////////
// dynamic variables
byte pin_arm = 48;
byte pin_hand = 46;
byte pin_servo_distance = 50;
byte pin_servo_color = 52;
byte pin_align = 38;

float rot_per_degree = 1.3/90.0;

// dynamic functions
float blank_luminosity = 0;
void debug(String text = "");
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

void reachWall(bool go_back = true, int delay_time = 600) {
  adjustFrontDistance(100, 1);
  int t0 = millis(); int t = 0;
  while ((t - t0) < delay_time) {
    motorR.walk(100); motorL.walk(100);
    t = millis();
  }
  if (go_back) {
    both.together(-basespeed, -0.07);
  }
}

void alignBack(byte speed = 100) {
  pinMode(pin_align, INPUT_PULLUP);

//
//  while (digitalRead(pin_align)) {
//    motorR.walk(-speed);
//    motorL.walk(-speed);
//  }

  unsigned long init_time = millis();
  while (millis() - init_time < 500) {
    motorR.walk(-speed);
    motorL.walk(-speed);
  }
  both.stop();
}

void rightCircumvent(float rot_left = 6.5, float rot_right = 1) {
  float right_coefficient = (rot_right / rot_left);
  float spinspeed = basespeed;
  both.together(spinspeed, rot_left, spinspeed * right_coefficient, rot_right);
  both.stop();
}


float getLuminosity() {
  uint16_t ambient = 0;
  apds.readAmbientLight(ambient);
  return float(ambient);
}

String getColor() {
  uint16_t ambient = 0;
  uint16_t red = 0;
  uint16_t green = 0;
  uint16_t blue = 0;

  apds.readAmbientLight(ambient);
  apds.readRedLight(red);
  apds.readGreenLight(green);
  apds.readBlueLight(blue);

  String current_color = "WHITE";
  float min_black = blank_luminosity * 0.15;
  float red_margin = 0.9; float possible_r; //0.85
  float green_margin = 0.85; float possible_g;
  float blue_margin = 0.85; float possible_b;

  float a = float(ambient);
  float r = float(red);
  float g = float(green);
  float b = float(blue);
  //
  //  Serial.print("a: " + String(a) + " ");
  //  Serial.print("r: " + String(r) + " ");
  //  Serial.print("g: " + String(g) + " ");
  //  Serial.print("b: " + String(b) + "");
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

  if (r <= min_black && g <= min_black && b <= min_black) {
    current_color = "BLACK";
  }

  //  Serial.println();
  //  Serial.println("blank: " + String(blank_luminosity));
  //  Serial.println("min_black: " + String(min_black));
  //  Serial.print(current_color);
  //  Serial.println();

  return current_color;
}

String frontColor() {
  ServoColor.write(10);
  return getColor();
}

String groundColor() {
  ServoColor.write(90);
  return getColor();
}

void writeText(String text = " ", byte size_ = 3) {
  display.clearDisplay();
  display.setTextSize(size_);
  display.setTextColor(WHITE);
  display.setCursor(0, 28);
  display.println(text); Serial.println(text);
  display.display();
}

void armAway() {
  lockArms();
  arm.write(135); // 0 (super front), 180 (super back)
  hand.write(90); // 180 (super open) 0 (super close)
  unlockArms();
}

void lockArms() {
  arm.attach(pin_arm); // 0(super front), 180(super back)
  hand.attach(pin_hand);
}

void unlockArms() {
  delay(1000);
  arm.detach();
  hand.detach();
}

void getCube() {
  lockArms();
  hand.write(180); // 180 (super open) 0 (super close)
  arm.write(0); // 0 (super front), 180 (super back)
  delay(1000);
  both.together(50, 0.2);
  hand.write(80); // 180 (super open) 0 (super close)
  delay(720);
  arm.write(135); // 0 (super front), 180 (super back)
  unlockArms();
}

void dropCube() {
  lockArms();
  arm.write(0); // 0 (super front), 180 (super back)
  delay(1000);
  both.together(-basespeed, -0.1); // rotations because cube drops a little forward
  hand.write(180); // 180 (super open) 0 (super close)
  delay(1000);
  arm.write(135); // 0 (super front), 180 (super back)
  delay(1000);
  armAway();
}

float customDistance(byte angle = 0) {
  ServoDistance.write(angle);
  return readDistance();
}

void followWall(float left_ratio = 0, float right_ratio = 0.3, float adjust_distance = 9) {
  if (rightDistance() > adjust_distance) {
    motorL.walk(basespeed);
    motorR.walk(basespeed * right_ratio);
  } else {
    motorL.walk(basespeed * left_ratio);
    motorR.walk(basespeed);
  }
}

bool hasFoundWall(float margin = 0.2) {
  return (getLuminosity() * margin > blank_luminosity);
}
float getProximity() {
  uint8_t proximity_data = 0;
  apds.readProximity(proximity_data);
  return float(255 - proximity_data);
}

void setup () {
  //////////////////////////////////// setup part ///////////////////////////////////////////////////
  Serial.begin(9600);
  while (! Serial) {
    delay(1);
  }

  Serial.println("reiniciou");
  Wire.begin();

  const float kp = 3.5; // 2.5
  const float ki = 1;
  const float kd = 0.15;
  // left motor:
  // left motor:
  motorL.hBridge(11, 12, 13);
  motorL.setEncoderPin(2, 5);
  motorL.setRR(30);
  motorL.setPIDconstants(kp, ki, kd);
  motorL.setPins();
  motorL.stop();
  attachInterrupt(digitalPinToInterrupt(2), interruptL, FALLING);


  // right motor:
  motorR.hBridge(9, 10, 8);
  motorR.setEncoderPin(3, 4);
  motorR.setRR(30);
  motorR.setPIDconstants(kp, ki, kd);
  motorR.setPins();
  motorR.stop();
  attachInterrupt(digitalPinToInterrupt(3), interruptR, FALLING);

  both.setGyreDegreesRatio(1.3, 90);

  // servo to move distance sensor
  ServoDistance.attach(pin_servo_distance); //0(right) 90(front) 180 (left)

  // servo to move color sensor
  ServoColor.attach(pin_servo_color);


  // servos to move the arm
  lockArms();
  armAway();

  // initialize DISPLAY with the I2C addr 0x3C
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // Clear the buffer.
  display.clearDisplay();


  if (!apds.init()) { // for color sensor
    Serial.println(F("Failed to boot APDS"));
    while (1);
  }
  apds.enableLightSensor(false);
  apds.enableProximitySensor(false);

  // for distance sensors:
  // Define o sensor 2 como entrada para fazer o pino SHUT_ARM ficar em nível alto
  //pinMode(SHUT_EYE, OUTPUT);
  //pinMode(SHUT_ARM, INPUT);

  // "Desliga" o sensor 1
  //digitalWrite(SHUT_EYE, LOW);
  //delay(2);

  // Altera o endereço do sensor 2
  //arm_lox.setAddress(0x32);

  // Religa o sensor 1 definindo ele como entrada
  //pinMode(SHUT_EYE, INPUT);
  // É possível alterar o endereço do sensor 1 apenas com o código abaixo
  // Como o sensor 2 já está com endereço diferente, não é necessário desligá-lo,
  // pois ele não interferirá na comunicação
  //eye_lox.setAdress(0x31);

  // Inicializa sensores
  eye_lox.init(); eye_lox.setTimeout(500);
  //arm_lox.init(); arm_lox.setTimeout(500);

  groundColor();
  rightDistance();
  frontDistance();
  frontColor();
  ////// wait button
  while (!left_button.isPressed() && !right_button.isPressed());

  blank_luminosity = getLuminosity();
  bool debug_mode = true; // debug mode
  if (debug_mode) {
    Serial.println("iniciando debug");
    RescueProcess();
    debug();
  }
  //////////////////////////////////// MAIN part ///////////////////////////////////////////////////
  rightDistance();
  frontColor();
  delay(200);
  blank_luminosity = getLuminosity();
  Serial.println("calibrated blank_luminosity: " + String(blank_luminosity));
  while (true) {
    followWall(0, 0, 9);
    bool has_obstacle = frontColor() == "BLUE";
    if (has_obstacle) { // obstacle
      frontDistance(); delay(200);
      has_obstacle = has_obstacle && frontDistance() < 10;
      rightDistance();
      if (!has_obstacle) continue;

      Serial.println("obstacle ahead");
      both.turnDegree(-turnspeed, -90);
      alignBack();
      while (!hasFoundWall()) {
        followWall(0, 0, 11);
      }
      reachWall();
    } else if (hasFoundWall()) {
      both.turnDegree(-turnspeed, -90);//80
    } else if (isRescueArena()) {
      break;
    }
  }
  ////////////////////// start rescue//////////////////// ////
  both.stop();
  RescueProcess();
}

bool isRescueArena() {
  return frontColor() == "BLACK";
}

bool isCircle() {
  return (groundColor() == "RED");
}

void RescueProcess() {
  frontDistance(); frontColor(); delay(200);
  basespeed = 65;
  turnspeed = 70;
  goto debugBlock;
  both.turnDegree(-turnspeed, -90);
  alignBack();
  // take down door
  float back_rotations = 0;
  both.stop();
  motorR.stopCounting(); motorR.startCounting();
  while (frontDistance() < 30) {
    motorR.walk(basespeed);
    motorL.walk(basespeed);
    back_rotations = motorR.getRotations();
    Serial.println("back_rotations: " + String(back_rotations));
  }
  motorR.stopCounting();
  both.stop();
  both.together(-basespeed, -back_rotations);
  alignBack();
  Serial.println("back_rotations: " + String(back_rotations));
  both.together(basespeed, 4.8);
  both.turnDegree(turnspeed, 90);
debugBlock:
  // EDIT HERE
  float total_cube_distance = 9.5; // distancia total do cubo ate a parede (old 3.4);
  float increment_cube_distance = 7; // distance entre os cubos

  float search_distance = 22;
  float max_search_distance[99]; for (int i = 0; i <= 99; i++) max_search_distance[i] = search_distance;
  float max_vertical_distance = 90;
  float after_get_cube_distance = 58;

  for (int cube_id = 0; cube_id <= 99; cube_id++) {
    writeText(String(total_cube_distance));
    reachWall();
    adjustFrontDistance(basespeed, total_cube_distance);
    both.turnDegree(turnspeed, 90);
    frontColor();
    adjustFrontDistance(basespeed, 17); /////17.5/// distance until cube

    motorL.startCounting();
    float resultant_rotations = 0;
    while (frontColor() == "RED") {
      motorL.walk(-basespeed); motorR.walk(basespeed);
    }
    resultant_rotations += motorL.getRotations();
    writeText(String(resultant_rotations));
    motorL.stopCounting();
    motorL.startCounting();
    while (frontColor() != "RED") {
      motorL.walk(basespeed); motorR.walk(-basespeed);
    }
    resultant_rotations += motorL.getRotations();
    writeText(String(resultant_rotations));
    motorL.stopCounting();
    both.stop();
    float side = resultant_rotations / abs(resultant_rotations);

    float correction_degrees = 7;
    both.turnDegree(turnspeed, correction_degrees);

    while (getProximity() == 0) {
      motorR.walk(-basespeed);
      motorL.walk(-basespeed);
    }

    while (getProximity() != 0) {
      motorR.walk(50);
      motorL.walk(50);
    }

    both.together(-basespeed, -0.05);
    getCube(); delay(200);

    both.together(turnspeed*-side, abs(resultant_rotations)*-side, turnspeed*side, abs(resultant_rotations)*side);
    both.turnDegree(-turnspeed, -correction_degrees);
    adjustFrontDistance(basespeed, after_get_cube_distance);

    both.turnDegree(turnspeed, 90);
    alignBack(maxspeed); both.together(basespeed, 0.2);
    both.turnDegree(turnspeed, 90);


    bool deliver_cube = false;
    byte turns = 0;

    groundColor(); delay(200);
    while (true) { //search for circle
      writeText(String(max_search_distance[turns]));
      adjustFrontDistance(maxspeed, max_search_distance[turns]); // this distance will change
      while (frontDistance() < max_vertical_distance) {
        motorR.walk(-basespeed); motorL.walk(-basespeed);
        deliver_cube = isCircle();
        if (deliver_cube) {
          writeText("found circle");

          resultant_rotations = 0;
          
//          debugBlock:
//          groundColor(); delay(200);
        
          both.turnDegree(turnspeed, 90);
          while (!isCircle()) {
            motorL.walk(-basespeed);
            motorR.walk(-basespeed);
          }

          while (isCircle()) {
            motorL.walk(-basespeed);
            motorR.walk(-basespeed);
          }
          both.stop();

          while (!isCircle()) {
            motorL.walk(basespeed);
            motorR.walk(basespeed);
          }
          both.stop();

          both.together(basespeed, 0.6);

          motorL.startCounting();
          while (isCircle()) {
            motorL.walk(turnspeed);
            motorR.walk(-turnspeed);
          }
          resultant_rotations += motorL.getRotations();
          writeText(String(motorL.getRotations()));
          motorL.stopCounting();
          both.stop();

          motorL.startCounting();
          while (!isCircle()) {
            motorL.walk(-turnspeed);
            motorR.walk(turnspeed);
          }
          resultant_rotations += motorL.getRotations();
          writeText(String(motorL.getRotations()));
          motorL.stopCounting();
          both.stop();

          correction_degrees = 50;
          writeText(String(correction_degrees));
          back_rotations = 0.5;
          both.turnDegree(-turnspeed, -correction_degrees);
          both.together(-basespeed, -back_rotations);

          float resultant_degrees = (resultant_rotations / rot_per_degree);
          writeText(String(resultant_degrees));
          dropCube();
   
          
          float side = resultant_degrees / abs(resultant_degrees);
          float calculus = abs(abs(resultant_degrees) - abs(correction_degrees));
          writeText(String(calculus));
          both.turnDegree(turnspeed*-side, calculus*-side);
          debug();
          both.turnDegree(-turnspeed, -90);
          break;
        }
      }
      if (deliver_cube) { // deliver cube
        total_cube_distance += increment_cube_distance;
        const byte avoid_turns_left = 1; // muda a distancia maxima da parede x voltas pra a esquerda
        const byte avoid_turns_right = 2; // muda a distancia maxima da parede x voltas pra a direita
        for (int future = -avoid_turns_left; future <= avoid_turns_right; future++) {
          max_search_distance[turns + future] = frontDistance() + 17;
        }

        adjustFrontDistance(basespeed, max_vertical_distance);
        both.turnDegree(basespeed, 90); reachWall();
        break;
      }

      adjustFrontDistance(basespeed, max_vertical_distance);
      turns++;

      // if not found circle:
      both.turnDegree(-turnspeed, -90);
//      alignBack();
      const byte max_horizontal_distance = 95;
      const byte each_turn_distance = 11;
      adjustFrontDistance(basespeed, max_horizontal_distance - (turns * each_turn_distance));
      both.turnDegree(turnspeed, 90);
    }
  }
}


float frontDistance() {
  ServoDistance.write(90);
  return (readDistance() - 5.7);
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
  float distance = eye_lox.readRangeSingleMillimeters() / 10.0;
  if (distance == 819) distance = 200;
  return distance;
}

bool hasCube() {
  const float distance = arm_lox.readRangeSingleMillimeters() / 10.0;
  return (distance <= 6);
}

float degreeToRad(float degrees = 0) {
  return (PI * degrees) / 180;
}

void debug(String text = "") {
  Serial.println(text);
  both.stop();
  while (true) {
    delay(1000);
  }
}


void loop () {
}
