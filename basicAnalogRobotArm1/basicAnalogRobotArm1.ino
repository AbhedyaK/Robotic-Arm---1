#include <Servo.h>
//#include <SoftwareSerial.h>

/*
 * Wiring
 * Base Servo - Digital I/O 3
 * Shoulder Joint/Servo - Digital I/O 5
 * Elbow Joint - Digital I/O 6
 * Wrist Joint - Digital I/O 9 
 * Gripper Servo - Digital I/O 10
 * 
 * Digitalirect/Absolute :- Used for static positions such as knobs or sliders.
 * 
 * Incremental :- Used for joysticks that return to their initial position.
 * 
 * .writeMicroseconds() :- 600ms -> fully counter-clockwise
 * 2400ms -> fully clock-wise
 * 1500ms -> the servo being centered 
 * all of the above for the robot geek servos used.
 * 
 * .writeMicroseconds :- 900ms -> fully counter-clockwise
 * 2100ms -> fully clockwise
 * 1500ms -> servo center
 */

#define SERIAL_DEBUG
 #define rg_gripper

 #define BASE 0 //horizontal axis 1
 #define SHOULDER 1 //vertical axis 2 same joystick no -> 1
 #define ELBOW 2 //vertical axis 3
 #define WRIST 3 //vertical axis 4
 #define GRIPPER 4 //connected to knob.

#define BUTTON1 12
#define BUTTON2 13 

 #define minBase 600 //as specified above full CCW 180 Degrees
 #define maxBase 2400 //full CW 180 degrees
 #define minShoulder 600 
 #define maxShoulder 2400
 #define minElbow 600
 #define maxElbow 2400
 #define minWrist 600
 #define maxWrist 2400

 #define maxGripper 2400 //fully open
 #define minGripper 750 //fully closed

//deadband limits -> some joysticks don't exaclty center at 512, this removes the drift
//that are off center
 #define deadbandLow 482 
 #define deadbandHigh 542

 Servo baseServo;
 Servo shoulderServo;
 Servo elbowServo;
 Servo wristServo;
 Servo gripperServo;

 //Initial positions of the servos
 int Base = 1500; //center as mentioned above
 int Shoulder = 1500; //center as mentioned above
 int Elbow = 960;
 int Wrist = 600;
 int Gripper = 1000;

 //Servo offsets (Error) +/- uS. Although Not Necessary 
// #define baseError 0 
// #define shoulderError 0
// #define elbowError 0 
// #define wristError 0
// #define gripperError 0

//Native values for sensors from (0-1023)
//initializing these sensors
int joyBaseVal = 0;     //present value of the base rotation knob (analog 0)
int joyShoulderVal = 0; //present value of the shoulder joystick (analog 1)
int joyElbowVal = 0;    //present value of the elbow joystick (analog 2)
int joyWristVal = 0;    //present value of the wrist joystick (analog 3)
int joyGripperVal = 0;  //present value of the gripper rotation knob (analog 4)

//knob values (base and gripper) will be mapped directly to the servo limits
int joyBaseMapped = 0;      //base knob value, mapped from 1-1023 to BASE_MIN-BASE_MAX
int joyShoulderMapped = 0;  //shoulder joystick value, mapped from 1-1023 to -speed to speed
int joyElbowMapped = 0;     //elbow joystick value, mapped from 1-1023 to -speed to speed
int joyWristMapped = 0;     //wrist joystick value, mapped from 1-1023 to -speed to speed
int joyGripperMapped = 0;   //gripper knob  value, mapped from 1-1023 to GRIPPER_MIN-GRIPPER_MAX

int buttonState = 0;
int buttonState1 = 0;

int speed = 10; //speed of the movement

int serialTimer = 0;

void setup() {
  Serial.begin(38400);
  //attaching the servos to the pins on the geekDuino shield on PWM side.
  baseServo.attach(3, minBase, maxBase);
  shoulderServo.attach(5, minShoulder, maxShoulder);
  elbowServo.attach(11, minElbow, maxElbow);
  wristServo.attach(9, minWrist, maxWrist);
  gripperServo.attach(10, minGripper, maxGripper);

  delay(1000);

 // set_servo(); //moving servos to deafult positions .
}

void loop() {

  //reading alnalog values to the sensors/joysticks
  joyBaseVal = analogRead(BASE); //will read the intialzed value of the rotation joystick
  joyShoulderVal = analogRead(SHOULDER);
  joyElbowVal = analogRead(ELBOW);
  joyWristVal = analogRead(WRIST);
  joyGripperVal = analogRead(GRIPPER);
  
  if(joyBaseVal > deadbandHigh || joyBaseVal < deadbandLow) {
    //map functions(value, fromLow, fromHigh, toLow, toHigh)
    joyBaseMapped = map(joyBaseVal, 0, 1023, -speed, speed); 
    //add mapped base joystick value to present 
    //Base Value (positive values of joyBaseMapped
    //will increase the position, negative values will decrease the position)
    Base += joyBaseMapped;
    if(Base < minBase) {
  Base = minBase;
} else if (Base > maxBase) {
  Base = maxBase;
}
  }
  
//only if joystick is outside the deadband zone
if(joyShoulderVal > deadbandHigh || joyShoulderVal < deadbandLow) {
  joyShoulderMapped = map(joyShoulderVal, 0 , 1023, -speed, speed);
  Shoulder = Shoulder + joyShoulderMapped;
  //Shoulder = Shoulder - joyShoulderMapped;
  

if(Shoulder < minShoulder) {
  Shoulder = minShoulder;
} else if (Shoulder > maxShoulder) {
  Shoulder = maxShoulder;
}
}

if(joyElbowVal > deadbandHigh || joyElbowVal < deadbandLow) {
  joyElbowMapped = map(joyElbowVal, 0, 1023, -speed, speed);
  Elbow += joyElbowMapped;

  
if(Elbow < minElbow) {
  Elbow = minElbow;
}else if (Elbow > maxElbow) {
  Elbow = maxElbow;
}

}

if(joyWristVal > deadbandHigh || joyWristVal < deadbandLow) {
  joyWristMapped = map(joyWristVal, 0, 1023, -speed, speed);
  Wrist += joyWristMapped;

  if(Wrist < minWrist) {
  Wrist = minWrist;
}else if (Wrist > maxWrist) {
  Wrist = maxWrist;
}

}

//Mapping analog joystick value to servo PWM signal range;
//not checking for deadband because in this case the gripper
//is adjusted using a knob which is static after movement and will
//not return to the original position .
joyGripperMapped = map(joyGripperVal, 0, 1023, minGripper, maxGripper);
Gripper = joyGripperMapped; //setting servo position to 

//enforcing upper/lower values for servo positioning variables


if(Gripper < minGripper) {
  Gripper = minGripper;
}else if (Gripper > maxGripper) {
  Gripper = maxGripper;
}

set_servo();
  
#ifdef SERIAL_DEBUG
  serialprintout();
  serialTimer++;
  #endif

buttonState = digitalRead(BUTTON1);
buttonState1 = digitalRead(BUTTON2);

if(buttonState == HIGH) {
  automate();
  
}

if(buttonState1 == HIGH ){
  automate1();
}

}

void set_servo() {
  baseServo.writeMicroseconds(Base); //+ baseError);
  shoulderServo.writeMicroseconds(Shoulder); //+ shoulderError);
  elbowServo.writeMicroseconds(Elbow); //+ elbowError);
  wristServo.writeMicroseconds(Wrist); //+ wristError);
  gripperServo.writeMicroseconds(Gripper); //+ gripperError);

  delay(10); //delay for response
}

void serialprintout() {
  if(serialTimer == 50) {
    serialTimer = 0;
    Serial.println("Base Value: ");
    Serial.println(Base); //+ baseError);
    
    Serial.println("Shoulder Value: ");3
    Serial.println(Shoulder); //+ shoulderError);
    
    Serial.println("Elbow Value: ");
    Serial.println(Elbow );//+ elbowError);
    
    Serial.println("Wrist Value: ");
    Serial.println(Wrist );//+ wristError);
    
    Serial.println("Gripper Value: ");
    Serial.println(Gripper); //+ gripperError);
  }
}

void automate(){
   Base = 2400;
   delay(10);
   Gripper = 2400;
   delay(10);
   Shoulder = 1783;
   delay(10);
   Elbow = 1043;
   delay(10);
   Wrist = 805;
   delay(10);
}

void automate1(){
  Base = 1515;
  delay(10);
  Shoulder = 1647;
  delay(10);
  Elbow = 845;
  delay(10);
  Wrist = 886;
  delay(10);
  Gripper = 1600;
  delay(10);
}






